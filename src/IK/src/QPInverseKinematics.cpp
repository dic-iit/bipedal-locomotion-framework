/**
 * @file QPInverseKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <memory>

#include <OsqpEigen/OsqpEigen.h>

#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::IK;

struct QPInverseKinematics::Impl
{
    struct TaskWithPriority
    {
        std::shared_ptr<Task> task;
        std::size_t priority;
        Eigen::VectorXd weight;
        Eigen::MatrixXd tmp; /**< This is a temporary matrix usefull to reduce dynamics allocation
                                in advance method */
    };

    IKState solution;

    System::VariablesHandler::VariableDescription robotVelocityVariable;

    OsqpEigen::Solver solver; /**< Optimization solver. */

    std::unordered_map<std::string, TaskWithPriority> tasks;

    std::vector<std::reference_wrapper<const TaskWithPriority>> constraints;
    std::vector<std::reference_wrapper<TaskWithPriority>> costs;

    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
    Eigen::MatrixXd constraintMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    bool isFirstIteration{true};
    bool isValid{false};
    bool isInitialized{false};

    bool initializeSolver()
    {
        constexpr auto logPrefix = "[QPInversekinematics::Impl::initializeSolver]";
        // Hessian matrix
        Eigen::SparseMatrix<double> hessianSparse = this->hessian.sparseView();
        if (!this->solver.data()->setHessianMatrix(hessianSparse))
        {
            log()->error("{} Unable to set the hessian matrix.", logPrefix);
            return false;
        }

        // gradient
        if (!this->solver.data()->setGradient(gradient))
        {
            log()->error("{} Unable to set the gradient vector.", logPrefix);
            return false;
        }

        Eigen::SparseMatrix<double> constraintsMatrixSparse = this->constraintMatrix.sparseView();
        if (!this->solver.data()->setLinearConstraintsMatrix(constraintsMatrixSparse))
        {
            log()->error("{} Unable to set the constraint matrix.", logPrefix);
            return false;
        }

        if (!this->solver.data()->setBounds(this->lowerBound, this->upperBound))
        {
            log()->error("{} Unable to set the bounds.", logPrefix);
            return false;
        }

        if (!this->solver.initSolver())
        {
            log()->error("{} Unable to initialize the solver.", logPrefix);
            return false;
        }

        return true;
    }

    bool updateSolver()
    {
        constexpr auto logPrefix = "[QPInversekinematics::Impl::updateSolver]";
        // Hessian matrix
        Eigen::SparseMatrix<double> hessianSparse = this->hessian.sparseView();
        if (!this->solver.updateHessianMatrix(hessianSparse))
        {
            log()->error("{} Unable to set the hessian matrix.", logPrefix);
            return false;
        }

        // gradient
        if (!this->solver.updateGradient(gradient))
        {
            log()->error("{} Unable to set the gradient vector.", logPrefix);
            return false;
        }

        Eigen::SparseMatrix<double> constraintsMatrixSparse = this->constraintMatrix.sparseView();
        if (!this->solver.updateLinearConstraintsMatrix(constraintsMatrixSparse))
        {
            log()->error("{} Unable to set the constraint matrix.", logPrefix);
            return false;
        }

        if (!this->solver.updateBounds(this->lowerBound, this->upperBound))
        {
            log()->error("{} Unable to set the bounds.", logPrefix);
            return false;
        }

        return true;
    }
};

QPInverseKinematics::QPInverseKinematics()
{
    m_pimpl = std::make_unique<QPInverseKinematics::Impl>();
}

QPInverseKinematics::~QPInverseKinematics() = default;

bool QPInverseKinematics::addTask(std::shared_ptr<Task> task,
                                  const std::string& taskName,
                                  std::size_t priority,
                                  std::optional<Eigen::Ref<const Eigen::VectorXd>> weight)
{
    constexpr auto logPrefix = "[QPInverseKinematics::addTask]";

    // check if the task already exist
    const bool taskExist = (m_pimpl->tasks.find(taskName) != m_pimpl->tasks.end());
    if (taskExist)
    {
        log()->error("{} The task named {} already exist.", logPrefix, taskName);
        return false;
    }

    if (priority != 0 && priority != 1)
    {
        log()->error("{} At the time being we support only priority equal to 0 or 1.", logPrefix);
        return false;
    }

    if (priority == 1 && !weight)
    {
        log()->error("{} In case of priority equal to 1 the weight is mandatory.", logPrefix);
        return false;
    }

    if (weight)
    {
        if (weight.value().size() != task->size())
        {
            log()->error("{} The size of the weight is not coherent with the size of the task. "
                         "Expected: {}. Given: {}.",
                         logPrefix,
                         m_pimpl->tasks[taskName].task->size(),
                         weight.value().size());
            return false;
        }
        m_pimpl->tasks[taskName].weight = weight.value();
    }

    // take the right task
    m_pimpl->tasks[taskName].task = task;
    m_pimpl->tasks[taskName].priority = priority;

    if (priority == 0)
    {
        m_pimpl->constraints.push_back(m_pimpl->tasks[taskName]);
    } else
    {
        m_pimpl->costs.push_back(m_pimpl->tasks[taskName]);
    }

    return true;
}

bool QPInverseKinematics::finalize(const System::VariablesHandler& handler)
{
    constexpr auto logPrefix = "[QPInverseKinematics::finalize]";

    // set the variable handler for all the tasks
    std::size_t numberOfConstraints = 0;
    for (auto& [name, task] : m_pimpl->tasks)
    {
        if (!task.task->setVariableHandler(handler))
        {
            log()->error("{} Error while setting the variable handler in the task named {}, having "
                         "the following description {}.",
                         logPrefix,
                         name,
                         task.task->getDescription());
            return false;
        }

        // if priority is equal to 0 the task is considered as hard constraint.
        if (task.priority == 0)
        {
            numberOfConstraints += task.task->size();
        }
    }

    // resize the temporary matrix usefull to reduce dynamics allocation when advance() is called
    for (auto& cost : m_pimpl->costs)
    {
        cost.get().tmp.resize(handler.getNumberOfVariables(), cost.get().weight.size());
    }

    m_pimpl->solver.data()->setNumberOfVariables(handler.getNumberOfVariables());
    m_pimpl->solver.data()->setNumberOfConstraints(numberOfConstraints);

    // resize matrices
    m_pimpl->hessian.resize(handler.getNumberOfVariables(), handler.getNumberOfVariables());
    m_pimpl->gradient.resize(handler.getNumberOfVariables());

    m_pimpl->constraintMatrix.resize(numberOfConstraints, handler.getNumberOfVariables());
    m_pimpl->upperBound.resize(numberOfConstraints);
    m_pimpl->lowerBound.resize(numberOfConstraints);

    if (!handler.getVariable(m_pimpl->robotVelocityVariable.name,
                             m_pimpl->robotVelocityVariable))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", logPrefix);
        return false;
    }

    return true;
}

std::vector<std::string> QPInverseKinematics::getTaskNames() const
{
    std::vector<std::string> tasksName;

    for (const auto& [name, task] : m_pimpl->tasks)
    {
        tasksName.push_back(name);
    }

    return tasksName;
}

bool QPInverseKinematics::advance()
{
    constexpr auto logPrefix = "[QPInverseKinematics::advance]";

    // when advance is called the previous solution is no more valid
    m_pimpl->isValid = false;

    if (!m_pimpl->isInitialized)
    {
        log()->error("{} Please call initialize() before advance().", logPrefix);
        return false;
    }

    // update of all the tasks
    for (auto& [name, task] : m_pimpl->tasks)
    {
        task.task->update();
    }

    // Compute the gradient and the hessian
    m_pimpl->hessian.setZero();
    m_pimpl->gradient.setZero();
    for (auto& cost : m_pimpl->costs)
    {
        Eigen::Ref<const Eigen::MatrixXd> A = cost.get().task->getA();
        Eigen::Ref<const Eigen::VectorXd> b = cost.get().task->getB();

        // Here we avoid to have dynamic allocation
        cost.get().tmp.noalias() = A.transpose() * cost.get().weight.asDiagonal();
        m_pimpl->hessian.noalias() += cost.get().tmp * A;
        m_pimpl->gradient.noalias() -= cost.get().tmp * b;
    }

    // compute the constraints
    std::size_t index = 0;
    for (const auto& constraint : m_pimpl->constraints)
    {
        Eigen::Ref<const Eigen::MatrixXd> A = constraint.get().task->getA();
        Eigen::Ref<const Eigen::VectorXd> b = constraint.get().task->getB();

        m_pimpl->constraintMatrix.middleRows(index, constraint.get().task->size()) = A;
        m_pimpl->upperBound.segment(index, constraint.get().task->size()) = b;

        if (constraint.get().task->type() == Task::Type::inequality)
        {
            m_pimpl->lowerBound.segment(index, constraint.get().task->size())
                .setConstant(-OsqpEigen::INFTY);
        } else
        {
            m_pimpl->lowerBound.segment(index, constraint.get().task->size()) = b;
        }

        index += constraint.get().task->size();
    }

    // update the solver
    if (!m_pimpl->isFirstIteration)
    {
        if (!m_pimpl->updateSolver())
        {
            log()->error("{} Unable to update the QP solver.", logPrefix);
            return false;
        }
    } else
    {
        if (!m_pimpl->initializeSolver())
        {
            log()->error("{} Unable to run the QP solver the first time.", logPrefix);
            return false;
        }
        m_pimpl->isFirstIteration = false;
    }

    // solve the QP
    if (!m_pimpl->solver.solve())
    {
        log()->error("{} Unable to to solve the problem.", logPrefix);
        return false;
    }

    // retrieve the solution
    constexpr std::size_t spatialVelocitySize = 6;
    const std::size_t joints = m_pimpl->robotVelocityVariable.size - spatialVelocitySize;

    m_pimpl->solution.jointVelocity
        = m_pimpl->solver.getSolution().segment(m_pimpl->robotVelocityVariable.offset
                                                    + spatialVelocitySize,
                                                joints);

    m_pimpl->solution.baseVelocity.coeffs()
        = m_pimpl->solver.getSolution().segment<spatialVelocitySize>(
            m_pimpl->robotVelocityVariable.offset);

    m_pimpl->isValid = true;

    return true;
}

bool QPInverseKinematics::isValid() const
{
    return m_pimpl->isValid;
}

const IKState& QPInverseKinematics::get() const
{
    return m_pimpl->solution;
}

std::weak_ptr<Task> QPInverseKinematics::getTask(const std::string& name) const
{
    constexpr auto logPrefix = "[QPInverseKinematics::getTask]";
    auto task = m_pimpl->tasks.find(name);

    if (task == m_pimpl->tasks.end())
    {
        log()->warn("{} The task named {} does not exist. A nullptr will be returned.",
                    logPrefix,
                    name);
        return std::shared_ptr<Task>(nullptr);
    }

    return task->second.task;
}

bool QPInverseKinematics::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[QPInverseKinematics::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("robot_velocity_variable_name", m_pimpl->robotVelocityVariable.name))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", logPrefix);
        return false;
    }

    bool isVerbose = false;
    if (!ptr->getParameter("verbosity", isVerbose))
    {
        log()->info("{} 'verbosity' not found. The following parameter will be used '{}'.",
                    logPrefix,
                    isVerbose);
    }

    // set the some internal parameter of osqp-eigen
    m_pimpl->solver.settings()->setVerbosity(isVerbose);

    m_pimpl->isInitialized = true;

    return true;
}
