/**
 * @file JointsTrackingTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/IK/JointsTrackingTask.h>

#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/Core/EigenHelpers.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::IK;

bool JointsTrackingTask::setVariableHandler(const System::VariablesHandler& variablesHandler)
{
    constexpr auto errorPrefix = "[JointsTrackingTask::setVariableHandler]";

    System::VariablesHandler::VariableDescription robotVelocityVariable;

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize method.", errorPrefix);
        return false;
    }


    if (!variablesHandler.getVariable(m_robotVelocityVariableName, robotVelocityVariable))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", errorPrefix);
        return false;
    }

    if (robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + 6)
    {
        log()->error("{} The size of the robot velocity variable does not match with the one "
                     "stored in kinDynComputations object. Expected: {}. Given: {}",
                     errorPrefix,
                     m_kinDyn->getNrOfDegreesOfFreedom() + 6,
                     robotVelocityVariable.size);
        return false;
    }


    // resize the matrices
    m_A.resize(m_kinDyn->getNrOfDegreesOfFreedom(), variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_kinDyn->getNrOfDegreesOfFreedom());

    // A is constant
    // here we assume that the first robot velocity is stored as [base_velocity;
    // joint_velocity]
    iDynTree::toEigen(this->subA(robotVelocityVariable))
        .rightCols(m_kinDyn->getNrOfDegreesOfFreedom())
        .setIdentity();

    return true;
}

bool JointsTrackingTask::initialize(
    std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[JointsTrackingTask::initialize] ";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{} KinDynComputations object is not valid.", errorPrefix);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("robot_velocity_variable_name", m_robotVelocityVariableName))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", errorPrefix);
        return false;
    }

    // set the gains for the controllers
    m_kp.resize(m_kinDyn->getNrOfDegreesOfFreedom());
    if (!ptr->getParameter("kp", m_kp))
    {
        log()->error("{} Error while retrieving the proportional gain.", errorPrefix);
        return false;
    }

    // set the description
    m_description = "Joint tracking task";

    m_zero = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_desiredJointPosition = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_desiredJointVelocity = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());
    m_jointPosition = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());

    m_isInitialized = true;

    return true;
}

bool JointsTrackingTask::update()
{
    constexpr auto errorPrefix = "[JointsTrackingTask::update]";

    if (!m_kinDyn->getJointPos(m_jointPosition))
    {
        log()->error("{} Unable to get the joint position.", errorPrefix);
        return false;
    }

    m_b = m_desiredJointVelocity + m_kp.asDiagonal() * (m_desiredJointPosition - m_jointPosition);

    return true;
}

bool JointsTrackingTask::setSetpoint(Eigen::Ref<const Eigen::VectorXd> jointPosition)
{
    return this->setSetpoint(jointPosition, m_zero);
}

bool JointsTrackingTask::setSetpoint(Eigen::Ref<const Eigen::VectorXd> jointPosition,
                                     Eigen::Ref<const Eigen::VectorXd> jointVelocity)
{
    constexpr auto errorPrefix = "[JointsTrackingTask::setSetpoint]";

    if (jointPosition.size() != m_kinDyn->getNrOfDegreesOfFreedom()
        || jointVelocity.size() != m_kinDyn->getNrOfDegreesOfFreedom())
    {
        log()->error("{} Wrong size of the desired reference setpoint. Expected size: {}. Given "
                     "joint position size: {}, given joint velocity size: {}.",
                     errorPrefix,
                     m_kinDyn->getNrOfDegreesOfFreedom(),
                     jointPosition.size(),
                     jointVelocity.size());
        return false;
    }

    m_desiredJointPosition = jointPosition;
    m_desiredJointVelocity = jointVelocity;

    return true;
}

std::size_t JointsTrackingTask::size() const
{
    if (m_kinDyn == nullptr)
    {
        log()->warn("[JointsTrackingTask::size] Please call setKinDyn method before. A size equal "
                    "to zero will be returned.");
        return 0;
    }
    return m_kinDyn->getNrOfDegreesOfFreedom();
}
