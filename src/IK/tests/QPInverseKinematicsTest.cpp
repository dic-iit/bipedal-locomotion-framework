/**
 * @file QPInverseKinematicsTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

// BipedalLocomotion
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/IK/CoMTask.h>
#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>

#include <BipedalLocomotion/System/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/System/ForwardEuler.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/ModelTestUtils.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;
using namespace BipedalLocomotion::Conversions;

constexpr auto robotVelocity = "robotVelocity";
constexpr double dT = 0.01;

struct InverseKinematicsAndTasks
{
    std::shared_ptr<IntegrationBasedIK> ik;
    std::shared_ptr<SE3Task> se3Task;
    std::shared_ptr<CoMTask> comTask;
    std::shared_ptr<JointTrackingTask> regularizationTask;
};

struct DesiredSetPoints
{
    Eigen::Vector3d CoMPosition;
    manif::SE3d endEffectorPose;
    Eigen::VectorXd joints;
};

struct System
{
    std::shared_ptr<ForwardEuler<FloatingBaseSystemKinematics>> integrator;
    std::shared_ptr<FloatingBaseSystemKinematics> dynamics;
};

std::shared_ptr<IParametersHandler> createParameterHandler()
{
    constexpr double gain = 5.0;

    auto parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);

    parameterHandler->setParameter("verbosity", false);

    /////// SE3 Task
    auto SE3ParameterHandler = std::make_shared<StdImplementation>();
    SE3ParameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);

    SE3ParameterHandler->setParameter("kp_linear", gain);
    SE3ParameterHandler->setParameter("kp_angular", gain);
    parameterHandler->setGroup("SE3_TASK", SE3ParameterHandler);

    /////// CoM task
    auto CoMParameterHandler = std::make_shared<StdImplementation>();
    CoMParameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    CoMParameterHandler->setParameter("kp_linear", gain);
    parameterHandler->setGroup("COM_TASK", CoMParameterHandler);

    /////// Joint regularization task
    auto jointRegularizationHandler = std::make_shared<StdImplementation>();
    jointRegularizationHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    parameterHandler->setGroup("REGULARIZATION_TASK", jointRegularizationHandler);

    return parameterHandler;
}

InverseKinematicsAndTasks createIK(std::shared_ptr<IParametersHandler> handler,
                                   std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                   const VariablesHandler& variables)
{
    // prepare the parameters related to the size of the system
    const Eigen::VectorXd kpRegularization = Eigen::VectorXd::Ones(kinDyn->model().getNrOfDOFs());
    const Eigen::VectorXd weightRegularization = kpRegularization;
    handler->getGroup("REGULARIZATION_TASK").lock()->setParameter("kp", kpRegularization);

    InverseKinematicsAndTasks out;

    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;

    out.ik = std::make_shared<QPInverseKinematics>();
    REQUIRE(out.ik->initialize(handler));

    out.se3Task = std::make_shared<SE3Task>();
    REQUIRE(out.se3Task->setKinDyn(kinDyn));
    REQUIRE(out.se3Task->initialize(handler->getGroup("SE3_TASK")));
    REQUIRE(out.ik->addTask(out.se3Task, "se3_task", highPriority));

    out.comTask = std::make_shared<CoMTask>();
    REQUIRE(out.comTask->setKinDyn(kinDyn));
    REQUIRE(out.comTask->initialize(handler->getGroup("COM_TASK")));
    REQUIRE(out.ik->addTask(out.comTask, "com_task", highPriority));

    out.regularizationTask = std::make_shared<JointTrackingTask>();


    REQUIRE(out.regularizationTask->setKinDyn(kinDyn));
    REQUIRE(out.regularizationTask->initialize(handler->getGroup("REGULARIZATION_TASK")));
    REQUIRE(out.ik->addTask(out.regularizationTask,
                            "regularization_task",
                            lowPriority,
                            weightRegularization));

    REQUIRE(out.ik->finalize(variables));

    return out;
}

DesiredSetPoints getDesiredReference(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                     std::size_t numberOfJoints)
{
    DesiredSetPoints out;

    const auto worldBasePos = iDynTree::getRandomTransform();
    const auto baseVel = iDynTree::Twist::Zero();

    iDynTree::VectorDynSize jointsPos(kinDyn->model().getNrOfDOFs());
    iDynTree::VectorDynSize jointsVel(kinDyn->model().getNrOfDOFs());
    iDynTree::Vector3 gravity;

    for (auto& joint : jointsPos)
    {
        joint = iDynTree::getRandomDouble();
    }

    for (auto& joint : jointsVel)
    {
        joint = 0;
    }

    for (auto& element : gravity)
    {
        element = iDynTree::getRandomDouble();
    }

    REQUIRE(kinDyn->setRobotState(worldBasePos, jointsPos, baseVel, jointsVel, gravity));

    // getCoMPosition and velocity
    out.CoMPosition = iDynTree::toEigen(kinDyn->getCenterOfMassPosition());
    const std::string controlledFrame = kinDyn->model().getFrameName(numberOfJoints);
    out.endEffectorPose = toManifPose(kinDyn->getWorldTransform(controlledFrame));
    out.joints.resize(jointsPos.size());
    out.joints = iDynTree::toEigen(jointsPos);

    return out;
}

System getSystem(int NrOfDOFs)
{
    System out;

    // create the System
    Eigen::Matrix<double, 6, 1> twist;
    twist.setZero();
    Eigen::VectorXd jointVelocity(NrOfDOFs);
    jointVelocity.setZero();
    Eigen::Matrix3d baseRotation;
    baseRotation.setIdentity();
    Eigen::Vector3d basePosition;
    basePosition.setZero();
    Eigen::VectorXd jointPosition(NrOfDOFs);
    jointPosition.setZero();

    out.dynamics = std::make_shared<FloatingBaseSystemKinematics>();
    out.dynamics->setState({basePosition, baseRotation, jointPosition});

    out.integrator = std::make_shared<ForwardEuler<FloatingBaseSystemKinematics>>(dT);
    out.integrator->setDynamicalSystem(out.dynamics);

    return out;
}

TEST_CASE("QP-IK")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = createParameterHandler();

    constexpr double tolerance = 1e-2;

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(
        iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    for (std::size_t numberOfJoints = 20; numberOfJoints < 60; numberOfJoints += 15)
    {
        DYNAMIC_SECTION("Model with " << numberOfJoints << " joints")
        {
            // create the model
            const iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints);
            REQUIRE(kinDyn->loadRobotModel(model));

            const auto desiredSetPoints = getDesiredReference(kinDyn, numberOfJoints);

            // Instantiate the handler
            VariablesHandler variablesHandler;
            variablesHandler.addVariable(robotVelocity, model.getNrOfDOFs() + 6);

            auto system = getSystem(model.getNrOfDOFs());

            // Set the frame name
            const std::string controlledFrame = model.getFrameName(numberOfJoints);
            parameterHandler->getGroup("SE3_TASK")
                .lock()
                ->setParameter("frame_name", controlledFrame);

            // create the IK
            auto ikAndTasks = createIK(parameterHandler, kinDyn, variablesHandler);

            REQUIRE(ikAndTasks.se3Task->setSetPoint(desiredSetPoints.endEffectorPose,
                                                    manif::SE3d::Tangent::Zero()));
            REQUIRE(ikAndTasks.comTask->setSetPoint(desiredSetPoints.CoMPosition,
                                                    Eigen::Vector3d::Zero()));
            REQUIRE(ikAndTasks.regularizationTask->setSetPoint(desiredSetPoints.joints));

            // propagate the inverse kinematics for
            constexpr std::size_t iterations = 1200;
            Eigen::Vector3d gravity;
            gravity << 0, 0, -9.81;
            Eigen::Matrix4d baseTransform;
            baseTransform.setZero();
            Eigen::Matrix<double, 6, 1> baseVelocity;
            baseVelocity.setZero();
            Eigen::VectorXd jointVelocity(model.getNrOfDOFs());
            jointVelocity.setZero();

            for (std::size_t iteration = 0; iteration < iterations; iteration++)
            {
                // get the solution of the integrator
                const auto& [basePosition, baseRotation, jointPosition]
                    = system.integrator->getSolution();

                // update the KinDynComputations object
                baseTransform.topLeftCorner<3, 3>() = baseRotation;
                baseTransform.topRightCorner<3, 1>() = basePosition;
                REQUIRE(kinDyn->setRobotState(baseTransform,
                                              jointPosition,
                                              baseVelocity,
                                              jointVelocity,
                                              gravity));

                // solve the IK
                REQUIRE(ikAndTasks.ik->advance());

                // get the output of the IK
                baseVelocity = ikAndTasks.ik->get().baseVelocity.coeffs();
                jointVelocity = ikAndTasks.ik->get().jointVelocity;

                // propagate the dynamical system
                system.dynamics->setControlInput({baseVelocity, jointVelocity});
                system.integrator->integrate(0, dT);
            }

            // check the CoM position
            REQUIRE(desiredSetPoints.CoMPosition.isApprox(toEigen(kinDyn->getCenterOfMassPosition()),
                                                          tolerance));

            // Check the end-effector pose error
            const manif::SE3d endEffectorPose  = toManifPose(kinDyn->getWorldTransform(controlledFrame));

            // please read it as (log(desiredSetPoints.endEffectorPose^-1 * endEffectorPose))^v
            const manif::SE3d::Tangent error = endEffectorPose - desiredSetPoints.endEffectorPose;
            REQUIRE(error.coeffs().isZero(tolerance));
        }
    }
}
