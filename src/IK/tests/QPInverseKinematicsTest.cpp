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

#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/IK/CoMTask.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/ModelTestUtils.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;
using namespace BipedalLocomotion::Conversions;

constexpr auto robotVelocity = "robotVelocity";

struct InverseKinematicsAndTasks
{
    std::shared_ptr<IntegrationBasedIK> ik;
    std::shared_ptr<SE3Task> se3Task;
    std::shared_ptr<CoMTask> comTask;
    std::shared_ptr<JointTrackingTask> regularizationTask;
};

std::shared_ptr<IParametersHandler> createParameterHandler()
{
    auto parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("robot_velocity_variable_name",
                                   robotVelocity);

    parameterHandler->setParameter("verbosity", true);

    /////// SE3 Task
    auto SE3ParameterHandler = std::make_shared<StdImplementation>();
    SE3ParameterHandler->setParameter("robot_velocity_variable_name",
                                      robotVelocity);

    SE3ParameterHandler->setParameter("kp_linear", 1.0);
    SE3ParameterHandler->setParameter("kp_angular", 1.0);

    parameterHandler->setGroup("SE3_TASK",  SE3ParameterHandler);

    /////// CoM task
    auto CoMParameterHandler = std::make_shared<StdImplementation>();
    CoMParameterHandler->setParameter("robot_velocity_variable_name",
                                      robotVelocity);

    CoMParameterHandler->setParameter("kp_linear", 1.0);
    parameterHandler->setGroup("COM_TASK",  CoMParameterHandler);

    auto jointRegularizationHandler = std::make_shared<StdImplementation>();
    jointRegularizationHandler->setParameter("robot_velocity_variable_name",
                                             robotVelocity);
    parameterHandler->setGroup("REGULARIZATION_TASK",  jointRegularizationHandler);

    return parameterHandler;
}

InverseKinematicsAndTasks createIK(std::shared_ptr<IParametersHandler> handler,
                                   std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                   const VariablesHandler& variables)
{
    InverseKinematicsAndTasks out;

    constexpr auto priority = 0;

    out.ik = std::make_shared<QPInverseKinematics>();
    REQUIRE(out.ik->initialize(handler));

    out.se3Task = std::make_shared<SE3Task>();
    REQUIRE(out.se3Task->setKinDyn(kinDyn));
    REQUIRE(out.se3Task->initialize(handler->getGroup("SE3_TASK")));
    REQUIRE(out.ik->addTask(out.se3Task, "se3_task", priority));

    out.comTask = std::make_shared<CoMTask>();
    REQUIRE(out.comTask->setKinDyn(kinDyn));
    REQUIRE(out.comTask->initialize(handler->getGroup("COM_TASK")));
    REQUIRE(out.ik->addTask(out.comTask, "com_task", priority));

    out.regularizationTask = std::make_shared<JointTrackingTask>();

    Eigen::VectorXd weight;
    REQUIRE(handler->getGroup("REGULARIZATION_TASK").lock()->getParameter("weight", weight));

    REQUIRE(out.regularizationTask->setKinDyn(kinDyn));
    REQUIRE(out.regularizationTask->initialize(handler->getGroup("REGULARIZATION_TASK")));
    REQUIRE(out.ik->addTask(out.regularizationTask, "regularization_task", 1, weight));

    REQUIRE(out.ik->finalize(variables));

    return out;
}

TEST_CASE("QP-IK")
{

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = createParameterHandler();

    constexpr double tolerance = 1e-3;

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    for (std::size_t numberOfJoints = 9; numberOfJoints < 50; numberOfJoints += 15)
    {
        DYNAMIC_SECTION("Model with " << numberOfJoints << " joints")
        {
            // create the model
            const iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints);
            REQUIRE(kinDyn->loadRobotModel(model));

            const auto worldBasePos = iDynTree::getRandomTransform();
            const auto baseVel = iDynTree::getRandomTwist();
            iDynTree::VectorDynSize jointsPos(model.getNrOfDOFs());
            iDynTree::VectorDynSize jointsVel(model.getNrOfDOFs());
            iDynTree::Vector3 gravity;

            for (auto& joint : jointsPos)
            {
                joint = iDynTree::getRandomDouble();
            }

            for (auto& joint : jointsVel)
            {
                joint = iDynTree::getRandomDouble();
            }

            for (auto& element : gravity)
            {
                element = iDynTree::getRandomDouble();
            }

            REQUIRE(kinDyn->setRobotState(worldBasePos, jointsPos, baseVel, jointsVel, gravity));

            // Instantiate the handler
            VariablesHandler variablesHandler;
            variablesHandler.addVariable(robotVelocity, model.getNrOfDOFs() + 6);


            // Set the frame name
            const std::string controlledFrame = model.getFrameName(numberOfJoints);
            parameterHandler->getGroup("SE3_TASK").lock()->setParameter("frame_name", controlledFrame);

            const Eigen::VectorXd kpRegularization = Eigen::VectorXd::Ones(model.getNrOfDOFs());
            const Eigen::VectorXd weightRegularization = Eigen::VectorXd::Ones(model.getNrOfDOFs());
            parameterHandler->getGroup("REGULARIZATION_TASK").lock()->setParameter("kp", kpRegularization);
            parameterHandler->getGroup("REGULARIZATION_TASK").lock()->setParameter("weight", weightRegularization);

            auto ikAndTasks = createIK(parameterHandler, kinDyn, variablesHandler);

            // getCoMPosition and velocity
            const Eigen::Vector3d desiredCoMPosition = iDynTree::toEigen(kinDyn->getCenterOfMassPosition());
            const Eigen::Vector3d desiredCoMVelocity = iDynTree::toEigen(kinDyn->getCenterOfMassVelocity());

            const manif::SE3d desiredPose = toManifPose(kinDyn->getWorldTransform(controlledFrame));
            const manif::SE3d::Tangent desiredVelocity = iDynTree::toEigen(kinDyn->getFrameVel(controlledFrame));


            REQUIRE(ikAndTasks.se3Task->setSetPoint(desiredPose, desiredVelocity));
            REQUIRE(ikAndTasks.comTask->setSetPoint(desiredCoMPosition, desiredCoMVelocity));
            REQUIRE(ikAndTasks.regularizationTask->setSetPoint(iDynTree::toEigen(jointsPos),
                                                               iDynTree::toEigen(jointsVel)));

            REQUIRE(ikAndTasks.ik->advance());

            // check that everything went fine
            REQUIRE(kinDyn->setRobotState(worldBasePos.asHomogeneousTransform(),
                                          jointsPos,
                                          ikAndTasks.ik->get().baseVelocity.coeffs(),
                                          ikAndTasks.ik->get().jointVelocity,
                                          gravity));

            REQUIRE(ikAndTasks.ik->get().jointVelocity.isApprox(iDynTree::toEigen(jointsVel), tolerance));
            REQUIRE(desiredCoMVelocity.isApprox(iDynTree::toEigen(kinDyn->getCenterOfMassVelocity()), tolerance));
            REQUIRE(desiredVelocity.isApprox(iDynTree::toEigen(kinDyn->getFrameVel(controlledFrame)), tolerance));
        }
    }
}
