/**
 * @file JointsTrackingTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_JOINT_TRACKING_TASK_H
#define BIPEDAL_LOCOMOTION_IK_JOINT_TRACKING_TASK_H

#include <memory>

#include <BipedalLocomotion/IK/Task.h>

#include <LieGroupControllers/ProportionalDerivativeController.h>

namespace BipedalLocomotion
{
namespace IK
{

/**
 * JointsTrackingTask is a concrete implementation of the Task. Please use this element if you
 * want to control the desired joint position of the robot.
 * The task represents the following equation
 * \f[
 * \begin{bmatrix} 0_6 & I_n \end{bmatrix} \nu = \dot{s} ^ *
 * \f]
 * where \f$0_6\f$ and \f$I_n\f$ are the zero and the identity matrix.
 * The desired joint velocity is chosen such that the joint will converge to the desired
 * trajectory and it is computed with a standard standard PD controller in \f$\mathbb{R}^n\f$.
 */
class JointsTrackingTask : public Task
{
    Eigen::VectorXd m_kp; /**< Proportional gain. */
    Eigen::VectorXd m_jointPosition; /**< Joints position in radians */
    Eigen::VectorXd m_desiredJointVelocity; /**< Desired joints velocity in radians per second. */
    Eigen::VectorXd m_desiredJointPosition; /**< Desired joints position in radians. */
    Eigen::VectorXd m_zero; /**< Vector containing zero elements. */

    std::string m_robotVelocityVariableName;

    bool m_isInitialized{false};
public:
    /**
     * Initialize the planner.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                      | Mandatory |
     * |:----------------------------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * |    `robot_velocity_variable_name`  | `string` |   Name of the variable contained in `VariablesHandler` describing the robot velocity   |    Yes    |
     * |             `kp       `            | `vector` |                                Proportional controller gain                            |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler);

    /**
     * Set the variable required by the task
     * @param variablesHandler reference to a variables handler.
     * @return True in case of success, false otherwise.
     */
    bool setVariableHandler(const System::VariablesHandler& variablesHandler) override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set the desired setpoint.
     * @param jointPosition vector containing the desired joint position in radians.
     * @note The desired velocity is implicitly set to zero.
     * @return True in case of success, false otherwise.
     */
    bool setSetpoint(Eigen::Ref<const Eigen::VectorXd> jointPosition);

    /**
     * Set the desired setpoint.
     * @param jointPosition vector containing the desired joint position in radians.
     * @param jointVelocity vector containing the desired joint velocity in radians per second.
     * @return True in case of success, false otherwise.
     */
    bool setSetpoint(Eigen::Ref<const Eigen::VectorXd> jointPosition,
                     Eigen::Ref<const Eigen::VectorXd> jointVelocity);

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;
};

} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_JOINT_TRACKING_TASK_H
