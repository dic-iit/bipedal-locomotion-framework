/**
 * @file CoMTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_COM_TASK_H
#define BIPEDAL_LOCOMOTION_IK_COM_TASK_H

#include <memory>

#include <manif/manif.h>

#include <BipedalLocomotion/IK/Task.h>

#include <LieGroupControllers/ProportionalController.h>

namespace BipedalLocomotion
{
namespace IK
{

/**
 * SE3Task is a concrete implementation of the Task. Please use this element if you
 * want to control the position and the orientation of a given frame rigidly attached to the robot.
 * The task assumes perfect control of the robot velocity \f$\nu\f$ that contains the base
 * linear and angular velocity expressed in mixed representation and the joints velocity.
 * The task represents the following equation
 * \f[
 * J \nu = \mathrm{v} ^ *
 * \f]
 * where \f$J\f$ is the robot jacobian and \f$\mathrm{v} ^ *\f$ is the desired velocity.
 * The desired velocity is chosen such that the frame will asymptotically converge to the
 * desired trajectory. The linear component of \f$\mathrm{v} ^ *\f$ is computed with a
 * standard Proportional controller in \f$R^3\f$ while the angular velocity is computed
 * by a Proportional controller in \f$SO(3)\f$.
 * @note Please refer to https://github.com/dic-iit/lie-group-controllers if you are interested in
 * the implementation of the PD controllers.
 * @note The SE3Task is technically not a \f$SE(3)\f$ space defined task, instead is a \f$SO(3)
 * \times \mathbb{R}^3\f$ task. Theoretically, there are differences between the two due to the
 * different definitions of exponential maps and logarithm maps. Please consider that here the MIXED
 * representation is used to define the 6d-velocity. You can find further details in Section 2.3.4
 * of https://traversaro.github.io/phd-thesis/traversaro-phd-thesis.pdf.
 */
class CoMTask : public Task
{
    LieGroupControllers::ProportionalControllerR3d m_R3Controller; /**< P Controller in R3 */

    System::VariablesHandler::VariableDescription m_robotVelocityVariable; /**< Variable
                                                                              describing the robot
                                                                              velocity (base +
                                                                              joint) */

    static constexpr std::size_t m_linearVelocitySize{3}; /**< Size of the linear velocity vector. */
    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector. */

    bool m_isInitialized{false}; /**< True if the task has been initialized. */

public:

    /**
     * Initialize the task.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                      | Mandatory |
     * |:----------------------------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * |   `robot_velocity_variable_name`   | `string` |   Name of the variable contained in `VariablesHandler` describing the robot velocity   |    Yes    |
     * |             `kp_linear`            | `double` |                             Gain of the position controller                            |    Yes    |
     * @return True in case of success, false otherwise.
     * Where the generalized robot velocity is a vector containing the base spatial-velocity
     * (expressed in mixed representation) and the joints velocity.
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler) override;

    /**
     * Set the set of variables required by the task. The variables are stored in the
     * System::VariablesHandler.
     * @param variablesHandler reference to a variables handler.
     * @note The handler must contain a variable named as the parameter
     * `robot_velocity_variable_name` stored in the parameter handler. The variable represents the
     * generalized velocity of the robot. Where the generalized robot velocity is a vector
     * containing the base spatial-velocity (expressed in mixed representation) and the joints
     * velocity.
     * @return True in case of success, false otherwise.
     */
    bool setVariableHandler(const System::VariablesHandler& variablesHandler) override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set the desired set-point of the trajectory.
     * @param position position of the CoM
     * @param velocity velocity of the CoM expressed in mixed representation
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(Eigen::Ref<const Eigen::Vector3d> position, Eigen::Ref<const Eigen::Vector3d> velocity);

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The CoMTask is an equality task.
     * @return the size of the task.
     */
    Type type() const override;
};

} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_SE3_TASK_H
