/**
 * @file InverseKinematics.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_INVERSE_KINEMATICS_H
#define BIPEDAL_LOCOMOTION_IK_INVERSE_KINEMATICS_H

#include <memory>
#include <string>
#include <optional>

#include <Eigen/Dense>
#include <manif/SE3.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/IK/Task.h>

namespace BipedalLocomotion
{

namespace IK
{

/**
 * State of the InverseKinematics
 */
struct IKState
{
    Eigen::VectorXd jointVelocity; /**< Joints velocity in rad per seconds */
    manif::SE3d::Tangent baseVelocity; /**< Mixed spatial velocity of the base */
};

/**
 * InverseKinematics implements the interface for the inverse kinematics. Please inherits this class
 * if you want to implement your custom IK.
 */
class InverseKinematics : public BipedalLocomotion::System::Advanceable<IKState>
{

public:
    /**
     * Initialize the planner.
     * @param handler pointer to the parameter handler.
     * @return true in case of success/false otherwise.
     */
    virtual bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler);

    /**
     * Add a task in the inverse kinematics
     * @param task pointer to a given task
     * @param priority is the priority of the task. The lower the number the higher the priority.
     * @param weight weight associated to the task.
     * @return true if the task has been added to the inverse kinematics.
     */
    virtual bool addTask(std::shared_ptr<Task> task,
                         const std::string& taskName,
                         std::size_t priority,
                         std::optional<Eigen::Ref<const Eigen::VectorXd>> weight = {}) = 0;

    /**
     * Get a vector containing the name of the tasks.
     * @return an std::vector containing all the names associated to the tasks
     */
    virtual std::vector<std::string> getTaskNames() const = 0;

    /**
     * Finalize the IK.
     * @param handler parameter handler.
     * @note You should call this method after you add ALL the tasks.
     * @return true in case of success, false otherwise.
     */
    virtual bool finalize(const System::VariablesHandler& handler) = 0;

    /**
     * Get a specific task
     * @param name name associated to the task.
     * @return a weak ptr associated to an existing task in the IK. If the task does not exist a
     * nullptr is returned.
     */
    virtual std::weak_ptr<Task> getTask(const std::string& name) const = 0;

    /**
     * Destructor.
     */
    virtual ~InverseKinematics() = default;
};
} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_INVERSE_KINEMATICS_H
