/**
 * @file SE3Task.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

bool SE3Task::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[SE3Task::setVariablesHandler] The task is not initialized. Please call "
                     "initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotVelocityVariable.name,
                                      m_robotVelocityVariable))
    {
        log()->error("[SE3Task::setVariablesHandler] Unable to get the variable named {}.",
                     m_robotVelocityVariable.name);
        return false;
    }

    // get the variable
    if (m_robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[SE3Task::setVariablesHandler] The size of the robot velocity variable is "
                     "different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotVelocityVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_spatialVelocitySize, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_spatialVelocitySize);

    return true;
}

bool SE3Task::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr std::string_view errorPrefix = "[SE3Task::initialize] ";

    std::string frameName = "Unknown";
    constexpr std::string_view descriptionPrefix = "SE3Task Optimal Control Element - Frame name: ";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{}, [{} {}] KinDynComputations object is not valid.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);

        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        log()->error("{}, [{} {}] The task supports only quantities expressed in MIXED "
                     "representation. Please provide a KinDynComputations with Frame velocity "
                     "representation set to MIXED_REPRESENTATION.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{}, [{} {}] The parameter handler is not valid.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    std::string robotVelocityVariableName;
    if (!ptr->getParameter("robot_velocity_variable_name", m_robotVelocityVariable.name))
    {
        log()->error("{}, [{} {}] Error while retrieving the robot velocity variable.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (!ptr->getParameter("frame_name", frameName)
        || (m_frameIndex = m_kinDyn->model().getFrameIndex(frameName))
               == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{}, [{} {}] Error while retrieving the frame that should be controlled.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    // set the gains for the controllers
    double kpLinear;
    double kpAngular;
    if (!ptr->getParameter("kp_linear", kpLinear))
    {
        log()->error("{}, [{} {}] Unable to get the proportional linear gain.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (!ptr->getParameter("kp_angular", kpAngular))
    {
        log()->error("{}, [{} {}] Unable to get the proportional angular gain.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    m_R3Controller.setGains(kpLinear);
    m_SO3Controller.setGains(kpAngular);

    // set the description
    m_description = std::string(descriptionPrefix) + frameName + ".";

    m_isInitialized = true;

    return true;
}

bool SE3Task::update()
{
    using namespace BipedalLocomotion::Conversions;
    using namespace iDynTree;

    // set the state
    m_SO3Controller.setState(toManifRot(m_kinDyn->getWorldTransform(m_frameIndex).getRotation()));
    m_R3Controller.setState(toEigen(m_kinDyn->getWorldTransform(m_frameIndex).getPosition()));

    // update the controller
    m_SO3Controller.computeControlLaw();
    m_R3Controller.computeControlLaw();

    m_b.head<3>() = m_R3Controller.getControl().coeffs();
    m_b.tail<3>() = m_SO3Controller.getControl().coeffs();

    if (!m_kinDyn->getFrameFreeFloatingJacobian(m_frameIndex,
                                                this->subA(m_robotVelocityVariable)))
    {
        log()->error("[SE3Task::update] Unable to get the jacobian.");
        return false;
    }

    return true;
}

bool SE3Task::setSetPoint(const manif::SE3d& I_H_F, const manif::SE3d::Tangent& mixedVelocity)
{

    bool ok = true;
    ok = ok && m_R3Controller.setDesiredState(I_H_F.translation());
    ok = ok && m_R3Controller.setFeedForward(mixedVelocity.v());

    ok = ok && m_SO3Controller.setDesiredState(I_H_F.quat());
    ok = ok && m_SO3Controller.setFeedForward(mixedVelocity.w());

    return ok;
}

std::size_t SE3Task::size() const
{
    return m_spatialVelocitySize;
}

SE3Task::Type SE3Task::type() const
{
    return Type::equality;
}
