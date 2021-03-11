/**
 * @file bipedal_locomotion.h
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <Eigen/Core>
#include <manif/SE3.h>
#include <pybind11/pybind11.h>

namespace BipedalLocomotion::Contacts
{
class PlannedContact;
}


namespace BipedalLocomotion::bindings
{
// Custom formatter of Eigen vectors
const Eigen::IOFormat FormatEigenVector //
    (Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

// Conversions from custom classes to string
std::string ToString(const manif::SE3d& se3);
std::string ToString(const Contacts::PlannedContact& contact);

// BaseTypes.cpp
void CreateBaseTypes(pybind11::module& module);

// QuinticSpline.cpp
void CreateQuinticSpline(pybind11::module& module);

// ParametersHandler.cpp
void CreateIParameterHandler(pybind11::module& module);
void CreateStdParameterHandler(pybind11::module& module);

// Contacts.cpp
void CreateContact(pybind11::module& module);
void CreateContactList(pybind11::module& module);
void CreateContactPhase(pybind11::module& module);
void CreateContactPhaseList(pybind11::module& module);

// SwingFootPlanner.cpp
void CreateSwingFootPlanner(pybind11::module& module);

// DCMPlanner.cpp
void CreateDCMPlanner(pybind11::module& module);

// TimeVaryingDCMPlanner.cpp
void CreateTimeVaryingDCMPlanner(pybind11::module& module);

// RobotInterface.cpp
void CreatePolyDriver(pybind11::module& module);
void CreatePolyDriverDescriptor(pybind11::module& module);
void CreateIRobotControl(pybind11::module& module);
void CreateYarpRobotControl(pybind11::module& module);
void CreateISensorBridge(pybind11::module& module);
void CreateYarpSensorBridge(pybind11::module& module);

// Constants.cpp
void CreateConstants(pybind11::module& module);

// ContactDetectors.cpp
void CreateContactDetector(pybind11::module& module);
void CreateSchmittTriggerUnit(pybind11::module& module);
void CreateSchmittTriggerDetector(pybind11::module& module);

// FloatingBaseEstimators.cpp
void CreateKinDynComputations(pybind11::module& module);
void CreateKinDynComputationsDescriptor(pybind11::module& module);
void CreateFloatingBaseEstimator(pybind11::module& module);
void CreateLeggedOdometry(pybind11::module& module);

// VariablesHandler.cpp
void CreateVariablesHandler(pybind11::module& module);

} // namespace BipedalLocomotion::bindings
