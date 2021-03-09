# Changelog
All notable changes to this project are documented in this file.

## [Unreleased]
### Added
- Implement IRobotControl python bindings (https://github.com/dic-iit/bipedal-locomotion-framework/pull/200)
- Implement ISensorBridge python bindings (https://github.com/dic-iit/bipedal-locomotion-framework/pull/203)
- Implement `LeggedOdometry` class as a part of `FloatingBaseEstimators` library and handle arbitrary contacts in `FloatingBaseEstimator`. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/151)
- Implement the possibility to set a desired reference trajectory in the TimeVaryingDCMPlanner. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/208)
- Implement SchmittTriggerDetector python bindings (https://github.com/dic-iit/bipedal-locomotion-framework/pull/213)
- Implement ModelComputationsHelper for quick construction of KinDynComputations object using parameters handler (https://github.com/dic-iit/bipedal-locomotion-framework/pull/216)
- Implement FloatingBaseEstimator and LeggedOdometry python bindings (https://github.com/dic-iit/bipedal-locomotion-framework/pull/218)
- Add spdlog as mandatory dependency of the project (https://github.com/dic-iit/bipedal-locomotion-framework/pull/225)
- Implement the inverse kinematics component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/229)

### Changed
- Move all the Contacts related classes in Contacts component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/204)
- Move all the ContactDetectors related classes in Contacts component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/209)
- The DCMPlanner and TimeVaryingDCMPlanner initialize functions take as input an std::weak_ptr. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/208)
- Use `Math::StandardAccelerationOfGravitation` instead of hardcoding 9.81. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/211)
- Convert iDynTree types in FloatingBaseEstimators component to Eigen/manif types (https://github.com/dic-iit/bipedal-locomotion-framework/pull/215)
- Use std::optional instead of raw pointer in ISensorBridge. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/226)

### Fixed
- Fix missing implementation of `YarpSensorBridge::getFailedSensorReads()`. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/202)
- Fixed `mas-imu-test` configuration files after FW fix.

## [0.1.0] - 2021-02-22
### Added
- The `CHANGELOG.md` file
- The `cmake/BipedalLocomotionControllersFindDepencies.cmake` file
- The `cmake/AddInstallRPATHSupport.cmake` file
- The `cmake/AddUninstallTarget.cmake` file
- The `cmake/FindEigen3.cmake` file
- The `cmake/InstallBasicPackageFiles.cmake` file
- Implement the first version of the `BipedalLocomotionControllers` interface
- Implement the  first version of the `YarpUtilities` library
- Implement `ParametersHandler` library (https://github.com/dic-iit/bipedal-locomotion-controllers/pull/13)
- Implement `GenericContainer::Vector` (https://github.com/dic-iit/bipedal-locomotion-controllers/pull/29)
- Implement `Estimators` library (https://github.com/dic-iit/bipedal-locomotion-controllers/pull/23)
- Implement `Contact` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/43 and https://github.com/dic-iit/bipedal-locomotion-framework/pull/45)
- Implement the first version of the `TimeVaryingDCMPlanner` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/61)
- Implement the Quintic Spline class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/83)
- Implement the `ConvexHullHelper` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/51)
- Implement the `DynamicalSystem` and `Integrator` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/46)
- Implement the `IRobotControl` interface and the YARP specialization (https://github.com/dic-iit/bipedal-locomotion-framework/pull/97, https://github.com/dic-iit/bipedal-locomotion-framework/pull/192)
- Add `SensorBridge` interface (https://github.com/dic-iit/bipedal-locomotion-framework/pull/87)
- Add the `YarpSensorBridge` Implementation (https://github.com/dic-iit/bipedal-locomotion-framework/pull/106)
- Added `CommonConversions`, `ManifConversions`, and `matioCppConversions` libraries to handle type conversions. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/138 and https://github.com/dic-iit/bipedal-locomotion-framework/pull/143)
- Implement the `JointPositionTracking` application. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/136)
- Initial implementation of Python bindings using pybind11 (https://github.com/dic-iit/bipedal-locomotion-framework/pull/134)
- Implement `FloatingBaseEstimatorDevice` YARP device for wrapping floating base estimation algorithms. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/130)
- Implement Continuous algebraic Riccati equation function (https://github.com/dic-iit/bipedal-locomotion-framework/pull/157)
- Implement YARP based `ROSPublisher` in the `YarpUtilities` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/156)
- Implement example YARP device `ROSPublisherTestDevice` for understanding the usage of `ROSPublisher`. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/160)
- Implement `TSID` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/167, https://github.com/dic-iit/bipedal-locomotion-framework/pull/170, https://github.com/dic-iit/bipedal-locomotion-framework/pull/178)
- Implement the `JointTrajectoryPlayer` application. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/169)29ed234a1c
- Implement `ContactDetectors` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/142)
- Added `mas-imu-test` application to check the output of MAS IMUs (https://github.com/dic-iit/bipedal-locomotion-framework/pull/62)
- Implement motor currents reading in `YarpSensorBridge`. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/187)

[Unreleased]: https://github.com/dic-iit/bipedal-locomotion-framework/compare/v0.1.0...master
[0.1.0]: https://github.com/dic-iit/bipedal-locomotion-framework/releases/tag/v0.1.0
