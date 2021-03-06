# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


if(FRAMEWORK_COMPILE_System)

  set(H_PREFIX include/BipedalLocomotion/System)

  # set target name
  add_bipedal_locomotion_library(
    NAME                   System
    PUBLIC_HEADERS         ${H_PREFIX}/Advanceable.h ${H_PREFIX}/VariablesHandler.h ${H_PREFIX}/DynamicalSystem.h ${H_PREFIX}/FloatingBaseSystemDynamics.h ${H_PREFIX}/DynamicalSystem.tpp ${H_PREFIX}/ContactWrench.h ${H_PREFIX}/LinearTimeInvariantSystem.h ${H_PREFIX}/Integrator.h ${H_PREFIX}/Integrator.tpp ${H_PREFIX}/FixedStepIntegrator.h ${H_PREFIX}/FixedStepIntegrator.tpp ${H_PREFIX}/Integrator.tpp ${H_PREFIX}/ForwardEuler.h ${H_PREFIX}/ForwardEuler.tpp ${H_PREFIX}/FloatingBaseSystemKinematics.h
    SOURCES                src/FloatingBaseSystemDynamics.cpp src/VariablesHandler.cpp src/ContactWrench.cpp src/LinearTimeInvariantSystem.cpp src/FloatingBaseSystemKinematics.cpp
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::ContactModels iDynTree::idyntree-high-level iDynTree::idyntree-model Eigen3::Eigen
    PRIVATE_LINK_LIBRARIES  BipedalLocomotion::CommonConversions BipedalLocomotion::Math
    SUBDIRECTORIES         tests
    )

endif()
