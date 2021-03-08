/**
 * @file InverseKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/IK/InverseKinematics.h>

using namespace BipedalLocomotion::IK;

bool InverseKinematics::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    return true;
};
