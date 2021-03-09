/**
 * @file Task.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/IK/Task.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

iDynTree::MatrixView<double> Task::subA(const VariablesHandler::VariableDescription& description)
{
    return iDynTree::MatrixView<double>(m_A).block(0, description.offset, m_A.rows(), description.size);
}

iDynTree::MatrixView<const double>
Task::subA(const VariablesHandler::VariableDescription& description) const
{
    return iDynTree::MatrixView<const double>(m_A).block(0, description.offset, m_A.rows(), description.size);
}

bool Task::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    m_kinDyn = kinDyn;
    return (m_kinDyn != nullptr) && (m_kinDyn->isValid());
}

bool Task::setVariableHandler(const VariablesHandler& variablesHandler)
{
    return true;
}

bool Task::initialize(std::weak_ptr<IParametersHandler> paramHandler)
{
    return true;
}

bool Task::update()
{
    return true;
}

Eigen::Ref<const Eigen::MatrixXd> Task::getA() const
{
    return m_A;
}

Eigen::Ref<const Eigen::VectorXd> Task::getB() const
{
    return m_b;
}

const std::string& Task::getDescription() const
{
    return m_description;
}
