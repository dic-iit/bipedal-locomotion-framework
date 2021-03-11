/**
 * @file VariablesHandler.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/VariablesHandler.h>

#include "bipedal_locomotion_framework.h"

void BipedalLocomotion::bindings::CreateVariablesHandler(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    py::class_<VariablesHandler> variablesHandler(module, "VariablesHandler");

    py::class_<VariablesHandler::VariableDescription>(module, "VariableDescription")
        .def(py::init())
        .def_readonly("offset", &VariablesHandler::VariableDescription::offset)
        .def_readonly("size", &VariablesHandler::VariableDescription::size)
        .def_readonly("name", &VariablesHandler::VariableDescription::name)
        .def("is_valid", &VariablesHandler::VariableDescription::isValid)
        .def_static("invalid_variable", &VariablesHandler::VariableDescription::InvalidVariable);


    variablesHandler.def(py::init())
        .def("add_variable", &VariablesHandler::addVariable, py::arg("name"), py::arg("size"))
        .def("get_variable",
             py::overload_cast<const std::string&>(&VariablesHandler::getVariable, py::const_),
             py::arg("name"))
        .def("get_number_of_variables", &VariablesHandler::getNumberOfVariables)
        .def("__str__", &VariablesHandler::toString);


}
