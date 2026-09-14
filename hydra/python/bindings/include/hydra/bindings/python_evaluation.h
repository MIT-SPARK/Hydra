#pragma once
#include <pybind11/pybind11.h>

namespace hydra::python::python_evaluation {
void addBindings(pybind11::module_& module);
}  // namespace hydra::python::python_evaluation
