
// This file was automatically generated by autogenu-jupyter (https://github.com/ohtsukalab/autogenu-jupyter). 
// The autogenu-jupyter copyright holders make no ownership claim of its contents. 

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "cgmres/multiple_shooting_cgmres_solver.hpp"
#include "cgmres/python/multiple_shooting_cgmres_solver.hpp"
#include "ocp.hpp"

#include <iostream>
#include <stdexcept>

namespace cgmres {
namespace python {

namespace py = pybind11;

constexpr int N = 50;
constexpr int kmax = 5;
DEFINE_PYBIND11_MODULE_MULTIPLE_SHOOTING_CGMRES_SOLVER(OCP_generated_code, N, kmax)


} // namespace python
} // namespace cgmres
