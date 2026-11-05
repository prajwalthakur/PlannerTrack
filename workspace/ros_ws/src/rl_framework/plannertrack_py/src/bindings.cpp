/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

#include "plannertrack_py/sim.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

/** \file
 * \brief pybind11 module \c plannertrack_py._core.
 */

namespace py = pybind11;

namespace plannertrack_py
{
// raceline_bindings.cpp
void bindRaceline(py::module_ & m);
}  // namespace plannertrack_py

PYBIND11_MODULE(_core, m)
{
	m.doc() =
	    "PlannerTrack simulator core (VehicleModelFactory/AgentModel) and minimum-curvature "
	    "raceline, without ROS.";
	plannertrack_py::bindRaceline(m);

	py::class_<plannertrack_py::Sim>(m, "Sim",
	    "Single-agent simulator stepped from Python; deterministic, no ROS graph.")
	    .def(py::init<const std::string &, const std::string &>(), py::arg("sim_yaml"),
	        py::arg("agent_yaml"),
	        "Build from the text of a sim.yaml and of one agent's node in agents.yaml.")
	    .def("reset", &plannertrack_py::Sim::reset,
	        "Return to the initial state configured in dynamics_params.")
	    .def("set_state", &plannertrack_py::Sim::setState, py::arg("state"),
	        "Overwrite the state vector (size nx).")
	    .def("get_state", &plannertrack_py::Sim::getState, "Copy of the current state vector.")
	    .def("step", &plannertrack_py::Sim::step, py::arg("u"), py::arg("n") = 1,
	        py::call_guard<py::gil_scoped_release>(),
	        "Hold input u (size nu) for n steps of dt; returns the new state.")
	    .def("xdot", &plannertrack_py::Sim::xdot, py::arg("state"), py::arg("u"),
	        "Continuous-time derivative dx/dt = f(state, u) of the dynamics plugin "
	        "(the function the simulator integrates); does not change the state.")
	    .def_property_readonly("dt", &plannertrack_py::Sim::dt, "Simulator step [s].")
	    .def_property_readonly("nx", &plannertrack_py::Sim::nx, "State dimension.")
	    .def_property_readonly("nu", &plannertrack_py::Sim::nu, "Input dimension.")
	    .def_property_readonly(
	        "dynamics_plugin", &plannertrack_py::Sim::dynamicsPlugin, "Loaded dynamics plugin name.");
}
