/*
# Copyright (c) 2016-2022 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver.
#
#    sas_robot_driver is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@g.ecc.u-tokyo.ac.jp
#
# ################################################################*/
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <sas_robot_kinematics/sas_robot_kinematics_interface.h>
#include <sas_robot_kinematics/sas_robot_kinematics_provider.h>

namespace py = pybind11;
using RKI = sas::RobotKinematicsInterface;
using RKP = sas::RobotKinematicsProvider;

PYBIND11_MODULE(_sas_robot_kinematics, m) {

    py::class_<RKI>(m, "RobotKinematicsInterface")
            .def(py::init<const std::string&>())
            .def("is_enabled",&RKI::is_enabled)
            .def("get_pose",&RKI::get_pose)
            .def("get_reference_frame",&RKI::get_reference_frame)
            .def("send_desired_pose",&RKI::send_desired_pose)
            .def("send_desired_interpolator_speed",&RKI::send_desired_interpolator_speed);

    py::class_<RKP>(m, "RobotKinematicsProvider")
            .def(py::init<const std::string&>())
            .def("get_desired_pose",&RKP::get_desired_pose)
            .def("get_desired_interpolator_speed",&RKP::get_desired_interpolator_speed)
            .def("is_enabled",&RKP::is_enabled)
            .def("send_pose",&RKP::send_pose)
            .def("send_reference_frame",&RKP::send_reference_frame);

}
