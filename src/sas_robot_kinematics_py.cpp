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
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <rclcpp/rclcpp.hpp>

#include <sas_robot_kinematics/sas_robot_kinematics_interface.h>
#include <sas_robot_kinematics/sas_robot_kinematics_provider.h>

namespace py = pybind11;
using RKC = sas::RobotKinematicsClient;
using RKS = sas::RobotKinematicsServer;

PYBIND11_MODULE(_sas_robot_kinematics, m) {

    py::class_<RKC>(m, "RobotKinematicsClient")
            .def(py::init<const std::shared_ptr<rclcpp::Node>&,const std::string&>())
            .def("is_enabled",&RKC::is_enabled)
            .def("get_pose",&RKC::get_pose)
            .def("get_reference_frame",&RKC::get_reference_frame)
            .def("send_desired_pose",&RKC::send_desired_pose)
            .def("send_desired_interpolator_speed",&RKC::send_desired_interpolator_speed);

    py::class_<RKS>(m, "RobotKinematicsServer")
            .def(py::init<const std::shared_ptr<rclcpp::Node>&,const std::string&>())
            .def("get_desired_pose",&RKS::get_desired_pose)
            .def("get_desired_interpolator_speed",&RKS::get_desired_interpolator_speed)
            .def("is_enabled",&RKS::is_enabled)
            .def("send_pose",&RKS::send_pose)
            .def("send_reference_frame",&RKS::send_reference_frame);

}
