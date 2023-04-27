cmake_minimum_required(VERSION 3.8)
project(sas_robot_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(pybind11 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_msgs REQUIRED)
find_package(sas_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

##### LIBRARY #####

add_library(${PROJECT_NAME} SHARED
    src/sas_robot_kinematics_interface.cpp
    src/sas_robot_kinematics_provider.cpp
    )

ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs std_msgs sas_msgs sas_common sas_core sas_conversions)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp geometry_msgs std_msgs sas_common sas_msgs sas_core sas_conversions)

target_link_libraries(${PROJECT_NAME}
    -ldqrobotics
    Eigen3::Eigen
    )

install(
    DIRECTORY include/
    DESTINATION include
    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    #ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

##END## LIBRARY #####
