cmake_minimum_required(VERSION 3.5)
project(ros_gz_gazebo_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER 6)

# ros node
add_executable(gz_gazebo_manager src/gz_gazebo_manager.cpp)

target_link_libraries(gz_gazebo_manager
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)

ament_target_dependencies(gz_gazebo_manager
    rclcpp 
    geometry_msgs
    ros_gz_interfaces
    ros_gz_bridge
)

# Install executables
install(TARGETS gz_gazebo_manager
  DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install python
ament_python_install_package(${PROJECT_NAME})

#Install executable scripts
install(PROGRAMS 
        scripts/gz_demo.py
    DESTINATION lib/${PROJECT_NAME})

ament_package()
