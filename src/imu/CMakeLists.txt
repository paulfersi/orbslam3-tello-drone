cmake_minimum_required(VERSION 3.5)
project(imu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tello_msgs REQUIRED) 

add_executable(imu
  src/imu_node.cpp
)
ament_target_dependencies(imu rclcpp sensor_msgs tello_msgs)


install(TARGETS imu DESTINATION lib/${PROJECT_NAME})

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
