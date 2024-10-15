cmake_minimum_required(VERSION 3.5)
project(drone_info_gui)

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

set(CMAKE_INCLUDE_CURRENT_DIR)
set(CMAKE_AUTOMOC ON)

# Emulate colcon
if ($ENV{CLION_IDE})
  message(STATUS "Running inside CLion")
  set(tello_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/tello_msgs/share/tello_msgs/cmake")
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(rclcpp REQUIRED)
find_package(tello_msgs REQUIRED)

set(DRONE_INFO_GUI_DEPS tello_msgs)

add_library(${PROJECT_NAME} src/drone_info_gui_node)

include_directories(include ${rclcpp_INCLUDE_DIRS})
add_executable(drone_info_gui_node src/drone_info_gui_node.cpp)
target_link_libraries(drone_info_gui Qt5::Widgets Qt5::Core ${DRIVER_NODE_LIBS})
target_link_libraries(drone_info_gui_node Qt5::Widgets Qt5::Core ${DRIVER_NODE_LIBS})

ament_target_dependencies(drone_info_gui rclcpp tello_msgs)
ament_target_dependencies(drone_info_gui_node rclcpp tello_msgs)



install(DIRECTORY 
    launch 
    DESTINATION share/${PROJECT_NAME}/
)

# Install executables
install(
  TARGETS drone_info_gui_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
