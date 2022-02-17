cmake_minimum_required(VERSION 3.5)
project(indayv_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(indyav_interfaces REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(xacro REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include  
)

#catkin_package(
#INCLUDE_DIRS include
#LIBRARIES backwheel_plugin
#CATKIN_DEPENDS indyav_control_generate_messages gazebo_ros_pkgs rclcpp  
#)

#add_executable(backwheel_plugin src/backwheel_plugin.cpp)
#ament_target_dependencies(backwheel_plugin rclcpp indyav_interfaces)     # CHANGE
#target_include_directories(backwheel_plugin PUBLIC
#	  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#	    $<INSTALL_INTERFACE:include>)

add_library(backwheel_plugin SHARED
	src/backwheel_plugin.cpp)
ament_target_dependencies(backwheel_plugin rclcpp indyav_interfaces)
target_link_libraries(backwheel_plugin
	${ament_cmake_LIBRARIES}
	${sensor_msgs_LIBRARIES}
	${std_msgs_LIBRARIES}
	${GAZEBO_LIBRARIES} 
	${rclcpp_LIBRARIES} 
)
install(TARGETS backwheel_plugin
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin)

ament_export_libraries(backwheel_plugin)
#add_library(
#  backwheel_plugin
#  src/backwheel_plugin.cpp  
#)
#target_link_libraries(
#  backwheel_plugin
#  ${catkin_LIBRARIES}  
#)

#ament_target_dependencies(backwheel_plugin rclcpp indyav_interfaces)

#add_dependencies(backwheel_plugin
  #include/wheel_plugin.hpp
  #  indyav_interfaces
  # ${catkin_EXPORTED_TARGETS}
  #)


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
