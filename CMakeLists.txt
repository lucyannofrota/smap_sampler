cmake_minimum_required(VERSION 3.5)
project(smap_sampler)

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
find_package(geometry_msgs REQUIRED)
find_package(smap_interfaces REQUIRED)

find_package(tf2 REQUIRED)
#find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
#find_package(orocos_kdl REQUIRED)
#find_library(OROCOS_KDL orocos-kdl)
#set(orocos_kdl_LIBRARIES ${OROCOS_KDL})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(smap_sampler_node src/smap_sampler_node.cpp)

target_include_directories(smap_sampler_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  smap_sampler_node
  rclcpp
  tf2
  #tf2_ros
  #tf2_msgs
  tf2_sensor_msgs
  sensor_msgs
  geometry_msgs
  smap_interfaces
)

#target_link_libraries(smap_sampler_node  ${orocos_kdl_LIBRARIES})

install(TARGETS smap_sampler_node
  DESTINATION lib/${PROJECT_NAME})

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

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
