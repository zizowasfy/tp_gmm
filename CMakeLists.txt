cmake_minimum_required(VERSION 3.8)
project(tp_gmm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gaussian.msg"
  "msg/GaussianMixture.msg"
  "srv/StartTPGMM.srv"
  "srv/ReproduceTPGMM.srv"
  "srv/DeformTPGMM.srv"
  "srv/PrepareSampling.srv"
  "srv/SamplingReport.srv"
  DEPENDENCIES geometry_msgs std_msgs moveit_msgs
)

# C++ Executables and Libraries
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

add_library(gmm_rviz_converter SHARED scripts/gmm_rviz_converter.cpp)
target_compile_features(gmm_rviz_converter PUBLIC c_std_99 cxx_std_17)
target_include_directories(gmm_rviz_converter PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/scripts>
)
ament_target_dependencies(gmm_rviz_converter
  rclcpp
  rclcpp_components
  geometry_msgs
  visualization_msgs
  std_msgs
)
rosidl_target_interfaces(gmm_rviz_converter ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_node(gmm_rviz_converter PLUGIN "tp_gmm::GMMRvizConverter" EXECUTABLE gmm_rviz_converter_node)

add_library(gmm_moveit_lib SHARED scripts/gmm_moveit_node.cpp)
target_compile_features(gmm_moveit_lib PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(gmm_moveit_lib
  rclcpp
  rclcpp_components
  geometry_msgs
  visualization_msgs
  shape_msgs
  moveit_msgs
)
rclcpp_components_register_node(gmm_moveit_lib PLUGIN "tp_gmm::GMMMoveIt" EXECUTABLE gmm_moveit_node)

add_library(gmm_constraint_sampler SHARED src/gmm_constraint_sampler.cpp)
target_compile_features(gmm_constraint_sampler PUBLIC cxx_std_17)
ament_target_dependencies(gmm_constraint_sampler rclcpp pluginlib moveit_core moveit_ros_planning
  moveit_msgs geometry_msgs visualization_msgs shape_msgs)
rosidl_target_interfaces(gmm_constraint_sampler ${PROJECT_NAME} "rosidl_typesupport_cpp")
pluginlib_export_plugin_description_file(moveit_core gmm_constraint_sampler_plugin_description.xml)

# Install Python scripts and modules
install(PROGRAMS
  scripts/tp_gmm_node.py
  scripts/collect_trajectories.py
  scripts/collect_trajectories_gen3_ros2.py
  scripts/demons_to_samples.py
  scripts/validate_policy.py
  scripts/run_experiments.py
  scripts/sampling_client.py
  scripts/compare_sampling_approaches.py
  scripts/plot_sampling_comparison.py
  scripts/sampling_fidelity.py
  scripts/standalone_policy.py
  scripts/dir_paths.py
  scripts/visualize_demonstrations.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ nodes/libraries
install(TARGETS
  gmm_constraint_sampler
  gmm_rviz_converter
  gmm_rviz_converter_node
  gmm_moveit_lib
  gmm_moveit_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES SAMPLING.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tasks DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  rclcpp_components
  geometry_msgs
  visualization_msgs
  shape_msgs
  std_msgs
  moveit_msgs
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_sampling_math tests/test_sampling_math.cpp)
  target_compile_features(test_sampling_math PRIVATE cxx_std_17)
  target_compile_options(test_sampling_math PRIVATE -UNDEBUG)
  add_test(NAME sampling_math COMMAND test_sampling_math)
endif()

ament_package()
