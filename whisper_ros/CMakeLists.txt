cmake_minimum_required(VERSION 3.8)
project(whisper_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(
  whisper_parameters
  src/parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/audio_buffer.cpp
  src/whisper_ros_component.cpp)

target_link_libraries(${PROJECT_NAME}_component
  whisper_parameters
)

rclcpp_components_register_nodes(${PROJECT_NAME}_component
 "whisper_ros::WhisperRosComponent")

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
