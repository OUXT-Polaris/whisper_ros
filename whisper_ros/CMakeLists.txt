cmake_minimum_required(VERSION 3.8)
project(whisper_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#add_definitions(DR_WAV_IMPLEMENTATION)

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

ament_target_dependencies(${PROJECT_NAME}_component whisper_cpp_vendor)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_audio_buffer test/src/test_audio_buffer.cpp)
  target_link_libraries(test_audio_buffer ${PROJECT_NAME}_component)
endif()

ament_auto_package()
