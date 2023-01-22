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

execute_process(
  COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/models
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

macro(download_model MODEL_TYPE)
  execute_process(
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/download-ggml-model.sh ${MODEL_TYPE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ggml-${MODEL_TYPE}.bin
    DESTINATION share/models
  )
endmacro()

download_model(tiny.en)
download_model(tiny)
download_model(base.en)
download_model(base)
download_model(small.en)
download_model(small)
download_model(medium.en)
download_model(medium)
download_model(large-v1)
download_model(large)

install(DIRECTORY
  launch scripts
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
