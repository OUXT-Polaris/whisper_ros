cmake_minimum_required(VERSION 3.8)
project(whisper_cpp_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

macro(download_model MODEL_TYPE)
  execute_process(
    COMMAND bash ${WHISPER_SRC_DIR}/models/download-ggml-model.sh ${MODEL_TYPE}
    WORKING_DIRECTORY ${WHISPER_SRC_DIR}
  )
  install(
    FILES ${WHISPER_SRC_DIR}/models/ggml-${MODEL_TYPE}.bin
    DESTINATION models
  )
endmacro()

macro(build_whisper_cpp)
  set(cmake_commands)
  set(cmake_configure_args -Wno-dev)
  set(cmake_configure_args -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install)
  set(WHISPER_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR}/whisper_cpp-ext-prefix/src/whisper_cpp-ext)

  include(ExternalProject)
  externalproject_add(whisper_cpp-ext
    GIT_REPOSITORY https://github.com/ggerganov/whisper.cpp.git
    GIT_TAG 1.0.4
    TIMEOUT 6000
    ${cmake_commands}
    CMAKE_ARGS
      ${cmake_configure_args}
  )

  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
  )
  install(
    FILES ${WHISPER_SRC_DIR}/ggml.h ${WHISPER_SRC_DIR}/whisper.h
    DESTINATION include
  )
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

  unset(WHISPER_SRC_DIR)
endmacro()

build_whisper_cpp()
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
