cmake_minimum_required(VERSION 3.8)
project(whisper_cpp_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

macro(build_whisper_cpp)
  set(cmake_commands)
  set(cmake_configure_args -Wno-dev)
  set(cmake_configure_args -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install)
  set(WHISPER_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR}/whisper_cpp-ext-prefix/src/whisper_cpp-ext)
  set(WHISPER_MODELS)

  include(ExternalProject)
  externalproject_add(whisper_cpp-ext
    GIT_REPOSITORY https://github.com/ggerganov/whisper.cpp.git
    GIT_TAG 1.1.0
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
    FILES ${WHISPER_SRC_DIR}/ggml.h ${WHISPER_SRC_DIR}/whisper.h ${WHISPER_SRC_DIR}/examples/dr_wav.h
    DESTINATION include
  )
  install(
    FILES ${WHISPER_SRC_DIR}/models/download-ggml-model.sh
    DESTINATION share/scripts
  )

  ament_export_libraries(whisper)

  unset(WHISPER_SRC_DIR)
endmacro()

build_whisper_cpp()
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
