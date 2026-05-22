#[[

  Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
  de Barcelona (UAB).
  
  This work is licensed under the terms of the MIT license.
  For a copy, see <https://opensource.org/licenses/MIT>.

]]

set (
  CARLA_CMAKE_MINIMUM_REQUIRED_VERSION
  3.27.2
)

cmake_minimum_required (
  VERSION
  ${CARLA_CMAKE_MINIMUM_REQUIRED_VERSION}
)

cmake_policy (SET CMP0097 NEW)
cmake_policy (SET CMP0091 NEW)
cmake_policy (SET CMP0074 NEW)
cmake_policy (SET CMP0077 NEW)
cmake_policy (SET CMP0117 NEW)

if (${CMAKE_MINOR_VERSION} GREATER_EQUAL 24)
  cmake_policy (SET CMP0135 NEW)
endif ()

# CMake 4.x removed support for cmake_minimum_required < 3.5; set the floor
# so third-party FetchContent dependencies with old requirements still build.
if (CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
  set (CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif ()

set (CARLA_VERSION_MAJOR 0)
set (CARLA_VERSION_MINOR 10)
set (CARLA_VERSION_PATCH 0)

set (
  CARLA_VERSION 
  ${CARLA_VERSION_MAJOR}.${CARLA_VERSION_MINOR}.${CARLA_VERSION_PATCH}
)

project (
  CARLA
  VERSION
    ${CARLA_VERSION}
  LANGUAGES
    C
    CXX
    ASM
  DESCRIPTION
    "Open-source simulator for autonomous driving research."
  HOMEPAGE_URL
    https://carla.org
)

set (
  CARLA_WORKSPACE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set (
  CARLA_BUILD_PATH
  ${CMAKE_BINARY_DIR}
)

set (
  CARLA_PACKAGE_PATH
  ${CARLA_BUILD_PATH}/Package
)

set (
  CARLA_UNREAL_PLUGINS_PATH
  ${CARLA_WORKSPACE_PATH}/Unreal/CarlaUnreal/Plugins
)

set (
  CARLA_LICENSE_FILE_PATH
  ${CARLA_WORKSPACE_PATH}/LICENSE
)

include (CheckCCompilerFlag)
include (CheckCXXCompilerFlag)
include (CheckLinkerFlag)
include (${CARLA_WORKSPACE_PATH}/CMake/Util.cmake)
include (${CARLA_WORKSPACE_PATH}/CMake/Options.cmake)
include (${CARLA_WORKSPACE_PATH}/CMake/Common.cmake)
include (${CARLA_WORKSPACE_PATH}/CMake/Dependencies.cmake)

if (BUILD_CARLA_CLIENT OR BUILD_CARLA_SERVER)
  add_subdirectory (LibCarla)
endif ()

if (BUILD_OSM_WORLD_RENDERER)
  add_subdirectory (osm-world-renderer)
endif ()

if (ENABLE_ROS2)
  add_subdirectory (Ros2Native)
endif()

# RclcppBridge - built with system compiler (NOT UE toolchain) because it
# links against rclcpp which is built with libstdc++.
# Only needed when ENABLE_RGL=ON; completely skipped otherwise so no ROS2
# environment is required for non-RGL builds.
if (ENABLE_RGL)
  include (ExternalProject)
  set (RCLCPP_BRIDGE_SRC "${CMAKE_SOURCE_DIR}/Unreal/CarlaUnreal/Plugins/CarlaRGL/ThirdParty/RclcppBridge")
  if (IS_DIRECTORY "${RCLCPP_BRIDGE_SRC}")
    # RclcppBridge needs `source /opt/ros/humble/setup.bash` equivalent:
    #   - CMAKE_PREFIX_PATH for find_package(rclcpp)
    #   - PYTHONPATH for ament_package (used by rclcpp's cmake config)
    # We construct these from AMENT_PREFIX_PATH or fall back to /opt/ros/humble.
    if (DEFINED ENV{AMENT_PREFIX_PATH})
      set (RCLCPP_BRIDGE_ROS_PREFIX "$ENV{AMENT_PREFIX_PATH}")
    else ()
      set (RCLCPP_BRIDGE_ROS_PREFIX "/opt/ros/humble")
    endif ()

    set (RCLCPP_BRIDGE_BUILD_DIR "${CMAKE_BINARY_DIR}/rclcpp-bridge-prefix/src/rclcpp-bridge-build")

    ExternalProject_add (
      rclcpp-bridge
      SOURCE_DIR ${RCLCPP_BRIDGE_SRC}
      BINARY_DIR ${RCLCPP_BRIDGE_BUILD_DIR}
      CONFIGURE_COMMAND
        ${CMAKE_COMMAND} -E env
          "PYTHONPATH=${RCLCPP_BRIDGE_ROS_PREFIX}/lib/python3.10/site-packages:${RCLCPP_BRIDGE_ROS_PREFIX}/local/lib/python3.10/dist-packages:$ENV{PYTHONPATH}"
          "AMENT_PREFIX_PATH=${RCLCPP_BRIDGE_ROS_PREFIX}"
        ${CMAKE_COMMAND}
          -S ${RCLCPP_BRIDGE_SRC}
          -B ${RCLCPP_BRIDGE_BUILD_DIR}
          -G Ninja
          -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
          -DCMAKE_PREFIX_PATH=${RCLCPP_BRIDGE_ROS_PREFIX}
          -DCMAKE_C_COMPILER=/usr/bin/gcc
          -DCMAKE_CXX_COMPILER=/usr/bin/g++
      BUILD_COMMAND
        ${CMAKE_COMMAND} --build ${RCLCPP_BRIDGE_BUILD_DIR}
      INSTALL_COMMAND ""
    )

    set (CARLA_RGL_PLUGIN_BINARY_PATH ${CMAKE_SOURCE_DIR}/Unreal/CarlaUnreal/Plugins/CarlaRGL/Binaries/Linux)
    file (MAKE_DIRECTORY ${CARLA_RGL_PLUGIN_BINARY_PATH})

    add_custom_command (
      TARGET rclcpp-bridge
      POST_BUILD
      COMMAND
        ${CMAKE_COMMAND} -E copy
        ${CMAKE_BINARY_DIR}/rclcpp-bridge-prefix/src/rclcpp-bridge-build/libRclcppBridge.so
        ${CARLA_RGL_PLUGIN_BINARY_PATH}
    )
  else ()
    message (WARNING "RclcppBridge source not found at ${RCLCPP_BRIDGE_SRC}. "
                     "RGL ROS2 publish will not work without it.")
  endif ()
endif ()

if (BUILD_PYTHON_API)
  add_subdirectory (PythonAPI)
endif ()

if (BUILD_CARLA_UNREAL)
  add_subdirectory (Unreal)
endif ()

if (BUILD_EXAMPLES)
  add_subdirectory (Examples)
endif ()

carla_add_custom_target (
  carla-help
  "Display this message."
  COMMAND ${CMAKE_COMMAND} -E cat "${CMAKE_CURRENT_BINARY_DIR}/Help.md"
)

carla_get_option_docs (CARLA_OPTION_DOCS)
carla_get_target_docs (CARLA_TARGET_DOCS)
set (CARLA_CMAKE_HELP_MESSAGE)
string (
  APPEND 
  CARLA_CMAKE_HELP_MESSAGE
  "# CARLA - CMake Help\n"
  "## CMake Targets\n"
  "${CARLA_TARGET_DOCS}\n"
  "## CMake Options\n"
  "${CARLA_OPTION_DOCS}\n"
)

file (
  WRITE
  ${CMAKE_CURRENT_BINARY_DIR}/Help.md
  ${CARLA_CMAKE_HELP_MESSAGE}
)

if (CMAKE_EXPORT_COMPILE_COMMANDS)
  configure_file (
    ${CARLA_WORKSPACE_PATH}/.clangd.in
    ${CARLA_WORKSPACE_PATH}/.clangd
  )
endif ()
