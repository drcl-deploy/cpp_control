cmake_minimum_required(VERSION 3.8)
project(cpp_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-pedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ───────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Optional robot-specific message packages
find_package(unitree_hg QUIET)
find_package(messages QUIET)

if(unitree_hg_FOUND)
  message(STATUS "Found unitree_hg -- Unitree HG backend enabled")
else()
  message(STATUS "unitree_hg NOT found -- Unitree HG backend disabled")
endif()
if(messages_FOUND)
  message(STATUS "Found messages -- DRCL deploy backend enabled")
else()
  message(STATUS "messages NOT found -- DRCL deploy backend disabled")
endif()

if(NOT unitree_hg_FOUND AND NOT messages_FOUND)
  message(FATAL_ERROR "At least one of unitree_hg or messages is required")
endif()

# ── ONNX Runtime ──────────────────────────────────────────────
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
    set(ONNXRUNTIME_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.22.0"
        CACHE PATH "ONNX Runtime root directory")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ONNXRUNTIME_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.22.0"
        CACHE PATH "ONNX Runtime root directory")
endif()

if(EXISTS "${ONNXRUNTIME_ROOT_DIR}")
    set(ONNXRUNTIME_INCLUDE_DIRS "${ONNXRUNTIME_ROOT_DIR}/include")
    set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_ROOT_DIR}/lib")
    set(ONNXRUNTIME_LIBRARIES "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so")
    message(STATUS "Found ONNX Runtime at ${ONNXRUNTIME_ROOT_DIR}")
else()
    message(WARNING "ONNX Runtime not found at ${ONNXRUNTIME_ROOT_DIR}")
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_RPATH "${ONNXRUNTIME_LIB_DIR}")
set(CMAKE_INSTALL_RPATH "${ONNXRUNTIME_LIB_DIR}")

include_directories(include)
if(ONNXRUNTIME_INCLUDE_DIRS)
    include_directories(${ONNXRUNTIME_INCLUDE_DIRS})
endif()

# ── Shared library: common (config, onnx, optionally crc) ────
set(COMMON_SOURCES src/config_loader.cpp src/onnx_policy.cpp)
if(unitree_hg_FOUND)
  list(APPEND COMMON_SOURCES src/motor_crc_hg.cpp)
endif()

add_library(cpp_control_common STATIC ${COMMON_SOURCES})
target_link_libraries(cpp_control_common ${ONNXRUNTIME_LIBRARIES} yaml-cpp)
ament_target_dependencies(cpp_control_common rclcpp)

if(unitree_hg_FOUND)
  target_compile_definitions(cpp_control_common PUBLIC HAS_UNITREE_HG)
  ament_target_dependencies(cpp_control_common unitree_hg)
endif()
if(messages_FOUND)
  target_compile_definitions(cpp_control_common PUBLIC HAS_MESSAGES)
endif()

# ── Level 0: base ────────────────────────────────────────
add_library(cpp_control_base STATIC src/base.cpp)
target_link_libraries(cpp_control_base cpp_control_common)
ament_target_dependencies(cpp_control_base rclcpp sensor_msgs)

# ── Level 1 + tasks: g1 (needs at least one backend) ─────────
if(unitree_hg_FOUND OR messages_FOUND)
  set(G1_DEPS rclcpp geometry_msgs)
  if(unitree_hg_FOUND)
    list(APPEND G1_DEPS unitree_hg)
  endif()
  if(messages_FOUND)
    list(APPEND G1_DEPS messages)
  endif()

  add_library(cpp_control_g1 STATIC src/robots/g1.cpp)
  target_link_libraries(cpp_control_g1 cpp_control_base)
  ament_target_dependencies(cpp_control_g1 ${G1_DEPS})

  # Task: locomotion (g1)
  add_executable(g1_locomotion_node src/tasks/locomotion/g1.cpp)
  target_link_libraries(g1_locomotion_node cpp_control_g1)
  ament_target_dependencies(g1_locomotion_node
    rclcpp std_msgs sensor_msgs geometry_msgs ${G1_DEPS})

  # Task: tracker (g1 textop)
  add_executable(g1_textop_node src/tasks/tracker/g1_textop.cpp)
  target_link_libraries(g1_textop_node cpp_control_g1)
  ament_target_dependencies(g1_textop_node
    rclcpp std_msgs sensor_msgs geometry_msgs ${G1_DEPS})

  list(APPEND INSTALL_TARGETS g1_locomotion_node g1_textop_node)
endif()

# ── Level 1 + tasks: mini_pi (needs messages) ────────────────
if(messages_FOUND)
  add_library(cpp_control_mini_pi STATIC src/robots/mini_pi.cpp)
  target_link_libraries(cpp_control_mini_pi cpp_control_base)
  ament_target_dependencies(cpp_control_mini_pi rclcpp messages geometry_msgs)

  # Task: locomotion (mini_pi)
  add_executable(mini_pi_locomotion_node src/tasks/locomotion/mini_pi.cpp)
  target_link_libraries(mini_pi_locomotion_node cpp_control_mini_pi)
  ament_target_dependencies(mini_pi_locomotion_node
    rclcpp std_msgs sensor_msgs geometry_msgs messages)

  # Task: dive (mini_pi)
  add_executable(mini_pi_dive_node src/tasks/dive/mini_pi.cpp)
  target_link_libraries(mini_pi_dive_node cpp_control_mini_pi)
  ament_target_dependencies(mini_pi_dive_node
    rclcpp std_msgs sensor_msgs geometry_msgs messages)

  list(APPEND INSTALL_TARGETS mini_pi_locomotion_node mini_pi_dive_node)
endif()

# ── Install ───────────────────────────────────────────────────
if(INSTALL_TARGETS)
  install(TARGETS ${INSTALL_TARGETS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch config models include
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS scripts/npz_motion_publisher.py
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
