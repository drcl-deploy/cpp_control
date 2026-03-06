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
find_package(unitree_hg REQUIRED)
find_package(messages REQUIRED)
find_package(yaml-cpp REQUIRED)

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

# ── Shared library: common (config, onnx, crc) ────────────────
add_library(cpp_control_common STATIC
  src/config_loader.cpp
  src/onnx_policy.cpp
  src/motor_crc_hg.cpp
)
target_link_libraries(cpp_control_common ${ONNXRUNTIME_LIBRARIES} yaml-cpp)
ament_target_dependencies(cpp_control_common rclcpp unitree_hg)

# ── Level 0: base ────────────────────────────────────────
add_library(cpp_control_base STATIC src/base.cpp)
target_link_libraries(cpp_control_base cpp_control_common)
ament_target_dependencies(cpp_control_base rclcpp sensor_msgs)

# ── Level 1: g1 ────────────────────────────────────
add_library(cpp_control_g1 STATIC src/robots/g1.cpp)
target_link_libraries(cpp_control_g1 cpp_control_base)
ament_target_dependencies(cpp_control_g1 rclcpp unitree_hg messages geometry_msgs)

# ── Task: locomotion (g1) ─────────────────────────────────────
add_executable(g1_locomotion_node src/tasks/locomotion/g1.cpp)
target_link_libraries(g1_locomotion_node cpp_control_g1)
ament_target_dependencies(g1_locomotion_node
  rclcpp std_msgs sensor_msgs geometry_msgs unitree_hg messages)

# ── Level 1: mini_pi ────────────────────────────────────
add_library(cpp_control_mini_pi STATIC src/robots/mini_pi.cpp)
target_link_libraries(cpp_control_mini_pi cpp_control_base)
ament_target_dependencies(cpp_control_mini_pi rclcpp messages geometry_msgs)

# ── Task: locomotion (mini_pi) ───────────────────────────────
add_executable(mini_pi_locomotion_node src/tasks/locomotion/mini_pi.cpp)
target_link_libraries(mini_pi_locomotion_node cpp_control_mini_pi)
ament_target_dependencies(mini_pi_locomotion_node
  rclcpp std_msgs sensor_msgs geometry_msgs messages)

# ── Install ───────────────────────────────────────────────────
install(TARGETS
  g1_locomotion_node
  mini_pi_locomotion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config models include
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
