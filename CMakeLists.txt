cmake_minimum_required(VERSION 3.8)
project(cpp_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-pedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ───────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)  # cnpy (npz motion clips)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionReference.msg"
)

# Optional robot-specific message packages
find_package(unitree_hg QUIET)
find_package(unitree_go QUIET)
find_package(messages QUIET)
find_package(vision_encoders QUIET)  # ImageTokens (vibe-sonic)

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
    set(_ort_arch "x64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(_ort_arch "aarch64")
endif()
file(GLOB _ort_candidates "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/onnxruntime-linux-${_ort_arch}-*")
list(SORT _ort_candidates ORDER DESCENDING)  # pick highest version
list(LENGTH _ort_candidates _ort_count)
if(_ort_count GREATER 0)
    list(GET _ort_candidates 0 _ort_found)
    set(ONNXRUNTIME_ROOT_DIR "${_ort_found}" CACHE PATH "ONNX Runtime root directory")
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

# ── Deploy infra: manifest-driven ONNX + g1 motion (ROS-free) ──
add_library(cpp_control_deploy STATIC
  src/common/deploy_manifest.cpp
  src/common/onnx_session.cpp
  src/common/g1/motion.cpp
  src/common/g1/sonic_stand.cpp
  thirdparty/cnpy/cnpy.cpp)
target_include_directories(cpp_control_deploy PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
target_link_libraries(cpp_control_deploy ${ONNXRUNTIME_LIBRARIES} yaml-cpp ZLIB::ZLIB)

# Vibe task contracts are ROS-free: validate artifact semantics in tests and
# share exactly the same profile selection with the runtime.
add_library(cpp_control_vibe_contract STATIC src/tasks/vibe/task_profile.cpp)
target_link_libraries(cpp_control_vibe_contract cpp_control_deploy)

# Deploy self-test (unit checks; optionally smoke-runs a real export):
#   ./build/cpp_control/deploy_selftest [policy.onnx [policy.manifest.json]]
add_executable(deploy_selftest tests/deploy_selftest.cpp)
target_link_libraries(deploy_selftest cpp_control_deploy cpp_control_vibe_contract)
if(BUILD_TESTING)
  add_test(NAME deploy_selftest COMMAND deploy_selftest)
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
    if(unitree_go_FOUND)
      list(APPEND G1_DEPS unitree_go)
    endif()
  endif()
  if(messages_FOUND)
    list(APPEND G1_DEPS messages)
  endif()

  add_library(cpp_control_g1 STATIC src/robots/g1.cpp)
  target_link_libraries(cpp_control_g1 cpp_control_base cpp_control_deploy)
  ament_target_dependencies(cpp_control_g1 ${G1_DEPS})

  # Tracker library (motion handling, WBC obs, frame alignment)
  add_library(cpp_control_g1_textop_tracker STATIC src/tasks/tracker/g1_textop_tracker.cpp)
  target_compile_definitions(cpp_control_g1_textop_tracker PRIVATE CPP_CONTROL_TEXTOP_TRACKER_LIB)
  target_link_libraries(cpp_control_g1_textop_tracker cpp_control_g1 cpp_control_deploy)
  ament_target_dependencies(cpp_control_g1_textop_tracker
    rclcpp std_msgs sensor_msgs geometry_msgs ${G1_DEPS})

  # Task: locomotion (g1)
  add_executable(g1_locomotion_node src/tasks/locomotion/g1.cpp)
  target_link_libraries(g1_locomotion_node cpp_control_g1)
  ament_target_dependencies(g1_locomotion_node
    rclcpp std_msgs sensor_msgs geometry_msgs ${G1_DEPS})

  # Task: tracker (g1 textop — standalone WBC)
  add_executable(g1_textop_node src/tasks/tracker/g1_textop_tracker.cpp)
  target_link_libraries(g1_textop_node cpp_control_g1 cpp_control_deploy)
  ament_target_dependencies(g1_textop_node
    rclcpp std_msgs sensor_msgs geometry_msgs ${G1_DEPS})

  # LEGACY (vibe v1, pre-manifest): kept as reference source, not built.
  # Uncomment to resurrect the residual nodes.
  # add_executable(g1_vibe_residual_node src/tasks/vibe/g1_residual.cpp)
  # target_link_libraries(g1_vibe_residual_node cpp_control_g1_textop_tracker)
  # ament_target_dependencies(g1_vibe_residual_node
  #   rclcpp std_msgs sensor_msgs geometry_msgs yaml-cpp ${G1_DEPS})
  #
  # add_library(cpp_control_vibe_residual STATIC src/tasks/vibe/g1_residual.cpp)
  # target_compile_definitions(cpp_control_vibe_residual PRIVATE CPP_CONTROL_VIBE_RESIDUAL_LIB)
  # target_link_libraries(cpp_control_vibe_residual cpp_control_g1_textop_tracker)
  # ament_target_dependencies(cpp_control_vibe_residual
  #   rclcpp std_msgs sensor_msgs geometry_msgs yaml-cpp ${G1_DEPS})
  #
  # add_executable(g1_vibe_residual_v2_node src/tasks/vibe/g1_residual_v2.cpp)
  # target_link_libraries(g1_vibe_residual_v2_node cpp_control_vibe_residual)
  # ament_target_dependencies(g1_vibe_residual_v2_node
  #   rclcpp std_msgs sensor_msgs geometry_msgs yaml-cpp ${G1_DEPS})

  # Task: tracker (g1 sonic — exported vibe policy + manifest)
  add_executable(g1_sonic_node src/tasks/tracker/g1_sonic.cpp)
  target_link_libraries(g1_sonic_node cpp_control_g1 cpp_control_deploy)
  ament_target_dependencies(g1_sonic_node
    rclcpp std_msgs sensor_msgs geometry_msgs yaml-cpp ${G1_DEPS})
  rosidl_target_interfaces(g1_sonic_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

  # Sonic tracker library (for the vibe-sonic node to inherit from)
  add_library(cpp_control_g1_sonic STATIC src/tasks/tracker/g1_sonic.cpp)
  target_compile_definitions(cpp_control_g1_sonic PRIVATE CPP_CONTROL_SONIC_LIB)
  target_link_libraries(cpp_control_g1_sonic cpp_control_g1 cpp_control_deploy)
  ament_target_dependencies(cpp_control_g1_sonic
    rclcpp std_msgs sensor_msgs geometry_msgs yaml-cpp ${G1_DEPS})
  rosidl_target_interfaces(cpp_control_g1_sonic ${PROJECT_NAME} "rosidl_typesupport_cpp")

  # Task: vibe (g1 vibe-sonic — extractor export + /enc/tokens)
  if(vision_encoders_FOUND)
    add_executable(g1_vibe_sonic_node src/tasks/vibe/g1_vibe_sonic.cpp)
    target_link_libraries(g1_vibe_sonic_node
      cpp_control_g1_sonic cpp_control_vibe_contract)
    ament_target_dependencies(g1_vibe_sonic_node
      rclcpp std_msgs sensor_msgs geometry_msgs yaml-cpp vision_encoders ${G1_DEPS})
    rosidl_target_interfaces(g1_vibe_sonic_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
    list(APPEND INSTALL_TARGETS g1_vibe_sonic_node)
  else()
    message(STATUS "vision_encoders NOT found -- g1_vibe_sonic_node disabled")
  endif()

  list(APPEND INSTALL_TARGETS g1_locomotion_node g1_textop_node g1_sonic_node)
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

install(PROGRAMS
  scripts/npz_motion_publisher.py
  # scripts/viser_ghost_g1_residual.py   # LEGACY (vibe v1) — reference only
  scripts/vibe/attention_overlay_viewer.py
  scripts/vibe/record_topics.sh
  scripts/vibe/replay_vibes.py
  scripts/vibe/replay_vibes.sh
  scripts/vibe/stream_vibes.sh
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
