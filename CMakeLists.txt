cmake_minimum_required(VERSION 3.8)
project(cpp_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
  # Disable pedantic warnings for ONNX Runtime
  add_compile_options(-Wno-pedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_hg REQUIRED)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# ONNX Runtime setup
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
    message(WARNING "ONNX Runtime not found at ${ONNXRUNTIME_ROOT_DIR}. "
                    "Please set ONNXRUNTIME_ROOT_DIR or place ONNX Runtime in thirdparty/")
endif()

# Set RPATH to find our ONNX Runtime instead of system's
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_RPATH "${ONNXRUNTIME_LIB_DIR}")
set(CMAKE_INSTALL_RPATH "${ONNXRUNTIME_LIB_DIR}")

# Include directories
include_directories(include)
if(ONNXRUNTIME_INCLUDE_DIRS)
    include_directories(${ONNXRUNTIME_INCLUDE_DIRS})
endif()

# Add locomotion ONNX controller executable
add_executable(locomotion_onnx_controller
  src/locomotion_onnx_controller.cpp
  src/config_loader.cpp
  src/onnx_policy.cpp
  src/motor_crc_hg.cpp
)

ament_target_dependencies(locomotion_onnx_controller
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  unitree_hg
)

target_link_libraries(locomotion_onnx_controller
  ${ONNXRUNTIME_LIBRARIES}
  yaml-cpp
)

# Install targets
install(TARGETS
  locomotion_onnx_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  models
  include
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
