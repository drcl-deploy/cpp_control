cmake_minimum_required(VERSION 3.16)
project(legged_odom)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(legged_model REQUIRED)
find_package(legged_estimation REQUIRED)

add_executable(legged_odom_node src/legged_odom_node.cpp)
# Eigen explicitly: legged_model's headers use it in their own interface, but
# the package exports it as a system include that does not reach a consumer's
# compile line on Jazzy, so <Eigen/Core> is not found without this.
target_link_libraries(legged_odom_node Eigen3::Eigen)

# Pinocchio's default joint collection is a boost::variant of 25 types, and
# boost::mpl::list is preprocessed for 20. Without these the build dies inside
# boost with "wrong number of template arguments (25, should be at least 0)",
# which names neither pinocchio nor the real cause. These are pinocchio's own
# documented requirements for anything that includes its headers; the two go
# together, because raising the limit only takes effect once the preprocessed
# headers are out of the way.
target_compile_definitions(legged_odom_node PRIVATE
  BOOST_MPL_LIMIT_LIST_SIZE=30
  BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
ament_target_dependencies(legged_odom_node
  rclcpp nav_msgs tf2_msgs std_srvs unitree_hg legged_model legged_estimation)

install(TARGETS legged_odom_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
