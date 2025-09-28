cmake_minimum_required(VERSION 3.8)
project(lidar_odometry_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)

# Force system spdlog to avoid conflicts
find_package(PkgConfig REQUIRED)
pkg_check_modules(SPDLOG REQUIRED spdlog)

# Thirdparty directories
set(THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lidar_odometry/thirdparty)
set(LIDAR_ODOMETRY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lidar_odometry)

# Build Ceres from thirdparty (needed for lidar_odometry library)
set(CERES_DIR ${THIRDPARTY_DIR}/ceres-solver)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build Ceres examples" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "Build Ceres benchmarks" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build Ceres tests" FORCE)
set(PROVIDE_UNINSTALL_TARGET OFF CACHE BOOL "Disable Ceres uninstall target" FORCE)
set(TBB OFF CACHE BOOL "Disable TBB for Ceres" FORCE)
set(SUITESPARSE OFF CACHE BOOL "Disable SuiteSparse for Ceres" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_TBB ON CACHE BOOL "Disable TBB package finding" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_SuiteSparse ON CACHE BOOL "Disable SuiteSparse package finding" FORCE)
add_subdirectory(${CERES_DIR} ${CMAKE_BINARY_DIR}/ceres-solver EXCLUDE_FROM_ALL)

# Include directories for lidar_odometry
include_directories(
  ${LIDAR_ODOMETRY_DIR}/src
  ${LIDAR_ODOMETRY_DIR}/app
  ${THIRDPARTY_DIR}/Sophus
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Direct static linking approach - much simpler!
set(LIDAR_ODOMETRY_SOURCES
    ${LIDAR_ODOMETRY_DIR}/src/database/LidarFrame.cpp
    ${LIDAR_ODOMETRY_DIR}/src/processing/FeatureExtractor.cpp
    ${LIDAR_ODOMETRY_DIR}/src/processing/Estimator.cpp
    ${LIDAR_ODOMETRY_DIR}/src/processing/IterativeClosestPoint.cpp
    ${LIDAR_ODOMETRY_DIR}/src/optimization/Parameters.cpp
    ${LIDAR_ODOMETRY_DIR}/src/optimization/Factors.cpp
    ${LIDAR_ODOMETRY_DIR}/src/optimization/AdaptiveMEstimator.cpp
    ${LIDAR_ODOMETRY_DIR}/src/util/Config.cpp
    ${LIDAR_ODOMETRY_DIR}/src/util/MathUtils.cpp
)

# Simple direct linking node
add_executable(lidar_odometry_node
  src/lidar_odometry_ros.cpp
  ${LIDAR_ODOMETRY_SOURCES}
)

ament_target_dependencies(lidar_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  std_msgs
  pcl_conversions
)

target_link_libraries(lidar_odometry_node
  ceres
  ${PCL_LIBRARIES}
  fmt::fmt
  ${SPDLOG_LIBRARIES}
)

target_compile_definitions(lidar_odometry_node PRIVATE ${PCL_DEFINITIONS})

target_include_directories(lidar_odometry_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${SPDLOG_INCLUDE_DIRS}
)

if(TARGET Eigen3::Eigen)
    target_link_libraries(lidar_odometry_node Eigen3::Eigen)
else()
    target_include_directories(lidar_odometry_node PRIVATE ${EIGEN3_INCLUDE_DIR})
endif()

# Install executables
install(TARGETS
  lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  lidar_odometry/config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
