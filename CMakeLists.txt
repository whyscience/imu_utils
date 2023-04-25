cmake_minimum_required(VERSION 3.8)
project(ros2_imu_utils)

# find dependencies from package.xml
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3  REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_executable(imu_an src/imu_an_ros2.cpp
  src/acc_lib/allan_acc.cpp
  src/acc_lib/fitallan_acc.cpp
  src/gyr_lib/allan_gyr.cpp
  src/gyr_lib/fitallan_gyr.cpp
)
target_link_libraries(imu_an ${OpenCV_LIBS} ${CERES_LIBRARIES})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)