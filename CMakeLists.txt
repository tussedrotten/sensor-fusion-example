cmake_minimum_required( VERSION 3.10 )

project(sensor-fusion-example)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

# Compiler flags
set(compiler_options -Wall -Wextra -Wpedantic)

# Add libraries.
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
find_package(GeographicLib REQUIRED)

add_library(${PROJECT_NAME}
  gps_imu_batch_fuser.h
  gps_source.h
  image_landmark_track_source.h
  imu_source.h
  raw_data_plotter.h
  gps_imu_batch_fuser.cpp
  gps_source.cpp
  image_landmark_track_source.cpp
  imu_source.cpp
  raw_data_plotter.cpp
  gps_imu_isam_fuser.cpp
  gps_imu_isam_fuser.h
  gps_imu_isam.cpp
  gps_imu_fixed_lag_fuser.cpp
  gps_imu_fixed_lag_fuser.h
  plot/plot.cpp
  plot/plot.h
  plot/plot_pose.h
  plot/python_object_manager.cpp
  plot/python_object_manager.h
  nav/attitude.cpp
  nav/attitude.h
  nav/coordinate_frame.h
  nav/ecef_coordinate_system.cpp
  nav/ecef_coordinate_system.h
  nav/geodetic_position.cpp
  nav/geodetic_position.h
  nav/local_coordinate_system.cpp
  nav/local_coordinate_system.h
  nav/navlab_utils.cpp
  nav/navlab_utils.h
  trigonometry.h
  )

target_link_libraries(${PROJECT_NAME} PUBLIC
  gtsam
  gtsam_unstable
  Eigen3::Eigen
  Sophus::Sophus
  ${OpenCV_LIBS}
  ${PYTHON_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  )

target_include_directories(${PROJECT_NAME} SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS}
  )

target_compile_options(${PROJECT_NAME} PRIVATE ${compiler_options})

add_executable(plot_raw_data plot_raw_data.cpp)
target_link_libraries(plot_raw_data PRIVATE ${PROJECT_NAME})

add_executable(gps_imu_batch gps_imu_batch.cpp)
target_link_libraries(gps_imu_batch PRIVATE ${PROJECT_NAME})

add_executable(gps_imu_isam gps_imu_isam.cpp)
target_link_libraries(gps_imu_isam PRIVATE ${PROJECT_NAME})

add_executable(gps_imu_fixed_lag gps_imu_fixed_lag.cpp)
target_link_libraries(gps_imu_fixed_lag PRIVATE ${PROJECT_NAME})
