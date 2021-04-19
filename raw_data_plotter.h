#pragma once

#include "nav/local_coordinate_system.h"
#include <memory>
#include <vector>

// Forward declarations.
struct TrackingFrame;
namespace nav{
class NavlabGPS;
class NavlabIMU;
}

class RawDataPlotter
{
public:
  explicit RawDataPlotter(const nav::NavlabGPS& init_gps);

  void addGpsData(const nav::NavlabGPS& gps);
  void addImuData(const nav::NavlabIMU& imu);
  void addTrackData(const TrackingFrame& track_frame);

  void plot();

private:
  // Coordinate system.
  nav::LocalCoordinateSystem local_;

  // GPS data.
  std::vector<double> gps_time;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> h;

  // IMU data.
  std::vector<double> imu_time;
  std::vector<double> drot_x;
  std::vector<double> drot_y;
  std::vector<double> drot_z;
  std::vector<double> dvel_x;
  std::vector<double> dvel_y;
  std::vector<double> dvel_z;

  // Track data.
  std::vector<double> track_time;
  std::vector<double> num_tracks;
};
