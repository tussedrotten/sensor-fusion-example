#pragma once

#include "nav/local_coordinate_system.h"
#include "gtsam/navigation/ImuBias.h"
#include "gtsam/navigation/NavState.h"
#include "gtsam/nonlinear/Values.h"

// Forward declarations.
namespace nav{
class Attitude;
class NavlabGPS;
class NavlabIMU;
}
namespace gtsam
{
class PreintegratedCombinedMeasurements;
class FixedLagSmoother;
}

class GpsImuFixedLagFuser
{
public:
  GpsImuFixedLagFuser(
      const nav::NavlabGPS& init_gps,
      const nav::Attitude& init_att,
      const nav::Attitude& init_att_sigmas,
      double imu_integration_interval_sec = 0.01);

  void addGpsData(const nav::NavlabGPS& gps, bool do_plot_update = true);
  void addImuData(const nav::NavlabIMU& imu);

private:
  nav::LocalCoordinateSystem local_;
  double last_gps_time;
  double dt_;

  size_t curr_state_num_ = 0;
  double last_imu_time_ = 0.0;
  std::shared_ptr<gtsam::FixedLagSmoother> smoother_;
  gtsam::Values last_estimate_;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrator_;
};

