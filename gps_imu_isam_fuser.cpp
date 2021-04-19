#include "gps_imu_isam_fuser.h"
#include "trigonometry.h"
#include "plot/plot.h"
#include "plot/plot_pose.h"
#include "nav/attitude.h"
#include "nav/navlab_utils.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/navigation/CombinedImuFactor.h"
#include "gtsam/navigation/GPSFactor.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"

using namespace gtsam;
using plot::Plot;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3

GpsImuIsamFuser::GpsImuIsamFuser(
    const nav::NavlabGPS& init_gps,
    const nav::Attitude& init_att,
    const nav::Attitude& init_att_sigmas,
    double imu_integration_interval_sec)
    : local_(init_gps.pos_geo)
    , last_gps_time{init_gps.time}
    , dt_{imu_integration_interval_sec}
{
  // Construct ISAM2 estimator.
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam_ = std::make_shared<ISAM2>(parameters);

  // Define priors at the origin in the local coordinate system.
  const Pose3 prior_pose(Rot3(init_att.toQuaternion()), Point3::Zero());
  const Vector3 prior_velocity = Vector3::Zero(); // Can also use velocity from GPS.
  const imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

  // Hard code uncertainty at first.
  const auto prior_pose_noise_model = noiseModel::Diagonal::Sigmas(
      (Vector(6) <<init_att_sigmas.roll(), init_att_sigmas.pitch(), init_att_sigmas.heading(),
          Vector3::Constant(0.5)).finished()
  ); // rad,rad,rad, m, m, m
  const auto prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 1e-2);  // m/s
  const auto prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

  // Add priors on initial states to graph.
  NonlinearFactorGraph graph;
  graph.addPrior(X(0), prior_pose, prior_pose_noise_model);
  graph.addPrior(V(0), prior_velocity, prior_velocity_noise_model);
  graph.addPrior(B(0), prior_imu_bias, prior_bias_noise_model);

  // Insert initial values for initial states.
  Values initial_estimate;
  initial_estimate.insert(X(0), prior_pose);
  initial_estimate.insert(V(0), prior_velocity);
  initial_estimate.insert(B(0), prior_imu_bias);
  last_estimate_ = initial_estimate;

  // Add priors to ISAM2.
  isam_->update(graph, initial_estimate);

  // Set up IMU-parameters.
  // Can move this outside the class, initialize preintegrator in constructor.
  constexpr double accel_noise_sigma = 7.8480e-04;
  constexpr double gyro_noise_sigma = 1.7500e-04;
  constexpr double accel_bias_rw_sigma = 0.03;
  constexpr double gyro_bias_rw_sigma = 0.0035;
  auto imu_params = PreintegrationCombinedParams::MakeSharedD(); // _D_  for z downwards (as in NED)
  imu_params->setAccelerometerCovariance(I_3x3 * (accel_noise_sigma * accel_noise_sigma));
  imu_params->setGyroscopeCovariance(I_3x3 * (gyro_noise_sigma * gyro_noise_sigma));
  imu_params->setIntegrationCovariance(I_3x3 * 1e-8);
  imu_params->setBiasAccCovariance(I_3x3 * (accel_bias_rw_sigma * accel_bias_rw_sigma));
  imu_params->setBiasOmegaCovariance(I_3x3 * (gyro_bias_rw_sigma * gyro_bias_rw_sigma));
  imu_params->setBiasAccOmegaInt(I_6x6 * 1e-5);

  imu_preintegrator_ = std::make_shared<PreintegratedCombinedMeasurements>(imu_params, prior_imu_bias);
}

void GpsImuIsamFuser::addGpsData(const nav::NavlabGPS& gps, bool do_plot_update)
{
  if (gps.time <= last_gps_time)
  {
    throw std::runtime_error("GPS measurement from before last GPS measurement");
  }
  if (gps.time <= last_imu_time_)
  {
    throw std::runtime_error("GPS measurement before last IMU measurement");
  }
  last_gps_time = gps.time;

  // We update the states at each GPS measurement.
  ++curr_state_num_;

  // Create new graph.
  NonlinearFactorGraph graph;

  // Add IMU factor.
  const CombinedImuFactor imu_factor(
      X(curr_state_num_ - 1), V(curr_state_num_ - 1),
      X(curr_state_num_), V(curr_state_num_),
      B(curr_state_num_ - 1), B(curr_state_num_),
      *imu_preintegrator_
  );
  graph.add(imu_factor);

  // Add GPS factor.
  const Vector3 local_ned_pos = local_.toLocalPosition(gps.pos_geo);
  const auto gps_noise = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 2.0)); // Use GPS noise?
  const GPSFactor gps_factor(X(curr_state_num_), local_ned_pos, gps_noise);
  graph.add(gps_factor);

  // Update initial estimates with last result.
  Values initial_estimate;

  // Add predicted initial estimates for the new states.
  const NavState prev_nav_state(
      last_estimate_.at<Pose3>(X(curr_state_num_-1)),
      last_estimate_.at<Vector3>(V(curr_state_num_-1)));
  const auto& prev_bias_state = last_estimate_.at<imuBias::ConstantBias>(B(curr_state_num_-1));
  const auto predicted_state = imu_preintegrator_->predict(prev_nav_state, prev_bias_state);

  initial_estimate.insert(X(curr_state_num_), predicted_state.pose());
  initial_estimate.insert(V(curr_state_num_), predicted_state.v());
  initial_estimate.insert(B(curr_state_num_), prev_bias_state);

  // Estimate all states.
  isam_->update(graph, initial_estimate);
  for (int i=0; i < 10; ++i)
  {
    isam_->update();
  }
  last_estimate_ = isam_->calculateEstimate();

  // Reset imu preintegrator.
  const auto& new_bias_state = last_estimate_.at<imuBias::ConstantBias>(B(curr_state_num_));
  imu_preintegrator_->resetIntegrationAndSetBias(new_bias_state);

  // Print out the marginal uncertainties for the current state.
//  const Marginals marginals(graph_, last_estimate_);
//  const Vector6 stddevs = marginals.marginalCovariance(X(curr_state_num_)).diagonal().array().sqrt();
//  std::cout << "Standard deviations for X(" << curr_state_num_ << "): " << stddevs.transpose() << std::endl;

  if (do_plot_update)
  {
    const auto currPose = last_estimate_.at<Pose3>(X(curr_state_num_));
    const Eigen::Quaterniond to_plot = Eigen::AngleAxisd(0.5 * trig::pi, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(trig::pi, Eigen::Vector3d::UnitX());
    const Sophus::SE3d curr_pose = Sophus::SE3d(to_plot, Eigen::Vector3d::Zero()) * Sophus::SE3d(currPose.rotation().toQuaternion(), currPose.translation());

    Plot::clf();
    Plot::plot({0.0}, {0.0}, "xr");
    plot::plotPose2d({curr_pose}, "r");
    Plot::axis("equal");
    Plot::pause(1e-3);
  }
}

void GpsImuIsamFuser::addImuData(const nav::NavlabIMU& imu)
{
  if (imu.time < last_gps_time)
  {
    throw std::runtime_error("IMU measurement from before last GPS measurement");
  }
  if (imu.time <= last_imu_time_)
  {
    throw std::runtime_error("IMU measurement before last IMU measurement");
  }

  double dt = 0.01;
  if (last_imu_time_ > 0.0)
  {
    dt = imu.time - last_imu_time_;
  }

  const Vector3 accel = imu.dvel / dt;
  const Vector3 omega = imu.drot / dt;

  // Accumulate IMU measurements between GPS measurements.
  imu_preintegrator_->integrateMeasurement(accel, omega, dt);

  last_imu_time_ = imu.time;
}
