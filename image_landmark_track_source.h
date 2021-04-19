#pragma once

#include "nav/geodetic_position.h"
#include "nav/attitude.h"
#include "Eigen/Dense"
#include "opencv2/core.hpp"
#include <deque>

/// \brief Intrinsic camera calibration parameters.
struct Intrinsics
{
  double fu;  /// Focal length in the horizontal u-direction.
  double fv;  /// Focal length in the vertical v-direction.
  double cu;  /// Principal coordinate in the horizontal u-direction.
  double cv;  /// Principal coordinate in the vertical v-direction.

  double k1;  /// First radial distortion coefficient.
  double k2;  /// Second radial distortion coefficient.
  double k3;  /// Third radial distortion coefficient.

  /// Shorthand for 5x1 vector.
  using Vector5d = Eigen::Matrix<double, 5, 1>;

  /// \brief Extract calibration matrix.
  Eigen::Matrix3d toCalibrationMatrix() const;

  /// \brief Extract calibration matrix.
  cv::Matx33d toCvCalibrationMatrix() const;

  /// \brief Extract distortion coefficients on the form [k1, k2, 0, 0, k3].
  Vector5d toDistortionCoefficientVector() const;

  cv::Mat toCvDistortionCoefficientVector() const;
};

/// \brief Operator for reading data from stream.
std::istream& operator>>(std::istream& is, Intrinsics& intrinsics);

struct Track
{
  size_t id;
  Eigen::Vector2d pixel;
  Eigen::Vector2d sigmas;
};

std::istream& operator>>(std::istream& is, Track& track);

struct InitialPose
{
  nav::GeodeticPosition pos;
  nav::Attitude att;
};

std::istream& operator>>(std::istream& is, InitialPose& init_pose);

struct TrackingFrame
{
  double time;
  InitialPose init;
  std::vector<Track> tracks;
};

std::istream& operator>>(std::istream& is, TrackingFrame& frame);

class ImageLandmarkTrackSource
{
public:
  ImageLandmarkTrackSource(const std::string& filename);

  bool hasNext() const;

  double nextTime() const;

  const TrackingFrame& next() const;

  void pop();

  const Intrinsics& calibration() const;

private:
  std::deque<TrackingFrame> data_;
  Intrinsics calibration_;
};
