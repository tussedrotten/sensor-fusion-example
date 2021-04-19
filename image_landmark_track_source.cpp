#include "image_landmark_track_source.h"
#include "opencv2/core/eigen.hpp"
#include "opencv2/calib3d.hpp"
#include <fstream>

Eigen::Matrix3d Intrinsics::toCalibrationMatrix() const
{
  Eigen::Matrix3d calib_mat;
  calib_mat <<
            fu, 0., cu,
      0., fv, cv,
      0., 0., 1.;

  return calib_mat;
}

cv::Matx33d Intrinsics::toCvCalibrationMatrix() const
{
  cv::Matx33d calib_matrix(
      fu, 0.0, cu,
      0.0, fv, cv,
      0.0, 0.0, 1.0
  );

  return calib_matrix;
}

Intrinsics::Vector5d Intrinsics::toDistortionCoefficientVector() const
{
  Vector5d dist_coeffs;
  dist_coeffs << k1, k2, 0., 0., k3;

  return dist_coeffs;
}

cv::Mat Intrinsics::toCvDistortionCoefficientVector() const
{
  auto dist_coeffs = toDistortionCoefficientVector();
  cv::Mat dist_coeffs_cv;
  cv::eigen2cv(dist_coeffs, dist_coeffs_cv);

  return dist_coeffs_cv;
}

std::istream& operator>>(std::istream& is, Intrinsics& intrinsics)
{
  is >> intrinsics.fu
     >> intrinsics.fv
     >> intrinsics.cu
     >> intrinsics.cv
     >> intrinsics.k1
     >> intrinsics.k2
     >> intrinsics.k3;

  return is;
}

std::istream& operator>>(std::istream& is, Track& track)
{
  is >> track.id;
  is >> track.pixel.x();
  is >> track.pixel.y();
  is >> track.sigmas.x();
  is >> track.sigmas.y();

  return is;
}

std::istream& operator>>(std::istream& is, InitialPose& init_pose)
{
  is >> init_pose.pos.latitude;
  is >> init_pose.pos.longitude;
  is >> init_pose.pos.altitude;
  is >> init_pose.att.x_rot;
  is >> init_pose.att.y_rot;
  is >> init_pose.att.z_rot;

  return is;
}

std::istream& operator>>(std::istream& is, TrackingFrame& frame)
{
  is >> frame.time;

  size_t num_tracks{};
  is >> num_tracks;

  is >> frame.init;

  if (!is)
  { return is; }

  for (size_t i = 0; i < num_tracks; ++i)
  {
    Track track;
    is >> track;
    frame.tracks.emplace_back(track);
  }

  return is;
}

ImageLandmarkTrackSource::ImageLandmarkTrackSource(const std::string& filename)
{
  std::ifstream is(filename);

  is >> calibration_;

  const auto dist_coeffs = calibration_.toCvDistortionCoefficientVector();
  const auto calib_matrix = calibration_.toCvCalibrationMatrix();

  TrackingFrame frame;
  while (is >> frame)
  {
    for (auto& track : frame.tracks)
    {
      std::vector<cv::Point2d> distorted{{track.pixel.x(), track.pixel.y()}};
      std::vector<cv::Point2d> undistorted(1);

      cv::undistortPoints(distorted, undistorted, calib_matrix, dist_coeffs, cv::Mat{}, calib_matrix);
      track.pixel.x() = undistorted[0].x;
      track.pixel.y() = undistorted[0].y;
    }

    data_.emplace_back(frame);
    frame = TrackingFrame{};
  }
}

bool ImageLandmarkTrackSource::hasNext() const
{
  return !data_.empty();
}

const Intrinsics& ImageLandmarkTrackSource::calibration() const
{
  return calibration_;
}

double ImageLandmarkTrackSource::nextTime() const
{
  if (data_.empty())
  { return std::numeric_limits<double>::infinity(); }

  return data_.front().time;
}

const TrackingFrame& ImageLandmarkTrackSource::next() const
{
  if (data_.empty())
  { throw std::out_of_range("Data is hasNext"); }

  return data_.front();
}

void ImageLandmarkTrackSource::pop()
{
  if (data_.empty())
  { throw std::out_of_range("Data is hasNext"); }

  data_.pop_front();
}
