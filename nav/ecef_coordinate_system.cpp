#include "ecef_coordinate_system.h"

#include "Eigen/Geometry"

using namespace nav;

ECEFCoordinateSystem::ECEFCoordinateSystem(const CoordinateFrame& frame)
    : frame_{frame}
{}

GeodeticPosition ECEFCoordinateSystem::toGeodeticPosition(const Eigen::Vector3d& ecef_pos)
{
  GeodeticPosition geodetic;
  GeographicLib::Geocentric::WGS84().Reverse(
      ecef_pos.x(), ecef_pos.y(), ecef_pos.z(),
      geodetic.latitude, geodetic.longitude, geodetic.altitude);
  return geodetic;
}

Sophus::SE3d ECEFCoordinateSystem::toEcefPose(const GeodeticPosition& geo_pos) const
{
  std::vector<double> matrix_data(9);
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rot_E_Ne(matrix_data.data());
  Eigen::Vector3d ecef;

  GeographicLib::Geocentric::WGS84().Forward(
      geo_pos.latitude, geo_pos.longitude, geo_pos.altitude,
      ecef.x(), ecef.y(), ecef.z(),
      matrix_data
  );

  if (frame_ == CoordinateFrame::NED)
  {
    Eigen::Matrix3d rot_E_N;
    rot_E_N << rot_E_Ne.col(1), rot_E_Ne.col(0), -rot_E_Ne.col(2);
    return {rot_E_N, ecef};
  }
  else
  {
    return {rot_E_Ne, ecef};
  }
}

Sophus::SE3d ECEFCoordinateSystem::toEcefPose(const GeodeticPosition& geo_pos, const Sophus::SO3d& att_frame_body) const
{
  Sophus::SE3d ecef_pose = toEcefPose(geo_pos);

  ecef_pose.so3() *= att_frame_body;

  return ecef_pose;
}
