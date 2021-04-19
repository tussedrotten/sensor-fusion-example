#include "local_coordinate_system.h"
#include "Eigen/Geometry"

using namespace nav;

LocalCoordinateSystem::LocalCoordinateSystem(const GeodeticPosition& origin, CoordinateFrame frame)
    : local_{origin.latitude, origin.longitude, origin.altitude}
    , frame_{frame}
{}

GeodeticPosition LocalCoordinateSystem::toGeodeticPosition(const Eigen::Vector3d& local_pos) const
{
  GeodeticPosition geo_pos;

  if (frame_ == CoordinateFrame::NED)
  {
    // Local is given in NED, but implicitly transformed to ENU below.
    local_.Reverse(local_pos.y(), local_pos.x(), -local_pos.z(),
                   geo_pos.latitude, geo_pos.longitude, geo_pos.altitude);
  }
  else
  {
    local_.Reverse(local_pos.x(), local_pos.y(), local_pos.z(),
                   geo_pos.latitude, geo_pos.longitude, geo_pos.altitude);
  }

  return geo_pos;
}

Sophus::SE3d LocalCoordinateSystem::toLocalPose(const GeodeticPosition& geo_pos, const Sophus::SO3d& att_frame_body) const
{
  // Use a std::vector as storage for a row-major Eigen::Matrix3d.
  std::vector<double> matrix_data(9);
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_local(matrix_data.data());
  Eigen::Vector3d t;

  // Transform geodetic coordinates to local (ENU-based) cartesian coordinate system.
  local_.Forward(geo_pos.latitude, geo_pos.longitude, geo_pos.altitude,
               t.x(), t.y(), t.z(),
               matrix_data);

  // Compute attitude in local (NED/ENU-based) coordinate frame (R_local works for both).
  const Sophus::SO3d att_local_body{Sophus::SO3d{R_local} * att_frame_body};

  if (frame_ == CoordinateFrame::NED)
  {
    // Compute position in local NED-based coordinate frame.
    t = Eigen::Vector3d{t.y(), t.x(), -t.z()};
  }

  return {att_local_body, t};
}

Eigen::Vector3d LocalCoordinateSystem::toLocalPosition(const GeodeticPosition& geo_pos) const
{
  // Transform geodetic coordinates to local (ENU-based) cartesian coordinate system.
  Eigen::Vector3d t;
  local_.Forward(geo_pos.latitude, geo_pos.longitude, geo_pos.altitude,
                 t.x(), t.y(), t.z());

  if (frame_ == CoordinateFrame::NED)
  {
    // Compute position in local (NED-based) coordinate system.
    t = Eigen::Vector3d{t.y(), t.x(), -t.z()};
  }

  return t;
}
