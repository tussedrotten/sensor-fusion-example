#pragma once

#include "coordinate_frame.h"
#include "geodetic_position.h"
#include "GeographicLib/LocalCartesian.hpp"
#include "sophus/se3.hpp"


namespace nav
{

/// \brief Computes and represents a local cartesian coordinate system.
class LocalCoordinateSystem
{
public:

  /// \brief Constructs a local coordinate system at the specified origin
  /// \param origin Origin for coordinate system
  /// \param frame Local coordinate frame
  explicit LocalCoordinateSystem(const GeodeticPosition& origin, CoordinateFrame frame = CoordinateFrame::NED);

  /// \brief Transforms local position to geodetic coordinate system.
  /// \param local_pos Local cartesian position
  GeodeticPosition toGeodeticPosition(const Eigen::Vector3d& local_pos) const;

  /// \brief Transforms geodetic position and attitude to pose in local coordinate system.
  /// \param geo_pos Geodetic position
  /// \param att_frame_body Attitude
  Sophus::SE3d toLocalPose(const GeodeticPosition& geo_pos, const Sophus::SO3d& att_frame_body) const;

  /// \brief Transforms geodetic position to local coordinate system.
  /// \param geo_pos Geodetic position
  Eigen::Vector3d toLocalPosition(const GeodeticPosition& geo_pos) const;

private:
  GeographicLib::LocalCartesian local_;
  CoordinateFrame frame_;
};

}
