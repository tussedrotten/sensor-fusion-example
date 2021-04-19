#pragma once

#include "coordinate_frame.h"
#include "geodetic_position.h"
#include "GeographicLib/Geocentric.hpp"
#include "sophus/se3.hpp"

namespace nav
{

/// \brief Converts between geodetic and geocentric coordinate systems
class ECEFCoordinateSystem
{
public:
  /// \brief Constructs an ECEF coordinate system for a given body frame.
  /// \param frame ENU or NED body frame
  explicit ECEFCoordinateSystem(const CoordinateFrame& frame = CoordinateFrame::NED);

  /// \brief Converts an ECEF position to a geodetic position
  /// \param ecef_pos position in standard ECEF coordinates
  /// \return geodetic position
  static GeodeticPosition toGeodeticPosition(const Eigen::Vector3d& ecef_pos);

  /// \brief Converts a geodetic body pose to an ECEF pose
  /// \param geo_pos geodetic position
  /// \return ECEF pose
  Sophus::SE3d toEcefPose(const GeodeticPosition& geo_pos) const;

  /// \brief Converts a geodetic body pose to an ECEF pose
  /// \param geo_pos geodetic position
  /// \param att_frame_body body attitude
  /// \return ECEF pose
  Sophus::SE3d toEcefPose(const GeodeticPosition& geo_pos, const Sophus::SO3d& att_frame_body) const;

private:
  CoordinateFrame frame_;
};
}
