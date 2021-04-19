#pragma once

#include "sophus/so3.hpp"

namespace nav
{

/// \brief Attitude in NED system represented as a set of Euler angles.
struct Attitude
{
  /// \brief Constructs a NED attitude with euler angles (0, 0, 0).
  Attitude();

  /// \brief Constructs a NED attitude with the specified euler angles.
  /// \param x_angle Angle around the x-axis in radians (aka roll).
  /// \param y_angle Angle around the y-axis in radians (aka pitch).
  /// \param z_angle Angle around the z-axis in radians (aka heading).
  Attitude(double x_angle, double y_angle, double z_angle);

  double x_rot;  /// Angle around x-axis in radians. Often called Roll in NED systems.
  double y_rot;  /// Angle around y-axis in radians. Often called Pitch in NED systems.
  double z_rot;  /// Angle around z-axis in radians. Often called Heading in NED systems.

  /// \brief Alias for x_rot.
  /// \return Roll angle in radians.
  double& roll();

  /// \brief Alias for x_rot.
  /// \return Roll angle in radians.
  const double& roll() const;

  /// \brief Alias for y_rot.
  /// \return Pitch angle in radians.
  double& pitch();

  /// \brief Alias for y_rot.
  /// \return Pitch angle in radians.
  const double& pitch() const;

  /// \brief Alias for z_rot.
  /// \return Heading angle in radians.
  double& heading();

  /// \brief Alias for z_rot.
  /// \return Heading angle in radians.
  const double& heading() const;

  /// \brief Convert to quaternion representation.
  Eigen::Quaterniond toQuaternion() const;

  /// \brief Convert to SO3 representation.
  Sophus::SO3d toSO3() const;
};

}
