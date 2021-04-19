#include "attitude.h"
#include "Eigen/Geometry"

namespace nav
{

Attitude::Attitude()
    : x_rot{}
    , y_rot{}
    , z_rot{}
{ }

Attitude::Attitude(double x_angle, double y_angle, double z_angle)
    : x_rot{x_angle}
    , y_rot{y_angle}
    , z_rot{z_angle}
{ }

Eigen::Quaterniond Attitude::toQuaternion() const
{
  return Eigen::AngleAxisd(z_rot, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(y_rot, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(x_rot, Eigen::Vector3d::UnitX());
}

Sophus::SO3d Attitude::toSO3() const
{
  return Sophus::SO3d(toQuaternion());
}

double& Attitude::roll()
{
  return x_rot;
}

const double& Attitude::roll() const
{
  return x_rot;
}

double& Attitude::pitch()
{
  return y_rot;
}

const double& Attitude::pitch() const
{
  return y_rot;
}

double& Attitude::heading()
{
  return z_rot;
}

const double& Attitude::heading() const
{
  return z_rot;
}

}

