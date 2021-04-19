#pragma once

#include "plot.h"
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"

namespace plot
{

/// \brief Plot a 2D pose.
template <typename Scalar>
void plotPose2d(const std::vector<Sophus::SE2<Scalar>>& poses, const std::string& color = "");

/// \brief Plot a 2D pose.
void plotPose2d(const std::vector<Sophus::SE2d>& poses, const std::string& color = "");

/// \brief Plot a 3D pose i 2D.
template <typename Scalar>
void plotPose2d(const std::vector<Sophus::SE3<Scalar>>& poses, const std::string& color = "");

/// \brief Plot a 3D pose i 2D.
void plotPose2d(const std::vector<Sophus::SE3d>& poses, const std::string& color = "");


// ----------- Implementation -----------


template <typename Scalar>
void plotPose2d(const std::vector<Sophus::SE2<Scalar>>& poses, const std::string& color)
{
  using plot::Plot;

  std::vector<Scalar> x;
  std::vector<Scalar> y;
  std::vector<Scalar> u;
  std::vector<Scalar> v;

  for (const auto& pose : poses)
  {
    const auto& translation = pose.translation();

    x.push_back(translation.x());
    y.push_back(translation.y());

    const auto& theta = pose.so2().log();

    u.push_back(std::cos(theta));
    v.push_back(std::sin(theta));
  }

  Plot::plot(x, y, "." + color, {{"markersize", "12"}});
  Plot::quiver(x, y, u, v, color);
}

void plotPose2d(const std::vector<Sophus::SE2d>& poses, const std::string& color)
{
  plotPose2d<double>(poses, color);
}

template<typename Scalar>
void plotPose2d(const std::vector<Sophus::SE3<Scalar>>& poses, const std::string& color)
{
  using plot::Plot;

  std::vector<Scalar> x;
  std::vector<Scalar> y;
  std::vector<Scalar> u;
  std::vector<Scalar> v;

  const Eigen::Vector3d forward_direction{Eigen::Vector3d::UnitX()};

  for (const auto& pose : poses)
  {
    const auto& translation = pose.translation();

    x.push_back(translation.x());
    y.push_back(translation.y());

    const auto& direction = pose.so3() * forward_direction;

    u.push_back(direction.x());
    v.push_back(direction.y());
  }

  Plot::plot(x, y, "." + color, {{"markersize", "12"}});
  Plot::quiver(x, y, u, v, color);
}

void plotPose2d(const std::vector<Sophus::SE3d>& poses, const std::string& color)
{
  plotPose2d<double>(poses, color);
}

}
