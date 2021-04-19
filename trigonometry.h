#pragma once

#include <cmath>

namespace trig {
  constexpr double pi = 3.14159265358979323846;

  template<typename Scalar>
  constexpr inline Scalar deg2rad(const Scalar &deg) {
    return static_cast<Scalar>(deg * (pi / 180));
  }

  template<typename Scalar>
  constexpr inline Scalar rad2deg(const Scalar &rad) {
    return static_cast<Scalar>(rad * (180 / pi));
  }
}