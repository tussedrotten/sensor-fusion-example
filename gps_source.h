#pragma once

#include "nav/navlab_utils.h"
#include <deque>

class GpsSource
{
public:
  GpsSource(const std::string& filename);

  bool hasNext() const;

  double nextTime();

  const nav::NavlabGPS& next() const;

  void pop();

private:
  std::deque<nav::NavlabGPS> data_;
};
