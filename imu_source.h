#pragma once

#include "nav/navlab_utils.h"
#include <deque>

class ImuSource
{
public:
  ImuSource(const std::string& filename);

  bool hasNext() const;

  double nextTime();

  const nav::NavlabIMU& next() const;

  void pop();

private:
  std::deque<nav::NavlabIMU> data_;
};
