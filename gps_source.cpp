#include "gps_source.h"

GpsSource::GpsSource(const std::string& filename)
{
  auto data = nav::readBinaryGpsFile(filename);

  for (auto& data_elem : data)
  {
    data_.emplace_back(std::move(data_elem));
  }
}

bool GpsSource::hasNext() const
{
  return !data_.empty();
}

double GpsSource::nextTime()
{
  if (data_.empty())
  { return std::numeric_limits<double>::infinity(); }

  return data_.front().time;
}

const nav::NavlabGPS& GpsSource::next() const
{
  if (data_.empty())
  { throw std::out_of_range("Data is hasNext"); }

  return data_.front();
}

void GpsSource::pop()
{
  if (data_.empty())
  { throw std::out_of_range("Data is hasNext"); }

  data_.pop_front();
}
