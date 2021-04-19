#include "imu_source.h"

ImuSource::ImuSource(const std::string& filename)
{
  auto data = nav::readBinaryImuFile(filename);

  for (auto& data_elem : data)
  {
    // Perform axis rearrangement.
    data_elem.drot.y() = -data_elem.drot.y();
    data_elem.drot.z() = -data_elem.drot.z();
    data_elem.dvel.y() = -data_elem.dvel.y();
    data_elem.dvel.z() = -data_elem.dvel.z();

    data_.emplace_back(std::move(data_elem));
  }
}

bool ImuSource::hasNext() const
{
  return !data_.empty();
}

double ImuSource::nextTime()
{
  if (data_.empty())
  { return std::numeric_limits<double>::infinity(); }

  return data_.front().time;
}

const nav::NavlabIMU& ImuSource::next() const
{
  if (data_.empty())
  { throw std::out_of_range("Data is hasNext"); }

  return data_.front();
}

void ImuSource::pop()
{
  if (data_.empty())
  { throw std::out_of_range("Data is hasNext"); }

  data_.pop_front();
}
