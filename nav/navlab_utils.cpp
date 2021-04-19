#include "navlab_utils.h"
#include <fstream>

namespace nav
{

NavlabGPS NavlabGPS::readFrom(std::istream& is)
{
  // Read entire row of 19 doubles.
  std::array<double, 19> raw_data;
  is.read(reinterpret_cast<char*>(raw_data.data()), raw_data.max_size() * sizeof(double));

  // Conversion factors.
  constexpr double us_to_s = 1e-6;
  constexpr double cm_to_m = 1e-2;
  constexpr double mm_to_m = 1e-3;

  NavlabGPS gps;
  gps.time = us_to_s * raw_data[0];
  gps.fix = raw_data[1];
  gps.fix_flags = raw_data[2];
  gps.pos_ecef = cm_to_m * Eigen::Vector3d(raw_data[3], raw_data[4], raw_data[5]);
  gps.pos_ecef_acc = cm_to_m * raw_data[6];
  gps.vel_ecef = cm_to_m * Eigen::Vector3d(raw_data[7], raw_data[8], raw_data[9]);
  gps.vel_ecef_acc = cm_to_m * raw_data[10];
  gps.pos_dop = raw_data[11];
  gps.num_sat = static_cast<int>(std::round(raw_data[12]));
  gps.pos_geo = GeodeticPosition(raw_data[14], raw_data[13], mm_to_m * raw_data[16]);
  gps.alt_ell = mm_to_m * raw_data[15];
  gps.hor_acc = mm_to_m * raw_data[17];
  gps.ver_acc = mm_to_m * raw_data[18];

  if (!is)
  {
    // istream is empty, so invalidate data
    gps.fix_flags = std::numeric_limits<double>::quiet_NaN();
  }

  return gps;
}

bool NavlabGPS::isValid() const
{
  // Follows check used in navlab.
  return
      fix >= 3.0 && fix <= 4.0 &&
      !std::isnan(fix_flags) &&
      pos_dop >= 0 && pos_dop <= 50.0 &&
      num_sat >= 4;
}

NavlabBarometer NavlabBarometer::readFrom(std::istream& is)
{
  // Read entire row of 2 doubles.
  std::array<double, 2> raw_data;
  is.read(reinterpret_cast<char*>(raw_data.data()), raw_data.max_size() * sizeof(double));

  // Conversion factors.
  constexpr double us_to_s = 1e-6;

  NavlabBarometer barometer;
  barometer.time = us_to_s * raw_data[0];
  barometer.pressure = raw_data[1];

  return barometer;
}

NavlabIMU NavlabIMU::readFrom(std::istream& is)
{
  // Read entire row of 19 doubles.
  std::array<double, 10> raw_data;
  is.read(reinterpret_cast<char*>(raw_data.data()), raw_data.max_size() * sizeof(double));

  // Conversion factors.
  constexpr double us_to_s = 1e-6;

  NavlabIMU imu;
  imu.time = us_to_s * raw_data[0];
  imu.status = raw_data[1];
  imu.temp = raw_data[2];
  imu.drot = Eigen::Vector3d(raw_data[3], raw_data[4], raw_data[5]);
  imu.dvel = Eigen::Vector3d(raw_data[6], raw_data[7], raw_data[8]);
  imu.status2 = raw_data[9];

  return imu;
}

std::vector<NavlabGPS> readBinaryGpsFile(const std::string& file_path)
{
  std::ifstream binary_file(file_path, std::ios::binary);

  std::vector<NavlabGPS> gps_data;
  for (NavlabGPS gps = NavlabGPS::readFrom(binary_file); binary_file; gps = NavlabGPS::readFrom(binary_file))
  {
    gps_data.push_back(gps);
  }

  return gps_data;
}

std::vector<NavlabBarometer> readBinaryBarometerFile(const std::string& file_path)
{
  std::ifstream bin_file(file_path, std::ios::binary);

  std::vector<NavlabBarometer> barometer_data;
  for (NavlabBarometer bar = NavlabBarometer::readFrom(bin_file); bin_file; bar = NavlabBarometer::readFrom(bin_file))
  {
    barometer_data.push_back(bar);
  }

  return barometer_data;
}

std::vector<NavlabIMU> readBinaryImuFile(const std::string& file_path)
{
  std::ifstream binary_file(file_path, std::ios::binary);

  std::vector<NavlabIMU> imu_data;
  for (NavlabIMU imu = NavlabIMU::readFrom(binary_file); binary_file; imu = NavlabIMU::readFrom(binary_file))
  {
    imu_data.push_back(imu);
  }

  return imu_data;
}

std::vector<double> readBinaryTimeSynchFile(const std::string& file_path)
{
  std::ifstream binary_file{file_path, std::ios::binary | std::ios::ate};
  return readBinaryTimeSynchStream(binary_file);
}

std::vector<double> readBinaryTimeSynchStream(std::istream& is)
{
  std::vector<double> timestamp_data;

  // Get position at end.
  is.seekg(0, std::ios::end);
  const auto size = static_cast<size_t>(is.tellg());

  const auto num_timestamps = size / sizeof(double);
  timestamp_data.resize(num_timestamps);

  // Seek back to start and read entire file to vector.
  is.seekg(0);
  is.read(reinterpret_cast<char*>(timestamp_data.data()), size);

  // Convert from microseconds to seconds.
  constexpr double us_to_s = 1e-6;
  for (auto& timestamp : timestamp_data)
  {
    timestamp *= us_to_s;
  }

  return timestamp_data;
}
}
