#pragma once

#include "geodetic_position.h"
#include "Eigen/Core"
#include <vector>

namespace nav
{

/// \brief Represents Navlab GPS data.
struct NavlabGPS
{
  double time;                /// Time in seconds (s)
  double fix;                 /// GPS fix
  double fix_flags;           /// GPS fix status flags
  Eigen::Vector3d pos_ecef;   /// ECEF position (m)
  double pos_ecef_acc;        /// 3D position accuracy estimate (m)
  Eigen::Vector3d vel_ecef;   /// ECEF Velocity (m/s)
  double vel_ecef_acc;        /// Speed Accuracy Estimate (m/s)
  double pos_dop;             /// Position DOP ([Dilution of precision](https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)))
  int num_sat;                /// Number of Satellites used in Nav Solution
  GeodeticPosition pos_geo;   /// Geodetic position (lat (deg), lon (deg), alt (m above sea level)
  double alt_ell;             /// Height (m above Ellipsoid)
  double hor_acc;             /// Horizontal Accuracy Estimate (m)
  double ver_acc;             /// Vertical Accuracy Estimate (m)

  /// \brief Reads binary Navlab GPS data from stream.
  /// The Navlab data file is assumed to be a table of doubles with these columns:
  /// - Column 1: Time in microseconds (us)
  /// - Column 2: GPS fix
  /// - Column 3: GPS fix status flags
  /// - Column 4: ECEF x-coordinate (cm)
  /// - Column 5: ECEF y-coordinate (cm)
  /// - Column 6: ECEF z-coordinate (cm)
  /// - Column 7: 3D position accuracy estimate (cm)
  /// - Column 8: ECEF X-Velocity (cm/s)
  /// - Column 9: ECEF Y-Velocity (cm/s)
  /// - Column 10: ECEF Z-Velocity (cm/s)
  /// - Column 11: Speed Accuracy Estimate (cm/s)
  /// - Column 12: Position DOP
  /// - Column 13: Number of Satellites used in Nav Solution
  /// - Column 14: Longitude (deg)
  /// - Column 15: Latitude (deg)
  /// - Column 16: Height (mm above Ellipsoid)
  /// - Column 17: Height (mm above sea level)
  /// - Column 18: Horizontal Accuracy Estimate (mm)
  /// - Column 19: Vertical Accuracy Estimate (mm)
  ///
  /// Conversion to seconds, meters and meters/second is performed.
  static NavlabGPS readFrom(std::istream& is);

  /// \brief Checks that element is valid.
  bool isValid() const;
};

/// \brief Represents Navlab GPS data.
struct NavlabIMU
{
  double time;          /// Time in seconds (s)
  double status;        /// Status word
  double temp;          /// Temperature (if available)
  Eigen::Vector3d drot; /// Delta rotations (rad)
  Eigen::Vector3d dvel; /// Delta velocities (m/s)
  double status2;       /// Status word 2 (if available)

  /// \brief Reads binary Navlab IMU data from stream.
  /// The Navlab data file is assumed to be a table of doubles with these columns:
  /// - Column 1: Time in microseconds (us)
  /// - Column 2: Status word
  /// - Column 3: Temperature
  /// - Column 4: Delta rotation X (rad)
  /// - Column 5: Delta rotation Y (rad)
  /// - Column 6: Delta rotation Z (rad)
  /// - Column 7: Delta velocity X (m/s)
  /// - Column 8: Delta velocity Y (m/s)
  /// - Column 9: Delta velocity Z (m/s)
  /// - Column 10: Status word 2
  ///
  /// Conversion to seconds is performed.
  static NavlabIMU readFrom(std::istream& is);
};

/// \brief Represents Navlab Barometer data.
struct NavlabBarometer
{
  double time;          /// Time in seconds (s)
  double pressure;      /// Pressure in hectopascals (hPa)

  /// \brief Reads binary Navlab Barometer data from stream.
  /// The Navlab data file is assumed to be a table of doubles with these columns:
  /// - Column 1: Time in microseconds (us)
  /// - Column 2: Barometer pressure in hectopascals (hPa)
  ///
  /// Conversion to seconds is performed.
  static NavlabBarometer readFrom(std::istream& is);
};

/// \brief Reads a binary Navlab GPS file.
/// \param file_path Path to binary Navlab GPS file.
/// \return Vector of GPS data.
/// \see NavlabGPS
std::vector<NavlabGPS> readBinaryGpsFile(const std::string& file_path);

/// \brief Reads a binary Navlab Barometer file.
/// \param file_path Path to binary Navlab Barometer file.
/// \return Vector of Barometer data.
/// \see NavlabBarometer
std::vector<NavlabBarometer> readBinaryBarometerFile(const std::string& file_path);

/// \brief Reads a binary Navlab IMU file.
/// \param file_path Path to binary Navlab IMU file.
/// \return Vector of IMU data.
/// \see NavlabIMU
std::vector<NavlabIMU> readBinaryImuFile(const std::string& file_path);

/// \brief Reads a binary Navlab TimeSynch file.
/// \param file_path Path to binary Navlab timestamp file.
/// \return Vector of timestamp data in seconds (s).
std::vector<double> readBinaryTimeSynchFile(const std::string& file_path);

/// \brief Reads a binary Navlab TimeSynch stream.
/// \param is Binary input stream.
/// \return Vector of timestamp data in seconds (s).
std::vector<double> readBinaryTimeSynchStream(std::istream& is);

}

