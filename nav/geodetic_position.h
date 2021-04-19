#pragma once


namespace nav
{

/// \brief Geodetic position
struct GeodeticPosition
{
  /// \brief Constructs a geodetic position at (0, 0, 0).
  GeodeticPosition();

  /// \brief Constructs a geodetic position.
  /// \param lat Latitude in degrees.
  /// \param lon Longitude in degrees.
  /// \param alt Altitude in meters.
  GeodeticPosition(double lat, double lon, double alt);

  double latitude;  /// Latitude in degrees.
  double longitude; /// Longitude in degrees.
  double altitude;  /// Altitude in meters.
};

}

