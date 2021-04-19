#include "geodetic_position.h"

namespace nav
{

GeodeticPosition::GeodeticPosition()
    : latitude{}
    , longitude{}
    , altitude{}
{ }

GeodeticPosition::GeodeticPosition(double lat, double lon, double alt)
    : latitude{lat}
    , longitude{lon}
    , altitude{alt}
{ }

}

