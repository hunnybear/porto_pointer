#include <GPS_Nav.h>

//testing max longitude = -88.240342
//testing min longitude = -88.272228
//testing max latitude = 40.119949
//testing min latitude = 40.098319

const int portos_count = 10;
gps_location portos[portos_count] = {
  gps_location{40.1048130593, -88.2547982305},
  gps_location{40.1070574655, -88.2714607282},
  gps_location{40.1178026368, -88.2471423981},
  gps_location{40.1059937712, -88.246202169},
  gps_location{40.1015551819, -88.2444249443},
  gps_location{40.109012835, -88.2680116205},
  gps_location{40.1167460592, -88.2454060974},
  gps_location{40.0983801511, -88.2479918665},
  gps_location{40.1020113597, -88.2694096506},
  gps_location{40.1096149973, -88.2434922247},
};

gps_location TEST_WEST[1] = {{40.106198, -88.252949}};
gps_location TEST_EAST[1] = {{40.106198, -88.250116}};
gps_location TEST_NORTH[1] = {{40.106838, -88.251967}};
gps_location TEST_SOUTH[1] = {{40.103757, -88.252122}};