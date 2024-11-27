#ifndef NMEA2000HANDLER_H
#define NMEA2000HANDLER_H

#include <NMEA2000.h> // Include the necessary NMEA2000 library

class NMEA2000Handler {
public:
    NMEA2000Handler(tNMEA2000* nmea2000);
    static void HandleNMEA2000Msg(const tN2kMsg& N2kMsg);
};

#endif // NMEA2000HANDLER_H