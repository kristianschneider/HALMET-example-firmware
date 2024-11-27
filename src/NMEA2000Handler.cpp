#include "NMEA2000Handler.h"

#include <N2kDeviceList.h>
#include <NMEA2000.h>     // Include the necessary NMEA2000 library
#include <N2kMessages.h>
#include <sensesp_app.h>  // Include SensESP app header

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

tN2kDeviceList *locN2KDeviceList;

Stream *OutputStream;
void SystemTime(const tN2kMsg &N2kMsg);
void Rudder(const tN2kMsg &N2kMsg);
void EngineRapid(const tN2kMsg &N2kMsg);
void EngineDynamicParameters(const tN2kMsg &N2kMsg);
void TransmissionParameters(const tN2kMsg &N2kMsg);
void TripFuelConsumption(const tN2kMsg &N2kMsg);
void Speed(const tN2kMsg &N2kMsg);
void WaterDepth(const tN2kMsg &N2kMsg);
void BinaryStatus(const tN2kMsg &N2kMsg);
void FluidLevel(const tN2kMsg &N2kMsg);
void OutsideEnvironmental(const tN2kMsg &N2kMsg);
void Temperature(const tN2kMsg &N2kMsg);
void TemperatureExt(const tN2kMsg &N2kMsg);
void DCStatus(const tN2kMsg &N2kMsg);
void BatteryConfigurationStatus(const tN2kMsg &N2kMsg);
void COGSOG(const tN2kMsg &N2kMsg);
void GNSS(const tN2kMsg &N2kMsg);
void LocalOffset(const tN2kMsg &N2kMsg);
void Attitude(const tN2kMsg &N2kMsg);
void Heading(const tN2kMsg &N2kMsg);
void Humidity(const tN2kMsg &N2kMsg);
void Pressure(const tN2kMsg &N2kMsg);
void UserDatumSettings(const tN2kMsg &N2kMsg);
void GNSSSatsInView(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[] = {{126992L, &SystemTime},
                                       {127245L, &Rudder},
                                       {127250L, &Heading},
                                       {127257L, &Attitude},
                                       {127488L, &EngineRapid},
                                       {127489L, &EngineDynamicParameters},
                                       {127493L, &TransmissionParameters},
                                       {127497L, &TripFuelConsumption},
                                       {127501L, &BinaryStatus},
                                       {127505L, &FluidLevel},
                                       {127506L, &DCStatus},
                                       {127513L, &BatteryConfigurationStatus},
                                       {128259L, &Speed},
                                       {128267L, &WaterDepth},
                                       {129026L, &COGSOG},
                                       {129029L, &GNSS},
                                       {129033L, &LocalOffset},
                                       {129045L, &UserDatumSettings},
                                       {129540L, &GNSSSatsInView},
                                       {130310L, &OutsideEnvironmental},
                                       {130312L, &Temperature},
                                       {130313L, &Humidity},
                                       {130314L, &Pressure},
                                       {130316L, &TemperatureExt},
                                       {0, 0}};

// pass nmea2000 object to NMEA2000Handler constructor
NMEA2000Handler::NMEA2000Handler(tNMEA2000 *nmea2000) {
  locN2KDeviceList = new tN2kDeviceList(nmea2000);
}

void NMEA2000Handler::HandleNMEA2000Msg(const tN2kMsg& N2kMsg) {
  int iHandler;

  // Find handler

  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 &&
                     !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN);
       iHandler++);

  // If handler is in list, call it.
  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

//implement the handler functions and print debugD for the data
void SystemTime(const tN2kMsg &N2kMsg) {
  debugD("System Time");
}

void Rudder(const tN2kMsg &N2kMsg) {
  debugD("Rudder");
}

void EngineRapid(const tN2kMsg &N2kMsg) {
  debugD("Engine Rapid");
}

void EngineDynamicParameters(const tN2kMsg &N2kMsg) {
   unsigned char EngineInstance;
  double EngineOilPress;
  double EngineOilTemp;
  double EngineCoolantTemp;
  double AltenatorVoltage;
  double FuelRate;
  double EngineHours;
  double EngineCoolantPress;
  double EngineFuelPress;
  int8_t EngineLoad;
  int8_t EngineTorque;
  tN2kEngineDiscreteStatus1 Status1;
  tN2kEngineDiscreteStatus2 Status2;

  if (ParseN2kEngineDynamicParam(N2kMsg, EngineInstance, EngineOilPress, EngineOilTemp, EngineCoolantTemp,
                                 AltenatorVoltage, FuelRate, EngineHours,
                                 EngineCoolantPress, EngineFuelPress,
                                 EngineLoad, EngineTorque, Status1, Status2))
  {
    // debugD("Engine dynamic parameters: %d", EngineInstance);
    // debugD("  oil pressure (Pa): %f", EngineOilPress);
    // debugD("  oil temp (C): %f", EngineOilTemp);
    // debugD("  coolant temp (C): %f", EngineCoolantTemp);
    // debugD("  altenator voltage (V): %f", AltenatorVoltage);
    // debugD("  fuel rate (l/h): %f", FuelRate);
    // debugD("  engine hours (h): %f", EngineHours);
    // debugD("  coolant pressure (Pa): %f", EngineCoolantPress);
    // debugD("  fuel pressure (Pa): %f", EngineFuelPress);
    // debugD("  engine load (%): %f", EngineLoad);
    // debugD("  engine torque (%): %f", EngineTorque);
  } else {
    debugD("Failed to parse PGN: %lu", N2kMsg.PGN);
  }
}

void TransmissionParameters(const tN2kMsg &N2kMsg) {
  debugD("Transmission Parameters");
}

void TripFuelConsumption(const tN2kMsg &N2kMsg) {
     unsigned char EngineInstance;
  double TripFuelUsed;
  double FuelRateAverage;
  double FuelRateEconomy;
  double InstantaneousFuelEconomy;

  if (ParseN2kEngineTripParameters(N2kMsg, EngineInstance, TripFuelUsed, FuelRateAverage, FuelRateEconomy, InstantaneousFuelEconomy))
  {
    debugD("Trip fuel consumption: %d", EngineInstance);
    debugD("  fuel used (l): %d", TripFuelUsed);
    debugD("  average fuel rate (l/h): %d", FuelRateAverage);
    debugD("  economy fuel rate (l/h): %d", FuelRateEconomy);
    debugD("  instantaneous fuel economy (l/h): %d", InstantaneousFuelEconomy);
  }
  else
  {
    debugD("Failed to parse PGN: ");
    debugD("PGN: %lu", N2kMsg.PGN);
  }
  debugD("Trip Fuel Consumption");
}

void Speed(const tN2kMsg &N2kMsg) {
  debugD("Speed");
}

void WaterDepth(const tN2kMsg &N2kMsg) {
  debugD("Water Depth");
}

void BinaryStatus(const tN2kMsg &N2kMsg) {
  debugD("Binary Status");
}

void COGSOG(const tN2kMsg &N2kMsg) {
  debugD("COG SOG");
}

void GNSS(const tN2kMsg &N2kMsg) {
  debugD("GNSS");
}

void LocalOffset(const tN2kMsg &N2kMsg) {
  debugD("Local Offset");
}

void UserDatumSettings(const tN2kMsg &N2kMsg) {
  debugD("User Datum Settings");
}

void GNSSSatsInView(const tN2kMsg &N2kMsg) {
  debugD("GNSS Sats In View");
}

void OutsideEnvironmental(const tN2kMsg &N2kMsg) {
  debugD("Outside Environmental");
}

void Temperature(const tN2kMsg &N2kMsg) {
  debugD("Temperature");
}

void Humidity(const tN2kMsg &N2kMsg) {
  debugD("Humidity");
}

void Pressure(const tN2kMsg &N2kMsg) {
  debugD("Pressure");
}

void TemperatureExt(const tN2kMsg &N2kMsg) {
  debugD("Temperature Ext");
}

void Heading(const tN2kMsg &N2kMsg) {
  debugD("Heading");
}

void Attitude(const tN2kMsg &N2kMsg) {
  debugD("Attitude");
}

void FluidLevel(const tN2kMsg &N2kMsg) {
  debugD("Fluid Level");
}

void DCStatus(const tN2kMsg &N2kMsg) {
  debugD("DC Status");
}

void BatteryConfigurationStatus(const tN2kMsg &N2kMsg) {
  debugD("Battery Configuration Status");
}