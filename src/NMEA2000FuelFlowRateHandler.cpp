#include "NMEA2000FuelFlowRateHandler.h"

#include <N2kMessages.h>
#include <NMEA2000.h>     
#include <sensesp_app.h>  

void NMEA2000FuelFlowRateHandler::setSignalKSender(
    std::function<void(const std::string&, float)> sender) {
  signalKSender = sender;
}

void NMEA2000FuelFlowRateHandler::EngineDynamicParameters(const tN2kMsg &N2kMsg) {
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

  if (ParseN2kEngineDynamicParam(N2kMsg, EngineInstance, EngineOilPress,
                                 EngineOilTemp, EngineCoolantTemp,
                                 AltenatorVoltage, FuelRate, EngineHours,
                                 EngineCoolantPress, EngineFuelPress,
                                 EngineLoad, EngineTorque, Status1, Status2)) {
    // debugD("Engine dynamic parameters: %d", EngineInstance);
    // debugD("  oil pressure (Pa): %f", EngineOilPress);
    // debugD("  oil temp (C): %f", EngineOilTemp);
    // debugD("  coolant temp (C): %f", EngineCoolantTemp);
    // debugD("  altenator voltage (V): %f", AltenatorVoltage);
      // debugI("fuel rate (l/h): %f", FuelRate);
    // debugD("  engine hours (h): %f", EngineHours);
    // debugD("  coolant pressure (Pa): %f", EngineCoolantPress);
    // debugD("  fuel pressure (Pa): %f", EngineFuelPress);
    // debugD("  engine load (%): %f", EngineLoad);
    // debugD("  engine torque (%): %f", EngineTorque)

    // We need to convert the fuel rate from l/h to m3/s
    //  1 l/h = 0.000277778 m3/s
    FuelRate = FuelRate / 3600.0;  // Convert l/h to l/s
    FuelRate = FuelRate * 0.001;   // Convert l/s to m3/s
    // Call the callback function with the converted fuel rate
    sendToSignalK("propulsion.engine.fuel.rate", FuelRate);
  } else {
    debugD("Failed to parse PGN: %lu", N2kMsg.PGN);
  }
}

void NMEA2000FuelFlowRateHandler::TripFuelConsumption(const tN2kMsg &N2kMsg) {
  unsigned char EngineInstance;
  double TripFuelUsed;
  double FuelRateAverage;
  double FuelRateEconomy;
  double InstantaneousFuelEconomy;

  if (ParseN2kEngineTripParameters(N2kMsg, EngineInstance, TripFuelUsed,
                                   FuelRateAverage, FuelRateEconomy,
                                   InstantaneousFuelEconomy)) {
    // debugI("Trip fuel consumption: %d", EngineInstance);
    //debugI("  fuel used (l): %f", TripFuelUsed);
    // debugI("  average fuel rate (l/h): %f", FuelRateAverage);
    // debugI("  economy fuel rate (l/h): %f", FuelRateEconomy);
    // debugI("  instantaneous fuel economy (l/h): %f",
    // InstantaneousFuelEconomy);
  } else {
    debugD("Failed to parse PGN: ");
    debugD("PGN: %lu", N2kMsg.PGN);
  }
}

void NMEA2000FuelFlowRateHandler::sendToSignalK(const std::string &path, float value) {
  if (signalKSender) {
    signalKSender(path, value);
  } else {
    debugD("SignalK sender not set. Cannot send data.");
  }
}
