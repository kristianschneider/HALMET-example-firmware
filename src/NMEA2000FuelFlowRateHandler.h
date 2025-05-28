#ifndef NMEA2000FuelFlowRateHandler_H
#define NMEA2000FuelFlowRateHandler_H
#include <string>
#include <NMEA2000.h>  // Include the necessary NMEA2000 library

#include <functional>  // For std::function

class NMEA2000FuelFlowRateHandler {
 public:
  void EngineDynamicParameters(const tN2kMsg& N2kMsg);
  void TripFuelConsumption(const tN2kMsg& N2kMsg);
  // Method to set a callback for sending data to SignalK
  void setSignalKSender(std::function<void(const std::string&, float)> sender);

 private:
  // Callback function to send data to SignalK
  std::function<void(const std::string&, float)> signalKSender;

  // Helper methods to process and send data
  void sendToSignalK(const std::string& path, float value);
};

#endif  // NMEA2000FuelFlowRateHandler_H