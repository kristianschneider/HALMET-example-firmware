// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <NMEA2000_esp32.h>

#include "debug_gps.h"
#include "n2k_senders.h"
#include "sensesp/net/discovery.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/system_status_led.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/wiring.h"
#include "sensesp_onewire/onewire_temperature.h"


// #ifdef ENABLE_SIGNALK
#include "sensesp_app_builder.h"
#define BUILDER_CLASS SensESPAppBuilder
// #else
// #include "sensesp_minimal_app_builder.h"
// #endif

#include "Arduino.h"
#include "NMEA2000Handler.h"
#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"

using namespace sensesp;
using namespace halmet;
using namespace sensesp::nmea0183;
using namespace sensesp::onewire;


// #ifndef ENABLE_SIGNALK
// #define BUILDER_CLASS SensESPMinimalAppBuilder
// SensESPMinimalApp* sensesp_app;
// Networking* networking;
// MDNSDiscovery* mdns_discovery;
// HTTPServer* http_server;
// SystemStatusLed* system_status_led;
// #endif

///////////// GPS serial config /////////////
constexpr int kGNSSBitRate = 9600;
constexpr int kGNSSRxPin = 13;
// set the Tx pin to -1 if you don't want to use it
constexpr int kGNSSTxPin = 15;

///////////// DS18S20  config /////////////
const int kDQPin = 4;
uint onewire_read_delay = 1000;



elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;
TwoWire* i2c;
Adafruit_SSD1306* display;
tNMEA2000* nmea2000;



// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

// Set the ADS1115 GAIN to adjust the analog input voltage range.
// On HALMET, this refers to the voltage range of the ADS1115 input
// AFTER the 33.3/3.3 voltage divider.

// GAIN_TWOTHIRDS: 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// GAIN_ONE:       1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// GAIN_TWO:       2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// GAIN_FOUR:      4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// GAIN_EIGHT:     8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// GAIN_SIXTEEN:   16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

const adsGain_t kADS1115Gain = GAIN_ONE;

/////////////////////////////////////////////////////////////////////
// Test output pin configuration. If ENABLE_TEST_OUTPUT_PIN is defined,
// GPIO 33 will output a pulse wave at 380 Hz with a 50% duty cycle.
// If this output and GND are connected to one of the digital inputs, it can
// be used to test that the frequency counter functionality is working.
//#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_33;
// With the default pulse rate of 100 pulses per revolution (configured in
// halmet_digital.cpp), this frequency corresponds to 3.8 r/s or about 228 rpm.
const int kTestOutputFrequency = 380;
#endif

/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // These calls can be used for fine-grained control over the logging level.
  // esp_log_level_set("*", esp_log_level_t::ESP_LOG_DEBUG);

  Serial.begin(115200);

  /////////////////////////////////////////////////////////////////////
  // Initialize the application framework

  // Construct the global SensESPApp() object
  BUILDER_CLASS builder;
  sensesp_app = (&builder)
                    // EDIT: Set a custom hostname for the app.
                    ->set_hostname("halmet")
                    // EDIT: Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    // EDIT: Enable OTA updates with a password.
                    //->enable_ota("my_ota_password")
                    ->get_app();

  
  //Setup GPS serial port
  NMEAGPS();

  OneWire();

  NMEA2K();

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();

  ads1115->setGain(kADS1115Gain);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);


  // #ifndef ENABLE_SIGNALK
  // Initialize components that would normally be present in SensESPApp
  // networking = new Networking("/System/WiFi Settings", "", "");
  // ConfigItem(networking);
  // mdns_discovery = new MDNSDiscovery();
  // http_server = new HTTPServer();
  // system_status_led = new SystemStatusLed(LED_BUILTIN);
  // #endif


  ///////////////////////////////////////////////////////////////////
  // Analog inputs



  // Connect the tank senders.
  // EDIT: To enable more tanks, uncomment the lines below.
  // auto tank_a1_volume = ConnectTankSender(ads1115, 0, "Fuel", "fuel.main", 3000,
  //                                         enable_signalk_output);
  // auto tank_a2_volume = ConnectTankSender(ads1115, 1, "A2");
  // auto tank_a3_volume = ConnectTankSender(ads1115, 2, "A3");
  // auto tank_a4_volume = ConnectTankSender(ads1115, 3, "A4");


  // if (display_present) {
  //   // EDIT: Duplicate the lines below to make the display show all your tanks.
  //   tank_a1_volume->connect_to(new LambdaConsumer<float>(
  //       [](float value) { PrintValue(display, 2, "Tank A1", 100 * value); }));
  // }

  // Read the voltage level of analog input A2
  auto a2_voltage = new ADS1115VoltageInput(ads1115, 1, "/Voltage A2");

  ConfigItem(a2_voltage)
      ->set_title("Analog Voltage A2")
      ->set_description("Voltage level of analog input A2")
      ->set_sort_order(3000);

  // a2_voltage->connect_to(new LambdaConsumer<float>(
  //     [](float value) { debugD("Voltage A2: %f", value); }));

  // If you want to output something else than the voltage value,
  // you can insert a suitable transform here.
  // For example, to convert the voltage to a distance with a conversion
  // factor of 0.17 m/V, you could use the following code:
  // auto a2_distance = new Linear(0.17, 0.0);
  // a2_voltage->connect_to(a2_distance);

// #ifdef ENABLE_SIGNALK
  a2_voltage->connect_to(
      new SKOutputFloat("Analog Voltage A2", "sensors.a2.voltage",
                        new SKMetadata("Analog Voltage A2", "V")));
  // Example of how to output the distance value to Signal K.
  // a2_distance->connect_to(
  //     new SKOutputFloat("Analog Distance A2", "sensors.a2.distance",
  //                       new SKMetadata("Analog Distance A2", "m")));
// #endif

  ///////////////////////////////////////////////////////////////////
  // Digital alarm inputs

  // EDIT: More alarm inputs can be defined by duplicating the lines below.
  // Make sure to not define a pin for both a tacho and an alarm.
  // auto alarm_d2_input = ConnectAlarmSender(kDigitalInputPin2, "D2");
  // auto alarm_d3_input = ConnectAlarmSender(kDigitalInputPin3, "D3");
  // // auto alarm_d4_input = ConnectAlarmSender(kDigitalInputPin4, "D4");

  // Update the alarm states based on the input value changes.
  // EDIT: If you added more alarm inputs, uncomment the respective lines below.
  // alarm_d2_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  // // In this example, alarm_d3_input is active low, so invert the value.
  // auto alarm_d3_inverted = alarm_d3_input->connect_to(
  //     new LambdaTransform<bool, bool>([](bool value) { return !value; }));
  // alarm_d3_inverted->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  // alarm_d4_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));



  // FIXME: Transmit the alarms over SK as well.

  ///////////////////////////////////////////////////////////////////
  // Digital tacho inputs

  // Connect the tacho senders. Engine name is "main".
  // EDIT: More tacho inputs can be defined by duplicating the line below.
  auto tacho_d1_frequency = ConnectTachoSender(kDigitalInputPin1, "main");


  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void NMEA2K() {
  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality
  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);
  nmea2000->SetMsgHandler(NMEA2000Handler::HandleNMEA2000Msg);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "20231229",  // Manufacturer's Model serial code (max 32 chars)
      104,         // Manufacturer's product code
      "HALMET",    // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      140,                     // Device function: Engine
      50,                      // Device class: Propulsion
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly,
                    71  // Default N2k node address
  );
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  event_loop()->onRepeat(1, []() {
    nmea2000->ParseMessages();
  });
}

void NMEAGPS() {
  HardwareSerial* serial = &Serial1;
  serial->begin(kGNSSBitRate, SERIAL_8N1, kGNSSRxPin, kGNSSTxPin);
  NMEA0183IOTask* nmea0183_io_task = new NMEA0183IOTask(serial);
  ConnectGNSS(&nmea0183_io_task->parser_, new GNSSData());
}

void OneWire() {
  // Setup dallas temperature sensors
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(kDQPin);
  // Measure exhaust temperature 1
  auto* exhaust_temp = new OneWireTemperature(dts, onewire_read_delay,"/exhaustTemperature1/oneWire");
  auto* exhaust_temp_calibration = new Linear(1.0, 0.0, "/exhaustTemperature1/linear");
  auto* exhaust_temp_sk_output = new SKOutputFloat("propulsion.main.exhaustTemperature1", "/exhaustTemperature1/skPath");

  exhaust_temp->connect_to(exhaust_temp_calibration)->connect_to(exhaust_temp_sk_output);

  // Measure exhaust temperature 2
  auto* exhaust_temp2 = new OneWireTemperature(dts, onewire_read_delay,"/exhaustTemperature2/oneWire");
  auto* exhaust_temp_calibration2 = new Linear(1.0, 0.0, "/exhaustTemperature2/linear");
  auto* exhaust_temp_sk_output2 = new SKOutputFloat("propulsion.main.exhaustTemperature2", "/exhaustTemperature2/skPath");
  exhaust_temp2->connect_to(exhaust_temp_calibration2)->connect_to(exhaust_temp_sk_output2);

  // Measure engine temperature 3
  auto* engine_temp = new OneWireTemperature(dts, onewire_read_delay,"/engineTemperature1/oneWire");
  auto* engine_temp_calibration = new Linear(1.0, 0.0, "/engineTemperature1/linear");
  auto* engine_temp_sk_output = new SKOutputFloat("propulsion.main.engineTemperature1", "/engineTemperature1/skPath");
  engine_temp->connect_to(engine_temp_calibration)->connect_to(engine_temp_sk_output);
}

void loop() { event_loop()->tick(); }
