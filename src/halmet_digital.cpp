#include "halmet_digital.h"
#include "sensesp/transforms/moving_average.h" 
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/ui/config_item.h"

using namespace sensesp;

const float kDefaultFrequencyScale = 1 / 5.;

FloatProducer* ConnectTachoSender(int pin, String name) {
  char config_path[80];
  char sk_path[80];
  char config_title[80];
  char config_description[80];

  snprintf(config_path, sizeof(config_path), "", name.c_str());
  snprintf(config_title, sizeof(config_title), "Tacho %s Pin", name.c_str());
  snprintf(config_description, sizeof(config_description), "Tacho %s Input Pin",
           name.c_str());
  auto tacho_input =
      new DigitalInputCounter(pin, INPUT, RISING, 500, config_path);

  ConfigItem(tacho_input)
      ->set_title(config_title)
      ->set_description(config_description);

  snprintf(config_path, sizeof(config_path), "/Tacho %s/Revolution Multiplier",
           name.c_str());
  snprintf(config_title, sizeof(config_title), "Tacho %s Multiplier",
           name.c_str());
  snprintf(config_description, sizeof(config_description),
           "Tacho %s Multiplier", name.c_str());
  auto tacho_frequency = new Frequency(kDefaultFrequencyScale, config_path);

  tacho_input->connect_to(tacho_frequency);
// configure a smoothing window of N samples (e.g. 10)
  snprintf(config_path, sizeof(config_path),
           "/Tacho %s/Smoothing window", name.c_str());
  snprintf(config_title, sizeof(config_title),
           "Tacho %s Smoothing window", name.c_str());
  snprintf(config_description, sizeof(config_description),
           "Number of samples to average for smoothing RPM", name.c_str());
  

  auto tacho_smoother = new MovingAverage(5, 1.0, config_path);
  
  ConfigItem(tacho_smoother)
      ->set_title(config_title)
      ->set_description(config_description);
  // connect the tacho frequency to the smoother
  tacho_frequency->connect_to(tacho_smoother);

  snprintf(config_path, sizeof(config_path), "/Tacho %s/Revolutions SK Path",
           name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.revolutions", name.c_str());
  snprintf(config_title, sizeof(config_title), "Tacho %s Signal K Path",
           name.c_str());
  snprintf(config_description, sizeof(config_description),
           "Tacho %s Signal K Path", name.c_str());

  auto tacho_frequency_sk_output = new SKOutputFloat(sk_path, config_path);

  ConfigItem(tacho_frequency_sk_output)
      ->set_title(config_title)
      ->set_description(config_description);

  tacho_smoother->connect_to(tacho_frequency_sk_output);

  return tacho_frequency;
}

BoolProducer* ConnectAlarmSender(int pin, String name) {
  char config_path[80];
  char sk_path[80];
  char config_title[80];
  char config_description[80];

  auto* alarm_input = new DigitalInputState(pin, INPUT, 100);

#ifdef ENABLE_SIGNALK
  snprintf(config_path, sizeof(config_path), "/Alarm %s/SK Path", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "alarm.%s", name.c_str());
  snprintf(config_title, sizeof(config_title), "Alarm %s Signal K Path",
           name.c_str());
  snprintf(config_description, sizeof(config_description),
           "Alarm %s Signal K Path", name.c_str());

  auto alarm_sk_output = new SKOutputBool(sk_path, config_path);

  ConfigItem(alarm_sk_output)
      ->set_title(config_title)
      ->set_description(config_description);

  alarm_input->connect_to(alarm_sk_output);
#endif

  return alarm_input;
}
