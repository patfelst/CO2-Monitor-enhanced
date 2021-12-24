#pragma once
//
//    FILE: co2_generic.h
//  AUTHOR: Patrick Felstead
// VERSION: 0.0.1
//    DATE: 2021-12-12
// PURPOSE: Generic class for any CO2 sensor
//

#include "Arduino.h"

// These are in m platformio.ini file, but you can set them here instead if preferred
// #define SENSOR_IS_SCD30
// #define SENSOR_IS_SGP30
// #define SENSOR_IS_SCD41

#if defined SENSOR_IS_SCD30
  #include "SparkFun_SCD30_Arduino_Library.h"
  #define co2_sensor_type_str "SCD-30"
#elif defined SENSOR_IS_SGP30
  #include "SGP30.h"
  #define co2_sensor_type_str "SGP-30"
#elif defined SENSOR_IS_SCD41
  #include "SensirionI2CScd4x.h"
  #define co2_sensor_type_str "SCD-41"
#endif

class CO2_generic {
 public:
  // Constructor
  CO2_generic(void);
  bool begin(void);
  bool get_co2(void);
  int16_t calibrate(uint16_t target);
  bool set_co2_device_settings(float t_offset, uint16_t altitude, bool asc);
  bool get_co2_device_settings(float &t_offset, uint16_t &altitude, bool &asc);
  void factory_reset(void);
  void sim_sensor(void);

// Create CO2 sensor object for the selected CO2 device
#if defined SENSOR_IS_SCD30
  SCD30 co2_sensor;
#elif defined SENSOR_IS_SGP30
  SGP30 co2_sensor;
#elif defined SENSOR_IS_SCD41
  SensirionI2CScd4x co2_sensor;
#endif

  uint16_t co2_level = 0;
  float temperature = 0.0;
  float humidity = 0.0;
  bool simulate_co2 = false;
  bool co2_updated = false;

 private:
  TwoWire *_wire;
};