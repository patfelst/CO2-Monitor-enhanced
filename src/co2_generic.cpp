//
//    FILE: co2_generic.cpp
//  AUTHOR: Patrick Felstead
// VERSION: 0.0.1
//    DATE: 2021-12-12
// PURPOSE: Generic class for any CO2 sensor
//
//
//  HISTORY:
//  0.0.1   2021-12-12  initial version
//

#include "co2_generic.h"

/////////////////////////////////////////////////////
//
// CONSTRUCTOR
//
CO2_generic::CO2_generic() {
  co2_level = 0;
  temperature = 0.0;
  humidity = 0.0;
  simulate_co2 = false;
  co2_updated = false;
}

bool CO2_generic::begin() {
  bool begin_ok = false;

#if defined SENSOR_IS_SCD30
  Wire.begin(CO2_SDA_PIN, CO2_SCL_PIN);                       // Could use Wire1 here (2nd I2C port on ESP32)
  begin_ok = co2_sensor.begin(Wire, true);  // Autocalibrate = true
#elif defined SENSOR_IS_SGP30
  begin_ok = co2_sensor.begin(CO2_SDA_PIN, CO2_SCL_PIN);
  co2_sensor.GenericReset();
#elif defined SENSOR_IS_SCD41
  uint16_t retries = 0;
  do {
    begin_ok = Wire.begin(CO2_SDA_PIN, CO2_SCL_PIN);  // Could use Wire1 here (2nd I2C port on ESP32)
    // Serial.printf("SCD-41 Wire.begin() = %s\n", begin_ok ? "ok" : "not ok");
    co2_sensor.begin(Wire);
    co2_sensor.stopPeriodicMeasurement();  // In case ESP32 just reset and SCD-41 already sending periodic updates
    begin_ok = (bool)(co2_sensor.startPeriodicMeasurement() == 0);
    Serial.printf("SCD-41 begin() = %s\n", begin_ok ? "ok" : "not ok");
    delay(10);
  } while (!begin_ok && retries++ < 2);

#endif

  return begin_ok;
}

bool CO2_generic::get_co2(void) {
/*************************************************************
                            SCD-30
*************************************************************/
#if defined SENSOR_IS_SCD30
  if (co2_sensor.dataAvailable()) {
    co2_level = co2_sensor.getCO2();
    temperature = co2_sensor.getTemperature();
    humidity = co2_sensor.getHumidity();
    co2_updated = true;
    return true;
  } else
    return false;

/*************************************************************
                            SGP-30
*************************************************************/
#elif defined SENSOR_IS_SGP30
  static uint32_t last_time = 0;
  if (millis() > last_time + 1000) {
    last_time = millis();
    co2_sensor.request();
  }

  if (co2_sensor.read()) {
    co2_updated = true;
    co2_level = co2_sensor.getCO2();
    temperature = 0;  // Not supported by this sensor
    humidity = 0;     // Not supported by this sensor
    return true;
  } else
    return false;

/*************************************************************
                            SCD-41
*************************************************************/
#elif defined SENSOR_IS_SCD41
  uint16_t data_ready = 0;
  uint16_t error = co2_sensor.getDataReadyStatus(data_ready);
  data_ready &= 0x7FF;  // New data is ready when lower 11-bits is > 0
  if (error) {
    co2_level = 0;       // Zero displayed as NaN
    co2_updated = true;  // Force update of LCD
    Serial.println("Error reading CO2 sensor during getDataReadyStatus(). Restarting I2C bus...");
    delay(1000);
    // If I2C sensor had bad connection, that has now come good, need to restart I2C bus
    begin();
  } else if (data_ready) {
    error = co2_sensor.readMeasurement(co2_level, temperature, humidity);
    if (error) {
      Serial.println("Error reading CO2 sensor during readMeasurement()");
      co2_level = 0;
    } else {
      co2_updated = true;
    }
  }
  return (data_ready > 0);

#endif
}

void CO2_generic::factory_reset(void) {
#if defined SENSOR_IS_SCD30
  // Not supported by this sensor

#elif defined SENSOR_IS_SGP30
  // Not supported by this sensor

#elif defined SENSOR_IS_SCD41
  // Reset to clear out previous calibration
  co2_sensor.stopPeriodicMeasurement();
  co2_sensor.performFactoryReset();
  delay(10000);  // Required by Sensirion SCD-41 datasheet
  co2_sensor.startPeriodicMeasurement();
#endif
}

int16_t CO2_generic::calibrate(uint16_t target) {
#if defined SENSOR_IS_SCD30
  bool cmd_ok = false;
  uint16_t FRC_fact;

  cmd_ok = co2_sensor.setForcedRecalibrationFactor(target);
  delay(400);  // Required by Sensirion SCD-30 datasheet
  if (cmd_ok) {
    co2_sensor.getForcedRecalibration(&FRC_fact);
    Serial.printf("%s calibration correction factor FRC=%d\n", co2_sensor_type_str, FRC_fact);
    return (int16_t)FRC_fact;
  } else
    return 0;

#elif defined SENSOR_IS_SGP30
  // There is no calibration function for SGP30. It is possible to read baseline value and store in EEPROM
  // and then read this stored value at power on reset and set it in the sensor
  return false;

#elif defined SENSOR_IS_SCD41
  const uint16_t correct_shift = 0x8000;
  uint16_t correction = 0;
  uint16_t error = co2_sensor.stopPeriodicMeasurement();
  delay(500);  // Required by Sensirion SCD-41 datasheet
  error = co2_sensor.performForcedRecalibration(target, correction);
  delay(400);  // Required by Sensirion SCD-41 datasheet
  co2_sensor.startPeriodicMeasurement();
  // Correction is = correction - 0x8000 = correction - correct_shift
  Serial.printf("%s cal error code=%d, correction=%d, [correction-%d]=%d\n",
                co2_sensor_type_str, error, correction, correct_shift, correction - correct_shift);
  if (error == 0 && correction != 0xFFFF)
    return (correction - correct_shift);  // Success
  else
    return 0;  // Failure

#endif
}

bool CO2_generic::set_co2_device_settings(float t_offset, uint16_t altitude, bool asc) {
#if defined SENSOR_IS_SCD30
  bool cmd_ok = false;
  // Note it takes some time for SCD-30 to apply this offset, give it a few minutes!
  cmd_ok = co2_sensor.setTemperatureOffset(t_offset);
  Serial.printf("Set temperature offset command: %s\n", cmd_ok ? "OK" : "ERROR");
  if (!cmd_ok) return false;

  delay(100);
  cmd_ok = co2_sensor.setAltitudeCompensation(altitude);
  Serial.printf("Set altitude compensation command: %s\n", cmd_ok ? "OK" : "ERROR");
  if (!cmd_ok) return false;

  // This is redundant becuase co2_sensor.begin() lets you specify if ASC is ON or OFF
  delay(100);
  cmd_ok = co2_sensor.setAutoSelfCalibration(asc);
  Serial.printf("Set ASC command: %s\n", cmd_ok ? "OK" : "ERROR");
  return cmd_ok;

#elif defined SENSOR_IS_SGP30
  // Not supported by this sensor

  return true;

#elif defined SENSOR_IS_SCD41
  uint16_t error = false;

  // Stop potentially previously started measurement - prevents a I2C "NACK" reponse with .startPeriodicMeasurement()
  co2_sensor.stopPeriodicMeasurement();
  error = co2_sensor.setTemperatureOffset(t_offset);
  Serial.printf("Set temperature offset command: %s\n", error == 0 ? "OK" : "ERROR");
  if (error) return false;

  error = co2_sensor.setSensorAltitude(altitude);
  Serial.printf("Set altitude compensation command: %s\n", error == 0 ? "OK" : "ERROR");
  if (error) return false;

  error = co2_sensor.setAutomaticSelfCalibration((uint16_t)asc);
  Serial.printf("Set ASC command: %s\n", error == 0 ? "OK" : "ERROR");

  if (error) {
    return false;
  } else {
    // Save new settings to SCD-41 EEPROM
    co2_sensor.persistSettings();
    delay(100);  // Just to ensure the write has occurred
    co2_sensor.startPeriodicMeasurement();
    return true;
  }
#endif
}

bool CO2_generic::get_co2_device_settings(float &t_offset, uint16_t &altitude, bool &asc) {
#if defined SENSOR_IS_SCD30
  delay(50);  // Need a small delay for SCD-30 temperature offset, otherwise reads zero...why?
  t_offset = co2_sensor.getTemperatureOffset();
  altitude = co2_sensor.getAltitudeCompensation();
  asc = co2_sensor.getAutoSelfCalibration();
  return true;

#elif defined SENSOR_IS_SGP30
  t_offset = 0;
  altitude = 0;
  asc = false;
  return true;

#elif defined SENSOR_IS_SCD41
  uint16_t error = false;
  uint16_t _asc;
  co2_sensor.stopPeriodicMeasurement();
  error = co2_sensor.getTemperatureOffset(t_offset);
  if (error) return false;
  error = co2_sensor.getSensorAltitude(altitude);
  if (error) return false;
  error = co2_sensor.getAutomaticSelfCalibration(_asc);
  asc = (bool)_asc;
  if (error)
    return false;
  else {
    co2_sensor.startPeriodicMeasurement();
    return true;
  }

#endif
}

void CO2_generic::sim_sensor(void) {
  static bool co2_rising = true;
  static bool temp_rising = true;
  static bool hum_rising = true;

  // Simulate CO2
  if (co2_rising && co2_level > 6000)
    co2_rising = false;
  else if (!co2_rising && co2_level < 400)
    co2_rising = true;

  if (co2_rising)
    co2_level += 250 + random(-50, 50);
  else
    co2_level -= (250 + random(-50, 50));

  co2_updated = true;

  // Simulate temperature
  if (temp_rising && temperature > 45)
    temp_rising = false;
  else if (!temp_rising && temperature < 10)
    temp_rising = true;

  if (temp_rising)
    temperature += 10.0 + (float)random(-30, 30) / 10.0;
  else
    temperature -= (10.0 + (float)random(-30, 30) / 10.0);

  if (hum_rising && humidity > 100)
    hum_rising = false;
  else if (!hum_rising && humidity < 10)
    hum_rising = true;

  if (hum_rising)
    humidity += 15.0 + (float)random(-50, 50) / 10.0;
  else
    humidity -= (15.0 + (float)random(-50, 50) / 10.0);
}