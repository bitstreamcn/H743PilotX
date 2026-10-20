#include "sensor.h"


/**************************************************************************/
/*!
    @brief  Prints sensor information to serial console
*/
/**************************************************************************/
void Adafruit_Sensor::printSensorDetails(void) {
  sensor_t sensor;
  getSensor(&sensor);
  println(F("------------------------------------"));
  printf(F("Sensor:       "));
  println(sensor.name);
  printf(F("Type:         "));
  switch ((sensors_type_t)sensor.type) {
  case SENSOR_TYPE_ACCELEROMETER:
    printf(F("Acceleration (m/s2)"));
    break;
  case SENSOR_TYPE_MAGNETIC_FIELD:
    printf(F("Magnetic (uT)"));
    break;
  case SENSOR_TYPE_ORIENTATION:
    printf(F("Orientation (degrees)"));
    break;
  case SENSOR_TYPE_GYROSCOPE:
    printf(F("Gyroscopic (rad/s)"));
    break;
  case SENSOR_TYPE_LIGHT:
    printf(F("Light (lux)"));
    break;
  case SENSOR_TYPE_PRESSURE:
    printf(F("Pressure (hPa)"));
    break;
  case SENSOR_TYPE_PROXIMITY:
    printf(F("Distance (cm)"));
    break;
  case SENSOR_TYPE_GRAVITY:
    printf(F("Gravity (m/s2)"));
    break;
  case SENSOR_TYPE_LINEAR_ACCELERATION:
    printf(F("Linear Acceleration (m/s2)"));
    break;
  case SENSOR_TYPE_ROTATION_VECTOR:
    printf(F("Rotation vector"));
    break;
  case SENSOR_TYPE_RELATIVE_HUMIDITY:
    printf(F("Relative Humidity (%%)"));
    break;
  case SENSOR_TYPE_AMBIENT_TEMPERATURE:
    printf(F("Ambient Temp (C)"));
    break;
  case SENSOR_TYPE_OBJECT_TEMPERATURE:
    printf(F("Object Temp (C)"));
    break;
  case SENSOR_TYPE_VOLTAGE:
    printf(F("Voltage (V)"));
    break;
  case SENSOR_TYPE_CURRENT:
    printf(F("Current (mA)"));
    break;
  case SENSOR_TYPE_COLOR:
    printf(F("Color (RGBA)"));
    break;
  case SENSOR_TYPE_TVOC:
    printf(F("Total Volatile Organic Compounds (ppb)"));
    break;
  case SENSOR_TYPE_VOC_INDEX:
    printf(F("Volatile Organic Compounds (Index)"));
    break;
  case SENSOR_TYPE_NOX_INDEX:
    printf(F("Nitrogen Oxides (Index)"));
    break;
  case SENSOR_TYPE_CO2:
    printf(F("Carbon Dioxide (ppm)"));
    break;
  case SENSOR_TYPE_ECO2:
    printf(F("Equivalent/estimated CO2 (ppm)"));
    break;
  case SENSOR_TYPE_PM10_STD:
    printf(F("Standard Particulate Matter 1.0 (ppm)"));
    break;
  case SENSOR_TYPE_PM25_STD:
    printf(F("Standard Particulate Matter 2.5 (ppm)"));
    break;
  case SENSOR_TYPE_PM100_STD:
    printf(F("Standard Particulate Matter 10.0 (ppm)"));
    break;
  case SENSOR_TYPE_PM10_ENV:
    printf(F("Environmental Particulate Matter 1.0 (ppm)"));
    break;
  case SENSOR_TYPE_PM25_ENV:
    printf(F("Environmental Particulate Matter 2.5 (ppm)"));
    break;
  case SENSOR_TYPE_PM100_ENV:
    printf(F("Environmental Particulate Matter 10.0 (ppm)"));
    break;
  case SENSOR_TYPE_GAS_RESISTANCE:
    printf(F("Gas Resistance (ohms)"));
    break;
  case SENSOR_TYPE_UNITLESS_PERCENT:
    printf(F("Unitless Percent (%%)"));
    break;
  case SENSOR_TYPE_ALTITUDE:
    printf(F("Altitude (m)"));
    break;
  }

  println();
  printf(F("Driver Ver:   "));
  //println(sensor.version);
  printf("%ld", sensor.version);
  printf(F("Unique ID:    "));
  //println(sensor.sensor_id);
  printf("%ld", sensor.sensor_id);
  printf(F("Min Value:    "));
  //println(sensor.min_value);
  printf("%f", sensor.min_value);
  printf(F("Max Value:    "));
  //println(sensor.max_value);
  printf("%f", sensor.max_value);
  printf(F("Resolution:   "));
  //println(sensor.resolution);
  printf("%f", sensor.resolution);
  println(F("------------------------------------\n"));
}
