
#include <Arduino.h>
#include "utils.h"

int getColorFadedToBlack(int color) {
  color = (color / 2);
  if (color < 0) {
    color = 0;
  }
  return color;
}

int getColorFadedToWhite(int color) {
  int prevColor = color;
  color = (color / 2) + 128;
  if (color > 255) {
    color = 255;
  }
  return color;
}

uint16_t updateSatColor(uint16_t color_565) {
  // if color is already white
  if (color_565 == 0xFFFF) {
    return 0xFFFF;
  }
  uint32_t color_rgb = _color16to24(color_565);
  const int r = getColorFadedToWhite(color_rgb >> 16);
  const int g = getColorFadedToWhite(color_rgb >> 8 & 0xFF);
  const int b = getColorFadedToWhite(color_rgb & 0xFF);
  return _color565(r, g, b);
}

// https://github.com/adafruit/Adafruit_BMP085_Unified/blob/95ae5cc07f14cb36955d946c2403ccebce9bb00f/Adafruit_BMP085_U.cpp#L361
// https://www.hackster.io/dragos-iosub/bme688-first-ai-gas-sensor-arduino-how-to-b8a0a6
/*!
* @brief     This converts a pressure measurement into a height in meters
* @details   The corrected sea-level pressure can be passed into the function if it is know, otherwise the standard 
*            atmospheric pressure of 1013.25hPa is used (see https://en.wikipedia.org/wiki/Atmospheric_pressure
* @param[in] seaLevel Sea-Level pressure in millibars
* @return    floating point altitude in meters.
*/
float altitude(float pressure, const float seaLevel=1013.25) 
{
  /*wikipedia equation - original Zanshin code*/
  static float Altitude;
  //BME680.getSensorData(temp,hum,press,gas); // Get the most recent values from the device
  Altitude = 44330.0*(1.0-pow((pressure/100.0)/seaLevel,0.1903)); // Convert into altitude in meters
  return(Altitude);
} // of method altitude()

float calculate_altitude( float pressure, bool metric = true, float seaLevelPressure = 101325)
{
  /*Equations courtesy of NOAA - code ported from BME280*/;
  float altitude = NAN;
  if (!isnan(pressure) && !isnan(seaLevelPressure)){
    altitude = 1000.0 * ( seaLevelPressure - pressure ) / 3386.3752577878;
  }
  return metric ? altitude * 0.3048 : altitude;
}

float temperatureCompensatedAltitude(int32_t pressure, float temp=21.0 /*Celsius*/, float seaLevel=1013.25) 
{
  /*Casio equation - code written by itbrainpower.net*/
  float Altitude;
  Altitude = (pow((seaLevel/((float)pressure/100.0)), (1/5.257))-1)*(temp + 273.15) / 0.0065; // Convert into altitude in meters
  return(Altitude);	//this are metric value
}


// Copied from: https://github.com/adafruit/Adafruit_BMP085_Unified/blob/master/Adafruit_BMP085_U.cpp
float bmp085_pressureToAltitude(float atmospheric, float seaLevel) {
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude
    (in meters), and atmospheric pressure (in hPa).

    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
// Copied from: https://github.com/adafruit/Adafruit_BMP085_Unified/blob/master/Adafruit_BMP085_U.cpp
float bmp085_seaLevelForAltitude(float altitude,
                                 float atmospheric) {
  // Equation taken from BMP180 datasheet (page 17):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  float ret = (atmospheric) / pow(1.0 - (altitude / 44330.0), 5.255);
  // Round to 2 decimal places
  ret = round(ret * 100) / 100.0;
  return ret;
}

float getWithMutex(float* f_p, SemaphoreHandle_t mutex) {
  float f = 0;
  if (xSemaphoreTake(mutex, 300 * portTICK_PERIOD_MS)) {
    f = *f_p;
    xSemaphoreGive(mutex);
  }
  return f;
}

double getWithMutex(double* f_p, SemaphoreHandle_t mutex) {
  double f = 0;
  if (xSemaphoreTake(mutex, 300 * portTICK_PERIOD_MS)) {
    f = *f_p;
    xSemaphoreGive(mutex);
  }
  return f;
}

int getWithMutex(int* f_p, SemaphoreHandle_t mutex) {
  int f = 0;
  if (xSemaphoreTake(mutex, 300 * portTICK_PERIOD_MS)) {
    f = *f_p;
    xSemaphoreGive(mutex);
  }
  return f;
}