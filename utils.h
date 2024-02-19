#ifndef __UTILS_H__
#define __UTILS_H__

#include <Arduino.h>

#define SCREEN_HEIGHT 170
#define SCREEN_WIDTH 320
#define COLOR_OFF_WHITE 0xE73C //off-white
// #define COLOR_GREY    0x8410
#define COLOR_GREY   ((0x17<<11)|(0x29<<5)|(0x17<<0)) //R<<11 | G<<5 | B ___ 5bit 6bit 5bit ___
#define COLOR_GREEN   ((0x2F<<11)|(0x2F<<5)|(0x2F<<0)) //R<<11 | G<<5 | B ___ 5bit 6bit 5bit ___

#define INVALID_DATA -9999.0f
#define DEFAULT_SEA_LEVEL 1013.25


// Copied from, because it is not a static function in that library (for no reason):
// https://github.com/Bodmer/TFT_eSPI/blob/cbf06d7a214938d884b21d5aeb465241c25ce774/TFT_eSPI.cpp#L4774C2-L4774C2
/***************************************************************************************
** Function name:           color565
** Description:             convert three 8-bit RGB levels to a 16-bit colour value
***************************************************************************************/
inline uint16_t _color565(uint8_t r, uint8_t g, uint8_t b) __attribute__((always_inline));
uint16_t _color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
// https://github.com/Bodmer/TFT_eSPI/blob/cbf06d7a214938d884b21d5aeb465241c25ce774/TFT_eSPI.cpp#L4807C1-L4818C2
/***************************************************************************************
** Function name:           color16to24
** Description:             convert 16-bit colour to a 24-bit 888 colour value
***************************************************************************************/
inline uint32_t _color16to24(uint16_t color565) __attribute__((always_inline));
uint32_t _color16to24(uint16_t color565) {
  uint8_t r = (color565 >> 8) & 0xF8; r |= (r >> 5);
  uint8_t g = (color565 >> 3) & 0xFC; g |= (g >> 6);
  uint8_t b = (color565 << 3) & 0xF8; b |= (b >> 5);
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | ((uint32_t)b << 0);
}

inline uint16_t blend2ColorsWithAlpha(uint32_t c1_rgb, uint32_t c2_rgb, float alpha) __attribute__((always_inline));
uint16_t blend2ColorsWithAlpha(uint32_t c1_rgb, uint32_t c2_rgb, float alpha) {
  const int r = (c1_rgb >> 16) * alpha + (c2_rgb >> 16) * (1. - alpha);
  const int g = (c1_rgb >> 8 & 0xFF) * alpha + (c2_rgb >> 8 & 0xFF) * (1. - alpha);
  const int b = (c1_rgb & 0xFF) * alpha + (c2_rgb & 0xFF) * (1. - alpha);
  return _color565(r, g, b);
}

inline uint16_t blend2ColorsWithAlpha(uint16_t c1, uint16_t c2, float alpha) __attribute__((always_inline));
uint16_t blend2ColorsWithAlpha(uint16_t c1, uint16_t c2, float alpha) {
  uint32_t c1_rgb = _color16to24(c1);
  uint32_t c2_rgb = _color16to24(c2);
  return blend2ColorsWithAlpha(c1_rgb, c2_rgb, alpha);
}

inline uint16_t blend2ColorsWithAlpha(uint32_t c1_rgb, uint16_t c2, float alpha) __attribute__((always_inline));
uint16_t blend2ColorsWithAlpha(uint32_t c1_rgb, uint16_t c2, float alpha) {
  uint32_t c2_rgb = _color16to24(c2);
  return blend2ColorsWithAlpha(c1_rgb, c2_rgb, alpha);
}

inline uint16_t blend2ColorsWithAlpha(uint16_t c1, uint32_t c2_rgb, float alpha) __attribute__((always_inline));
uint16_t blend2ColorsWithAlpha(uint16_t c1, uint32_t c2_rgb, float alpha) {
  uint32_t c1_rgb = _color16to24(c1);
  return blend2ColorsWithAlpha(c1_rgb, c2_rgb, alpha);
}

int getColorFadedToBlack(int color);
int getColorFadedToWhite(int color);
uint16_t updateSatColor(uint16_t color_565);


// Copied from: https://github.com/adafruit/Adafruit_BMP085_Unified/blob/master/Adafruit_BMP085_U.cpp
float bmp085_pressureToAltitude(float atmospheric, float seaLevel=1013.25);
float bmp085_seaLevelForAltitude(float altitude, float atmospheric);


float getWithMutex(float* f_p, SemaphoreHandle_t mutex);
double getWithMutex(double* f_p, SemaphoreHandle_t mutex);
int getWithMutex(int* f_p, SemaphoreHandle_t mutex);

#endif