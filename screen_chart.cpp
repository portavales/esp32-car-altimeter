#include "screen_chart.h"

#include "fonts.h"
#include "utils.h"


void initFakeAltitudeHistoryForTestingTheCharting(float data_history[SCREEN_WIDTH]) {
  // Set random seed
  randomSeed(0);
  //random(0, 100)
  int i = 0;
  float last_alt = 1000;
  for (; i < SCREEN_WIDTH && i < 50; i++) {
    // Slope of 10 percent
    (data_history)[i] = 1000 + (i * 10);
    last_alt = (data_history)[i];
  }
  for (int j = 0; i < SCREEN_WIDTH && j < 10; j++) {
    // Slope of 30 percent
    (data_history)[i] = last_alt + 30;
    last_alt = (data_history)[i];
    i++;
  }
  for (int j = 0; i < SCREEN_WIDTH && j < 15; j++) {
      (data_history)[i] = last_alt - 15;
    last_alt = (data_history)[i];
    i++;
  }
  for (int j = 0; i < SCREEN_WIDTH && j < 50; j++) {
      (data_history)[i] = last_alt - 10;
    last_alt = (data_history)[i];
    i++;
  }
  for (int j = 0; i < SCREEN_WIDTH && j < 50; j++) {
      (data_history)[i] = last_alt - 5;
    last_alt = (data_history)[i];
    i++;
  }
  for (int j = 0; i < SCREEN_WIDTH && j < 10; j++) {
    (data_history)[i] = last_alt + 1;
    last_alt = (data_history)[i];
    i++;
  }
  for (int j = 0; i < SCREEN_WIDTH && j < 10; j++) {
    (data_history)[i] = last_alt + 10;
    last_alt = (data_history)[i];
    i++;
  }
  for (int j = 0; i < SCREEN_WIDTH && j < 100; j++) {
    (data_history)[i] = last_alt - 4;
    last_alt = (data_history)[i];
    i++;
  }
  for (int j = 0; i < SCREEN_WIDTH && j < 100; j++) {
    (data_history)[i] = last_alt + 1;
    last_alt = (data_history)[i];
    i++;
  }
}


static uint16_t sunsetColorsGradient(float value) {
  // Ensure value is between 0 and 1
  if (value < 0.0f) value = 0.0f;
  if (value > 1.0f) value = 1.0f;

  // Define sunset colors in 16-bit format
  const uint32_t gradient[] = {
    0x264653,
    0x2a9d8f,
    0xe9c46a,
    0xf4a261,
    0xe76f51
    };
  const int size_of_gradient = sizeof(gradient) / sizeof(gradient[0]);

  value = value * (size_of_gradient - 1);
  int ind = int(value);
  float alpha = value - ind;
  return blend2ColorsWithAlpha(gradient[ind+1], gradient[ind], alpha);
}

ScreenChart::ScreenChart() {
  // Init to null the config
  memset(&_config, 0, sizeof(_config));
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    _data_history[i] = INVALID_DATA;
  }
}

void ScreenChart::showTitle() {
  _show_title_ts = millis() + SHOW_TITLE_PERIOD;
}

bool ScreenChart::begin(TFT_eSprite* screen_p) {
  _screen = screen_p;
  showTitle();
  return true;
}

void ScreenChart::set_config(const ScreenChart_config_t& config) {
  _config = config;
  //initFakeAltitudeHistoryForTestingTheCharting(_data_history);
}

void ScreenChart::addToHistory() {
  if (millis() > _next_data_save_ts) {
    _next_data_save_ts = millis() + _save_data_period;
    float data = _config.add_to_history_cb();
    if (data == INVALID_DATA) {
      return;
    }
    _data_history[_data_history_index] = data;
    //Serial.printf("addToHistory: _data_history[%d]: %f\n", _data_history_index, _data_history[_data_history_index]);
    _data_history_index++;
    if (_data_history_index >= SCREEN_WIDTH) {
      _data_history_index = 0;
    }
  }
}


void ScreenChart::draw() {
  drawGPS();
  if (millis() < _show_title_ts) {
    _screen->loadFont(NotoSansMono_Bold_50);
    // hackly measured for the NotoSansMono_Bold_50 font
    #define PIXELS_PER_CHAR 30   //(int)(320/10.6)
    _screen->setTextColor(TFT_WHITE, TFT_BLACK);
    _screen->setTextWrap(false, false);
    const int x = (SCREEN_WIDTH - (_config.title.length() * PIXELS_PER_CHAR)) / 2;
    _screen->drawString(_config.title, x, 70);
    _screen->unloadFont(); 
  } else {
    drawNumbers();
  }

  plotHistory();
}

// Draw GPS Icon with number of sattelites
void ScreenChart::drawGPS() {
  if (!_config.gps_quality_cb || !_config.sattelites_cb || !_config.gps_altitude_is_updated_cb) {
    return;
  }
  _screen->setTextColor(TFT_WHITE, TFT_BLACK);
  const int sattelites = _config.sattelites_cb();
  uint16_t sat_color = _config.gps_quality_cb();
  if (!_config.gps_altitude_is_updated_cb()) {
    sat_color = TFT_RED;
  }
  if (sattelites >= 5) {
    _screen->drawSmoothArc(SCREEN_WIDTH - 20, 20, 18, 15, 180, 270, sat_color, TFT_BLACK, true);
  }
  if (sattelites >= 4) {
    _screen->drawSmoothArc(SCREEN_WIDTH - 20, 20, 12, 9, 180, 270, sat_color, TFT_BLACK, true);
  }
  if (sattelites >= 3) {
    _screen->drawSmoothArc(SCREEN_WIDTH - 20, 20, 6, 3, 180, 270, sat_color, TFT_BLACK, true);
  }
  _screen->drawSmoothArc(SCREEN_WIDTH - 20, 20, 0, 10, 315, 135, sat_color, TFT_BLACK, true);

  _screen->loadFont(NotoSansMono_Bold_14);
  _screen->drawString(String("Sats"), SCREEN_WIDTH - 37, 32);
  _screen->unloadFont(); 

  _screen->loadFont(NotoSansMono_Regular_30);
  _screen->drawString(String(sattelites), SCREEN_WIDTH - 28, 45);
  _screen->unloadFont(); 
}


void ScreenChart::drawNumbers() {
  _screen->setTextColor(TFT_WHITE, TFT_BLACK);

  const int field_1 = _config.field_1_cb();
  const int field_2 = _config.field_2_cb();
  const int field_3 = _config.field_3_cb();
  const int field_4 = _config.field_4_cb();

  _screen->loadFont(NotoSansMono_Bold_20);
  _screen->drawString(_config.field_1, 5, 5);
  _screen->drawString(_config.field_2, 3, SCREEN_HEIGHT-60);
  _screen->drawString(_config.field_3, 100, SCREEN_HEIGHT-60);
  _screen->drawString(_config.field_4, 230, SCREEN_HEIGHT-60);
  _screen->unloadFont(); 

  _screen->loadFont(NotoSansMono_Bold_100);
  _screen->drawString(String(field_1), 10, 20);
  _screen->unloadFont();

  _screen->loadFont(NotoSansMono_Bold_50);
  _screen->drawString(String(field_2), 3, SCREEN_HEIGHT-45);
  _screen->drawString(String(field_3), 100, SCREEN_HEIGHT-45);
  _screen->drawString(String(field_4), 230, SCREEN_HEIGHT-45);
  _screen->unloadFont();

  // Draw horizontal line
  _screen->drawLine(0, SCREEN_HEIGHT-65, SCREEN_WIDTH, SCREEN_HEIGHT-65, COLOR_OFF_WHITE);
  // Draw vertical line
  _screen->drawLine(95, SCREEN_HEIGHT-60, 95, SCREEN_HEIGHT, COLOR_OFF_WHITE);
  // Draw vertical line
  _screen->drawLine(225, SCREEN_HEIGHT-60, 225, SCREEN_HEIGHT, COLOR_OFF_WHITE);


  /*
  // https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BME680-688-IAQ-meaning/td-p/45196
  // Set font color based on IAQ
  uint16_t iaq_color = TFT_RED;
  if (sensorsState.iaq <= 100) {
    iaq_color = TFT_GREEN;
  } else if (sensorsState.iaq <= 150) {
    iaq_color = TFT_YELLOW;
  } else if (sensorsState.iaq <= 200) {
    iaq_color = TFT_ORANGE;
  } else if (sensorsState.iaq <= 250) {
    iaq_color = TFT_RED;
  } else if (sensorsState.iaq <= 350) {
    iaq_color = TFT_PURPLE;
  } else {
    iaq_color = TFT_MAROON;
  }
  //screen.setTextColor(iaq_color, TFT_BLACK);
  */
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void ScreenChart::plotHistory() {
  static uint16_t sunset_colors_lut[SCREEN_HEIGHT];
  static uint32_t sunset_colors_lut32[SCREEN_HEIGHT];
  static bool init = false;
  if (!init) {
    for (int i = 0; i < SCREEN_HEIGHT; i++) {
      float f = 1.0 - (i / (float)SCREEN_HEIGHT);
      sunset_colors_lut[i] = sunsetColorsGradient(f);
      sunset_colors_lut32[i] = _color16to24(sunset_colors_lut[i]);
    }
    init = true;
  }
  float max_alt = 0;
  float min_alt = 999999;
  int index_max = SCREEN_WIDTH;
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    //Serial.printf("plotHistory 1: _data_history[%d]: %f\n", i, _data_history[i]);
    if (_data_history[i] == INVALID_DATA) {
      index_max = i;
      break;
    }
    max_alt = _data_history[i] > max_alt ? _data_history[i] : max_alt;
    min_alt = _data_history[i] < min_alt ? _data_history[i] : min_alt;
  }
  // Draw altitude history
  for (int i = 0; i < index_max; i++) {
    const int index = (_data_history_index + i ) % index_max;
    const float data_point = _data_history[index];
    //Serial.printf("plotHistory 2: _data_history[%d]: %f\n", index, _data_history[index]);
    if (data_point == INVALID_DATA) {
      break;
    }
    const int x = i;
    // Scale altitude to screen height
    const int alt_y = map_float(data_point, min_alt, max_alt, SCREEN_HEIGHT-1, 0);
    for (int y=alt_y; y < SCREEN_HEIGHT; y++) {
      const uint16_t screen_color = _screen->readPixel(x, y);
      uint16_t pix_color = sunset_colors_lut[y];
      if (screen_color > TFT_DARKGREY || screen_color == TFT_GREEN || screen_color == TFT_YELLOW || screen_color == TFT_RED) {
        const uint32_t pix_color32 = sunset_colors_lut32[y];
        if (abs(y-alt_y) <= 3) {
          pix_color = blend2ColorsWithAlpha(screen_color, pix_color32, 0.50);
        } else {
          pix_color = blend2ColorsWithAlpha(screen_color, pix_color32, 0.90);
        }
      }
      _screen->drawPixel(x, y, pix_color);
    }
  }
}