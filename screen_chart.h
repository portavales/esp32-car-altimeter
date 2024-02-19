#ifndef __SCREEN_CHART_H__
#define __SCREEN_CHART_H__

#include <Arduino.h>
#include "utils.h"
#include <TFT_eSPI.h>

// callback that returns a float
typedef float (*CallbackFloat)();
typedef int (*CallbackInt)();
typedef bool (*CallbackBool)();

// How long to show the title of the screen when we change to it
#define SHOW_TITLE_PERIOD 1000 * 3

typedef struct {
  String title;
  String field_1;
  CallbackInt field_1_cb;
  CallbackFloat add_to_history_cb;

  String field_2;
  CallbackInt field_2_cb;
  String field_3;
  CallbackInt field_3_cb;
  String field_4;
  CallbackInt field_4_cb;

  CallbackInt gps_quality_cb;
  CallbackInt sattelites_cb;
  CallbackBool gps_altitude_is_updated_cb;
} ScreenChart_config_t;

class ScreenChart{
public:
  ScreenChart();
  bool begin(TFT_eSprite* screen_p);
  void set_config(const ScreenChart_config_t& config);

  void showTitle();
  void addToHistory();
  void draw();

  void set_plot_history_period(int period) { // in seconds
    //Serial.printf("%s set_plot_history_period: %d\n", _config.title, period);
    _save_data_period = 1000 * period;
  }

private:
  void drawGPS();
  void drawNumbers();
  void plotHistory();

  TFT_eSprite* _screen;

  ScreenChart_config_t _config;

  float _data_history[SCREEN_WIDTH];
  int _data_history_index = 0;
  uint32_t _next_data_save_ts = 0;

  uint32_t _save_data_period = 1000 * 1;

  uint32_t _show_title_ts = 0;
};

#endif
