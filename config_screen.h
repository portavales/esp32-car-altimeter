#ifndef __ALTIMETER_CONFIG_SCREEN_H__
#define __ALTIMETER_CONFIG_SCREEN_H__

#include <Preferences.h>
#include <Arduino.h>
#include "utils.h"
#include <TFT_eSPI.h>

typedef struct {
  uint8_t screen_rotation = 3;
  uint8_t brightness = 2;
  uint8_t plot_history_period = 1;
  bool debug = false;
} altimeter_config_t;

// How long to show the title of the screen when we change to it
#define SHOW_TITLE_PERIOD 1000 * 3

class AltimeterConfigScreen {
public:
  AltimeterConfigScreen();
  bool begin(TFT_eSprite* screen_p, Preferences* prefs_p);
  void set_config(const altimeter_config_t& config);
  const altimeter_config_t& get_config() {
    return _config;
  }
  void loadState();
  void set_active();
  bool is_active() {
    return _is_active;
  }

  void draw();
  void button1();
  void button2();
private:
  void showTitle();
  void saveState();

  TFT_eSprite* _screen;
  Preferences* _prefs;
  altimeter_config_t _config;
  uint32_t _show_title_ts = 0;
  bool _is_active = false;


  int _selected_field = 0;
  static const char* version;
};

#endif
