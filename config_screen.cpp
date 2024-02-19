#include "config_screen.h"

#include "fonts.h"
#include "utils.h"

const char* AltimeterConfigScreen::version = "v0.0.2";

AltimeterConfigScreen::AltimeterConfigScreen() {
  // Init to null the config
  memset(&_config, 0, sizeof(_config));
  _config.screen_rotation = 1;
  _config.brightness = 2;
  _config.plot_history_period = 1;
  _config.debug = false;
}


bool AltimeterConfigScreen::begin(TFT_eSprite* screen_p, Preferences* prefs_p) {
  _prefs = prefs_p;
  _screen = screen_p;
  showTitle();
  return true;
}

void AltimeterConfigScreen::set_config(const altimeter_config_t& config) {
  _config = config;
}

void AltimeterConfigScreen::saveState() {
  _prefs->begin("car-altimeter", false); 
  _prefs->putInt("screen_rotation", _config.screen_rotation);
  _prefs->putInt("brightness", _config.brightness);
  _prefs->putInt("plot_history", _config.plot_history_period); // Preferences lib needs the name of the pref to be a short string and the int is a uint8 AFAICT
  _prefs->putBool("debug", _config.debug);
  _prefs->end();
}

void AltimeterConfigScreen::loadState() {
  _prefs->begin("car-altimeter", false); 
  _config.screen_rotation = _prefs->getInt("screen_rotation", 1);
  if (_config.screen_rotation != 1 && _config.screen_rotation != 3) {
    _config.screen_rotation = 1;
  }
  _config.brightness = _prefs->getInt("brightness", 2);
  if (_config.brightness < 1 || _config.brightness > 3) {
    _config.brightness = 2;
  }
  _config.plot_history_period = _prefs->getInt("plot_history", 1);
  if (_config.plot_history_period != 1 && _config.plot_history_period != 5 && _config.plot_history_period != 10) {
    _config.plot_history_period = 1;
  }
  _config.debug = _prefs->getBool("debug", false);
  _prefs->end();
}

void AltimeterConfigScreen::showTitle() {
  _show_title_ts = millis() + SHOW_TITLE_PERIOD;
}

void AltimeterConfigScreen::button1() {
  // _config.screen_rotation = (_config.screen_rotation + 1) % 4;

  switch(_selected_field) {
    case 0:
      if (_config.screen_rotation == 1) {
        _config.screen_rotation = 3;
      } else {
        _config.screen_rotation = 1;
      }
      break;
    case 1:
      _config.brightness = _config.brightness + 1;
      if (_config.brightness > 3) {
        _config.brightness = 1;
      }
      break;
    case 2:
      static int plot_history_period_values[] = {1, 5, 10};
      for (int i = 0; i < 3; i++) {
        if (plot_history_period_values[i] == _config.plot_history_period) {
          _config.plot_history_period = plot_history_period_values[(i + 1) % 3];
          break;
        }
      }
      break;
    case 3:
      _config.debug = !_config.debug;
      break;
    case 4:
      saveState();
      _is_active = false;
      break;
  }
}


void AltimeterConfigScreen::button2() {
  // _config.screen_rotation = (_config.screen_rotation + 1) % 4;
  _selected_field = (_selected_field + 1) % 5;
}


void AltimeterConfigScreen::draw() {
  static const String title = "Config";

  if (millis() < _show_title_ts) {
    _screen->loadFont(NotoSansMono_Bold_50);
    // hackly measured for the NotoSansMono_Bold_50 font
    #define PIXELS_PER_CHAR 30   //(int)(320/10.6)
    _screen->setTextColor(COLOR_OFF_WHITE, TFT_BLACK);
    _screen->setTextWrap(false, false);
    const int x = (SCREEN_WIDTH - (title.length() * PIXELS_PER_CHAR)) / 2;
    _screen->drawString(title, x, 70);
    _screen->unloadFont(); 
  } else {

    _screen->setTextSize(2);
    if (_selected_field == 0) {
      _screen->setTextColor(COLOR_OFF_WHITE, TFT_BLUE);
    } else {
      _screen->setTextColor(COLOR_GREY, TFT_BLACK);
    }
    String screen_rotation_text = "Screen Rotation: ";
    if (_config.screen_rotation == 1) {
      screen_rotation_text += "[->]";
    } else if (_config.screen_rotation == 3) {
      screen_rotation_text += "[<-]";
    }
    _screen->drawString(screen_rotation_text, 10, 30);


    if (_selected_field == 1) {
      _screen->setTextColor(COLOR_OFF_WHITE, TFT_BLUE);
    } else {
      _screen->setTextColor(COLOR_GREY, TFT_BLACK);
    }
    _screen->drawString(String("Brightness: ") + String(_config.brightness), 10, 50);

    if (_selected_field == 2) {
      _screen->setTextColor(COLOR_OFF_WHITE, TFT_BLUE);
    } else {
      _screen->setTextColor(COLOR_GREY, TFT_BLACK);
    }
    String plot_history_text = "Plot History: ";
    if (_config.plot_history_period == 1) {
      plot_history_text += "5 mins";
    } else if (_config.plot_history_period == 5) {
      plot_history_text += "30 mins";
    } else if (_config.plot_history_period == 10) {
      plot_history_text += "1 hour";
    }
    _screen->drawString(plot_history_text, 10, 70);


    if (_selected_field == 3) {
      _screen->setTextColor(COLOR_OFF_WHITE, TFT_BLUE);
    } else {
      _screen->setTextColor(COLOR_GREY, TFT_BLACK);
    }
    String debug_text = "Debug Info: ";
    if (_config.debug == true) {
      debug_text += "enabled";
    } else {
      debug_text += "disabled";
    }
    _screen->drawString(debug_text, 10, 90);

    if (_selected_field == 4) {
      _screen->setTextColor(COLOR_OFF_WHITE, TFT_BLUE);
    } else {
      _screen->setTextColor(COLOR_GREY, TFT_BLACK);
    }
    _screen->drawString("SAVE", 10, 110);

    _screen->setTextColor(COLOR_GREY, TFT_BLACK);
    _screen->drawString("version: " + String(version), 10, 150);
  }
}

void AltimeterConfigScreen::set_active() {
  _is_active = true;
  _selected_field = 0;
  showTitle();
}