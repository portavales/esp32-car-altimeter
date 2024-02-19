
// ESP32 CPU temperature sensor
// https://github.com/espressif/arduino-esp32/issues/2422#issuecomment-1211527653
#include "driver/temp_sensor.h"    //legacy esp32 temp sensor driver. https://github.com/espressif/esp-idf/blob/master/components/driver/test_apps/legacy_rtc_temp_driver/main/test_rtc_temp_driver.c


#include "pin_config.h"
#include <TFT_eSPI.h> // Include the TFT library https://github.com/Bodmer/TFT_eSPI

#include <bsec.h>
#include <BH1750.h>
#include <TinyGPSPlus.h> // http://arduiniana.org/libraries/tinygpsplus/
#include <HardwareSerial.h>

// https://lonelybinary.com/en-nz/blogs/learn/persistent-data-storage-on-esp32-leveraging-eeprom-and-preferences-for-permanent-data-retention#:~:text=Therefore%2C%20it%20is%20highly%20recommended,is%20stored%20in%20plain%20text.
// https://www.aranacorp.com/en/using-the-eeprom-with-the-esp32/
#include <Preferences.h>
Preferences prefs;

#include "fonts.h"
#include "utils.h"
#include "screen_chart.h"
#include "config_screen.h"


#define CONFIG_BUTTON_PRESS_DURATION 3000
#define SEALEVEL_MAX_AGE 1000 * 60 * 60 * 48 // 48 hours
#define MAX_GPS_FIX_AGE 1000 * 2 // 2 seconds

#define FPS 30
#define DRAW_RATE (1000 / FPS)
#define SENSOR_RATE 100
#define GPS_RATE 100
#define BME_STATE_SAVE_PERIOD	UINT32_C(60 * 60 * 1000) // 60 minutes
#define SAVE_SENSOR_STATE_PERIOD	UINT32_C(60 * 60 * 1000) // 60 minutes
#define SAVE_ALTITUDE_PERIOD 10   // Seconds



AltimeterConfigScreen config_screen;
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite screen = TFT_eSprite(&tft);


BH1750 lightMeter;
Bsec bme_sensor;
// Accuracy values description
// https://community.bosch-sensortec.com/t5/Question-and-answers/What-does-the-IAQ-accuracy-mean-in-BSEC/qaq-p/5935
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};
struct SensorsDataState_t {
  double iaq = INVALID_DATA;
  int iaqAccuracy = 0;
  double staticIaq = 0;
  double co2Equivalent = INVALID_DATA;
  double breathVocEquivalent = 0;
  double rawTemperature = 0;
  double pressure = INVALID_DATA;
  float seaLevel = DEFAULT_SEA_LEVEL;
  uint32_t seaLevel_last_update = 0;
  float altitude = INVALID_DATA;
  double rawHumidity = 0;
  double gasResistance = 0;
  int stabStatus = 0;
  int runInStatus = 0;
  double temperature = INVALID_DATA;
  double humidity = 0;
  double gasPercentage = 0;
};
SemaphoreHandle_t sensorsState_mutex = NULL;
SensorsDataState_t sensorsState = {};


TinyGPSPlus tiny_gps;
HardwareSerial SerialGPS(1);
struct GpsDataState_t {
  double originLat = 0;
  double originLon = 0;
  double originAlt = 0;
  double totalDist = 0;
  double altMax = -99999;
  double altMin = 99999;
  double spdMax = 0;

  float hdop = 99.99;
  int satellites = 0;
  double altitude = INVALID_DATA;
  uint32_t altitude_last_update = 0;
  int quality = TFT_RED;
  uint16_t color = TFT_RED;
};
SemaphoreHandle_t gpsState_mutex = NULL;
GpsDataState_t gpsState = {};


// Screens history and global objects / singletons.
float altitude_history[SCREEN_WIDTH] = {0};
int altitude_history_index = 0;
ScreenChart altitude_chart;
ScreenChart co2_chart;
ScreenChart iaq_chart;
ScreenChart temperature_chart;
static ScreenChart* g_all_charts[] = {
  &altitude_chart,
  &co2_chart,
  &iaq_chart,
  &temperature_chart
};
static const int g_sizeof_all_charts = sizeof(g_all_charts) / sizeof(g_all_charts[0]); // 4
static uint16_t g_currently_selected_chart = 0;


void gps_task(void *pvParameters);
void sensors_task (void *pvParameters);
void checkBmeSensorStatus(void);
void bmeLoadState(void);
void bmeSaveState(void);
void saveSensorState();
void loadSensorState();
void saveScreenChoice();
void loadScreenChoice();
void setDisplayBrightness(int brightness);
void readLight();
void setScreenRotation(int rotation);
void setAllChartsPlotHistoryPeriod(int period);




altimeter_config_t g_altimeter_config;
void setAltimeterConfig(const altimeter_config_t& config) {
  g_altimeter_config = config;
  setScreenRotation(config.screen_rotation);
  readLight();
  setAllChartsPlotHistoryPeriod(g_altimeter_config.plot_history_period);
}


/*************************************************************************
 * Light & Brightness
 ************************************************************************/
#define BRIGHTNESS_FACTOR g_altimeter_config.brightness
int getBoundedBrightnessFromLux(float lux) {
  int brightness = BRIGHTNESS_FACTOR * 25 + (1.35 * BRIGHTNESS_FACTOR) * lux;
  if (brightness < 0) {
    brightness = 0;
  } else if (brightness > 255) {
    brightness = 255;
  }
  return brightness;
}
static int g_current_brightness = 0;
void setDisplayBrightness(int brightness) {
  if (brightness < 0) {
    brightness = 0;
  } else if (brightness > 255) {
    brightness = 255;
  }
  if (brightness == g_current_brightness) {
    // Serial.print("Brightness is already: "); Serial.println(brightness);
    return;
  }
  //Serial.print("Setting brightness to: "); Serial.println(brightness);
  ledcSetup(0, 10000, 8);
  ledcAttachPin(PIN_LCD_BL, 0);
  ledcWrite(0, brightness);
  g_current_brightness = brightness;
}


static float g_lux = INVALID_DATA;
static float g_prev_lux = INVALID_DATA;
void readLight() {
  // Above 255 we already saturate the brightness
  float new_lux = lightMeter.readLightLevel() < 255./BRIGHTNESS_FACTOR ? lightMeter.readLightLevel() : 255./BRIGHTNESS_FACTOR;
  if (g_prev_lux == INVALID_DATA) {
    g_prev_lux = ((int)(new_lux * 100)) / 100.0;
  }
  if (g_lux == INVALID_DATA) {
    g_lux = ((int)(new_lux * 100)) / 100.0;
  }
  g_lux = g_lux * 0.5 + new_lux * 0.5;
  // Round to 2 decimal places
  g_lux = ((int)(g_lux * 100)) / 100.0;

  //Serial.print("g_lux: "); Serial.print(g_lux);
  //Serial.print("  g_prev_lux: "); Serial.println(g_prev_lux);
  const int new_brightness = getBoundedBrightnessFromLux(g_lux);
  const int prev_brightness = getBoundedBrightnessFromLux(g_prev_lux);
  if (new_brightness == prev_brightness) {
    setDisplayBrightness(new_brightness);
  }
  g_prev_lux = g_lux;
}


/*************************************************************************
 * GPS
 ************************************************************************/
float getAltitude() {
  float altitude = INVALID_DATA;
  if (xSemaphoreTake(sensorsState_mutex, 300 * portTICK_PERIOD_MS)) { // take the mutex, max wait time 300ms
    altitude = sensorsState.altitude;
    xSemaphoreGive(sensorsState_mutex); // give the mutex back
  }
  return altitude;
}
bool gpsAltitudeIsUpdated() {
  return millis() - gpsState.altitude_last_update < MAX_GPS_FIX_AGE;
}
bool gpsSignalIsGoodQuality() {
  return gpsState.altitude != INVALID_DATA && gpsState.quality == TFT_GREEN;
}
bool gpsSignalIsMediumQuality() {
  return gpsState.altitude != INVALID_DATA && gpsState.quality == TFT_YELLOW;
}


/*************************************************************************
 * Draw errors and temporary information to the screen.
 ************************************************************************/
String show_momentarily_info = "";
uint32_t show_momentarily_info_start_time = 0;
String print_screen_info[(SCREEN_HEIGHT/10) - 1] = {"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""};
uint8_t print_screen_line = 0;
void drawScreenInfo(bool push_sprite = false) {
  for (uint8_t i = 0; i < (SCREEN_HEIGHT/10) - 1; i++) {
    screen.setCursor(20, 10 + i * 10);
    screen.setTextColor(TFT_WHITE, TFT_RED);
    if (print_screen_info[i].length() > 0) {
      screen.print(print_screen_info[i]);
    }
  }
  
  if (millis() - show_momentarily_info_start_time > 2000) {
    show_momentarily_info = "";
  }
  if (show_momentarily_info.length() > 0) {
    screen.setCursor(20, (SCREEN_HEIGHT/2 - 10));
    screen.setTextColor(TFT_WHITE, 0x999999);
    screen.print(show_momentarily_info);
  }

  if (push_sprite) {
    screen.pushSprite(0, 0);
  }
}
void showMomentarily(String line) {
  Serial.println(line);
  show_momentarily_info = line;
  show_momentarily_info_start_time = millis();
}
void printInScreen(String line, bool push_sprite = false) {
  Serial.println(line);
  print_screen_info[print_screen_line] = line;
  print_screen_line++;
  if (print_screen_line >= (SCREEN_HEIGHT/10) - 1) print_screen_line = 0;
  drawScreenInfo(push_sprite);
}
void clearScreenInfo(bool push_sprite = false) {
  for (uint8_t i = 0; i < (SCREEN_HEIGHT/10) - 1; i++) {
    print_screen_info[i] = "";
  }
  print_screen_line = 0;
  drawScreenInfo(push_sprite);
}



void initCPUTempSensor() {
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor.dac_offset = TSENS_DAC_L2;  //TSENS_DAC_L2 is default   L4(-40℃ ~ 20℃), L2(-10℃ ~ 80℃) L1(20℃ ~ 100℃) L0(50℃ ~ 125℃)
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
}

void setup() {
  Serial.begin(115200);
  // Wait for serial. Use it for debugging
  // while (!Serial) { delay(10); }

  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);

  // Set up the LCD backlight brightness PWM channel
  ledcSetup(0, 10000, 8);
  ledcAttachPin(PIN_LCD_BL, 0);

  initCPUTempSensor();

  // Init TFT
  tft.begin();
  tft.writecommand(0x11);
  // tft.fillScreen(TFT_GREEN); // For debugging
  // digitalWrite(PIN_LCD_BL, HIGH); // Turn on/off the backlight
  tft.writecommand(ST7789_DISPON); //turn on display  
  // setDisplayBrightness(200);


  setScreenRotation(3);
  screen.createSprite(tft.width(), tft.height());
  Serial.print("TFT Height: "); Serial.println(tft.height());
  Serial.print("TFT Width: "); Serial.println(tft.width());

  setDisplayBrightness(20);
  setCpuFrequencyMhz(80);

  // Baudrate for the GPS Module is 9600
  // TX of GPS Module is connected to RX of ESP32
#define PIN_GPS_TX 18
#define PIN_GPS_RX 17
  SerialGPS.begin(9600, SERIAL_8N1, PIN_GPS_TX, PIN_GPS_RX);  // tem que ser 18 e 17 in that order
  //memset(&gpsState, 0, sizeof(gpsState));
  gpsState_mutex = xSemaphoreCreateMutex();  // crete a mutex object
  sensorsState_mutex = xSemaphoreCreateMutex();  // crete a mutex object

// Configure SCL for serial connection on GPIO 1, SDA on GPIO 2
#define PIN_SCL 1
#define PIN_SDA 2
  Wire1.setPins(PIN_SDA, PIN_SCL);
  Wire1.begin();
  Wire1.setClock(50000);


  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire1)) {
    printInScreen("Error initialising light sensor.", true);
    delay(2000);
  }
  int c = 0;
  while (!lightMeter.measurementReady(true)) {
    if (c % 10 == 0 && c > 0) {
      printInScreen("Waiting for light sensor to be ready...", true);
    }
    delay(100);
    c++;
    if (c > 50) {
      printInScreen("Error waiting for light sensor measurement.", true);
      delay(2000);
      break;
    }
  }
  readLight();

  bme_sensor.begin(BME68X_I2C_ADDR_HIGH, Wire1);
  checkBmeSensorStatus();
  bmeLoadState();
  loadSensorState();
  loadScreenChoice();


  bme_sensor.setConfig(bsec_config_iaq);
  bsec_virtual_sensor_t sensorList[13] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
  };
  bme_sensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
  checkBmeSensorStatus();

  xTaskCreatePinnedToCore (
    gps_task,     // Function to implement the task
    "gps_task",   // Name of the task
    10000,      // Stack size in words
    NULL,      // Task input parameter
    1000,         // Priority of the task
    NULL,      // Task handle.
    0          // Core where the task should run
  );
  xTaskCreatePinnedToCore (
    sensors_task,     // Function to implement the task
    "sensors_task",   // Name of the task
    10000,      // Stack size in words
    NULL,      // Task input parameter
    5,         // Priority of the task
    NULL,      // Task handle.
    0          // Core where the task should run
  );


  /*************************************************************************
   * Configure each screen chart
   ************************************************************************/
  ScreenChart_config_t altitude_config = {
    .title = "Altitude",
    .field_1 = "Altitude",
    .field_1_cb = [](){ return (int)getAltitude(); },
    .add_to_history_cb = &getAltitude,

    .field_2 = "Speed",
    .field_2_cb = [](){ return (int)tiny_gps.speed.kmph(); }, // TODO Add mutex here
    .field_3 = "CO2",
    .field_3_cb = [](){ return (int)getWithMutex(&(sensorsState.co2Equivalent), sensorsState_mutex); },
    .field_4 = "Air Qual",
    .field_4_cb = [](){ return (int)getWithMutex(&(sensorsState.iaq), sensorsState_mutex); },

    .gps_quality_cb = [](){ return (int)getWithMutex(&(gpsState.quality), gpsState_mutex); },
    .sattelites_cb = [](){ return (int)getWithMutex(&(gpsState.satellites), gpsState_mutex); },
    .gps_altitude_is_updated_cb = &gpsAltitudeIsUpdated  // TODO Add mutex here
  };
  altitude_chart.begin(&screen);
  altitude_chart.set_config(altitude_config);

  ScreenChart_config_t co2_config = {
    .title = "CO2",
    .field_1 = "CO2",
    .field_1_cb = [](){ return (int)getWithMutex(&(sensorsState.co2Equivalent), sensorsState_mutex); },
    .add_to_history_cb = [](){ return (float)getWithMutex(&(sensorsState.co2Equivalent), sensorsState_mutex); },

    .field_2 = "Air Qual",
    .field_2_cb = [](){ return (int)getWithMutex(&(sensorsState.iaq), sensorsState_mutex); },
    .field_3 = "Altitude",
    .field_3_cb = [](){ return (int)getAltitude(); },
    .field_4 = "Temp",
    .field_4_cb = [](){ return (int)getWithMutex(&(sensorsState.temperature), sensorsState_mutex); },

    .gps_quality_cb = [](){ return (int)getWithMutex(&(gpsState.quality), gpsState_mutex); },
    .sattelites_cb = [](){ return (int)getWithMutex(&(gpsState.satellites), gpsState_mutex); },
    .gps_altitude_is_updated_cb = &gpsAltitudeIsUpdated  // TODO Add mutex here
  };
  co2_chart.begin(&screen);
  co2_chart.set_config(co2_config);

  ScreenChart_config_t iaq_config = {
    .title = "Air Quality",
    .field_1 = "Air Qual",
    .field_1_cb = [](){ return (int)getWithMutex(&(sensorsState.iaq), sensorsState_mutex); },
    .add_to_history_cb = [](){ return (float)getWithMutex(&(sensorsState.iaq), sensorsState_mutex); },

    .field_2 = "CO2",
    .field_2_cb = [](){ return (int)getWithMutex(&(sensorsState.co2Equivalent), sensorsState_mutex); },
    .field_3 = "Altitude",
    .field_3_cb = [](){ return (int)getAltitude(); },
    .field_4 = "Temp",
    .field_4_cb = [](){ return (int)getWithMutex(&(sensorsState.temperature), sensorsState_mutex); },
    
    .gps_quality_cb = [](){ return (int)getWithMutex(&(gpsState.quality), gpsState_mutex); },
    .sattelites_cb = [](){ return (int)getWithMutex(&(gpsState.satellites), gpsState_mutex); },
    .gps_altitude_is_updated_cb = &gpsAltitudeIsUpdated  // TODO Add mutex here
  };
  iaq_chart.begin(&screen);
  iaq_chart.set_config(iaq_config);

  ScreenChart_config_t temperature_config = {
    .title = "Temperature",
    .field_1 = "Temperature",
    .field_1_cb = [](){ return (int)getWithMutex(&(sensorsState.temperature), sensorsState_mutex); },
    .add_to_history_cb = [](){ return (float)getWithMutex(&(sensorsState.temperature), sensorsState_mutex); },

    .field_2 = "Air Qual",
    .field_2_cb = [](){ return (int)getWithMutex(&(sensorsState.iaq), sensorsState_mutex); },
    .field_3 = "CO2",
    .field_3_cb = [](){ return (int)getWithMutex(&(sensorsState.co2Equivalent), sensorsState_mutex); },
    .field_4 = "Alt",
    .field_4_cb = [](){ return (int)getAltitude(); },
    
    .gps_quality_cb = [](){ return (int)getWithMutex(&(gpsState.quality), gpsState_mutex); },
    .sattelites_cb = [](){ return (int)getWithMutex(&(gpsState.satellites), gpsState_mutex); },
    .gps_altitude_is_updated_cb = &gpsAltitudeIsUpdated  // TODO Add mutex here
  };
  temperature_chart.begin(&screen);
  temperature_chart.set_config(temperature_config);


  // Loading the config needs to come last as it will set the plot history to all charts
  config_screen.begin(&screen, &prefs);
  config_screen.loadState();
  setAltimeterConfig(config_screen.get_config());
}


void setAllChartsPlotHistoryPeriod(int period) {
  for (int i = 0; i < g_sizeof_all_charts; i++) {
    g_all_charts[i]->set_plot_history_period(period);
  }
}


//Function to check for the button being pushed 
void buttonPressed() {
  #define BUTTON_DEBOUNCE_DELAY 300
  static unsigned long button1_last_state_low = 0;
  static unsigned long button1_last_pressed = 0;
  static unsigned long button2_last_state_low = 0;
  static unsigned long button2_last_pressed = 0;
  static bool button1_pressed = false;
  static bool button2_pressed = false;

  bool button1_released = false;
  bool button2_released = false;
  int button1_state = digitalRead(PIN_BUTTON_1); //Read the button state
  if (button1_state == LOW && (millis() - button1_last_state_low) > BUTTON_DEBOUNCE_DELAY) { //If the button is pressed and it has been longer than the debounce delay
    button1_last_state_low = millis();
    if (!button1_pressed) {
      button1_last_pressed = millis();
    }
    button1_pressed = true;
    if (millis() - button1_last_pressed > CONFIG_BUTTON_PRESS_DURATION) {
      config_screen.set_active();
    }
  } else if (button1_state == HIGH && button1_pressed) {
    button1_pressed = false;
    button1_released = true;
  }

  int button2_state = digitalRead(PIN_BUTTON_2); //Read the button state
  if (button2_state == LOW && (millis() - button2_last_state_low) > BUTTON_DEBOUNCE_DELAY) { //If the button is pressed and it has been longer than the debounce delay
    button2_last_state_low = millis();
    if (!button2_pressed) {
      button2_last_pressed = millis();
    }
    if (millis() - button2_last_pressed > CONFIG_BUTTON_PRESS_DURATION) {
      config_screen.set_active();
    }
    button2_pressed = true;
  } else if (button2_state == HIGH && button2_pressed) {
    button2_pressed = false;
    button2_released = true;
  }

  if (!config_screen.is_active()) { 
    if (button1_released) {
      g_currently_selected_chart = (g_currently_selected_chart + 1) % g_sizeof_all_charts;
      g_all_charts[g_currently_selected_chart]->showTitle();
      saveScreenChoice();
    } else if (button2_released) {
      g_currently_selected_chart = ((uint16_t)(g_currently_selected_chart - 1)) % g_sizeof_all_charts;
      g_all_charts[g_currently_selected_chart]->showTitle();
      saveScreenChoice();
    }
  } else {
    if (button1_released && (millis() - button1_last_pressed < CONFIG_BUTTON_PRESS_DURATION)) { // Avoid pressing the button while entering the config mode
      config_screen.button1();
      setAltimeterConfig(config_screen.get_config());
    }
    if (button2_released && (millis() - button2_last_pressed < CONFIG_BUTTON_PRESS_DURATION)) {
      config_screen.button2();
      setAltimeterConfig(config_screen.get_config());
    }
  }
}


/*************************************************************************
 * Main loop draws the screen
 ************************************************************************/
void loop() {
  buttonPressed();

  // Call addToHistory() on all charts
  for (int i = 0; i < g_sizeof_all_charts; i++) {
    g_all_charts[i]->addToHistory();
  }

  screen.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, TFT_BLACK);

  if (config_screen.is_active()) {
    config_screen.draw();
  } else {
    g_all_charts[g_currently_selected_chart]->draw();
  }


  // Draw Debug Info
  if (config_screen.get_config().debug) {
    // DEBUG INFO
    screen.setTextSize(2);
    screen.setTextColor(TFT_WHITE, TFT_BLUE);
    screen.drawString(String("hdop:") + String(tiny_gps.hdop.hdop()), 0, 0);
    screen.drawString(String("iaq:") + String(int(sensorsState.iaqAccuracy)), 130, 0);
    screen.drawString(String("lux:") + String(int(g_lux)), 0, 20);
    screen.drawString(String("bri:") + String(int(g_current_brightness)), 130, 20);
    /* 
    screen.drawString(
      String("gps:") + 
      String(" ") + String(int(tiny_gps.satellites.value())) +
      String(" ") + String(int(tiny_gps.satellites.isValid())) +
      String(" ") + String(int(tiny_gps.satellites.age()/1000))
      , 320/2 + 40, 0);
    */

    static uint32_t last_fps_ts = 0;
    const int fps = int(1000.0 / (millis() - last_fps_ts));
    last_fps_ts = millis();
    screen.drawString(String("fps:") + String(fps), 250, 0);
  }

  screen.pushSprite(0, 0);
  static uint32_t last_draw_ts = 0;
  int32_t delay_time = DRAW_RATE - (millis() - last_draw_ts);
  last_draw_ts = millis();
  if (delay_time < 0) delay_time = 1;
  delay(delay_time);
}


void checkBmeSensorStatus(void)
{
  if (bme_sensor.bsecStatus != BSEC_OK) {
    if (bme_sensor.bsecStatus < BSEC_OK) {
      String output = "BSEC error code : " + String(bme_sensor.bsecStatus);
      printInScreen(output, true);
      delay(2000);
    } else {
      String output = "BSEC warning code : " + String(bme_sensor.bsecStatus);
      printInScreen(output, true);
      delay(500);
    }
  }

  if (bme_sensor.bme68xStatus != BME68X_OK) {
    if (bme_sensor.bme68xStatus < BME68X_OK) {
      String output = "BME68X error code : " + String(bme_sensor.bme68xStatus);
      printInScreen(output, true);
      delay(2000);
    } else {
      String output = "BME68X warning code : " + String(bme_sensor.bme68xStatus);
      printInScreen(output, true);
      delay(500);
    }
  }
}


static uint32_t last_gps_fix = 0;
void gps_task(void *pvParameters) {
  while (true) {
    if (SerialGPS.available() > 0) {
      bool end_of_sentence = false;
      while (!end_of_sentence) {
        if (SerialGPS.available() == 0) {
          delay(1);
          continue;
        }
        // char c = SerialGPS.read();
        // // if printable char, add to sentence buffer
        // if (isprint(c)) {
        //   Serial.print(c);
        // }
        end_of_sentence = tiny_gps.encode(SerialGPS.read());
      }
    }
    last_gps_fix = millis();
    if (millis() - last_gps_fix > MAX_GPS_FIX_AGE) {
      if (xSemaphoreTake(gpsState_mutex, 300 * portTICK_PERIOD_MS)) { // take the mutex, max wait time 300ms
        gpsState.satellites = 0;
        gpsState.quality = TFT_RED;
        xSemaphoreGive(gpsState_mutex); // give the mutex back
        // Serial.println("GPS Fix too old");
      }
    }

    if (xSemaphoreTake(gpsState_mutex, 300 * portTICK_PERIOD_MS)) { // take the mutex, max wait time 300ms
      gpsState.hdop = tiny_gps.hdop.hdop();

      gpsState.satellites = 0;
      if (tiny_gps.satellites.isValid() && tiny_gps.satellites.age() <= MAX_GPS_FIX_AGE) {
        gpsState.satellites = tiny_gps.satellites.value();
      }
      // Serial.println("GPS is valid: " + String(tiny_gps.satellites.isValid()));
      // Serial.println("GPS is age: " + String(tiny_gps.satellites.age()));
      // Serial.println("GPS satellites: " + String(tiny_gps.satellites.value()));

      gpsState.quality = TFT_RED;
      //if (gpsState.satellites >= 5) {
      if (gpsState.hdop <= 5) { // https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
        gpsState.quality = TFT_GREEN;
      } else if (gpsState.hdop <= 20) {
        gpsState.quality = TFT_YELLOW;
      } else {
        // Serial.println("GPS TFT_RED 1");
        gpsState.quality = TFT_RED;
      }
      if (!tiny_gps.altitude.isValid() || tiny_gps.altitude.age() > MAX_GPS_FIX_AGE) {
        gpsState.quality = TFT_RED;
      }
      if (gpsState.satellites >= 0) {
        if (tiny_gps.altitude.isUpdated()) {
          tiny_gps.altitude.value(); // Set updated = false;
          gpsState.altitude_last_update = millis();
          gpsState.altitude = tiny_gps.altitude.meters();
          // Serial.println("GPS altitude: " + String(gpsState.altitude));
        }else {
          // Serial.println("GPS altitude not updated: " + String(tiny_gps.altitude.meters()));
          // Serial.println("lat: " + String(tiny_gps.location.lat()));
          // Serial.println("lon: " + String(tiny_gps.location.lng()));
        }
        if (gpsState.originLat != 0) {
          gpsState.totalDist += TinyGPSPlus::distanceBetween(tiny_gps.location.lat(), tiny_gps.location.lng(), gpsState.originLat, gpsState.originLon);
        }
        gpsState.originLat = tiny_gps.location.lat();
        gpsState.originLon = tiny_gps.location.lng();
        gpsState.originAlt = tiny_gps.altitude.meters();

        if (tiny_gps.speed.kmph() > gpsState.spdMax) {
          gpsState.spdMax = tiny_gps.speed.kmph();
        }
        if (tiny_gps.altitude.meters() > gpsState.altMax) {
          gpsState.altMax = tiny_gps.altitude.meters();
        }
        if (tiny_gps.altitude.meters() < gpsState.altMin) {
          gpsState.altMin = tiny_gps.altitude.meters();
        }
      }
      xSemaphoreGive(gpsState_mutex); // give the mutex back
    }
    delay(GPS_RATE);
  }
}


static bool seaLevel_updated_after_boot = false;
void sensors_task (void *pvParameters) {

  // temperature offset:
  //  - 240mhz
  //  - ambient: 24
  //  - cpu: 42.50
  //  - bme: 32.0
  //
  //  - 80mhz
  //  - ambient: 23
  //  - cpu: 37.5
  //  - bme: 29.78
  //
  //  - 80mhz no display
  //  - ambient: 24
  //  - cpu: 37.5
  //  - bme: 30.48
  //
  //  - 80mhz no display
  //  - ambient: 24
  //  - cpu: 40.5
  //  - bme: 31.3
  bme_sensor.setTemperatureOffset(6.0);
  while (true) {
    if (bme_sensor.run()) { // If new data is available
      String output = "";
      output += " iaq " + String(bme_sensor.iaq);
      output += " iaqAccuracy " + String(bme_sensor.iaqAccuracy);
      output += " staticIaq " + String(bme_sensor.staticIaq);
      output += " co2Equivalent " + String(bme_sensor.co2Equivalent);
      output += " breathVocEquivalent " + String(bme_sensor.breathVocEquivalent);
      output += " rawTemperature " + String(bme_sensor.rawTemperature);
      output += " pressure " + String(bme_sensor.pressure);
      output += " rawHumidity " + String(bme_sensor.rawHumidity);
      output += " gasResistance " + String(bme_sensor.gasResistance);
      output += " stabStatus " + String(bme_sensor.stabStatus);
      output += " runInStatus " + String(bme_sensor.runInStatus);
      output += " temperature " + String(bme_sensor.temperature);
      output += " humidity " + String(bme_sensor.humidity);
      output += " gasPercentage " + String(bme_sensor.gasPercentage);
      //Serial.println(output);

      // We need to take the temperature of the ESP32 CPU to estimate the
      // temperature offset for the BME sensor. Since the BME sensor is sitting
      // right next to the CPU inside the cramped case, the temperature of the
      // CPU will affect the temperature reading of the BME sensor. We need to
      // subtract the temperature of the CPU.
      float result = 0;
      temp_sensor_read_celsius(&result); 
      // Serial.println("CPU Temperature: " + String(result));
      // Serial.println("BME Temperature " + String(bme_sensor.temperature));
      result = result - 30.5;
      if (result < 0) { result = 0; }
      bme_sensor.setTemperatureOffset(result);
      // Serial.println("Temperature offset " + String(result));

/*
      Serial.println("Pressure " + String(bme_sensor.pressure));
      Serial.println("temperatureCompensatedAltitude " + String(temperatureCompensatedAltitude(bme_sensor.pressure, bme_sensor.temperature - 10)));
      Serial.println("altitude " + String(altitude(bme_sensor.pressure)));
      Serial.println("calculate_altitude " + String(calculate_altitude(bme_sensor.pressure)));
      Serial.println("bmp085_pressureToAltitude " + String(bmp085_pressureToAltitude(bme_sensor.pressure)));

      Serial.println("Temperature " + String(bme_sensor.temperature));
*/

      float altitude = INVALID_DATA;
      float gps_altitude = INVALID_DATA;
      bool gps_high_quality = false;
      bool gps_medium_quality = false;
      if (xSemaphoreTake(gpsState_mutex, 300 * portTICK_PERIOD_MS)) { // take the mutex, max wait time 300ms
        if (gpsAltitudeIsUpdated()) {
          gps_high_quality = gpsSignalIsGoodQuality();
          gps_medium_quality = gpsSignalIsMediumQuality();
          if (gps_high_quality || gps_medium_quality) {
            gps_altitude = gpsState.altitude;
          }
        }
        xSemaphoreGive(gpsState_mutex); // give the mutex back
      }

      float new_seaLevel = DEFAULT_SEA_LEVEL;
      xSemaphoreTake(sensorsState_mutex, 300 * portTICK_PERIOD_MS); // take the mutex, max wait time 300ms
      const float bmp085_altitude = bmp085_pressureToAltitude(bme_sensor.pressure / 100.0 /* convert to hPa */, sensorsState.seaLevel);
      if (gps_altitude == INVALID_DATA) {
        // Use previous seaLevel recorded in the sensorsState
        altitude = bmp085_altitude;
      } else {
        if (gps_high_quality) {
          altitude = gps_altitude;
          new_seaLevel = bmp085_seaLevelForAltitude(altitude, bme_sensor.pressure / 100.0 /* convert to hPa */);
        } else if (gps_medium_quality) {
          altitude = bmp085_altitude * 0.5 + gps_altitude * 0.5;
          // This will exponentially smooth the sealLevel towards the gps altitude when the gps signal is medium quality
          new_seaLevel = bmp085_seaLevelForAltitude(altitude, bme_sensor.pressure / 100.0 /* convert to hPa */);
        }
        
      }

      sensorsState.iaq = bme_sensor.iaq;
      sensorsState.iaqAccuracy = bme_sensor.iaqAccuracy;
      sensorsState.staticIaq = bme_sensor.staticIaq;
      sensorsState.co2Equivalent = bme_sensor.co2Equivalent;
      sensorsState.breathVocEquivalent = bme_sensor.breathVocEquivalent;
      sensorsState.rawTemperature = bme_sensor.rawTemperature;
      sensorsState.pressure = bme_sensor.pressure;
      sensorsState.altitude = altitude;
      if (new_seaLevel != DEFAULT_SEA_LEVEL || (millis() - sensorsState.seaLevel_last_update > SEALEVEL_MAX_AGE)) {
        sensorsState.seaLevel = new_seaLevel;
        sensorsState.seaLevel_last_update = millis();
        seaLevel_updated_after_boot = true;
      }
      sensorsState.rawHumidity = bme_sensor.rawHumidity;
      sensorsState.gasResistance = bme_sensor.gasResistance;
      sensorsState.stabStatus = bme_sensor.stabStatus;
      sensorsState.runInStatus = bme_sensor.runInStatus;
      sensorsState.temperature = bme_sensor.temperature;
      sensorsState.humidity = bme_sensor.humidity;
      sensorsState.gasPercentage = bme_sensor.gasPercentage;

      saveSensorState();
      xSemaphoreGive(sensorsState_mutex); // give the mutex back
    } else {
      checkBmeSensorStatus();
    }
    while (!lightMeter.measurementReady(true)) {
      yield();
    }
    readLight();

    bmeSaveState();
    delay(SENSOR_RATE);
  }
}

void saveSensorState() {
  static uint32_t sensor_state_update_time_ms = 0;
  if (sensorsState.altitude == INVALID_DATA && sensorsState.pressure == INVALID_DATA && sensorsState.seaLevel == DEFAULT_SEA_LEVEL) {
    return;
  }
  bool shoould_save = 
      (sensor_state_update_time_ms == 0 && seaLevel_updated_after_boot) ||
      (
        (millis() - sensor_state_update_time_ms) >= SAVE_SENSOR_STATE_PERIOD &&
        (millis() - sensorsState.seaLevel_last_update) < SAVE_SENSOR_STATE_PERIOD
      );

  if (!shoould_save) { return; }

  Serial.println("Saving sensor state");
  sensor_state_update_time_ms = millis();
  prefs.begin("car-altimeter", false); 
  prefs.putFloat("seaLevel", sensorsState.seaLevel);
  prefs.putInt("seaLevel_last_update", sensorsState.seaLevel_last_update);
  prefs.putFloat("altitude", sensorsState.altitude);
  prefs.putFloat("pressure", sensorsState.pressure);
  prefs.end();
}

void loadSensorState() {
  prefs.begin("car-altimeter", false); 
  sensorsState.seaLevel_last_update = prefs.getInt("seaLevel_last_update", 0);
  if (sensorsState.seaLevel_last_update == 0 || (millis() - sensorsState.seaLevel_last_update > SEALEVEL_MAX_AGE)) {
    prefs.end();
    return;
  }
  sensorsState.seaLevel = prefs.getFloat("seaLevel", DEFAULT_SEA_LEVEL);
  sensorsState.altitude = prefs.getFloat("altitude", INVALID_DATA);
  sensorsState.pressure = prefs.getFloat("pressure", INVALID_DATA);
  prefs.end();
}

void setScreenRotation(int rotation) {
  if (rotation == 0) {
    rotation = 3;
  }
  tft.setRotation(rotation);
  screen.setRotation(rotation);
}

void saveScreenChoice() {
  prefs.begin("car-altimeter", false); 
  prefs.putInt("screen_choice", g_currently_selected_chart);
  prefs.end();
}

void loadScreenChoice() {
  prefs.begin("car-altimeter", false); 
  g_currently_selected_chart = prefs.getInt("screen_choice", 0);
  prefs.end();
}

uint8_t bme_bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
void bmeLoadState(void) {
  prefs.begin("car-altimeter", false); 
  // int bme_state_size = prefs.getInt("bme_state_size", 0);
  // if (bme_state_size == 0) return;

  // Serial.println("BME Reading state from disk");
  size_t bme_state_len = prefs.getBytesLength("bme_state");
  if (bme_state_len == 0) {
    prefs.end();
    // Serial.println("BME No state in disk");
    return;
  }
  if (bme_state_len != BSEC_MAX_STATE_BLOB_SIZE) {
    String msg = "Error: BME size in disk: " + String(bme_state_len) + " != " + String(BSEC_MAX_STATE_BLOB_SIZE);
    printInScreen(msg, true);
    prefs.end();
    delay(500);
    return;
  }
  Serial.println("BME Valid state size, setting BME state.");
  prefs.getBytes("bme_state", bme_bsecState, bme_state_len);
  bme_sensor.setState(bme_bsecState);
  checkBmeSensorStatus();
  prefs.end();
}

void bmeSaveState(void) {
  static uint32_t state_update_time_ms = 0;
  bool update = false;
  if (state_update_time_ms == 0) {
    /* First state update when IAQ accuracy is >= 3 */
    if (bme_sensor.iaqAccuracy >= 3) {
      Serial.println("BME First state update");
      update = true;
    }
  } else {
    /* Update every BME_STATE_SAVE_PERIOD minutes */
    if ((millis() - state_update_time_ms) > BME_STATE_SAVE_PERIOD) {
      update = true;
    }
  }

  if (update) {
    state_update_time_ms = millis();
    bme_sensor.getState(bme_bsecState);
    checkBmeSensorStatus();
    // Serial.println("BME Writing state to disk");
    prefs.begin("car-altimeter", false); 
    prefs.putBytes("bme_state", bme_bsecState, BSEC_MAX_STATE_BLOB_SIZE);
    prefs.end();
  }
}

