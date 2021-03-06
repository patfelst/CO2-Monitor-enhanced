#include <DFRobot_VEML7700.h>
#include <FastLED.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <esp_sntp.h>

#include "DSEG7Modern40.h"
#include "DSEG7ModernBold60.h"
#include "RunningAverage.h"
#include "TickTwo.h"
#include "co2_generic.h"
#include "time.h"
#include "wifi_credentials.h"

// TODO Check scaling of bargraph
// TODO Save 24-hour history to SD card
// TODO add MENU system to set options

// General defines
#define sw_version "v0.7.0"
bool debug_mode = false;  // Set true to output some serial debug text
#define TFT_BACKGND           TFT_BLACK
#define max_adc_value         110  // max ADC value corresponds to max LCD and LED brightness
#define led_brightness_pc_low 20
#define lcd_brightness_low    50

// Uncomment this to apply temperature offset and altitude, only do once, then re-flash with this commented out
// #define UPDATE_SETTINGS
#define temperature_offset 10.0  // Temperature offset for CO2 sensor based temperature sensor
#define altitude           88    // altitude in metres used for CO2 sensor

// RGB LED defines
#define LED_COUNT 10
#define LED_PIN   25

// LDR light sensor pin
#define LDR_PIN 35

// LCD text and graphics coordinates
#define lcd_width  320
#define lcd_height 240

// CO2 value - text coordinates
#define co2_value_x (M5.Lcd.width() - 53)
#define co2_value_y 30

// CO2 units - text coordinates
#define co2_units_x M5.Lcd.width()
#define co2_units_y (co2_value_y + 3)

// Temp and humidity - text coordinates
#define temp_units_colour TFT_LIGHTGRAY
#define temp_units_bg     TFT_BACKGND
#define temp_val_colour   TFT_LIGHTGRAY
#define temp_val_bg       TFT_BACKGND
#define temp_align        bottom_right
#define temp_val_x        120                     // temperature X coordinate
#define temp_val_y        (M5.Lcd.height() - 15)  // temperature Y coordinate
#define humid_align       bottom_right
#define humid_val_x       (M5.Lcd.width() - 53)  // humidity X coordinate
#define humid_val_y       temp_val_y             // humidity Y coordinate

// CO2 effect on people - text coordinates
#define effect_align bottom_centre
#define effect_txt_x (M5.Lcd.width() / 2)
#define effect_txt_y (M5.Lcd.height() - 85)

// Time and date - text coordinates
#define time_align top_right
#define time_txt_x M5.Lcd.width()
#define time_txt_y 0
#define date_txt_x time_txt_x
#define date_txt_y (time_txt_y + 30)

// NTP connection - text coordinates
#define ntp_msg_x (M5.Lcd.width() / 2)
#define ntp_msg_y 130

// Battery icon data
#define batt_spr_x       0    // Battery sprite X location on LCD
#define batt_spr_y       0    // Battery sprite Y location on LCD
#define batt_spr_wdth    113  // Battery sprite width
#define batt_spr_ht      20   // Battery sprite height
#define batt_rect_width  30
#define batt_rect_height 14
#define batt_button_wdth 4
#define batt_button_ht   6

// CO2 history sizes for circular buffers
#if defined SENSOR_IS_SCD30
  #define co2_sec_per_sample    2   // SCD-30 CO2 sensor sample rate is 1 sample / 2 sec
  #define co2_raw_hist_disp_pts 24  // 2 sec / sample * 24 samples = 48 seconds total
#elif defined SENSOR_IS_SCD41
  #define co2_sec_per_sample    5   // SCD-41 CO2 sensor sample rate is 1 sample / 5 sec
  #define co2_raw_hist_disp_pts 24  // 5 sec / sample * 24 samples = 120 seconds total
#elif defined SENSOR_IS_SGP30
  #define co2_sec_per_sample    1   // SCD-41 CO2 sensor sample rate is 1 sample / 5 sec
  #define co2_raw_hist_disp_pts 24  // 1 sec / sample * 24 samples = 24 seconds total
#endif

#define co2_raw_pts_per_min (60 / co2_sec_per_sample)   // Number of samples per minute
#define co2_raw_hist_pts    (60 * co2_raw_pts_per_min)  // Store 1 hour of raw points
#define co2_minute_hist_pts 60                          // Store 1 hour of minute history
#define co2_hour_hist_pts   24                          // Store 1 day of hour history

// CO2 bargraph display
#define co2_minute_hist_disp_pts 30  // Only display last 30 minutes otherwise bars are too narrow
#define co2_hour_hist_disp_pts   24  // Only display last 12 hours otherwise bars are too narrow

// Gaps between bar graphs on CO2 history
#define raw_bar_gap  3
#define mins_bar_gap 2
#define hour_bar_gap 2

// CO2 history box - sprite dimensions
#define co2_hist_spr_x  10   //  Location to push bargraph sprite
#define co2_hist_spr_y  100  //  Location to push bargraph sprite
#define co2_hist_spr_w  300  // Bargraph sprite width
#define co2_hist_spr_h  140  // Bargraph Sprite height
#define co2_spr_title_x 3    // X coordinates of co2 title
#define co2_spr_title_y 3    // X coordinates of co2 title

// Circular gauge pointer
#define gauge_ptr_spr_w  20
#define gauge_ptr_spr_h  20
#define gauge_tick_spr_w 3
#define gauge_tick_spr_h 15
#define rad_1            105
#define rad_2            145
#define arc_x            (lcd_width / 2)
#define arc_y            160

// Function prototypes
void start_co2_sensor(bool);
void display_time(void);
bool connect_wifi(uint8_t max_tries = 15);
void sync_rtc_to_ntp(void);
void disp_batt_wrapper(void);
void disp_batt_symbol(uint16_t batt_x, uint16_t batt_y, bool disp_volts);
void display_co2_effect(const char* effect, int32_t colour);
void display_co2_value(uint16_t co2, int32_t colour);
void display_co2_units();
void display_temp_humid(float temp, float humid);
void co2_to_colour(uint16_t co2, uint32_t& led_colour, int32_t& lcd_colour, char* txt);
void read_lux_sensor(void);
void display_lux_val();
void set_rgb_led(uint8_t brightness, uint32_t colour);
void save_co2_history(void);
void main_display(void);
uint16_t co2_to_bargraph_ht(float co2);
void display_title_timespan(const char* timespan);
void display_ave_co2(float ave);
void display_max_co2(float max);
void display_min_co2(float min);
void display_wait_msg(const char* msg);
void scd_x_forced_cal(uint16_t target_co2);
void scd_x_settings(float temp_offs, uint16_t alt, bool ASC);
void sim_sensor_wrapper(void);
void draw_circular_gauge_scale(void);
void draw_circular_gauge_pointer(uint16_t percent);

// Object creation
DFRobot_VEML7700 lux;
CO2_generic co2;
CRGB leds[LED_COUNT];                           // WS2812 RGB LED object
TickTwo clock_display(display_time, 1000);      // Schedule time to display once per second
TickTwo batt_display(disp_batt_wrapper, 5000);  // Schedule display battery icon every 5 seconds
TickTwo co2_display(main_display, 500);         // Schedule CO2 display twice per second
TickTwo co2_history(save_co2_history, 1000);    // Schedule save CO2 history every second
TickTwo sim(sim_sensor_wrapper, 5000);          // Schedule simulation of the SCD-30 every 5 seconds
TickTwo read_lux(read_lux_sensor, 5000);        // Schedule read of lux sensor and set LCD and RGB LED brightness
m5::rtc_time_t RTCtime;
m5::rtc_date_t RTCdate;
M5Canvas batt_sprite(&M5.Lcd);                        // Sprite for battery icon and percentage text
M5Canvas co2_hist_sprite(&M5.Lcd);                    // Sprite for CO2 history bargraph
M5Canvas gauge_pointer(&M5.Lcd);                      // Sprite for semi circular gauge triangle pointer
M5Canvas gauge_ticks(&M5.Lcd);                        // Sprite for semi circular gauge scale ticks
RunningAverage co2_raw_hist(co2_raw_hist_pts);        // Circular buffer for raw CO2 samples
RunningAverage co2_minute_hist(co2_minute_hist_pts);  // Circular buffer for minute CO2 samples
RunningAverage co2_hour_hist(co2_hour_hist_pts);      // Circular buffer for hour CO2 samples

enum {
  display_tem_hum,
  dispaly_gauge,
  display_hist_raw,
  display_hist_minute,
  display_hist_hour,
  display_lux,
  display_settings,
};
uint8_t display_state = display_tem_hum;
bool display_init = false;
uint32_t led_brightness_pc = 0;
uint8_t lcd_brightness_pc = 0;
float lux_float;

/*
-----------------
  setup(void)
-----------------
*/
void setup(void) {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;  // default=115200. if "Serial" is not needed, set it to 0.
  cfg.clear_display = true;      // default=true. clear the screen when begin.
  cfg.output_power = true;       // default=true. use external port 5V output.
  cfg.internal_imu = true;       // default=true. use internal IMU.
  cfg.internal_rtc = true;       // default=true. use internal RTC.
  cfg.external_imu = false;      // default=false. use Unit Accel & Gyro.
  cfg.external_rtc = false;      // default=false. use Unit RTC.
  cfg.led_brightness = 0;        // default= 0. Green LED brightness (0=off / 255=max) (??? not NeoPixel)

  M5.begin(cfg);
  M5.Lcd.setBrightness(180);  // Core2 LCD backlight brightness

  lux.begin();
  // lux.setIntegrationTime(DFRobot_VEML7700::ALS_INTEGRATION_400ms);

  // Setup RGB LED
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, LED_COUNT);
  set_rgb_led(100, CRGB::Fuchsia);  // RGB LED brightness to 100%

  // Create battery icon sprite
  batt_sprite.createSprite(batt_spr_wdth, batt_spr_ht);

  // Create CO2 history bargraph sprite
  co2_hist_sprite.createSprite(co2_hist_spr_w, co2_hist_spr_h);

  // Create semi circular gauge pointer sprite
  gauge_pointer.createSprite(gauge_ptr_spr_w, gauge_ptr_spr_h);
  gauge_pointer.setPivot(gauge_ptr_spr_w / 2, rad_1 - 5);

  // create semi circular gauge scale ticks sprite
  gauge_ticks.createSprite(gauge_tick_spr_w, gauge_tick_spr_h);
  gauge_ticks.setPivot(1, rad_2 + gauge_tick_spr_h + 1);

  M5.Speaker.begin();
  // The setVolume function can be set the master volume in the range of 0-255.
  M5.Speaker.setVolume((10 * 255) / 100);
  // M5.Speaker.setAllChannelVolume((35 * 255) / 100);

  // const uint16_t C4 = 261.626;
  // const uint16_t E4 = 329.628;
  // const uint16_t G4 = 391.995;
  const uint16_t C6 = 1046.50;
  // const uint16_t E6 = 1318.51;
  const uint16_t G6 = 1567.98;
  uint16_t chord_duration_ms = 200;
  M5.Speaker.tone(C6, chord_duration_ms, 0, false);
  // delay(250);
  M5.Speaker.tone(G6, chord_duration_ms, 1, false);

  // Start CO2 sensor and display sensor settings
  start_co2_sensor(true);

#if (defined UPDATE_SETTINGS) && (defined SENSOR_IS_SCD41 || defined SENSOR_IS_SCD30)
  scd_x_settings(temperature_offset, altitude, true);  // Uncomment to update the settings one time, which get saved to SCD-x EEPROM
#endif

  // If no sensor detected, switch to simulation mode
  if (co2.simulate_co2) {
    if (debug_mode) Serial.printf("%s CO2 sensor NOT connected, switching to simulation mode\n", co2_sensor_type_str);
    co2.co2_level = 400;
    co2.temperature = 20.0;
    co2.humidity = 25.0;
    sim.start();
    // Simulate co2 history for buffers that take ages to fill
    for (int i = 0; i < co2_raw_hist_pts - 1; i++) {
      co2_raw_hist.addValue((float)random(400, 5000));
    }
    for (int i = 0; i < co2_minute_hist_disp_pts - 1; i++) {
      co2_minute_hist.addValue((float)random(400, 5000));
    }
    for (int i = 0; i < co2_hour_hist_disp_pts + 1; i++) {
      co2_hour_hist.addValue((float)random(400, 5000));
    }
  }

  display_init = true;

  // Clear the co2 circular buffers
  co2_raw_hist.clear();
  co2_minute_hist.clear();
  co2_hour_hist.clear();

  // Start scheduled tasks
  clock_display.start();
  batt_display.start();
  co2_history.start();
  co2_display.start();
  read_lux.start();
}

/*
-----------------
  loop(void)
-----------------
*/
void loop(void) {
  M5.update();  // check touch buttons

  // Indicate calibration mode will be entered while BtnB is being held
  if (M5.BtnC.isHolding()) {
    display_co2_effect("Hold to Calibrate", TFT_CYAN);
  } else
    co2_display.update();

  // Scheduled tasks update
  clock_display.update();
  batt_display.update();
  co2_history.update();
  read_lux.update();

  // Enter calibration mode after BtnB held for 5 seconds
  if (M5.BtnC.pressedFor(5000)) {
    scd_x_forced_cal(425);  // We just assume outdoor "fresh air" is 425 ppm, it will be pretty close
    display_state = display_tem_hum;
    display_init = true;
  }

  // Connect to WiFi to sync ESP32's RTC to internet NTP sever
  if (M5.BtnA.pressedFor(1000)) {
    M5.Lcd.clear();
    // Connect to WiFi
    if (connect_wifi()) {
      delay(500);
      sync_rtc_to_ntp();
      WiFi.disconnect(true);  // Disconnect wifi
      WiFi.mode(WIFI_OFF);    // Set the wifi mode to off
    }
    delay(2000);
    display_init = true;
  }

  // Check for user change display type
  auto td = M5.Touch.getDetail();
  if (td.wasPressed()) {
    if (td.x > lcd_width / 2 && td.y < lcd_height / 2) {
      display_init = true;
      if (display_state == display_settings)
        display_state = display_tem_hum;
      else
        display_state++;
    } else if (td.x <= lcd_width / 2 && td.y < lcd_height / 2) {
      display_init = true;
      display_state = display_tem_hum;
    }
  }

  if (co2.simulate_co2)
    sim.update();
  else {
    // Check if data is available from CO2 sensor
    co2.get_co2();
  }
}

/*
-----------------
  Display the CO2, temperature and humidity screen, or
  One of three CO2 history bargraphs
-----------------
*/
void main_display(void) {
  // Get LED and LCD colour based on CO2 level
  uint32_t co2_led_colour = 0;
  int32_t co2_lcd_colour = 0;
  int32_t co2_lcd_colour2 = 0;
  char txt_msg[50] = "";
  int last_sample = 0;
  uint16_t bar_w = 0;
  uint16_t bar_h = 0;
  uint16_t i_shift = 0;
  static bool display_drawn_in_colour = false;

  co2_to_colour(co2.co2_level, co2_led_colour, co2_lcd_colour, txt_msg);
  co2_lcd_colour2 = co2_lcd_colour;
  set_rgb_led(led_brightness_pc, co2_led_colour);  // Neopixel RGB LED colour and brightness

#if defined SENSOR_IS_SGP30
  // Don't blink the co2 value as it updates at 1Hz
  if (co2.co2_updated || display_init) {
    co2.co2_updated = false;
    display_drawn_in_colour = false;
  } else if (display_drawn_in_colour)
    return;
  else
    display_drawn_in_colour = true;
#else
  static uint32_t highlight_timer = millis();
  // Blink the co2 value white for half a second to indicate an update
  if (co2.co2_updated || display_init) {
    highlight_timer = millis();
    co2.co2_updated = false;
    display_drawn_in_colour = false;
    co2_lcd_colour = TFT_WHITE;
  } else if ((millis() < highlight_timer + 500) || display_drawn_in_colour)
    return;
  else
    display_drawn_in_colour = true;
#endif

  M5.Lcd.setTextPadding(0);

  // Display SIM in title bar if in Simulate mode
  if (co2.simulate_co2) {
    M5.Lcd.setTextColor(TFT_ORANGE);
    M5.Lcd.setFont(&fonts::FreeSans12pt7b);
    if (display_state == dispaly_gauge) {
      M5.Lcd.setTextDatum(bottom_centre);
      M5.Lcd.drawString("SIM!", lcd_width / 2, lcd_height);
    } else
      M5.Lcd.setTextDatum(top_centre);
    M5.Lcd.drawString("SIM!", lcd_width / 2 + 20, time_txt_y);
  }

  // Prepare to display CO2 history bargraph
  if (display_init && (display_state == display_hist_raw || display_state == display_hist_minute || display_state == display_hist_hour)) {
    display_init = false;
    M5.Lcd.clear();
    co2_hist_sprite.setTextDatum(top_left);
    co2_hist_sprite.setTextColor(TFT_ORANGE, TFT_BLACK);
    co2_hist_sprite.setFont(&fonts::FreeSans9pt7b);
    co2_hist_sprite.clear(TFT_BLACK);
    co2_hist_sprite.drawRect(0, 0, co2_hist_spr_w, co2_hist_spr_h, TFT_DARKGRAY);
  }

  switch (display_state) {
    // Temperature and Humidity and CO2 effect on people
    case display_tem_hum:
      if (display_init) {
        display_init = false;
        M5.Lcd.clear();
        display_co2_units();
      }
      display_co2_value(co2.co2_level, co2_lcd_colour);
      display_temp_humid(co2.temperature, co2.humidity);
      display_co2_effect(txt_msg, co2_lcd_colour2);
      break;

    case display_lux:
      if (display_init) {
        display_init = false;
        M5.Lcd.clear();
      }
      display_lux_val();
      break;

    case dispaly_gauge:
      if (display_init) {
        display_init = false;
        M5.Lcd.clear();
        draw_circular_gauge_scale();
        display_co2_units();
      }
      display_co2_value(co2.co2_level, co2_lcd_colour);
      draw_circular_gauge_pointer((co2.co2_level * 100) / 2500);
      break;

    case display_settings:
      if (display_init) {
        // Display co2 settings without starting the sensor
        start_co2_sensor(false);
        display_init = true;
        display_state = display_tem_hum;
      }
      break;

    // Raw CO2 history for last two minutes
    case display_hist_raw:
      display_co2_value(co2.co2_level, co2_lcd_colour);
      display_co2_units();
      // Erase the old bargraph, but not the outer border
      co2_hist_sprite.fillRect(1, 1, co2_hist_spr_w - 2, co2_hist_spr_h - 2, TFT_BLACK);
      sprintf(txt_msg, "<=%ds=>", co2_raw_hist_disp_pts * co2_sec_per_sample);
      display_title_timespan(txt_msg);

      if (co2_raw_hist.getCount() > 0) {
        // Draw a bargraph of co2 history - for last 1 minute (12 samples)
        last_sample = co2_raw_hist.getCount() - 1;                       // -1 as first idx == 0
        bar_w = (co2_hist_spr_w / co2_raw_hist_disp_pts) - raw_bar_gap;  // -2 for 1 pixel gap on each side of bar

        // If circular buffer has more points than being displayed, use a "shift" variable so x-axis remain on screen
        if (last_sample >= co2_raw_hist_disp_pts)
          i_shift = last_sample - co2_raw_hist_disp_pts + 1;

        for (int i = last_sample; i >= (last_sample - co2_raw_hist_disp_pts + 1) && i >= 0; i--) {
          bar_h = co2_to_bargraph_ht(co2_raw_hist.getValue(i));
          co2_to_colour(co2_raw_hist.getValue(i), co2_led_colour, co2_lcd_colour, nullptr);
          co2_hist_sprite.fillRect(7 + ((i - i_shift) * (bar_w + raw_bar_gap)), co2_hist_spr_h - bar_h + 1, bar_w, bar_h - 2, co2_lcd_colour);
        }
        // Display the average and max CO2 level in this history period
        display_min_co2(co2_raw_hist.getMinInBufferLast(co2_raw_hist_disp_pts));
        // display_ave_co2(co2_raw_hist.getAverageLast(co2_raw_hist_disp_pts));
        display_max_co2(co2_raw_hist.getMaxInBufferLast(co2_raw_hist_disp_pts));
      } else
        display_wait_msg("Wait for next raw sample");

      // Display the sprite
      co2_hist_sprite.pushSprite(co2_hist_spr_x, co2_hist_spr_y);
      break;

    // Last 30 minutes of CO2 history, each bar is an average of 1 minute of raw CO2
    case display_hist_minute:
      display_co2_value(co2.co2_level, co2_lcd_colour);
      display_co2_units();
      // Erase the old bargraph, but not the outer border
      co2_hist_sprite.fillRect(1, 1, co2_hist_spr_w - 2, co2_hist_spr_h - 2, TFT_BLACK);
      sprintf(txt_msg, "<=%dm=>", co2_minute_hist_disp_pts);
      display_title_timespan(txt_msg);

      if (co2_minute_hist.getCount() > 0) {
        // Draw a bargraph of co2 history - for last 30 minutes (30 samples)
        last_sample = co2_minute_hist.getCount() - 1;                        // -1 as first idx == 0
        bar_w = (co2_hist_spr_w / co2_minute_hist_disp_pts) - mins_bar_gap;  // -2 for 1 pixel gap on each side of bar

        // If circular buffer has more points than being displayed, use a "shift" variable so x-axis remain on screen
        if (last_sample >= co2_minute_hist_disp_pts)
          i_shift = last_sample - co2_minute_hist_disp_pts + 1;

        for (int i = last_sample; i >= (last_sample - co2_minute_hist_disp_pts + 1) && i >= 0; i--) {
          bar_h = co2_to_bargraph_ht(co2_minute_hist.getValue(i));
          co2_to_colour(co2_minute_hist.getValue(i), co2_led_colour, co2_lcd_colour, nullptr);
          co2_hist_sprite.fillRect(1 + ((i - i_shift) * (bar_w + mins_bar_gap)), co2_hist_spr_h - bar_h + 1, bar_w, bar_h - 2, co2_lcd_colour);
        }

        // Display the average and max CO2 level in this history period
        display_min_co2(co2_minute_hist.getMinInBufferLast(co2_minute_hist_disp_pts));
        // display_ave_co2(co2_minute_hist.getAverageLast(co2_minute_hist_disp_pts));
        display_max_co2(co2_minute_hist.getMaxInBufferLast(co2_minute_hist_disp_pts));
      } else
        display_wait_msg("Wait for next minute");
      co2_hist_sprite.pushSprite(co2_hist_spr_x, co2_hist_spr_y);
      break;

    // Last 24 hours of CO2 history, each bar is an average of 60 minutes of CO2 history
    case display_hist_hour:
      display_co2_value(co2.co2_level, co2_lcd_colour);
      display_co2_units();
      // Erase the old bargraph, but not the outer border
      co2_hist_sprite.fillRect(1, 1, co2_hist_spr_w - 2, co2_hist_spr_h - 2, TFT_BLACK);
      sprintf(txt_msg, "<=%dhr=>", co2_hour_hist_disp_pts);
      display_title_timespan(txt_msg);

      if (co2_hour_hist.getCount() > 0) {
        // Draw a bargraph of co2 history - for last 12 hours (12 samples)
        last_sample = co2_hour_hist.getCount() - 1;                        // -1 as first idx == 0
        bar_w = (co2_hist_spr_w / co2_hour_hist_disp_pts) - hour_bar_gap;  // -4 for 2 pixel gap on each side of bar
        // If circular buffer has more points than being displayed, use a "shift" variable so x-axis remain on screen
        if (last_sample >= co2_hour_hist_disp_pts)
          i_shift = last_sample - co2_hour_hist_disp_pts + 1;

        for (int i = last_sample; i >= (last_sample - co2_hour_hist_disp_pts + 1) && i >= 0; i--) {
          bar_h = co2_to_bargraph_ht(co2_hour_hist.getValue(i));
          co2_to_colour(co2_hour_hist.getValue(i), co2_led_colour, co2_lcd_colour, nullptr);
          co2_hist_sprite.fillRect(7 + ((i - i_shift) * (bar_w + hour_bar_gap)), co2_hist_spr_h - bar_h + 1, bar_w, bar_h - 2, co2_lcd_colour);
        }

        // Display the average and max CO2 level in this history period
        display_min_co2(co2_hour_hist.getMinInBufferLast(co2_hour_hist_disp_pts));
        // display_ave_co2(co2_hour_hist.getAverageLast(co2_hour_hist_disp_pts));
        display_max_co2(co2_hour_hist.getMaxInBufferLast(co2_hour_hist_disp_pts));

      } else
        display_wait_msg("Wait for next hour");
      co2_hist_sprite.pushSprite(co2_hist_spr_x, co2_hist_spr_y);
      break;

    default:
      break;
  }
}

/*
-----------------
  Save co2 history to circular buffers. Raw co2 level is sampled from SCD-30 every 5 seconds.
  Saves history in 3 circular buffers:
    co2_raw_hist:     Raw co2 samples
    co2_minute_hist:  1 minute CO2 samples
    co2_hour_hist:    1 hour CO2 samples

    To assist in visualisation for the user, as the minute and hour history is building in 1 element
    the element has the cumulative average up to tha point in time.

    Mental model of "RunningAverage" circular buffer. Once full, the previous values are on the RHS of the array, i.e.:
      getValue(size - 1) is the last value added
      getValue(size - 2) is the second last value added, and so on
-----------------
*/
void save_co2_history(void) {
  // static uint32_t samples = 0;

  // SCD-30 samples once per 2 seconds, this will sync history to the RTC at 2 second rate
  if (!(RTCtime.seconds % co2_sec_per_sample)) {
    if (co2.co2_level > 0)
      co2_raw_hist.addValue(co2.co2_level);
    else
      return;

    // samples++;
    // Serial.printf("sync sec=%d, co2=%d, count=%d, total samples=%d\n", RTCtime.seconds, co2.co2_level, co2_raw_hist.getCount(), samples);

    // int last = co2_raw_hist.getCount() - 1;  // -1 as first idx == 0
    // Serial.println("**** Raw history ***");
    // for (int i = last; i > last - co2_raw_hist_pts && i >= 0; i--) {
    //   Serial.print(co2_raw_hist.getValue(i), 0);
    //   Serial.print("\t");
    // }
    // Serial.println();
    // Serial.printf("From the last %d points: Min=%.0f, Ave=%.1f, Max=%.0f\n\n",
    //               co2_raw_hist.getCount() < co2_raw_hist_disp_pts ? co2_raw_hist.getCount() : co2_raw_hist_disp_pts,
    //               co2_raw_hist.getMinInBufferLast(co2_raw_hist_disp_pts),
    //               co2_raw_hist.getAverageLast(co2_raw_hist_disp_pts),
    //               co2_raw_hist.getMaxInBufferLast(co2_raw_hist_disp_pts));

    // Save the 1-minute history
    if (RTCtime.seconds == 0) {
      co2_minute_hist.addValue(co2_raw_hist.getAverageLast(co2_raw_pts_per_min));

      // last = co2_minute_hist.getCount() - 1;
      // Serial.println("**************************");
      // Serial.println("***** Minute history *****");
      // Serial.println("**************************");
      // for (int i = last; i > last - co2_minute_hist_pts && i >= 0; i--) {
      //   Serial.print(co2_minute_hist.getValue(i), 1);
      //   Serial.print("\t");
      // }
      // Serial.println();
      // Serial.printf("From the last %d points: Min=%.0f, Ave=%.1f, Max=%.0f\n\n",
      //               co2_minute_hist.getCount(),
      //               co2_minute_hist.getMinInBuffer(),
      //               co2_minute_hist.getAverage(),
      //               co2_minute_hist.getMaxInBuffer());

      // Save the 1-hour history
      if (RTCtime.minutes == 0) {
        co2_hour_hist.addValue(co2_raw_hist.getAverage());

        // last = co2_hour_hist.getCount() - 1;
        // Serial.println("****************************************************");
        // Serial.println("******************* Hour history *******************");
        // Serial.println("****************************************************");
        // for (int i = last; i > last - co2_hour_hist_pts && i >= 0; i--) {
        //   Serial.print(co2_hour_hist.getValue(i), 1);
        //   Serial.print("\t");
        // }
        // Serial.println();
        // Serial.printf("From the last %d points: Min=%.0f, Ave=%.1f, Max=%.0f\n\n",
        //               co2_hour_hist.getCount(),
        //               co2_hour_hist.getMinInBuffer(),
        //               co2_hour_hist.getAverage(),
        //               co2_hour_hist.getMaxInBuffer());
      }
    }
  }
}

/*
-----------------
  Convert co2 value into a bargraph height in pixels
  Use google sheets to calculate the scale factor
  https://docs.google.com/spreadsheets/d/1Qu3fMzD7Rvy_9NZa7CwBIezJ-XfksOBFPso_ldFfgO4/edit#gid=0
-----------------
*/
uint16_t co2_to_bargraph_ht(float co2) {
  if (co2 < 400.0) co2 = 400.0;
  uint16_t bar_height = (uint16_t)(30.0 * (log(co2) / log(2.0) - 8.64));
  if (bar_height > co2_hist_spr_h - 30)
    bar_height = co2_hist_spr_h - 30;
  return bar_height;
}

/*
-----------------
  Display the x-axis total timespan text
-----------------
*/
void display_title_timespan(const char* timespan) {
  co2_hist_sprite.setFont(&fonts::FreeSans9pt7b);
  co2_hist_sprite.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  co2_hist_sprite.setTextDatum(top_centre);
  co2_hist_sprite.drawString(timespan, co2_hist_spr_w / 2, co2_spr_title_y);
}

/*
-----------------
  Display min co2 on history bargraph sprite
-----------------
*/
void display_min_co2(float min) {
  char txt[40] = "";
  uint32_t led_colour;
  int32_t lcd_colour;
  int32_t x = co2_spr_title_x;

  co2_hist_sprite.setFont(&fonts::FreeSans9pt7b);
  co2_hist_sprite.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  co2_hist_sprite.setTextDatum(top_left);
  co2_hist_sprite.drawString("Min:", x, co2_spr_title_y);
  sprintf(txt, "%.0f", min);
  co2_to_colour(min, led_colour, lcd_colour, nullptr);
  co2_hist_sprite.setTextColor(lcd_colour, TFT_BLACK);
  x += 40;
  co2_hist_sprite.drawString(txt, x, co2_spr_title_y);
}

/*
-----------------
  Display average co2 on history bargraph sprite
-----------------
*/
void display_ave_co2(float ave) {
  char txt[40] = "";
  uint32_t led_colour;
  int32_t lcd_colour;
  int32_t x = co2_spr_title_x;

  co2_hist_sprite.setFont(&fonts::FreeSans9pt7b);
  co2_hist_sprite.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  co2_hist_sprite.setTextDatum(top_left);
  co2_hist_sprite.drawString("Ave:", x, co2_spr_title_y);
  sprintf(txt, "%.0f", ave);
  co2_to_colour(ave, led_colour, lcd_colour, nullptr);
  co2_hist_sprite.setTextColor(lcd_colour, TFT_BLACK);
  x += 40;
  co2_hist_sprite.drawString(txt, x, co2_spr_title_y);
}

/*
-----------------
  Display maximum co2 on history bargraph sprite
-----------------
*/
void display_max_co2(float max) {
  uint32_t led_colour;
  int32_t lcd_colour;
  char txt[40] = "";
  int32_t x = co2_spr_title_x + 205;

  co2_hist_sprite.setFont(&fonts::FreeSans9pt7b);
  co2_hist_sprite.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  co2_hist_sprite.setTextDatum(top_left);
  co2_hist_sprite.drawString("Max:", x, co2_spr_title_y);
  sprintf(txt, "%.0f", max);
  co2_to_colour(max, led_colour, lcd_colour, nullptr);
  co2_hist_sprite.setTextColor(lcd_colour, TFT_BLACK);
  x += 40;
  co2_hist_sprite.drawString(txt, x, co2_spr_title_y);
}

/*
-----------------
  Display wait message in bargraph sprite
-----------------
*/
void display_wait_msg(const char* msg) {
  co2_hist_sprite.setFont(&fonts::FreeSans12pt7b);
  co2_hist_sprite.setTextDatum(top_centre);
  co2_hist_sprite.setTextColor(TFT_RED, TFT_BLACK);
  co2_hist_sprite.drawString(msg, co2_hist_spr_w / 2, co2_spr_title_y + 30);
}

/*
-----------------
  Display temperature and humidity
  Coordinates of temp and humidity text are at bottom right of the values and units are relative to that
-----------------
*/
void display_temp_humid(float temp, float humid) {
  char txt[30] = "";

  // Prepare to display temp and humidity
  M5.Lcd.setFont(&DSEG7_Modern_Regular_40);
  M5.Lcd.setTextPadding(105);

  // Display temperature on LCD
  M5.Lcd.setTextDatum(temp_align);
  M5.Lcd.setTextColor(temp_val_colour, temp_val_bg);
  sprintf(txt, "%2.1f", temp);
  M5.Lcd.drawString(txt, temp_val_x, temp_val_y);

  // Display temperature units "??C"
  M5.Lcd.setTextPadding(0);
  M5.Lcd.setTextDatum(bottom_left);
  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  uint32_t deg_sym_x = temp_val_x + 7;
  uint32_t deg_sym_y = temp_val_y - M5.Lcd.fontHeight();
  M5.Lcd.setTextColor(temp_units_colour, temp_units_bg);
  M5.Lcd.drawCircle(deg_sym_x, deg_sym_y, 4, temp_units_colour);  // Degree symbol
  deg_sym_x += 7;
  M5.Lcd.drawString("C", deg_sym_x, temp_val_y + 3);  // "C after degree symbol"

  // Display humidity on LCD
  M5.Lcd.setFont(&DSEG7_Modern_Regular_40);
  M5.Lcd.setTextPadding(105);
  M5.Lcd.setTextDatum(humid_align);
  M5.Lcd.setTextColor(temp_val_colour, temp_val_bg);
  sprintf(txt, "%3.0f", humid);
  M5.Lcd.drawString(txt, humid_val_x, humid_val_y);

  // Display humidity units "% RH"
  M5.Lcd.setTextPadding(0);
  M5.Lcd.setTextDatum(bottom_left);
  M5.Lcd.setTextColor(temp_units_colour, temp_units_bg);
  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  uint32_t humid_unit_x = humid_val_x + 5;
  uint32_t humid_unit_y = humid_val_y - 20;
  M5.Lcd.drawString("%", humid_unit_x, humid_unit_y);
  humid_unit_y += (20 + 3);
  M5.Lcd.drawString("RH", humid_unit_x, humid_unit_y);
}

/*
-----------------
  Display CO2 value
  SCD-30 has max CO2 range of 5,000 ppm
-----------------
*/
void display_co2_value(uint16_t co2, int32_t colour) {
  char txt[30] = "";
  int32_t xx = 0;
  int32_t yy = 0;

  if (display_state == dispaly_gauge) {
    M5.Lcd.setFont(&DSEG7_Modern_Regular_40);
    M5.Lcd.setTextPadding(170);
    M5.Lcd.setTextDatum(bottom_center);
    xx = lcd_width / 2;
    yy = lcd_height - 70;
  } else {
    M5.Lcd.setFont(&DSEG7_Modern_Bold_60);
    M5.Lcd.setTextPadding(240);
    M5.Lcd.setTextDatum(top_right);
    xx = co2_value_x;
    yy = co2_value_y;
  }

  if (co2 == 0) {
    // Don't display zero values
    M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    M5.Lcd.drawString("NAN", xx, yy);
  } else {
    M5.Lcd.setTextColor(colour, TFT_BLACK);
    sprintf(txt, "%d", co2);
    M5.Lcd.drawString(txt, xx, yy);
  }
}

/*
-----------------
  Display CO2 units
-----------------
*/
void display_co2_units() {
  M5.Lcd.setTextPadding(0);
  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  M5.Lcd.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);

  if (display_state == dispaly_gauge) {
    M5.Lcd.setTextDatum(bottom_center);
    M5.Lcd.drawString("CO2 ppm", lcd_width / 2, lcd_height - 30);
  } else {
    M5.Lcd.setTextDatum(top_right);
    M5.Lcd.drawString("CO2", co2_units_x, co2_units_y);
    M5.Lcd.drawString("ppm", co2_units_x, co2_units_y + 30);
  }
}

/*
-----------------
  Display CO2 effect on people
-----------------
*/
void display_co2_effect(const char* effect, int32_t colour) {
  M5.Lcd.setFont(&fonts::FreeSans18pt7b);
  M5.Lcd.setTextDatum(effect_align);
  M5.Lcd.setTextPadding(M5.Lcd.width() - 40);
  M5.Lcd.setTextColor(colour, TFT_BLACK);
  M5.Lcd.drawString(effect, effect_txt_x, effect_txt_y);
}

/*
-----------------
  Set the Neopixel RGB LED brightness in % and colour
-----------------
*/
void set_rgb_led(uint8_t brightness_pc, uint32_t colour) {
  FastLED.setBrightness((brightness_pc * 255) / 100);
  // M5 Core2 base has x10 LEDs around the base

  // if (brightness <= led_brightness_pc_low) {
  //   fill_solid(leds, LED_COUNT, CRGB::Black);
  //   leds[0] = colour;  // Top right LED
  //   leds[9] = colour;  // Top left LED
  // } else
  fill_solid(leds, LED_COUNT, colour);
  FastLED.show();
}

/*
-----------------
  Display lux sensor value
-----------------
*/
#define lux_lev_1  3.0
#define lux_lev_2  40.0
#define lux_lev_3  100.0

void display_lux_val() {
  char lux_str[40] = "";
  int32_t x = 10;
  int32_t y = 50;

  // Display lux and brightness on LCD
  static bool colour_toggle = false;
  colour_toggle = !colour_toggle;
  // const int color_true = M5.Lcd.color565(255, 255, 0);   // Yelllow
  // const int color_false = M5.Lcd.color565(255, 145, 0);  // Orange
  const int color_true = TFT_WHITE;
  const int color_false = TFT_GREEN;

  M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
  M5.Lcd.setTextDatum(top_left);
  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  M5.Lcd.setTextPadding(110);

  M5.Lcd.drawString("Lux:", x, y);
  y += 27;
  M5.Lcd.drawString("LED brightness:", x, y);
  y += 27;
  M5.Lcd.drawString("LCD brightness:", x, y);
  y += 50;
  M5.Lcd.setTextColor(TFT_WHITE, TFT_DARKGRAY);
  M5.Lcd.drawString("Lux levels", x, y);
  y += 27;
  sprintf(lux_str, "%.0f--%.0f--%.0f", lux_lev_1, lux_lev_2, lux_lev_3);
  M5.Lcd.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  M5.Lcd.drawString(lux_str, x, y);

  y = 50;
  x = M5.Lcd.width();
  M5.Lcd.setTextDatum(top_right);
  M5.Lcd.setTextColor(colour_toggle ? color_true : color_false, TFT_BLACK);

  sprintf(lux_str, "%.1f", lux_float);
  M5.Lcd.drawString(lux_str, x, y);

  y += 27;
  sprintf(lux_str, "%3d%%", led_brightness_pc);
  M5.Lcd.drawString(lux_str, x, y);

  y += 27;
  sprintf(lux_str, "%3d%%", lcd_brightness_pc);
  M5.Lcd.drawString(lux_str, x, y);
}

/*
-----------------
  Read VEML7700 lux sensor and set LED and LCD brightness
-----------------
*/
void read_lux_sensor(void) {
  lux.getALSLux(lux_float);
  // Auto ranging lux, can take up to 5 or 6 seconds in very low light
  // lux.getAutoALSLux(lux_float);

  if (lux_float < lux_lev_1) {
    led_brightness_pc = 5;
    lcd_brightness_pc = 30;
  } else if (lux_float < lux_lev_2) {
    led_brightness_pc = 50;
    lcd_brightness_pc = 50;
  } else if (lux_float < lux_lev_3) {
    led_brightness_pc = 70;
    lcd_brightness_pc = 70;
  } else {
    led_brightness_pc = 100;
    lcd_brightness_pc = 100;
  }

  if (debug_mode) Serial.printf("Lux=%.3f, Brightness: LED=%d%%, LCD=%d%%\n\n", lux_float, led_brightness_pc, lcd_brightness_pc);

  // Set RGB LED brightness
  FastLED.setBrightness((led_brightness_pc * 255) / 100);
  FastLED.show();

  // Set M5 Stack Core2 LCD brightness
  M5.Lcd.setBrightness((lcd_brightness_pc * 255) / 100);  // Core2 LCD backlight brightness
}

/*
-----------------
  Obtain LED and LCD text colour from CO2 level
-----------------
*/
void co2_to_colour(uint16_t co2, uint32_t& led_colour, int32_t& lcd_colour, char* txt) {
  // Set the RGB LEDs based on CO2 level from https://www.kane.co.uk/knowledge-centre/what-are-safe-levels-of-co-and-co2-in-rooms
  // Assume indoor CO2 levels
  switch (co2) {
    case 1 ... 1000:
      // Should never be below 400, but wind currents on the sensor can make this happen
      // So just detect anything above zero, zero is normally a bad connection with the sensor
      // CO2 400 - 1000: OK good air exchange
      if (txt != nullptr) strcpy(txt, "Good air quality");
      led_colour = CRGB::Green;
      lcd_colour = TFT_GREEN;
      break;

    case 1001 ... 2000:
      // CO2 1000 - 2000: Drowsy and poor air
      if (txt != nullptr) strcpy(txt, "Drowsy, poor air");
      led_colour = CRGB::Yellow;
      lcd_colour = TFT_YELLOW;
      break;

    // 5,000 is max value reported by Sensirion SCD-30 (I've actually seen 6,250 but that's not as per the datasheet)
    case 2001 ... 5000:
      //    Headaches, sleepiness and stagnant, stale, stuffy air.
      //    Poor concentration, loss of attention, increased heart rate and slight nausea may be present.
      if (txt != nullptr) strcpy(txt, "Headache, sleepy");
      led_colour = CRGB::Red;
      lcd_colour = TFT_RED;
      break;

    // 10,000 is max value reported by Sensirion SCD-30
    case 5001 ... 65535:
      //    Workplace 8-hr exposure limit
      if (txt != nullptr) strcpy(txt, "8-hr exposure limit");
      led_colour = CRGB::Red;
      lcd_colour = TFT_RED;
      break;

    case 0:
    default:
      // PINK indicates an error
      if (txt != nullptr) strcpy(txt, "Bad CO2 read :(");
      led_colour = CRGB::Pink;
      lcd_colour = TFT_MAGENTA;
      break;
  }
}

/*
-----------------
  Wrapper function for calling from TickTwo scheduler which cannot accept function parameters
-----------------
*/
void disp_batt_wrapper(void) {
  disp_batt_symbol(batt_spr_x, batt_spr_y, false);
}

/*
-----------------
  Display Core2 battery symbol, % charge, and voltage
  batt_x - X coordinate of battery sprite
  batt_y - Y coordinate of battery sprite
  batt_volt - LiPo voltage. Range is 3.7V to 4.2V
  disp_volts - boolean, display voltage when true
-----------------
*/
void disp_batt_symbol(uint16_t batt_x, uint16_t batt_y, bool disp_volts) {
  float batt_volt = 0.0;
  int32_t batt_percent = 0;
  int32_t batt_fill_length = 0;
  uint16_t fill_colour = TFT_DARKGRAY;
  uint16_t outline_colour = TFT_LIGHTGRAY;
  uint16_t txt_colour = TFT_LIGHTGRAY;
  const uint16_t erase_fill_colour = TFT_BACKGND;

  // Read battery voltage
  batt_volt = M5.Power.Axp192.getBatteryVoltage();
  batt_percent = M5.Power.getBatteryLevel();
  batt_fill_length = (batt_percent * batt_rect_width) / 100;
  if (debug_mode) Serial.printf("BatVoltage= %.1f, BattLevel=%d\n", batt_volt, batt_percent);

  // Clear the old values
  batt_sprite.fillSprite(erase_fill_colour);  // Clear the battery icon sprite

  // Display battery percentage
  char txt[20] = "";
  batt_sprite.setTextColor(txt_colour, TFT_BLACK);
  batt_sprite.setFont(&FreeSans12pt7b);
  batt_sprite.setTextDatum(middle_right);
  batt_sprite.setTextPadding(64);

  uint16_t spr_x = 67;               // X-axis offset of battery icon and voltage text in sprite
  uint16_t spr_y = batt_spr_ht / 2;  // Y-axis offset of battery icon and voltage text in sprite

  // If no battery is present, and running from USB power
  if (batt_volt < 2.0) {
    // batt_sprite.drawString("USB Pwr", spr_x + 30, spr_y);
    // batt_sprite.pushSprite(batt_x, batt_y);
    return;
  }

  sprintf(txt, "%3d%%", batt_percent);
  batt_sprite.drawString(txt, spr_x, spr_y);

  if (disp_volts) {
    spr_x += 73;
    sprintf(txt, "%.2fV", batt_volt);
    batt_sprite.drawString(txt, spr_x, spr_y);
  }

  if (batt_percent < 20)
    fill_colour = TFT_RED;
  else if (batt_percent >= 20 && batt_percent < 50)
    fill_colour = TFT_ORANGE;
  else if (batt_percent >= 50)
    fill_colour = TFT_DARKGREEN;

  // Draw the battery symbol outline
  spr_x += 10;
  spr_y = ((batt_spr_ht - batt_rect_height) / 2) - 1;
  batt_sprite.drawRect(spr_x, spr_y, batt_rect_width, batt_rect_height, outline_colour);

  // Draw the button on top of the battery - intentional gap from the main battery rectangle
  batt_sprite.fillRect(spr_x + batt_rect_width + 1, (batt_spr_ht - batt_button_ht) / 2, batt_button_wdth, batt_button_ht, outline_colour);

  // Erase the old battery level
  batt_sprite.fillRect(spr_x + 1, spr_y + 1, batt_rect_width - 2, batt_rect_height - 2, erase_fill_colour);

  // Draw the current battery level
  batt_sprite.fillRect(spr_x + 1, spr_y + 1, batt_fill_length - 2, batt_rect_height - 2, fill_colour);

  // Draw lighning bolt symbol
  if (M5.Power.isCharging()) {
    uint16_t cntre_x = spr_x + (batt_rect_width / 2);
    uint16_t cntre_y = spr_y + (batt_rect_height / 2) - 1;
    batt_sprite.fillTriangle(cntre_x - 15, cntre_y - 2, cntre_x, cntre_y, cntre_x + 2, cntre_y + 6, TFT_ORANGE);
    batt_sprite.fillTriangle(cntre_x + 15, cntre_y + 2, cntre_x, cntre_y, cntre_x - 2, cntre_y - 6, TFT_ORANGE);
  }

  // Display the sprite
  batt_sprite.pushSprite(batt_x, batt_y);
}

/*
-----------------
  Connect ESP32 to WiFi
  max_tries - how many 500ms delays to wait for WiFi to connect. Default is 15 (in function declaration)
  connected - true if WiFi is connected
-----------------
*/
bool connect_wifi(uint8_t max_tries) {
  bool connected = false;
  char msg[20] = "";
  uint8_t tries_count = 0;

  // Start WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);

  // Display WiFi starting message
  M5.Lcd.setTextDatum(top_center);
  M5.Lcd.setFont(&fonts::FreeSans18pt7b);
  M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Lcd.drawString("Starting WiFi", M5.Lcd.width() / 2, 80);

  // Set location where "connecting..." dots will appear
  M5.Lcd.setCursor(110, 120);

  do {
    delay(500);
    M5.Lcd.print(".");
    tries_count++;
    connected = (WiFi.status() == WL_CONNECTED);
  } while (!connected && (tries_count < max_tries));

  // M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  if (connected)
    strcpy(msg, "Connected!");
  else
    // WiFi not connected
    strcpy(msg, "Not connected");
  M5.Lcd.setTextPadding(280);
  M5.Lcd.drawString(msg, M5.Lcd.width() / 2, 80);

  return connected;
}

/*
-----------------
  Configures the timezone, queries NTP for data and time, and sets RTC chip with the result
-----------------
*/
void sync_rtc_to_ntp(void) {
  char time_txt[80] = "";
  struct tm* timeinfo;

  M5.Lcd.setTextDatum(top_center);
  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.setTextPadding(M5.Lcd.textWidth("Syncing to NTP time"));
  M5.Lcd.drawString("Syncing to NTP time", ntp_msg_x, ntp_msg_y);

  // See example: https://github.com/m5stack/M5Unified/blob/master/examples/Basic/Rtc/Rtc.ino
  // See pull request: https://github.com/m5stack/M5Unified/pull/20
  //
  // Also try "0.au.pool.ntp.org"
  // configTzTime("ACST-9:30ACDT,M10.1.0,M4.1.0/3", "pool.ntp.org");  // ACST = Australian Central Standard Time (timezone)
  configTzTime("ACST-9:30ACDT,M10.1.0,M4.1.0/3", "0.au.pool.ntp.org");  // ACST = Australian Central Standard Time (timezone)

  // Epoch time variable, i.e. number of seconds since 1-1-1970 UTC
  time_t t;

  // Wait until NTP sync has finished. Sets ESP32 internal RTC
  Serial.println("Syncing with NTP time");
  // Set location where "connecting..." dots will appear
  M5.Lcd.setCursor(110, 160);
  M5.Lcd.setFont(&fonts::FreeSans18pt7b);  // Set larger font to display probress dots "....""

  while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET) {
    Serial.print('.');
    M5.Lcd.print(".");
    delay(1000);
  }
  Serial.println("\r\n NTP Connected.");
  t = time(nullptr) + 1;  // Advance one second
  while (t > time(nullptr))
    ;                            /// Synchronization in seconds
  timeinfo = localtime(&t);      // Convert epoch time to a "tm" structure
  M5.Rtc.setDateTime(timeinfo);  // Writes the date and time to the Core2's external RTC chip

#if (CORE_DEBUG_LEVEL == 4)
  strftime(time_txt, 80, "%A %e-%m-%Y, %H:%M:%S", timeinfo);
  log_d("NTP time is: %s", time_txt);

  log_d("timeinfo: tm_mday=%d tm_mon=%d tm_wday=%d tm_year=%d",
        (uint8_t)timeinfo->tm_hour, (uint8_t)(timeinfo->tm_mon + 1), (uint8_t)timeinfo->tm_wday, (uint16_t)(timeinfo->tm_year + 1900));
  log_d("timeinfo: tm_hour=%d tm_min=%d tm_sec=%d\n", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
#endif

  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.drawString("RTC synced to NTP", ntp_msg_x, ntp_msg_y);
}

/*
-----------------
  Get time from M5 Stack Core2 RTC and display on LCD
-----------------
*/
void display_time(void) {
#define str_size 50
  char time_str[str_size];

  // Read time from real-time clock
  auto dt = M5.Rtc.getDateTime();
  RTCtime.seconds = dt.time.seconds;  // Pass the time to the global var RTCtime
  RTCtime.minutes = dt.time.minutes;  // Pass the time to the global var RTCtime

  // Display date
  sprintf(time_str, "%02d-%02d-%04d", dt.date.date, dt.date.month, dt.date.year);
  M5.Lcd.setTextDatum(time_align);
  M5.Lcd.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  M5.Lcd.setTextPadding(96);
  // M5.Lcd.drawString(time_str, date_txt_x, date_txt_y);

  // Display time
  sprintf(time_str, "%02d:%02d:%02d", dt.time.hours, dt.time.minutes, dt.time.seconds);
  M5.Lcd.drawString(time_str, time_txt_x, time_txt_y);
}

/*
-----------------
  Perform a FRC calibration on either a SCD-30 or SCD-41 CO2 sensor
  Needs sensor to be placed into a known CO2 concentration for 3 mins before calibration
-----------------
*/
void scd_x_forced_cal(uint16_t target_co2) {
  uint32_t start_time = millis();
  uint32_t duration = 0;
  uint32_t last_disp_time = 0;
  int32_t x = 0;
  int32_t y = 5;
  char txt[40] = "";

  M5.Lcd.clear();
  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  M5.Lcd.setTextDatum(top_left);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setTextPadding(M5.Lcd.width());

#if defined SENSOR_IS_SCD41 || defined SENSOR_IS_SCD30
  M5.Lcd.drawString("Really CALIBRATE CO2?", x, y);
  M5.Lcd.drawString("BtnB to CANCEL!", x, y + 30);
  M5.Lcd.drawString("BtnA to Continue", x, y + 60);
#elif defined SENSOR_IS_SGP30
  M5.Lcd.drawString("Can't calibrate", x, y);
  sprintf(txt, "%s CO2 sensor", co2_sensor_type_str);
  M5.Lcd.drawString(txt, x, y + 30);
  delay(3000);
  M5.Lcd.clear();
  return;
#endif

  bool btnA = false;
  bool btnB = false;
  do {
    M5.update();
    delay(1);
    btnA = M5.BtnA.wasClicked();
    btnB = M5.BtnB.wasClicked();
  } while (!btnA && !btnB);

  if (btnB) {
    M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    M5.Lcd.drawString("Calibration cancelled", x, y + 90);
    delay(1000);
    M5.Lcd.clear();
    return;
  }

  x = M5.Lcd.width() / 2;
  M5.Lcd.clear();
  M5.Lcd.setFont(&fonts::FreeSans18pt7b);
  M5.Lcd.setTextDatum(top_center);
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Lcd.drawString("Calibrate sensor", x, y);

  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  M5.Lcd.setTextDatum(top_left);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setTextPadding(M5.Lcd.width());
  x = 0;
  y = 40;

#if defined SENSOR_IS_SCD41
  M5.Lcd.drawString("Wait 10s for factory reset", x, y);
  Serial.println("Wait 10s for factory reset");
  co2.factory_reset();
#endif

  Serial.printf("Put in CO2=%d ppm for 3 minutes, or press BtnA when ready.\n", target_co2);
  sprintf(txt, "Put in CO2=%d ppm 3 mins", target_co2);
  M5.Lcd.drawString(txt, x, y);
  y += 30;
  // M5.Lcd.setTextPadding(280);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.drawString("BtnB to cancel calibration", x, y + 60);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

  do {
    if (co2.get_co2()) {
      Serial.printf("Pre-cal CO2=%d ppm\n", co2.co2_level);
      sprintf(txt, "Pre-cal CO2=%d ppm", co2.co2_level);
      M5.Lcd.drawString(txt, x, y);
    }

    duration = (millis() - start_time) / 1000;
    if (duration > last_disp_time) {
      last_disp_time = duration;
      Serial.printf("Wait for = %d sec\n", (3 * 60) - duration);
      sprintf(txt, "Wait for %3d sec", (3 * 60) - duration);
      M5.Lcd.drawString(txt, x, y + 30);
    }
    M5.update();
    if (M5.BtnB.wasClicked()) {
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
      M5.Lcd.drawString("Calibration cancelled", x, y + 90);
      delay(1000);
      M5.Lcd.clear();
      return;
    }
  } while (!M5.BtnA.wasClicked() && duration < (3 * 60));

  Serial.printf("Calibrating to %d ppm\n", target_co2);
  y += 60;
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.drawString("Calibrating NOW!", x, y);
  y += 30;

  int16_t correction = 0;
  correction = co2.calibrate(target_co2);
  delay(400);  // Required by Sensirion SCD-41 datasheet
  if (correction == 0) {
    Serial.println("Error trying to execute calibration");
    M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    M5.Lcd.drawString("Error during calibration", x, y);
  } else {
    Serial.printf("FRC calibration factor = %d\n", correction);
    sprintf(txt, "Cal correction %d ppm", correction);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.drawString(txt, x, y);
  }

  start_time = millis();
  do {
    delay(200);
  } while (!co2.get_co2() && (millis() < start_time + 5000));

  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  Serial.printf("Post-cal CO2 = %d\n", co2.co2_level);
  y += 30;
  sprintf(txt, "Post-cal CO2 = %d ppm", co2.co2_level);
  M5.Lcd.drawString(txt, x, y);
  y += 30;
  M5.Lcd.drawString("Press BtnA to exit", x, y);

  do {
    M5.update();
    delay(1);
  } while (!M5.BtnA.wasClicked());
  M5.Lcd.clear();
}

/*
-----------------
  Set SCD-30 SO2 sensor settings
  Note: Temperature offset can only be positive. Also it takes some minutes for it to take effect.
  This function persists SCD-30 settings to EEPROM. Note this EEPROM only lasts 2,000 write cycles.
-----------------
*/
void scd_x_settings(float temp_offs, uint16_t alt, bool ASC) {
  float _temp_offs = 0.0;
  uint16_t _alt = 0;
  bool _ASC = false;
  bool cmd_ok;

  Serial.printf("\n********* Start of function %s() *********\n", __func__);

  cmd_ok = co2.set_co2_device_settings(temp_offs, alt, ASC);

  if (!cmd_ok)
    Serial.printf("Error setting %s: temperature offset=%.2f ??C, altitude=%d, ASC calibration %s\n",
                  co2_sensor_type_str, temp_offs, alt, ASC == 1 ? "ON" : "OFF");
  else {
    co2.get_co2_device_settings(_temp_offs, _alt, _ASC);
    Serial.printf("%s Settings OK: temperature offset=%.2f ??C, altitude=%d, ASC calibration %s\n",
                  co2_sensor_type_str, _temp_offs, _alt, _ASC == 1 ? "ON" : "OFF");
  }
  Serial.println();
  Serial.printf("********* End of function %s() *********\n", __func__);
}

/*
-----------------
  Search for CO2 sensor and display startup message on LCD
-----------------
*/
void start_co2_sensor(bool start_co2) {
  int32_t x = M5.Lcd.width() / 2;
  int32_t y = 5;
  char txt[50] = "";
  bool settings_not_applicable = false;
#define co2_info_y     95
#define co2_info_y_inc 27
#define rect_y         84
#define max_retries    5
#define dot_height     20
#define dot_width      20
#define dot_gap        5
#define dot_x_start    85

#if defined SENSOR_IS_SGP30
  settings_not_applicable = true;
#endif

  Serial.printf("\n********* Start of function %s() *********\n", __func__);

  // Display product title
  M5.Lcd.clear();
  M5.Lcd.setFont(&fonts::FreeSansBold24pt7b);
  M5.Lcd.setTextDatum(top_center);
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Lcd.drawString("CO2 Monitor", x, y);

  // Display labels
  y = 57;
  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  sprintf(txt, "%s settings", co2_sensor_type_str);
  M5.Lcd.drawString(txt, x, y);
  M5.Lcd.drawRect(10, rect_y, M5.Lcd.width() - 20, 93, TFT_DARKGRAY);

  x = 20;
  y = co2_info_y;
  M5.Lcd.setTextDatum(top_left);

  // Attempt to connect to Sensirion CO2 sensor
  bool sensor_found = false;
  if (start_co2) {
    uint16_t retries = 0;
    uint16_t dot_x = 0;
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Lcd.drawString("Searching for CO2 sensor", x, y);

    do {
      sensor_found = co2.begin();
      Serial.printf("%s sensor present: %s\n", co2_sensor_type_str, sensor_found ? "Yes" : "No");
      M5.Lcd.fillRoundRect(dot_x_start + dot_x, y + 45, dot_width, dot_height, 3, TFT_LIGHTGREY);
      dot_x += (dot_width + dot_gap);
      // delay(100);
    } while (!sensor_found && retries++ < max_retries);
  } else
    sensor_found = true;

  // Clear inside the rectangle
  M5.Lcd.fillRect(11, rect_y + 1, M5.Lcd.width() - 22, 91, TFT_BLACK);

  // If sensor not found, enter simulation mode
  co2.simulate_co2 = !sensor_found;

  y = co2_info_y;
  if (co2.simulate_co2) {
    Serial.printf("Simulated %s CO2 sensor\n", co2_sensor_type_str);
    M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    M5.Lcd.drawString("No CO2 sensor detected", x, y);
    y += co2_info_y_inc;
    M5.Lcd.drawString("Switched to", x, y);
    y += co2_info_y_inc;
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Lcd.setFont(&fonts::FreeSansBold12pt7b);
    M5.Lcd.drawString("SIMULATION mode.", x, y);
    M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  } else {
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.drawString("Self Calibration:", x, y);
    y += co2_info_y_inc;
    M5.Lcd.drawString("Altitude:", x, y);
    y += co2_info_y_inc;
    M5.Lcd.drawString("Temp. offset:", x, y);

    // Display values
    y = co2_info_y;
    x = 210;
    bool self_cal;
    uint16_t alt;
    // uint16_t error;
    float temp_offset;

    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);

    co2.get_co2_device_settings(temp_offset, alt, self_cal);

    // Display SCD-30 or SCD-41 Automatic Self-Calibration (ASC) setting
    if (co2.simulate_co2) {
      strcpy(txt, "sim");  // Self cal
    } else if (settings_not_applicable) {
      strcpy(txt, "N/A");
      Serial.printf("No Automatic Self Calibration for %s CO2 sensor\n", co2_sensor_type_str);
    } else {
      sprintf(txt, "%s", self_cal == 1 ? "On" : "Off");  // Self cal
      Serial.printf("Sensirion %s Auto Self Cal (ASC) is %s\n", co2_sensor_type_str, self_cal ? "ON" : "OFF");
    }
    M5.Lcd.drawString(txt, x, y);

    // Display SCD-30 or SCD-41 Altitude setting
    y += co2_info_y_inc;
    if (co2.simulate_co2)
      strcpy(txt, "sim");  // Self cal
    else if (settings_not_applicable) {
      strcpy(txt, "N/A");
      Serial.printf("No altitude setting for %s CO2 sensor\n", co2_sensor_type_str);
    } else {
      sprintf(txt, "%d m", alt);
      Serial.printf("Sensirion %s altitude is %d m (AMSL)\n", co2_sensor_type_str, alt);
    }
    M5.Lcd.drawString(txt, x, y);

    // Display SCD-30 or SCD-41 temperature offset setting
    y += co2_info_y_inc;
    if (co2.simulate_co2) {
      strcpy(txt, "sim");            // Self cal
      M5.Lcd.drawString(txt, x, y);  // Temperature offset
    } else if (settings_not_applicable) {
      strcpy(txt, "N/A");
      Serial.printf("No temperature offset for %s CO2 sensor\n", co2_sensor_type_str);
      M5.Lcd.drawString(txt, x, y);  // Temperature offset
    } else {
      Serial.printf("Sensirion %s temperature offset is %.1f??C\n", co2_sensor_type_str, temp_offset);
      sprintf(txt, "%1.1f   C", temp_offset);
      M5.Lcd.drawString(txt, x, y);               // Temperature offset
      M5.Lcd.drawCircle(x + 48, y, 4, TFT_CYAN);  // Degree symbol
    }
  }

  M5.Lcd.setTextDatum(top_center);
  x = M5.Lcd.width() / 2;
  y += 40;
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Lcd.drawString("Tap screen to continue", x, y);

  // Display software version
  M5.Lcd.setTextDatum(bottom_right);
  M5.Lcd.setTextColor(TFT_DARKGRAY, TFT_BLACK);
  M5.Lcd.setFont(&fonts::FreeSans9pt7b);
  M5.Lcd.drawString("Version: " sw_version, M5.Lcd.width(), M5.Lcd.height());

  // Wait up to 20s for user to press touch BtnA
  auto td = M5.Touch.getDetail();
  uint32_t timeout = millis();
  do {
    M5.update();
    td = M5.Touch.getDetail();  // Read the buttons
    delay(10);
  } while (!td.wasPressed() && millis() < (timeout + 20000));

  // Clear out previous button press so don't go to next display
  do {
    M5.update();
    td = M5.Touch.getDetail();  // Read the buttons
    delay(10);
  } while (td.wasPressed());

  Serial.printf("********* End of function %s() *********\n", __func__);
}

void sim_sensor_wrapper(void) {
  co2.sim_sensor();
}

/*
-----------------
  Draw a triangular pointer using rotated sprite on semi-circle gauge
  Gauge value is passed in as a percentage:
  0% = 250??
  50% = 0??
  100% = 110??
  angle span = 220??
-----------------
*/
void draw_circular_gauge_pointer(uint16_t percent) {
  uint16_t angle = 0;
  static uint16_t last_angle = 0;

  // Erase the old pointer
  gauge_pointer.clear(TFT_BLACK);
  gauge_pointer.pushRotateZoom(arc_x, arc_y, last_angle, 1, 1);

  // Draw the new pointer sprite
  gauge_pointer.fillTriangle(0, 20, gauge_ptr_spr_w / 2, 0, gauge_ptr_spr_w, 20, TFT_WHITE);

  // calculate the angle based on value in percent
  if (percent > 100) percent = 100;
  angle = 250 + ((220 * percent) / 100);

  // if (angle >= 360) angle -= 360;

  // Display the pointer sprite
  gauge_pointer.pushRotateZoom(arc_x, arc_y, angle, 1, 1);

  // Remember the current angle to erase on next update
  last_angle = angle;
}

/*
-----------------
  Draw a semi circular gauge scale for CO2
  fillArc function defines 0?? at display EAST, 180?? at WEST, 270?? at NORTH
  0 = 160??
  max = 20??
  angle span = 220??

  GREEN     CO2 <= 1000
  YELLOW    CO2 1001-2000
  RED       CO2 2001-5000
-----------------
*/
void draw_circular_gauge_scale(void) {
  uint16_t start_angle = 160;
  uint16_t end_angle = 0;

  // Start = 160??, End = 160 + (2/5 * 220) = 248??
  end_angle = start_angle + (2000 * 220) / 5000;
  M5.Lcd.fillArc(arc_x, arc_y, rad_1, rad_2, start_angle, end_angle, TFT_DARKGREEN);
  M5.Lcd.drawArc(arc_x, arc_y, rad_1 - 1, rad_2 + 1, start_angle, end_angle, TFT_DARKGREY);

  // Start = 248??, End = 248 + (2/5 * 220) = 336??
  start_angle = end_angle;
  end_angle = start_angle + (2000 * 220) / 5000;
  M5.Lcd.fillArc(arc_x, arc_y, rad_1, rad_2, start_angle, end_angle, TFT_ORANGE);
  M5.Lcd.drawArc(arc_x, arc_y, rad_1 - 1, rad_2 + 1, start_angle, end_angle, TFT_DARKGREY);

  // Start = 336, End = 336 + (1/5 * 220) = 20?? (i.e. 380??-360??)
  start_angle = end_angle;
  end_angle = start_angle + (1000 * 220) / 5000;
  M5.Lcd.fillArc(arc_x, arc_y, rad_1, rad_2, start_angle, end_angle, TFT_RED);
  M5.Lcd.drawArc(arc_x, arc_y, rad_1 - 1, rad_2 + 1, start_angle, end_angle, TFT_DARKGREY);

  // Draw scale MINOR tick marks
  uint16_t value = 0;
  uint16_t angle = 0;
  gauge_ticks.clear(TFT_BLACK);
  gauge_ticks.drawRect(0, gauge_tick_spr_h - 3, 2, 3, TFT_LIGHTGREY);
  for (value = 0; value <= 2500; value += 5) {
    if (value % 125 == 0) {
      angle = 250 + ((220 * value) / 2500);
      gauge_ticks.pushRotateZoom(arc_x, arc_y, angle, 1, 1);
    }
  }

  // Draw scale MAJOR tick marks
  gauge_ticks.drawRect(0, gauge_tick_spr_h - 8, 2, 8, TFT_LIGHTGREY);
  for (value = 0; value <= 2500; value += 50) {
    if (value % 250 == 0) {
      angle = 250 + ((220 * value) / 2500);
      gauge_ticks.pushRotateZoom(arc_x, arc_y, angle, 1, 1);
    }
  }

  M5.Lcd.setTextDatum(bottom_centre);
  M5.Lcd.setFont(&fonts::FreeSans9pt7b);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.drawString("0", 40, lcd_height - 35);
  M5.Lcd.drawString("500", 50, 120);
  M5.Lcd.drawString("1000", 115, 55);
  M5.Lcd.drawString("1500", 205, 55);
  M5.Lcd.drawString("2000", lcd_width - 45, 120);
  M5.Lcd.drawString("2500", lcd_width - 40, lcd_height - 40);
}
