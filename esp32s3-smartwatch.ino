#include <Wire.h>
#include <Arduino.h>
#include "pin_config.h"
#include <lvgl.h>
#include <time.h>
#include <sys/time.h>

#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "lv_conf.h"
#include "ui.h"
#include "XPowersLib.h"

#include "HWCDC.h"
HWCDC USBSerial;

XPowersPMU power;

uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_disp_t *disp;
lv_color_t *disp_draw_buf;
lv_disp_draw_buf_t draw_buf;

// #define DIRECT_RENDER_MODE  // Disabled to reduce memory usage

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
  LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RESET /* RST */,
                                      0 /* rotation */, LCD_WIDTH, LCD_HEIGHT,
                                      22 /* col_offset1 */,
                                      0 /* row_offset1 */,
                                      0 /* col_offset2 */,
                                      0 /* row_offset2 */);


std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);

std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, TP_INT, Arduino_IIC_Touch_Interrupt));

void Arduino_IIC_Touch_Interrupt(void) {
  FT3168->IIC_Interrupt_Flag = true;
}

#if LV_USE_LOG != 0
void my_print(lv_log_level_t level, const char *buf) {
  LV_UNUSED(level);
  USBSerial.println(buf);
  USBSerial.flush();
}
#endif

uint32_t millis_cb(void) {
  return millis();
}

/* LVGL calls it when a rendered image needs to copied to the display*/
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
#ifndef DIRECT_RENDER_MODE
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);

  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
#endif  // #ifndef DIRECT_RENDER_MODE

  /*Call it to tell LVGL you are ready*/
  lv_disp_flush_ready(disp_drv);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  static int32_t last_x = 0;
  static int32_t last_y = 0;

  // Check if there's a touch interrupt
  if (FT3168->IIC_Interrupt_Flag == true) {
    FT3168->IIC_Interrupt_Flag = false;

    // Get number of touch points
    int32_t touch_count = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

    if (touch_count > 0) {
      // Touch detected - read coordinates
      last_x = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
      last_y = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

      data->state = LV_INDEV_STATE_PR;
      data->point.x = last_x;
      data->point.y = last_y;

      USBSerial.print("Touch PRESSED x: ");
      USBSerial.print(last_x);
      USBSerial.print(" y: ");
      USBSerial.println(last_y);
    } else {
      // No touch - released
      data->state = LV_INDEV_STATE_REL;
      data->point.x = last_x;
      data->point.y = last_y;
      USBSerial.println("Touch RELEASED");
    }
  } else {
    // No interrupt - return last known state (released)
    data->state = LV_INDEV_STATE_REL;
    data->point.x = last_x;
    data->point.y = last_y;
  }
}

void my_disp_rounder(lv_disp_drv_t *disp_drv, lv_area_t *area) {
  uint16_t x1 = area->x1;
  uint16_t x2 = area->x2;

  uint16_t y1 = area->y1;
  uint16_t y2 = area->y2;

  // round the start of coordinate down to the nearest 2M number
  area->x1 = (x1 >> 1) << 1;
  area->y1 = (y1 >> 1) << 1;
  // round the end of coordinate up to the nearest 2N+1 number
  area->x2 = ((x2 >> 1) << 1) + 1;
  area->y2 = ((y2 >> 1) << 1) + 1;
}

// Update time labels on the UI
void update_time_labels(lv_timer_t * timer) {
  struct tm timeinfo;
  time_t now = time(NULL);
  localtime_r(&now, &timeinfo);

  char time_str[3];
  const char* days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
  char date_str[6];

  // Update Hour
  snprintf(time_str, sizeof(time_str), "%02d", timeinfo.tm_hour);
  lv_label_set_text(ui_Hour, time_str);

  // Update Minutes
  snprintf(time_str, sizeof(time_str), "%02d", timeinfo.tm_min);
  lv_label_set_text(ui_Minutes, time_str);

  // Update Seconds
  snprintf(time_str, sizeof(time_str), "%02d", timeinfo.tm_sec);
  lv_label_set_text(ui_Seconds, time_str);

  // Update Day of week
  lv_label_set_text(ui_Day, days[timeinfo.tm_wday]);

  // Update Date (DD/MM format)
  snprintf(date_str, sizeof(date_str), "%02d/%02d", timeinfo.tm_mday, timeinfo.tm_mon + 1);
  lv_label_set_text(ui_Date, date_str);
}

// Update battery status on the UI
void update_battery_status(lv_timer_t * timer) {
  if (power.isBatteryConnect()) {
    // Get battery percentage
    int battery_percent = power.getBatteryPercent();

    // Update battery bar value (0-100)
    lv_bar_set_value(ui_BatteryBar, battery_percent, LV_ANIM_ON);

    // Update battery label text
    char battery_str[6];
    snprintf(battery_str, sizeof(battery_str), "%d%%", battery_percent);
    lv_label_set_text(ui_BatteryLabel, battery_str);

    // Change color based on battery level
    if (battery_percent > 50) {
      lv_obj_set_style_text_color(ui_BatteryLabel, lv_color_hex(0x00F943), LV_PART_MAIN | LV_STATE_DEFAULT); // Green
    } else if (battery_percent > 20) {
      lv_obj_set_style_text_color(ui_BatteryLabel, lv_color_hex(0xFFA500), LV_PART_MAIN | LV_STATE_DEFAULT); // Orange
    } else {
      lv_obj_set_style_text_color(ui_BatteryLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT); // Red
    }
  } else {
    lv_label_set_text(ui_BatteryLabel, "N/A");
    lv_bar_set_value(ui_BatteryBar, 0, LV_ANIM_OFF);
  }
}

void setup() {
#ifdef DEV_DEVICE_INIT
  DEV_DEVICE_INIT();
#endif

  USBSerial.begin(115200);
  // USBSerial.setDebugOutput(true);
  // while(!USBSerial);
  USBSerial.println("Arduino_GFX LVGL_Arduino_v9 example ");
  String LVGL_Arduino = String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  USBSerial.println(LVGL_Arduino);

  // Init Display
  if (!gfx->begin()) {
    USBSerial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(RGB565_BLACK);

  Wire.begin(IIC_SDA, IIC_SCL);

  while (FT3168->begin() == false) {
    USBSerial.println("FT3168 initialization fail");
    delay(2000);
  }
  USBSerial.println("FT3168 initialization successfully");

  FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                 FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);

  // Initialize AXP2101 Power Management
  if (!power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    USBSerial.println("Failed to initialize AXP2101 power management!");
  } else {
    USBSerial.println("AXP2101 initialized successfully");

    // Configure power management
    power.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    power.clearIrqStatus();

    // Enable battery detection and voltage measurement
    power.enableBattDetection();
    power.enableVbusVoltageMeasure();
    power.enableBattVoltageMeasure();
    power.enableSystemVoltageMeasure();
  }

  lv_init();

  /* register print function for debugging */
#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  screenWidth = gfx->width();
  screenHeight = gfx->height();

#ifdef DIRECT_RENDER_MODE
  bufSize = screenWidth * screenHeight;
#else
  bufSize = screenWidth * 50;
#endif

#ifdef ESP32
#if defined(DIRECT_RENDER_MODE) && (defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL))
  disp_draw_buf = (lv_color_t *)gfx->getFramebuffer();
#else   // !(defined(DIRECT_RENDER_MODE) && (defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL)))
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf) {
    // remove MALLOC_CAP_INTERNAL flag try again
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
  }
#endif  // !(defined(DIRECT_RENDER_MODE) && (defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL)))
#else   // !ESP32
  USBSerial.println("LVGL disp_draw_buf heap_caps_malloc failed! malloc again...");
  disp_draw_buf = (lv_color_t *)malloc(bufSize * 2);
#endif  // !ESP32
  if (!disp_draw_buf) {
    USBSerial.println("LVGL disp_draw_buf allocate failed!");
  } else {
    // Initialize display buffer
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, bufSize);

    // Initialize display driver
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.rounder_cb = my_disp_rounder;
    disp = lv_disp_drv_register(&disp_drv);

    // Initialize input device driver
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // Initialize SquareLine Studio UI
    ui_init();

    // Set initial time (example: October 10, 2025, 14:30:00)
    // You can replace this with NTP time sync or RTC
    struct timeval tv;
    struct tm timeinfo;
    timeinfo.tm_year = 2025 - 1900;  // Years since 1900
    timeinfo.tm_mon = 10 - 1;        // Month (0-11)
    timeinfo.tm_mday = 10;           // Day of month
    timeinfo.tm_hour = 14;           // Hour (0-23)
    timeinfo.tm_min = 30;            // Minute
    timeinfo.tm_sec = 0;             // Second
    tv.tv_sec = mktime(&timeinfo);
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);

    // Create LVGL timer to update time labels every 1000ms (1 second)
    lv_timer_create(update_time_labels, 1000, NULL);

    // Create LVGL timer to update battery status every 5000ms (5 seconds)
    lv_timer_create(update_battery_status, 5000, NULL);

    // Call once immediately to show current time and battery
    update_time_labels(NULL);
    update_battery_status(NULL);
  }

  USBSerial.println("Setup done");
}

void loop() {
  static uint32_t last_tick = 0;
  uint32_t current_tick = millis();
  uint32_t elapsed = current_tick - last_tick;

  lv_tick_inc(elapsed); /* Tell LVGL how much time has elapsed */
  last_tick = current_tick;

  lv_timer_handler(); /* let the GUI do its work */

#ifdef DIRECT_RENDER_MODE
#if defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL)
  gfx->flush();
#else   // !(defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL))
  gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif  // !(defined(CANVAS) || defined(RGB_PANEL) || defined(DSI_PANEL))
#else   // !DIRECT_RENDER_MODE
#ifdef CANVAS
  gfx->flush();
#endif
#endif  // !DIRECT_RENDER_MODE

  delay(5);
}
