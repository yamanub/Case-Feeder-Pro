/*
 * display_config.h
 * Display + capacitive-touch pin/geometry constants for the
 * 2.8" ESP32-S3 Display module (LCDWiki ES3C28P, ILI9341 + FT6336).
 *
 * The display SPI pins are configured via build flags in platformio.ini
 * (TFT_eSPI). These are the touch I2C pins and screen geometry consumed
 * by lvgl_driver.h. Part of Case Feeder Pro. Licensed under GPL-3.0.
 */
#ifndef DISPLAY_CONFIG_H
#define DISPLAY_CONFIG_H

// ===== DISPLAY GEOMETRY =====
#define SCREEN_WIDTH    320
#define SCREEN_HEIGHT   240
#define SCREEN_ROTATION         1   // 1 = landscape (USB on the right)
#define SCREEN_ROTATION_FLIPPED 3   // 3 = landscape turned 180 (USB on the left)

// Default orientation: build with -DSCREEN_FLIPPED=1 to start with the screen
// turned 180 degrees. Settings > Flip screen changes it on the unit; that
// choice is stored on the LCD and wins over this default.
#ifndef SCREEN_FLIPPED
#define SCREEN_FLIPPED 0
#endif

// ===== TOUCH PANEL (FT6336, I2C - onboard) =====
#define TOUCH_SDA 16
#define TOUCH_SCL 15
#define TOUCH_INT 17
#define TOUCH_RST 18

#endif // DISPLAY_CONFIG_H
