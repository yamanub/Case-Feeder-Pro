/**
 * @file lvgl_driver.h
 * LVGL v8 display and touch driver for the 2.8" ESP32-S3 display module.
 * Uses TFT_eSPI for the display (ILI9341) and FT6336 for capacitive touch.
 * TFT_eSPI is configured via build flags in platformio.ini.
 * Part of Case Feeder Pro. Licensed under GPL-3.0.
 */

#ifndef LVGL_DRIVER_H
#define LVGL_DRIVER_H

#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <FT6336.h>
#include "display_config.h"

// Display buffer size - 10 rows of pixels (~1/24 of the 320x240 frame) per
// buffer, for memory efficiency. LVGL flushes dirty areas in slices this tall.
#define DRAW_BUF_SIZE (SCREEN_WIDTH * 10)

// TFT and touch objects - managed by this driver
static TFT_eSPI* _tft = nullptr;
static FT6336* _touch = nullptr;
// Screen turned 180 degrees? Set before lvgl_driver_init() (from the saved
// setting / build default) and changed live with lvgl_set_flipped().
static bool _screenFlipped = (SCREEN_FLIPPED != 0);
static lv_disp_draw_buf_t _draw_buf;
static lv_disp_drv_t _disp_drv;
static lv_indev_drv_t _touch_drv;
static lv_color_t* _buf1 = nullptr;
static lv_color_t* _buf2 = nullptr;

/**
 * Display flush callback - sends pixels to TFT
 */
static void lvgl_display_flush(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
    if (!_tft) return;

    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    _tft->startWrite();
    _tft->setAddrWindow(area->x1, area->y1, w, h);
    _tft->pushColors((uint16_t*)&color_p->full, w * h, true);
    _tft->endWrite();

    lv_disp_flush_ready(disp);
}

/**
 * Touch read callback - reads touch position from FT6336
 * Handles coordinate transformation for landscape mode
 */
static void lvgl_touch_read(lv_indev_drv_t* indev, lv_indev_data_t* data) {
    if (!_touch) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }

    _touch->read();

    if (_touch->isTouched) {
        // Get raw touch coordinates (FT6336 is portrait 240x320)
        int rawX = _touch->points[0].x;
        int rawY = _touch->points[0].y;

        // Transform for landscape mode (rotation 1)
        // Portrait FT6336: 240x320, Landscape screen: 320x240
        // Screen X = Touch Y, Screen Y = 240 - Touch X
        int screenX = rawY;
        int screenY = SCREEN_HEIGHT - 1 - rawX;
        // Flipped 180 (rotation 3): the same mapping mirrored on both axes.
        if (_screenFlipped) {
            screenX = SCREEN_WIDTH - 1 - screenX;
            screenY = SCREEN_HEIGHT - 1 - screenY;
        }

        // Clamp to screen bounds
        if (screenX < 0) screenX = 0;
        if (screenX >= SCREEN_WIDTH) screenX = SCREEN_WIDTH - 1;
        if (screenY < 0) screenY = 0;
        if (screenY >= SCREEN_HEIGHT) screenY = SCREEN_HEIGHT - 1;

        data->point.x = screenX;
        data->point.y = screenY;
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

/**
 * Initialize LVGL with TFT_eSPI display and FT6336 touch
 * TFT_eSPI is configured via build flags in platformio.ini
 */
inline void lvgl_driver_init(void) {
    Serial.println("LVGL: Initializing driver...");

    // Create and initialize TFT display
    _tft = new TFT_eSPI();
    _tft->init();
    _tft->invertDisplay(true);  // Required for this ILI9341 display
    _tft->setRotation(_screenFlipped ? SCREEN_ROTATION_FLIPPED : SCREEN_ROTATION);
    _tft->fillScreen(TFT_BLACK);

    // Setup backlight
    pinMode(45, OUTPUT);
    digitalWrite(45, HIGH);

    Serial.println("LVGL: TFT initialized");

    // Initialize LVGL library
    lv_init();
    Serial.println("LVGL: Core initialized");

    // Allocate draw buffers
    _buf1 = (lv_color_t*)heap_caps_malloc(DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    _buf2 = (lv_color_t*)heap_caps_malloc(DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);

    if (!_buf1) {
        Serial.println("LVGL: ERROR - Failed to allocate draw buffer!");
        return;
    }

    if (_buf2) {
        lv_disp_draw_buf_init(&_draw_buf, _buf1, _buf2, DRAW_BUF_SIZE);
        Serial.println("LVGL: Using double buffer mode");
    } else {
        lv_disp_draw_buf_init(&_draw_buf, _buf1, nullptr, DRAW_BUF_SIZE);
        Serial.println("LVGL: Using single buffer mode");
    }

    // Initialize display driver
    lv_disp_drv_init(&_disp_drv);
    _disp_drv.hor_res = SCREEN_WIDTH;
    _disp_drv.ver_res = SCREEN_HEIGHT;
    _disp_drv.flush_cb = lvgl_display_flush;
    _disp_drv.draw_buf = &_draw_buf;
    lv_disp_drv_register(&_disp_drv);

    Serial.printf("LVGL: Display driver registered (%dx%d)\n", SCREEN_WIDTH, SCREEN_HEIGHT);

    // Touch controller (pins in display_config.h)
    _touch = new FT6336(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, 240, 320);
    _touch->begin();
    _touch->setThreshold(40);  // Lower threshold = more sensitive (default ~80)
    Serial.println("LVGL: Touch initialized (FT6336, threshold=40)");

    // Initialize touch driver
    lv_indev_drv_init(&_touch_drv);
    _touch_drv.type = LV_INDEV_TYPE_POINTER;
    _touch_drv.read_cb = lvgl_touch_read;
    lv_indev_drv_register(&_touch_drv);
    Serial.println("LVGL: Touch driver registered");

    Serial.println("LVGL: Driver initialization complete");
}

/**
 * Get the TFT_eSPI instance (for splash screen or direct drawing if needed)
 */
inline TFT_eSPI* lvgl_get_tft(void) {
    return _tft;
}

/**
 * Choose the 180-degree flip. Before lvgl_driver_init() it just sets the
 * startup orientation; afterwards it rotates the panel and touch mapping live
 * and forces a full redraw (the layout is identical either way).
 */
inline void lvgl_set_flipped(bool flipped) {
    _screenFlipped = flipped;
    if (!_tft) return;
    _tft->setRotation(flipped ? SCREEN_ROTATION_FLIPPED : SCREEN_ROTATION);
    lv_obj_invalidate(lv_scr_act());
    lv_obj_invalidate(lv_layer_top());
}

inline bool lvgl_is_flipped(void) {
    return _screenFlipped;
}

#endif // LVGL_DRIVER_H
