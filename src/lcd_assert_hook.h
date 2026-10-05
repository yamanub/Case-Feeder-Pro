// LVGL assert hook (LV_ASSERT_HANDLER in lv_conf.h). LVGL's default handler
// spins forever; this one, defined in lcd_terminal.cpp, restarts the LCD.
// Part of Case Feeder Pro. Licensed under GPL-3.0.
#ifndef LCD_ASSERT_HOOK_H
#define LCD_ASSERT_HOOK_H

#ifdef __cplusplus
extern "C" {
#endif

void lcd_lvgl_assert_hook(void);

#ifdef __cplusplus
}
#endif

#endif
