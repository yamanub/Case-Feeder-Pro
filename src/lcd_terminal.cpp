// ============================================================
//  Case Feeder Pro - LCD firmware
//  LCDWiki ES3C28P 2.8" ESP32-S3 display (ILI9341 + FT6336 touch)
//  Copyright (C) the Case Feeder Pro contributors. Licensed under GPL-3.0.
//
//  The touchscreen for the controller. It lays itself out from what the
//  controller reports:
//    - one feeder:  the full single screen, titled CASE FEEDER PRO or
//                   BULLET FEEDER PRO (speed presets, rate, TEMP, STOP/GO)
//    - two feeders: DUAL FEEDER PRO, split in halves - CASE left, BULLET
//                   right, each with - / + speed, rate and its own STOP/GO
//  Settings > Machine (press and hold) switches the controller between case,
//  bullet and dual; the screen follows on its own. The last layout is
//  remembered so the right screen comes up at power-on.
//
//  The LCD displays state, offers touch control and sounds the alerts. The
//  controller owns all motor and safety behavior and keeps running if the
//  LCD is unplugged or restarting.
//
//  Link: 115200 8-N-1 on GPIO43 (TX) / GPIO44 (RX).
// ============================================================

#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>
#include <lvgl.h>
#include "lvgl_driver.h"
#include "audio_manager.h"
#include "feeder_protocol.h"
#include "lcd_assert_hook.h"

using namespace FeederProtocol;

static constexpr uint8_t  PIN_CONTROLLER_TX = 43;
static constexpr uint8_t  PIN_CONTROLLER_RX = 44;
static constexpr uint32_t CONNECTION_TIMEOUT_MS = 3000;
#define FW_VERSION "1.0.0"

// LVGL calls this on an internal error; restart instead of hanging.
extern "C" void lcd_lvgl_assert_hook(void) {
    esp_restart();
}

HardwareSerial ControllerSerial(1);
AudioManager   audio;

// ---- Theme ----------------------------------------------------------------
#define COL_SCR_BG     0x0C0F14
#define COL_BAR_BG     0x151B24
#define COL_PANEL_BG   0x121822
#define COL_PANEL_BRD  0x2C3E55
#define COL_TAB_BG     0x2B6CB0
#define COL_TEXT       0xFFFFFF
#define COL_MUTED      0x8A97A6
#define COL_GREEN      0x39D353
#define COL_AMBER      0xFFB300
#define COL_REDST      0xFF5252
#define COL_RED_TOP    0xE5453C
#define COL_RED_BOT    0xB5271F
#define COL_GRN_TOP    0x43A047
#define COL_GRN_BOT    0x2E7D32
#define COL_GRY_TOP    0x6B7280
#define COL_GRY_BOT    0x434954
#define COL_SIL_TOP    0xFBFCFD
#define COL_SIL_BOT    0xC2C8D0
#define COL_SIL_BRD    0x9AA2AD
#define COL_SIL_TXT    0x161A1F
#define COL_SEL_TOP    0x5AA7F0
#define COL_SEL_BOT    0x1E6FD0
#define COL_SEL_BRD    0x1257A8
#define COL_PURPLE     0xAB47BC

#define NUM_SPEED_PRESETS 5
static const uint8_t SPEED_PRESETS[NUM_SPEED_PRESETS] = { 20, 40, 60, 80, 100 };
#define WARN_STEP_SECONDS 15
#define NUM_WARN_OPTIONS  6                // 15..90 s

// Rate (parts per hour): rolling window over recent counts.
#define RATE_RING        64
#define RATE_WINDOW_MS   30000UL
#define RATE_DECAY_GRACE 2
#define RATE_ROUND       25

// ---- Per-feeder data + its widgets on the current screen ------------------
struct FeederView {
    StatusPayload st;
    uint8_t  product;          // ProductType, from the controller
    bool     have;
    uint32_t lastFrameAt;
    uint32_t fedTimes[RATE_RING];
    uint8_t  ringHead, ringLen;
    uint16_t lastSeenCount;
    bool     haveSeenCount;
    float    rateSmoothed;
    uint32_t lastRateAt;
    // widgets (null when this feeder is not on screen)
    lv_obj_t *stateLed, *stateLbl, *speedVal, *speedBar, *rateLbl, *goBtn, *goLbl;
    lv_obj_t *stallModal, *idleModal, *warnModal;
    // refresh cache
    bool     warnDismissed, warnShown;
    int      lastState, lastFlags, lastPct, lastHave, lastRate;
};
static FeederView views[MAX_FEEDERS];

// What the screen shows: a MachineMask (which feeders), plus the product of
// the single feeder.
static uint8_t uiShown   = 0;
static uint8_t uiProduct = PRODUCT_CASE;
static uint8_t wantShown = MACHINE_CASE, wantProduct = PRODUCT_CASE;
static bool isDual()       { return uiShown == MACHINE_DUAL; }
static uint8_t singleIdx() { return uiShown == MACHINE_BULLET ? 1 : 0; }
static bool shownOnScreen(uint8_t i) { return (uiShown >> i) & 1U; }

static uint32_t lastValidFrameAt = 0, lastPingAt = 0, lastStatusRequestAt = 0;
static bool     audioEnabled = true;
static int      audioVolume  = 2;

// Shared widgets
static lv_obj_t *ui_tempLabel, *ui_healthLbl, *ui_linkLed, *ui_linkLbl;
static lv_obj_t *ui_settingsModal, *ui_audioSwitch, *ui_flipSwitch, *ui_machineBtnLbl;
static lv_obj_t *ui_warnDd[MAX_FEEDERS];
static lv_obj_t *ui_volBtns[3], *ui_volLbls[3];
static lv_obj_t *ui_tempModal, *ui_tempBox, *ui_tempTitle, *ui_tempMsg;
static lv_obj_t *ui_resetModal, *ui_resetTitle;
static uint8_t   resetTarget = 0;
static lv_obj_t *ui_machineModal, *ui_machineOpt[3], *ui_machineOptLbl[3], *ui_machineInfo,
                *ui_machineNote, *ui_machineApply;
static uint8_t   machineChoice = MACHINE_CASE;
static lv_obj_t *ui_speedBtns[NUM_SPEED_PRESETS], *ui_speedBtnLbls[NUM_SPEED_PRESETS];   // single screen
// Shared refresh cache
static int      sharedLastLink = -1, sharedLastTemp = INT16_MIN + 1;
static char     sharedLastHealth[40] = "";
static uint8_t  tempAckLevel = 0, shownTempLevel = 0;
static int      shownTempWho = -1;

struct RxParser {
    enum Stage : uint8_t { WAIT_SOF, READ_LEN, READ_BODY, CRC_LOW, CRC_HIGH, WAIT_EOF } stage = WAIT_SOF;
    uint8_t length = 0, index = 0;
    uint8_t body[MAX_PAYLOAD + 1]{};
    uint16_t receivedCrc = 0;
} rx;

// ---- Wording by product -----------------------------------------------------
static bool isBullet(uint8_t product) { return product == PRODUCT_BULLET; }
static const char* nameOf(const FeederView& v)    { return isBullet(v.product) ? "BULLET" : "CASE"; }
static const char* rateTabOf(const FeederView& v) { return isBullet(v.product) ? "BULLETS/HR" : "CASES/HR"; }
static const char* holdOf(const FeederView& v)    { return isBullet(v.product) ? "ROUND HOLD" : "CASE HOLD"; }
static const char* nounOf(const FeederView& v)    { return isBullet(v.product) ? "bullets" : "cases"; }

// ---- Link -------------------------------------------------------------------
static void sendFrame(uint8_t command, const void* payload = nullptr, uint8_t payloadLength = 0) {
    if (payloadLength > MAX_PAYLOAD) return;
    uint8_t frame[MAX_PAYLOAD + 7];
    size_t i = 0;
    frame[i++] = SOF;
    frame[i++] = (uint8_t)(payloadLength + 1);
    frame[i++] = command;
    if (payloadLength && payload) { memcpy(&frame[i], payload, payloadLength); i += payloadLength; }
    const uint16_t crc = crc16(&frame[1], payloadLength + 2);
    frame[i++] = (uint8_t)crc;
    frame[i++] = (uint8_t)(crc >> 8);
    frame[i++] = EOF_MARKER;
    ControllerSerial.write(frame, i);
}

// Command for one feeder: its payload followed by the feeder index.
static void sendTo(uint8_t command, uint8_t feeder, const void* payload = nullptr, uint8_t len = 0) {
    uint8_t buf[MAX_PAYLOAD];
    if (len) memcpy(buf, payload, len);
    buf[len] = feeder;
    sendFrame(command, buf, len + 1);
}

static bool controllerLinked() { return (millis() - lastValidFrameAt) < CONNECTION_TIMEOUT_MS; }
static bool feederLinked(const FeederView& v) {
    return v.have && (millis() - v.lastFrameAt) < CONNECTION_TIMEOUT_MS;
}

// ---- Rate -------------------------------------------------------------------
static void recordFed(FeederView& v, uint32_t t) {
    v.fedTimes[v.ringHead] = t;
    v.ringHead = (v.ringHead + 1) % RATE_RING;
    if (v.ringLen < RATE_RING) v.ringLen++;
}

// Parts per hour from the counts inside the window. When feeding stops, the
// rate decays instead of freezing at its last value.
static int computeRate(FeederView& v, uint32_t now) {
    uint32_t oldest = 0, newest = 0;
    int inWin = 0;
    for (uint8_t i = 0; i < v.ringLen; i++) {
        const uint32_t t = v.fedTimes[i];
        if (t == 0 || now - t > RATE_WINDOW_MS) continue;
        if (inWin == 0 || t < oldest) oldest = t;
        if (inWin == 0 || t > newest) newest = t;
        inWin++;
    }
    if (inWin < 2 || newest <= oldest) return 0;
    const uint32_t span = newest - oldest;
    const uint32_t avgGap = span / (inWin - 1);
    int rate = (int)((uint64_t)(inWin - 1) * 3600000ULL / span);
    const uint32_t sinceLast = now - newest;
    if (sinceLast > avgGap * RATE_DECAY_GRACE) {
        const int cap = (int)(3600000ULL / (sinceLast < 1 ? 1 : sinceLast));
        if (cap < rate) rate = cap;
    }
    return rate;
}

static void fmt_commas(char* out, uint32_t v) {
    char tmp[12];
    const int n = snprintf(tmp, sizeof(tmp), "%lu", (unsigned long)v);
    const int commas = (n - 1) / 3;
    int oi = n + commas;
    out[oi--] = '\0';
    int c = 0;
    for (int i = n - 1; i >= 0; i--) {
        out[oi--] = tmp[i];
        if (++c % 3 == 0 && i != 0) out[oi--] = ',';
    }
}

static const char* stateText(const FeederView& v) {
    switch (v.st.state) {
        case STATE_RUNNING:      return "RUNNING";
        case STATE_BEAM_BLOCKED: return holdOf(v);
        case STATE_JAM_REVERSE:
        case STATE_JAM_PAUSE:    return isDual() ? "JAM" : "JAM RECOVERY";
        case STATE_FAULT:        return "FAULT";
        case STATE_IDLE_SHUTOFF: return "IDLE";
        case STATE_PAUSED:       return "PAUSED";
        default:                 return "?";
    }
}

static uint32_t stateColour(uint8_t s) {
    switch (s) {
        case STATE_RUNNING:      return COL_GREEN;
        case STATE_BEAM_BLOCKED: return 0x1E88E5;
        case STATE_JAM_REVERSE:
        case STATE_JAM_PAUSE:    return 0xFFA726;
        case STATE_FAULT:        return COL_REDST;
        case STATE_IDLE_SHUTOFF: return COL_AMBER;
        default:                 return COL_PURPLE;
    }
}

// ---- UI helpers -------------------------------------------------------------
static void ui_grad(lv_obj_t* o, uint32_t top, uint32_t bot) {
    lv_obj_set_style_bg_color(o, lv_color_hex(top), 0);
    lv_obj_set_style_bg_grad_color(o, lv_color_hex(bot), 0);
    lv_obj_set_style_bg_grad_dir(o, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_shadow_width(o, 0, 0);
}

static void ui_style_card(lv_obj_t* c) {
    lv_obj_set_style_bg_color(c, lv_color_hex(COL_PANEL_BG), 0);
    lv_obj_set_style_border_color(c, lv_color_hex(COL_PANEL_BRD), 0);
    lv_obj_set_style_border_width(c, 1, 0);
    lv_obj_set_style_radius(c, 6, 0);
    lv_obj_set_style_pad_all(c, 0, 0);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
}

static lv_obj_t* ui_tab(lv_obj_t* parent, const char* txt, const lv_font_t* font) {
    lv_obj_t* tab = lv_label_create(parent);
    lv_label_set_text(tab, txt);
    lv_obj_set_style_text_color(tab, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_style_text_font(tab, font, 0);
    lv_obj_set_style_bg_color(tab, lv_color_hex(COL_TAB_BG), 0);
    lv_obj_set_style_bg_opa(tab, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(tab, 6, 0);
    lv_obj_set_style_pad_ver(tab, 2, 0);
    lv_obj_set_style_radius(tab, 3, 0);
    return tab;
}

static lv_obj_t* ui_label(lv_obj_t* parent, const char* txt, const lv_font_t* font, uint32_t col) {
    lv_obj_t* l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(col), 0);
    return l;
}

static lv_obj_t* ui_bar_strip(lv_obj_t* scr, int h, lv_align_t align) {
    lv_obj_t* bar = lv_obj_create(scr);
    lv_obj_set_size(bar, SCREEN_WIDTH, h);
    lv_obj_align(bar, align, 0, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(COL_BAR_BG), 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_pad_all(bar, 0, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    return bar;
}

// Dimmed full-screen layer that holds a popup box. Starts hidden.
static lv_obj_t* ui_overlay() {
    lv_obj_t* ov = lv_obj_create(lv_layer_top());
    lv_obj_set_size(ov, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_style_bg_color(ov, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(ov, LV_OPA_60, 0);
    lv_obj_set_style_border_width(ov, 0, 0);
    lv_obj_set_style_pad_all(ov, 0, 0);
    lv_obj_clear_flag(ov, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(ov, LV_OBJ_FLAG_HIDDEN);
    return ov;
}

static lv_obj_t* ui_box(lv_obj_t* ov, int w, int h, uint32_t border, uint32_t bg, int borderW) {
    lv_obj_t* box = lv_obj_create(ov);
    lv_obj_set_size(box, w, h);
    lv_obj_center(box);
    lv_obj_set_style_bg_color(box, lv_color_hex(bg), 0);
    lv_obj_set_style_border_color(box, lv_color_hex(border), 0);
    lv_obj_set_style_border_width(box, borderW, 0);
    lv_obj_set_style_radius(box, 12, 0);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    return box;
}

static lv_obj_t* ui_button(lv_obj_t* parent, int w, int h, const char* txt, const lv_font_t* font,
                           lv_event_cb_t cb, void* user, lv_event_code_t code = LV_EVENT_CLICKED) {
    lv_obj_t* b = lv_btn_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_radius(b, 8, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_add_event_cb(b, cb, code, user);
    lv_obj_t* l = ui_label(b, txt, font, COL_TEXT);
    lv_obj_center(l);
    return b;
}

// Alert popup: title, message, one button. Title/message can be reworded live.
static lv_obj_t* ui_modal(uint32_t accent, uint32_t boxBg, const char* title, const char* msg,
                          const char* btnText, lv_event_cb_t cb, void* user, uint32_t btnColor,
                          lv_obj_t** titleOut = nullptr, lv_obj_t** msgOut = nullptr,
                          lv_obj_t** boxOut = nullptr) {
    lv_obj_t* ov = ui_overlay();
    lv_obj_t* box = ui_box(ov, 290, 200, accent, boxBg, 4);
    lv_obj_t* t = ui_label(box, title, isDual() ? &lv_font_montserrat_22 : &lv_font_montserrat_28, accent);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_t* m = ui_label(box, msg, &lv_font_montserrat_16, COL_TEXT);
    lv_label_set_long_mode(m, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(m, 250);
    lv_obj_set_style_text_align(m, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(m, LV_ALIGN_CENTER, 0, -2);
    lv_obj_t* btn = ui_button(box, 170, 50, btnText, &lv_font_montserrat_20, cb, user);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(btnColor), 0);
    if (titleOut) *titleOut = t;
    if (msgOut) *msgOut = m;
    if (boxOut) *boxOut = box;
    return ov;
}

// ---- Callbacks --------------------------------------------------------------
static uint8_t idxOf(lv_event_t* e) { return (uint8_t)(intptr_t)lv_event_get_user_data(e); }

static void ui_go_cb(lv_event_t* e) {
    const uint8_t i = idxOf(e);
    FeederView& v = views[i];
    if (!feederLinked(v)) return;
    if (v.st.state == STATE_FAULT || v.st.state == STATE_IDLE_SHUTOFF) { sendTo(CMD_CLEAR_FAULT, i); return; }
    const uint8_t run = (v.st.state == STATE_PAUSED) ? 1 : 0;
    sendTo(CMD_SET_RUN, i, &run, 1);
}

// Dual screen: - / + step through the presets from wherever the speed is now.
static void stepSpeed(uint8_t i, int dir) {
    FeederView& v = views[i];
    if (!feederLinked(v)) return;
    const uint8_t cur = v.st.speedPercent;
    uint8_t next;
    if (dir > 0) {
        next = SPEED_PRESETS[NUM_SPEED_PRESETS - 1];
        for (int k = 0; k < NUM_SPEED_PRESETS; k++) if (SPEED_PRESETS[k] > cur) { next = SPEED_PRESETS[k]; break; }
    } else {
        next = SPEED_PRESETS[0];
        for (int k = NUM_SPEED_PRESETS - 1; k >= 0; k--) if (SPEED_PRESETS[k] < cur) { next = SPEED_PRESETS[k]; break; }
    }
    sendTo(CMD_SET_SPEED_PERCENT, i, &next, 1);
}
static void ui_minus_cb(lv_event_t* e) { stepSpeed(idxOf(e), -1); }
static void ui_plus_cb(lv_event_t* e)  { stepSpeed(idxOf(e), +1); }

// Single screen: tap a preset.
static void ui_speed_btn_cb(lv_event_t* e) {
    const uint8_t pct = SPEED_PRESETS[idxOf(e)];
    sendTo(CMD_SET_SPEED_PERCENT, singleIdx(), &pct, 1);
}

static void ui_rate_longpress_cb(lv_event_t* e) {
    resetTarget = idxOf(e);
    if (isDual()) lv_label_set_text_fmt(ui_resetTitle, "Reset %s rate?", nameOf(views[resetTarget]));
    else          lv_label_set_text(ui_resetTitle, "Reset rate?");
    lv_obj_clear_flag(ui_resetModal, LV_OBJ_FLAG_HIDDEN);
}
static void ui_reset_yes_cb(lv_event_t*) {
    FeederView& v = views[resetTarget];
    v.ringLen = 0; v.ringHead = 0; v.rateSmoothed = 0; v.lastRate = -1;
    if (v.rateLbl) lv_label_set_text(v.rateLbl, "0");
    lv_obj_add_flag(ui_resetModal, LV_OBJ_FLAG_HIDDEN);
}
static void ui_reset_no_cb(lv_event_t*) { lv_obj_add_flag(ui_resetModal, LV_OBJ_FLAG_HIDDEN); }

static void ui_resume_cb(lv_event_t* e) { sendTo(CMD_CLEAR_FAULT, idxOf(e)); }
static void ui_warn_dismiss_cb(lv_event_t* e) {
    FeederView& v = views[idxOf(e)];
    v.warnDismissed = true;
    v.warnShown = false;
    lv_obj_add_flag(v.warnModal, LV_OBJ_FLAG_HIDDEN);
}
// Dismissing acknowledges that band; the popup returns only if it gets hotter,
// and everything re-arms once every driver is back below 120 C.
static void ui_temp_dismiss_cb(lv_event_t*) {
    tempAckLevel = shownTempLevel;
    lv_obj_add_flag(ui_tempModal, LV_OBJ_FLAG_HIDDEN);
}

// Alerts, volume and screen flip are stored on the LCD. (The warning time
// belongs to the controller and is stored there.)
static void saveUiPrefs() {
    Preferences p;
    p.begin("ui", false);
    p.putBool("alerts", audioEnabled);
    p.putUChar("vol", (uint8_t)audioVolume);
    p.putBool("flip", lvgl_is_flipped());
    p.end();
}

static void ui_apply_volume() {
    for (int i = 0; i < 3; i++) {
        const bool sel = (i == audioVolume - 1);
        ui_grad(ui_volBtns[i], sel ? COL_SEL_TOP : COL_SIL_TOP, sel ? COL_SEL_BOT : COL_SIL_BOT);
        lv_obj_set_style_border_color(ui_volBtns[i], lv_color_hex(sel ? COL_SEL_BRD : COL_SIL_BRD), 0);
        lv_obj_set_style_text_color(ui_volLbls[i], lv_color_hex(sel ? COL_TEXT : COL_SIL_TXT), 0);
    }
}
static void ui_audio_cb(lv_event_t*) {
    audioEnabled = lv_obj_has_state(ui_audioSwitch, LV_STATE_CHECKED);
    saveUiPrefs();
    if (audioEnabled) audio.beep(1000, 100);
}
static void ui_vol_cb(lv_event_t* e) {
    audioVolume = (int)idxOf(e) + 1;
    audio.setVolume(audioVolume);
    saveUiPrefs();
    ui_apply_volume();
    if (audioEnabled) audio.beep(1000, 100);
}
static void ui_flip_cb(lv_event_t*) {
    lvgl_set_flipped(lv_obj_has_state(ui_flipSwitch, LV_STATE_CHECKED));
    saveUiPrefs();
}
static void ui_warn_dd_cb(lv_event_t* e) {
    const uint8_t i = idxOf(e);
    const uint16_t sec = (uint16_t)((lv_dropdown_get_selected(ui_warnDd[i]) + 1) * WARN_STEP_SECONDS);
    sendTo(CMD_SET_WARN_SECONDS, i, &sec, 2);
}

static const char* machineShort(uint8_t mask) {
    switch (mask) {
        case MACHINE_BULLET: return "BULLET";
        case MACHINE_DUAL:   return "DUAL";
        default:             return "CASE";
    }
}

static void ui_settings_open_cb(lv_event_t*) {
    if (audioEnabled) lv_obj_add_state(ui_audioSwitch, LV_STATE_CHECKED);
    else              lv_obj_clear_state(ui_audioSwitch, LV_STATE_CHECKED);
    if (lvgl_is_flipped()) lv_obj_add_state(ui_flipSwitch, LV_STATE_CHECKED);
    else                   lv_obj_clear_state(ui_flipSwitch, LV_STATE_CHECKED);
    for (uint8_t i = 0; i < MAX_FEEDERS; i++) {
        if (!ui_warnDd[i]) continue;
        long idx = (long)(views[i].st.warnSeconds / WARN_STEP_SECONDS) - 1;
        if (idx < 0) idx = 0;
        if (idx > NUM_WARN_OPTIONS - 1) idx = NUM_WARN_OPTIONS - 1;
        lv_dropdown_set_selected(ui_warnDd[i], (uint16_t)idx);
    }
    lv_label_set_text(ui_machineBtnLbl, machineShort(uiShown));
    ui_apply_volume();
    lv_obj_clear_flag(ui_settingsModal, LV_OBJ_FLAG_HIDDEN);
}
static void ui_settings_close_cb(lv_event_t*) { lv_obj_add_flag(ui_settingsModal, LV_OBJ_FLAG_HIDDEN); }

// ---- Machine chooser (Settings > Machine, press and hold) --------------------
static const uint8_t MACHINE_OPTIONS[3] = { MACHINE_CASE, MACHINE_BULLET, MACHINE_DUAL };

static bool allFeedersStopped() {
    for (uint8_t i = 0; i < MAX_FEEDERS; i++) {
        if (!shownOnScreen(i)) continue;
        const FeederView& v = views[i];
        if (!feederLinked(v)) return false;
        if (v.st.state != STATE_PAUSED && v.st.state != STATE_FAULT && v.st.state != STATE_IDLE_SHUTOFF) return false;
    }
    return true;
}

static void ui_machine_show(const char* note) {
    for (int k = 0; k < 3; k++) {
        const bool sel = MACHINE_OPTIONS[k] == machineChoice;
        ui_grad(ui_machineOpt[k], sel ? COL_SEL_TOP : COL_SIL_TOP, sel ? COL_SEL_BOT : COL_SIL_BOT);
        lv_obj_set_style_border_color(ui_machineOpt[k], lv_color_hex(sel ? COL_SEL_BRD : COL_SIL_BRD), 0);
        lv_obj_set_style_text_color(ui_machineOptLbl[k], lv_color_hex(sel ? COL_TEXT : COL_SIL_TXT), 0);
    }
    switch (machineChoice) {
        case MACHINE_CASE:   lv_label_set_text(ui_machineInfo, "Case feeder only.\nMotor on X, beam on Y-STOP."); break;
        case MACHINE_BULLET: lv_label_set_text(ui_machineInfo, "Bullet feeder only.\nMotor on E, beam on Z-STOP."); break;
        default:             lv_label_set_text(ui_machineInfo, "Case: motor X, beam Y-STOP.\nBullet: motor E, beam Z-STOP."); break;
    }
    const bool canApply = controllerLinked() && allFeedersStopped();
    if (note)                     lv_label_set_text(ui_machineNote, note);
    else if (!controllerLinked()) lv_label_set_text(ui_machineNote, "No controller link.");
    else if (!canApply)           lv_label_set_text(ui_machineNote, "Stop every feeder first.");
    else                          lv_label_set_text(ui_machineNote, "");
    if (canApply) lv_obj_clear_state(ui_machineApply, LV_STATE_DISABLED);
    else          lv_obj_add_state(ui_machineApply, LV_STATE_DISABLED);
}

static void ui_machine_open_cb(lv_event_t*) {
    machineChoice = uiShown ? uiShown : MACHINE_CASE;
    ui_machine_show(nullptr);
    lv_obj_add_flag(ui_settingsModal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_machineModal, LV_OBJ_FLAG_HIDDEN);
}
static void ui_machine_opt_cb(lv_event_t* e) {
    machineChoice = MACHINE_OPTIONS[idxOf(e)];
    ui_machine_show(nullptr);
}
static void ui_machine_cancel_cb(lv_event_t*) { lv_obj_add_flag(ui_machineModal, LV_OBJ_FLAG_HIDDEN); }
static void ui_machine_apply_cb(lv_event_t*) {
    lv_obj_add_flag(ui_machineModal, LV_OBJ_FLAG_HIDDEN);
    if (machineChoice == uiShown) return;
    sendFrame(CMD_SET_MACHINE, &machineChoice, 1);   // the screen follows the next status frames
}

// ---- Shared popups ------------------------------------------------------------
static lv_obj_t* ui_settings_row(lv_obj_t* box, const char* txt, int y) {
    lv_obj_t* l = ui_label(box, txt, &lv_font_montserrat_16, COL_TEXT);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 8, y + 3);
    return l;
}

static lv_obj_t* ui_switch(lv_obj_t* box, int y, lv_event_cb_t cb) {
    lv_obj_t* sw = lv_switch_create(box);
    lv_obj_set_size(sw, 48, 24);
    lv_obj_align(sw, LV_ALIGN_TOP_RIGHT, -8, y + 1);
    lv_obj_set_style_bg_color(sw, lv_color_hex(COL_GREEN), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_ext_click_area(sw, 16);
    lv_obj_add_event_cb(sw, cb, LV_EVENT_VALUE_CHANGED, NULL);
    return sw;
}

static lv_obj_t* ui_warn_dropdown(lv_obj_t* box, int y, uint8_t feeder) {
    lv_obj_t* dd = lv_dropdown_create(box);
    lv_dropdown_set_options_static(dd, "15 s\n30 s\n45 s\n60 s\n75 s\n90 s");
    lv_obj_set_size(dd, 104, 27);
    lv_obj_align(dd, LV_ALIGN_TOP_RIGHT, -8, y);
    lv_obj_set_style_text_font(dd, &lv_font_montserrat_16, 0);
    lv_obj_set_style_bg_color(dd, lv_color_hex(COL_BAR_BG), 0);
    lv_obj_set_style_text_color(dd, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_style_border_color(dd, lv_color_hex(COL_PANEL_BRD), 0);
    lv_obj_set_style_border_width(dd, 1, 0);
    lv_obj_set_style_pad_ver(dd, 3, 0);
    lv_obj_add_event_cb(dd, ui_warn_dd_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)feeder);
    lv_obj_t* list = lv_dropdown_get_list(dd);
    lv_obj_set_style_text_font(list, &lv_font_montserrat_16, 0);
    lv_obj_set_style_bg_color(list, lv_color_hex(COL_BAR_BG), 0);
    lv_obj_set_style_text_color(list, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_style_bg_color(list, lv_color_hex(COL_TAB_BG), LV_PART_SELECTED | LV_STATE_CHECKED);
    return dd;
}

static void ui_make_settings() {
    lv_obj_t* ov = ui_overlay();
    lv_obj_t* box = ui_box(ov, 300, 236, COL_TAB_BG, COL_PANEL_BG, 3);
    lv_obj_set_style_pad_all(box, 0, 0);
    lv_obj_t* t = ui_label(box, LV_SYMBOL_SETTINGS "  SETTINGS", &lv_font_montserrat_16, COL_TEXT);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 3);

    // Dual has one extra row (a warning time per feeder), so its rows are tighter.
    const int pitch = isDual() ? 29 : 34;
    int y = 23;
    ui_settings_row(box, "Alerts", y);
    ui_audioSwitch = ui_switch(box, y, ui_audio_cb);
    y += pitch;
    for (uint8_t i = 0; i < MAX_FEEDERS; i++) ui_warnDd[i] = nullptr;
    if (isDual()) {
        ui_settings_row(box, "Case warn", y);
        ui_warnDd[0] = ui_warn_dropdown(box, y, 0);
        y += pitch;
        ui_settings_row(box, "Bullet warn", y);
        ui_warnDd[1] = ui_warn_dropdown(box, y, 1);
    } else {
        ui_settings_row(box, "Early warn", y);
        ui_warnDd[singleIdx()] = ui_warn_dropdown(box, y, singleIdx());
    }
    y += pitch;
    ui_settings_row(box, "Flip screen", y);
    ui_flipSwitch = ui_switch(box, y, ui_flip_cb);
    y += pitch;

    // Machine: press and hold, so a stray tap cannot change it.
    lv_obj_t* ml = ui_settings_row(box, "Machine", y);
    lv_obj_t* mh = ui_label(box, "(hold)", &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align_to(mh, ml, LV_ALIGN_OUT_RIGHT_MID, 6, 1);
    lv_obj_t* mb = lv_btn_create(box);
    lv_obj_set_size(mb, 104, 27);
    lv_obj_align(mb, LV_ALIGN_TOP_RIGHT, -8, y);
    lv_obj_set_style_radius(mb, 5, 0);
    lv_obj_set_style_border_width(mb, 1, 0);
    lv_obj_set_style_border_color(mb, lv_color_hex(COL_SIL_BRD), 0);
    ui_grad(mb, COL_SIL_TOP, COL_SIL_BOT);
    lv_obj_add_event_cb(mb, ui_machine_open_cb, LV_EVENT_LONG_PRESSED, NULL);
    ui_machineBtnLbl = ui_label(mb, machineShort(uiShown), &lv_font_montserrat_16, COL_SIL_TXT);
    lv_obj_center(ui_machineBtnLbl);
    y += pitch;

    ui_settings_row(box, "Volume", y);
    const char* volTxt[3] = { "LOW", "MED", "HIGH" };
    for (int i = 0; i < 3; i++) {
        lv_obj_t* b = lv_btn_create(box);
        lv_obj_set_size(b, 58, 27);
        lv_obj_align(b, LV_ALIGN_TOP_RIGHT, -8 - (2 - i) * 62, y);
        lv_obj_set_style_radius(b, 5, 0);
        lv_obj_set_style_border_width(b, 1, 0);
        lv_obj_add_event_cb(b, ui_vol_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_t* l = ui_label(b, volTxt[i], &lv_font_montserrat_12, COL_SIL_TXT);
        lv_obj_center(l);
        ui_volBtns[i] = b;
        ui_volLbls[i] = l;
    }

    lv_obj_t* cb = ui_button(box, 140, 27, LV_SYMBOL_OK "  CLOSE", &lv_font_montserrat_16, ui_settings_close_cb, NULL);
    lv_obj_align(cb, LV_ALIGN_BOTTOM_MID, 0, -4);
    ui_grad(cb, COL_GRY_TOP, COL_GRY_BOT);
    ui_settingsModal = ov;
}

static void ui_make_machine() {
    lv_obj_t* ov = ui_overlay();
    lv_obj_t* box = ui_box(ov, 300, 226, COL_TAB_BG, COL_PANEL_BG, 3);
    lv_obj_set_style_pad_all(box, 0, 0);
    lv_obj_t* t = ui_label(box, "MACHINE", &lv_font_montserrat_20, COL_TEXT);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 6);
    for (int k = 0; k < 3; k++) {
        lv_obj_t* b = lv_btn_create(box);
        lv_obj_set_size(b, 88, 40);
        lv_obj_align(b, LV_ALIGN_TOP_MID, (k - 1) * 94, 36);
        lv_obj_set_style_radius(b, 6, 0);
        lv_obj_set_style_border_width(b, 1, 0);
        lv_obj_set_style_shadow_width(b, 0, 0);
        lv_obj_add_event_cb(b, ui_machine_opt_cb, LV_EVENT_CLICKED, (void*)(intptr_t)k);
        lv_obj_t* l = ui_label(b, machineShort(MACHINE_OPTIONS[k]), &lv_font_montserrat_18, COL_SIL_TXT);
        lv_obj_center(l);
        ui_machineOpt[k] = b;
        ui_machineOptLbl[k] = l;
    }
    ui_machineInfo = ui_label(box, "", &lv_font_montserrat_14, COL_TEXT);
    lv_obj_set_style_text_align(ui_machineInfo, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(ui_machineInfo, LV_ALIGN_TOP_MID, 0, 86);
    ui_machineNote = ui_label(box, "", &lv_font_montserrat_14, COL_AMBER);
    lv_label_set_long_mode(ui_machineNote, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(ui_machineNote, 280);
    lv_obj_set_style_text_align(ui_machineNote, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(ui_machineNote, LV_ALIGN_TOP_MID, 0, 128);

    lv_obj_t* no = ui_button(box, 120, 40, "CANCEL", &lv_font_montserrat_18, ui_machine_cancel_cb, NULL);
    lv_obj_align(no, LV_ALIGN_BOTTOM_MID, -66, -8);
    ui_grad(no, COL_GRY_TOP, COL_GRY_BOT);
    ui_machineApply = ui_button(box, 120, 40, LV_SYMBOL_OK " APPLY", &lv_font_montserrat_18, ui_machine_apply_cb, NULL);
    lv_obj_align(ui_machineApply, LV_ALIGN_BOTTOM_MID, 66, -8);
    ui_grad(ui_machineApply, COL_GRN_TOP, COL_GRN_BOT);
    lv_obj_set_style_bg_color(ui_machineApply, lv_color_hex(COL_GRY_BOT), LV_STATE_DISABLED);
    lv_obj_set_style_bg_grad_color(ui_machineApply, lv_color_hex(COL_GRY_BOT), LV_STATE_DISABLED);
    ui_machineModal = ov;
}

static void ui_make_reset_confirm() {
    lv_obj_t* ov = ui_overlay();
    lv_obj_t* box = ui_box(ov, 280, 150, COL_AMBER, COL_PANEL_BG, 3);
    lv_obj_set_style_pad_all(box, 0, 0);
    ui_resetTitle = ui_label(box, "Reset rate?", &lv_font_montserrat_20, COL_TEXT);
    lv_obj_align(ui_resetTitle, LV_ALIGN_TOP_MID, 0, 16);
    lv_obj_t* no = ui_button(box, 120, 44, "NO", &lv_font_montserrat_20, ui_reset_no_cb, NULL);
    lv_obj_align(no, LV_ALIGN_BOTTOM_MID, -66, -12);
    ui_grad(no, COL_GRY_TOP, COL_GRY_BOT);
    lv_obj_t* yes = ui_button(box, 120, 44, LV_SYMBOL_REFRESH "  YES", &lv_font_montserrat_20, ui_reset_yes_cb, NULL);
    lv_obj_align(yes, LV_ALIGN_BOTTOM_MID, 66, -12);
    ui_grad(yes, COL_RED_TOP, COL_RED_BOT);
    ui_resetModal = ov;
}

// Stall / idle / empty popups for one feeder. The dual screen names the
// machine in the title; the single screen keeps the short titles.
static void ui_make_feeder_modals(uint8_t i) {
    FeederView& v = views[i];
    char t1[32], t2[32], t3[32], m2[64], m3[112];
    if (isDual()) {
        snprintf(t1, sizeof(t1), LV_SYMBOL_WARNING " %s STALL", nameOf(v));
        snprintf(t2, sizeof(t2), LV_SYMBOL_POWER " %s IDLE", nameOf(v));
        snprintf(t3, sizeof(t3), LV_SYMBOL_WARNING " %s EMPTY", nameOf(v));
        snprintf(m2, sizeof(m2), "Out of %s and stopped. Reload and resume.", nounOf(v));
        snprintf(m3, sizeof(m3), "%s feeder is out - still running. Reload now, or it will auto-stop soon.",
                 isBullet(v.product) ? "Bullet" : "Case");
    } else {
        snprintf(t1, sizeof(t1), LV_SYMBOL_WARNING "  STALL");
        snprintf(t2, sizeof(t2), LV_SYMBOL_POWER "  AUTO SHUTOFF");
        snprintf(t3, sizeof(t3), LV_SYMBOL_WARNING "  FEEDER EMPTY");
        snprintf(m2, sizeof(m2), "Out of %s! Reload and continue.", nounOf(v));
        snprintf(m3, sizeof(m3), "Out of %s - still running. Reload now, or the feeder will auto-stop soon.", nounOf(v));
    }
    v.stallModal = ui_modal(0xFF1744, 0x3A0A0A, t1,
        isDual() ? "Jammed and could not self-clear. Clear the jam, then resume."
                 : "Motor jammed and could not self-clear. Clear the jam, then resume.",
        LV_SYMBOL_PLAY "  RESUME", ui_resume_cb, (void*)(intptr_t)i, 0x2E7D32);
    v.idleModal = ui_modal(0xFFB300, 0x1A1A1A, t2, m2,
        LV_SYMBOL_PLAY "  RESUME", ui_resume_cb, (void*)(intptr_t)i, 0x2E7D32);
    v.warnModal = ui_modal(0xFFD54F, 0x2A2410, t3, m3,
        LV_SYMBOL_OK "  DISMISS", ui_warn_dismiss_cb, (void*)(intptr_t)i, 0x6B7280);
}

static void ui_make_gear(lv_obj_t* bar, int h) {
    lv_obj_t* gear = lv_btn_create(bar);
    lv_obj_set_size(gear, 26, h - 2);
    lv_obj_align(gear, LV_ALIGN_RIGHT_MID, -2, 0);
    lv_obj_set_style_bg_opa(gear, LV_OPA_TRANSP, 0);
    lv_obj_set_style_shadow_width(gear, 0, 0);
    lv_obj_set_style_border_width(gear, 0, 0);
    lv_obj_set_style_pad_all(gear, 0, 0);
    lv_obj_set_ext_click_area(gear, 24);
    lv_obj_add_event_cb(gear, ui_settings_open_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* gi = ui_label(gear, LV_SYMBOL_SETTINGS, &lv_font_montserrat_16, COL_MUTED);
    lv_obj_center(gi);
}

// ---- Single screen --------------------------------------------------------------
static void ui_apply_speed_buttons(uint8_t percent) {
    for (int i = 0; i < NUM_SPEED_PRESETS; i++) {
        const bool sel = (SPEED_PRESETS[i] == percent);
        ui_grad(ui_speedBtns[i], sel ? COL_SEL_TOP : COL_SIL_TOP, sel ? COL_SEL_BOT : COL_SIL_BOT);
        lv_obj_set_style_border_color(ui_speedBtns[i], lv_color_hex(sel ? COL_SEL_BRD : COL_SIL_BRD), 0);
        lv_obj_set_style_text_color(ui_speedBtnLbls[i], lv_color_hex(sel ? COL_TEXT : COL_SIL_TXT), 0);
    }
}

static void createSingle(uint8_t i) {
    FeederView& v = views[i];
    lv_obj_t* scr = lv_scr_act();

    lv_obj_t* bar = ui_bar_strip(scr, 26, LV_ALIGN_TOP_MID);
    lv_obj_t* title = ui_label(bar, isBullet(v.product) ? "BULLET FEEDER PRO" : "CASE FEEDER PRO",
                               &lv_font_montserrat_20, COL_TEXT);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);
    ui_make_gear(bar, 26);

    // Speed panel: [FEED SPEED] value and bar; preset row below.
    lv_obj_t* sp = lv_obj_create(scr);
    lv_obj_set_size(sp, 312, 84);
    lv_obj_align(sp, LV_ALIGN_TOP_MID, 0, 28);
    ui_style_card(sp);
    lv_obj_t* tab = ui_tab(sp, "FEED SPEED", &lv_font_montserrat_10);
    lv_obj_align(tab, LV_ALIGN_TOP_LEFT, 5, 5);
    v.speedVal = ui_label(sp, "--%", &lv_font_montserrat_20, COL_TEXT);
    lv_obj_align(v.speedVal, LV_ALIGN_TOP_LEFT, 86, 3);
    v.speedBar = lv_bar_create(sp);
    lv_obj_set_size(v.speedBar, 150, 12);
    lv_obj_align(v.speedBar, LV_ALIGN_TOP_RIGHT, -8, 8);
    lv_bar_set_range(v.speedBar, 0, 100);
    lv_obj_set_style_bg_color(v.speedBar, lv_color_hex(COL_BAR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_color(v.speedBar, lv_color_hex(COL_SEL_TOP), LV_PART_INDICATOR);
    const int pbw = 56, pbh = 50, pgap = 5;
    const int px0 = (312 - (NUM_SPEED_PRESETS * pbw + (NUM_SPEED_PRESETS - 1) * pgap)) / 2;
    for (int k = 0; k < NUM_SPEED_PRESETS; k++) {
        lv_obj_t* b = lv_btn_create(sp);
        lv_obj_set_size(b, pbw, pbh);
        lv_obj_align(b, LV_ALIGN_TOP_LEFT, px0 + k * (pbw + pgap), 28);
        lv_obj_set_style_radius(b, 5, 0);
        lv_obj_set_style_border_width(b, 1, 0);
        lv_obj_set_style_shadow_width(b, 0, 0);
        lv_obj_add_event_cb(b, ui_speed_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)k);
        lv_obj_t* l = lv_label_create(b);
        lv_label_set_text_fmt(l, "%u", SPEED_PRESETS[k]);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
        lv_obj_center(l);
        ui_speedBtns[k] = b;
        ui_speedBtnLbls[k] = l;
    }
    ui_apply_speed_buttons(0);

    // Left column: rate over TEMP. Right: a double-height STOP/GO.
    const int cardW = 153, cardY = 116, colH = 94, cardGap = 4;
    const int cardH = (colH - cardGap) / 2;
    lv_obj_t* rc = lv_obj_create(scr);
    lv_obj_set_size(rc, cardW, cardH);
    lv_obj_align(rc, LV_ALIGN_TOP_LEFT, 4, cardY);
    ui_style_card(rc);
    lv_obj_t* rt = ui_tab(rc, rateTabOf(v), &lv_font_montserrat_10);
    lv_obj_align(rt, LV_ALIGN_TOP_LEFT, 5, 5);
    lv_obj_add_flag(rc, LV_OBJ_FLAG_CLICKABLE);                    // long-press to reset
    lv_obj_add_event_cb(rc, ui_rate_longpress_cb, LV_EVENT_LONG_PRESSED, (void*)(intptr_t)i);
    v.rateLbl = ui_label(rc, "0", &lv_font_montserrat_24, COL_TEXT);
    lv_obj_align(v.rateLbl, LV_ALIGN_BOTTOM_RIGHT, -8, -4);

    // Controller board temperature (the RP2040's own sensor).
    lv_obj_t* tc = lv_obj_create(scr);
    lv_obj_set_size(tc, cardW, cardH);
    lv_obj_align(tc, LV_ALIGN_TOP_LEFT, 4, cardY + cardH + cardGap);
    ui_style_card(tc);
    lv_obj_t* tt = ui_tab(tc, "TEMP", &lv_font_montserrat_10);
    lv_obj_align(tt, LV_ALIGN_TOP_LEFT, 5, 5);
    ui_tempLabel = ui_label(tc, "--", &lv_font_montserrat_24, COL_TEXT);
    lv_obj_align(ui_tempLabel, LV_ALIGN_BOTTOM_RIGHT, -8, -4);

    v.goBtn = lv_btn_create(scr);
    lv_obj_set_size(v.goBtn, cardW, colH);
    lv_obj_align(v.goBtn, LV_ALIGN_TOP_RIGHT, -4, cardY);
    lv_obj_set_style_radius(v.goBtn, 6, 0);
    ui_grad(v.goBtn, COL_GRN_TOP, COL_GRN_BOT);
    lv_obj_add_event_cb(v.goBtn, ui_go_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    v.goLbl = ui_label(v.goBtn, LV_SYMBOL_PLAY "  GO", &lv_font_montserrat_28, COL_TEXT);
    lv_obj_center(v.goLbl);

    // Status bar: feeder state | health | firmware
    lv_obj_t* st = ui_bar_strip(scr, 26, LV_ALIGN_BOTTOM_MID);
    v.stateLed = lv_led_create(st);
    lv_obj_set_size(v.stateLed, 12, 12);
    lv_obj_align(v.stateLed, LV_ALIGN_LEFT_MID, 8, 0);
    lv_led_set_color(v.stateLed, lv_color_hex(COL_MUTED));
    v.stateLbl = ui_label(st, "LINK...", &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align(v.stateLbl, LV_ALIGN_LEFT_MID, 26, 0);
    ui_healthLbl = ui_label(st, "CONNECTING", &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align(ui_healthLbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t* fw = ui_label(st, "FW " FW_VERSION, &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align(fw, LV_ALIGN_RIGHT_MID, -8, 0);

    ui_make_feeder_modals(i);
}

// ---- Dual screen ----------------------------------------------------------------
static lv_obj_t* ui_step_btn(lv_obj_t* parent, const char* sym, lv_align_t align, int x,
                             lv_event_cb_t cb, uint8_t feeder) {
    lv_obj_t* b = lv_btn_create(parent);
    lv_obj_set_size(b, 38, 38);
    lv_obj_align(b, align, x, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(COL_SIL_BRD), 0);
    ui_grad(b, COL_SIL_TOP, COL_SIL_BOT);
    lv_obj_set_ext_click_area(b, 4);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, (void*)(intptr_t)feeder);
    lv_obj_t* l = ui_label(b, sym, &lv_font_montserrat_20, COL_SIL_TXT);
    lv_obj_center(l);
    return b;
}

static void ui_make_half(uint8_t i, int x0) {
    FeederView& v = views[i];
    lv_obj_t* scr = lv_scr_act();
    const int w = 154;

    lv_obj_t* tab = ui_tab(scr, nameOf(v), &lv_font_montserrat_12);
    lv_obj_align(tab, LV_ALIGN_TOP_LEFT, x0, 28);
    v.stateLbl = ui_label(scr, "LINK...", &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align(v.stateLbl, LV_ALIGN_TOP_RIGHT, -(SCREEN_WIDTH - (x0 + w)), 30);
    v.stateLed = lv_led_create(scr);
    lv_obj_set_size(v.stateLed, 9, 9);
    lv_obj_align_to(v.stateLed, v.stateLbl, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_led_set_color(v.stateLed, lv_color_hex(COL_MUTED));

    lv_obj_t* sp = lv_obj_create(scr);
    lv_obj_set_size(sp, w, 48);
    lv_obj_align(sp, LV_ALIGN_TOP_LEFT, x0, 47);
    ui_style_card(sp);
    ui_step_btn(sp, LV_SYMBOL_MINUS, LV_ALIGN_LEFT_MID, 4, ui_minus_cb, i);
    ui_step_btn(sp, LV_SYMBOL_PLUS, LV_ALIGN_RIGHT_MID, -4, ui_plus_cb, i);
    lv_obj_t* cap = ui_label(sp, "FEED SPEED", &lv_font_montserrat_10, COL_MUTED);
    lv_obj_align(cap, LV_ALIGN_TOP_MID, 0, 2);
    v.speedVal = ui_label(sp, "--%", &lv_font_montserrat_20, COL_TEXT);
    lv_obj_align(v.speedVal, LV_ALIGN_TOP_MID, 0, 13);
    v.speedBar = lv_bar_create(sp);
    lv_obj_set_size(v.speedBar, 56, 5);
    lv_obj_align(v.speedBar, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_bar_set_range(v.speedBar, 0, 100);
    lv_obj_set_style_bg_color(v.speedBar, lv_color_hex(COL_BAR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_color(v.speedBar, lv_color_hex(COL_SEL_TOP), LV_PART_INDICATOR);

    lv_obj_t* rc = lv_obj_create(scr);
    lv_obj_set_size(rc, w, 32);
    lv_obj_align(rc, LV_ALIGN_TOP_LEFT, x0, 99);
    ui_style_card(rc);
    lv_obj_add_flag(rc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(rc, ui_rate_longpress_cb, LV_EVENT_LONG_PRESSED, (void*)(intptr_t)i);
    lv_obj_t* rt = ui_tab(rc, rateTabOf(v), &lv_font_montserrat_10);
    lv_obj_align(rt, LV_ALIGN_LEFT_MID, 4, 0);
    v.rateLbl = ui_label(rc, "0", &lv_font_montserrat_20, COL_TEXT);
    lv_obj_align(v.rateLbl, LV_ALIGN_RIGHT_MID, -6, 0);

    v.goBtn = lv_btn_create(scr);
    lv_obj_set_size(v.goBtn, w, 76);
    lv_obj_align(v.goBtn, LV_ALIGN_TOP_LEFT, x0, 135);
    lv_obj_set_style_radius(v.goBtn, 6, 0);
    ui_grad(v.goBtn, COL_GRN_TOP, COL_GRN_BOT);
    lv_obj_add_event_cb(v.goBtn, ui_go_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    v.goLbl = ui_label(v.goBtn, LV_SYMBOL_PLAY "  GO", &lv_font_montserrat_22, COL_TEXT);
    lv_obj_center(v.goLbl);

    ui_make_feeder_modals(i);
}

static void createDual() {
    lv_obj_t* scr = lv_scr_act();
    lv_obj_t* bar = ui_bar_strip(scr, 24, LV_ALIGN_TOP_MID);
    ui_tempLabel = ui_label(bar, "--", &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align(ui_tempLabel, LV_ALIGN_LEFT_MID, 8, 0);
    lv_obj_t* title = ui_label(bar, "DUAL FEEDER PRO", &lv_font_montserrat_16, COL_TEXT);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);
    ui_make_gear(bar, 24);

    lv_obj_t* div = lv_obj_create(scr);
    lv_obj_set_size(div, 2, 186);
    lv_obj_align(div, LV_ALIGN_TOP_MID, 0, 27);
    lv_obj_set_style_bg_color(div, lv_color_hex(COL_PANEL_BRD), 0);
    lv_obj_set_style_border_width(div, 0, 0);
    lv_obj_set_style_radius(div, 0, 0);

    ui_make_half(0, 3);
    ui_make_half(1, 163);

    lv_obj_t* st = ui_bar_strip(scr, 24, LV_ALIGN_BOTTOM_MID);
    ui_linkLed = lv_led_create(st);
    lv_obj_set_size(ui_linkLed, 10, 10);
    lv_obj_align(ui_linkLed, LV_ALIGN_LEFT_MID, 8, 0);
    ui_linkLbl = ui_label(st, "LINK...", &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align(ui_linkLbl, LV_ALIGN_LEFT_MID, 24, 0);
    ui_healthLbl = ui_label(st, "CONNECTING", &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align(ui_healthLbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t* fw = ui_label(st, "FW " FW_VERSION, &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align(fw, LV_ALIGN_RIGHT_MID, -8, 0);
}

// ---- Build / rebuild the whole screen ----------------------------------------------
static void forgetWidgets() {
    for (FeederView& v : views) {
        v.stateLed = v.stateLbl = v.speedVal = v.speedBar = v.rateLbl = v.goBtn = v.goLbl = nullptr;
        v.stallModal = v.idleModal = v.warnModal = nullptr;
        v.warnDismissed = v.warnShown = false;
        v.lastState = v.lastFlags = v.lastPct = v.lastHave = v.lastRate = -2;
        v.lastRateAt = 0;
    }
    ui_tempLabel = ui_healthLbl = ui_linkLed = ui_linkLbl = nullptr;
    for (lv_obj_t*& b : ui_speedBtns) b = nullptr;
    for (lv_obj_t*& b : ui_warnDd) b = nullptr;
    sharedLastLink = -1;
    sharedLastTemp = INT16_MIN + 1;
    sharedLastHealth[0] = '\0';
    shownTempLevel = 0;
    shownTempWho = -1;
}

static void buildUi(uint8_t shown, uint8_t product) {
    lv_obj_clean(lv_layer_top());
    lv_obj_clean(lv_scr_act());
    forgetWidgets();
    uiShown = shown;
    uiProduct = product;
    if (shown != MACHINE_DUAL) views[singleIdx()].product = product;

    lv_obj_t* scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(COL_SCR_BG), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    if (shown == MACHINE_DUAL) createDual();
    else                       createSingle(singleIdx());

    ui_tempModal = ui_modal(0xFF1744, 0x3A0A0A, "DRIVER HOT", "", LV_SYMBOL_OK "  DISMISS",
                            ui_temp_dismiss_cb, NULL, 0x6B7280, &ui_tempTitle, &ui_tempMsg, &ui_tempBox);
    ui_make_reset_confirm();
    ui_make_settings();
    ui_make_machine();

    Preferences p;   // come back up on this layout at the next power-on
    p.begin("ui", false);
    if (p.getUChar("shown", 0) != shown)     p.putUChar("shown", shown);
    if (p.getUChar("prod", 0xFF) != product) p.putUChar("prod", product);
    p.end();
}

// ---- Refresh ----------------------------------------------------------------------
static void refreshFeeder(uint8_t i) {
    FeederView& v = views[i];
    if (!v.goBtn) return;
    const bool linked = feederLinked(v);
    const int state = linked ? (int)v.st.state : -1;
    const int flags = linked ? (int)v.st.flags : -1;

    if (state != v.lastState || flags != v.lastFlags || (int)linked != v.lastHave) {
        v.lastState = state; v.lastFlags = flags; v.lastHave = (int)linked;
        uint32_t col; const char* txt;
        if (!linked)                                            { col = COL_REDST; txt = "NO LINK"; }
        else if (isDual() && !(v.st.flags & FLAG_DRIVER_READY)) { col = COL_REDST; txt = "CHECK DRIVER"; }
        else                                                    { col = stateColour(v.st.state); txt = stateText(v); }
        lv_led_set_color(v.stateLed, lv_color_hex(col));
        lv_led_on(v.stateLed);
        lv_obj_set_style_text_color(v.stateLbl, lv_color_hex(col), 0);
        lv_label_set_text(v.stateLbl, txt);
        if (isDual()) lv_obj_align_to(v.stateLed, v.stateLbl, LV_ALIGN_OUT_LEFT_MID, -5, 0);

        const bool showGo = !linked || v.st.state == STATE_PAUSED;
        lv_label_set_text(v.goLbl, showGo ? LV_SYMBOL_PLAY "  GO" : LV_SYMBOL_STOP "  STOP");
        ui_grad(v.goBtn, showGo ? COL_GRN_TOP : COL_RED_TOP, showGo ? COL_GRN_BOT : COL_RED_BOT);

        if (linked && v.st.state == STATE_FAULT) lv_obj_clear_flag(v.stallModal, LV_OBJ_FLAG_HIDDEN);
        else                                     lv_obj_add_flag(v.stallModal, LV_OBJ_FLAG_HIDDEN);
        if (linked && v.st.state == STATE_IDLE_SHUTOFF) lv_obj_clear_flag(v.idleModal, LV_OBJ_FLAG_HIDDEN);
        else                                            lv_obj_add_flag(v.idleModal, LV_OBJ_FLAG_HIDDEN);
    }

    // Empty-feeder notice follows the controller's flag, so reloading clears
    // it by itself; a dismiss holds until the condition clears.
    const bool warnNow = linked && (v.st.flags & FLAG_EMPTY_WARN) && v.st.state == STATE_RUNNING;
    if (!warnNow) v.warnDismissed = false;
    const bool showWarn = warnNow && !v.warnDismissed;
    if (showWarn != v.warnShown) {
        v.warnShown = showWarn;
        if (showWarn) lv_obj_clear_flag(v.warnModal, LV_OBJ_FLAG_HIDDEN);
        else          lv_obj_add_flag(v.warnModal, LV_OBJ_FLAG_HIDDEN);
    }

    if (linked && (int)v.st.speedPercent != v.lastPct) {
        v.lastPct = v.st.speedPercent;
        lv_label_set_text_fmt(v.speedVal, "%u%%", v.st.speedPercent);
        lv_bar_set_value(v.speedBar, v.st.speedPercent, LV_ANIM_ON);
        if (!isDual()) ui_apply_speed_buttons(v.st.speedPercent);
    }

    const uint32_t now = millis();
    if (now - v.lastRateAt >= 2000) {
        v.lastRateAt = now;
        const int target = computeRate(v, now);
        v.rateSmoothed += (target - v.rateSmoothed) * 0.25f;
        const int rate = ((int)(v.rateSmoothed + 0.5f) + RATE_ROUND / 2) / RATE_ROUND * RATE_ROUND;
        if (rate != v.lastRate) {
            v.lastRate = rate;
            char buf[16];
            fmt_commas(buf, (uint32_t)rate);
            lv_label_set_text(v.rateLbl, buf);
        }
    }
}

// The most urgent issue across the feeders on screen, for the status bar.
static void healthText(char* out, size_t n, uint32_t& col) {
    if (!controllerLinked()) { snprintf(out, n, "CONTROLLER OFFLINE"); col = COL_REDST; return; }
    int best = -1;
    snprintf(out, n, "SYSTEM OK");
    col = COL_GREEN;
    for (uint8_t i = 0; i < MAX_FEEDERS; i++) {
        if (!shownOnScreen(i)) continue;
        const FeederView& v = views[i];
        if (!feederLinked(v)) continue;
        int prio = 0; const char* txt = nullptr; uint32_t c = COL_GREEN;
        const uint8_t dt = v.st.driverTemp;
        if (!(v.st.flags & FLAG_DRIVER_READY))     { prio = 7; txt = "CHECK DRIVER";       c = COL_REDST; }
        else if (v.st.state == STATE_FAULT)        { prio = 7; txt = "FAULT";              c = COL_REDST; }
        else if (dt >= DRIVER_TEMP_SHUTDOWN)       { prio = 6; txt = "DRIVER OVERHEATED";  c = COL_REDST; }
        else if (dt >= DRIVER_TEMP_150)            { prio = 5; txt = "DRIVER OVERHEATING"; c = COL_REDST; }
        else if (dt >= DRIVER_TEMP_120)            { prio = 4; txt = "DRIVER HOT";         c = COL_AMBER; }
        else if (v.st.state == STATE_IDLE_SHUTOFF) { prio = 3; txt = "IDLE SHUTOFF";       c = COL_AMBER; }
        else if (!isDual() && v.st.state == STATE_PAUSED) { prio = 1; txt = "PAUSED";      c = COL_PURPLE; }
        if (txt && prio > best) {
            best = prio;
            col = c;
            if (isDual()) snprintf(out, n, "%s %s", nameOf(v), txt);
            else          snprintf(out, n, "%s", txt);
        }
    }
}

static void refreshShared() {
    const bool linked = controllerLinked();
    if (ui_linkLed && (int)linked != sharedLastLink) {
        sharedLastLink = (int)linked;
        lv_led_set_color(ui_linkLed, lv_color_hex(linked ? COL_GREEN : COL_REDST));
        lv_led_on(ui_linkLed);
        lv_obj_set_style_text_color(ui_linkLbl, lv_color_hex(linked ? COL_GREEN : COL_REDST), 0);
        lv_label_set_text(ui_linkLbl, linked ? "LINKED" : "NO LINK");
    }

    char h[40];
    uint32_t hcol;
    healthText(h, sizeof(h), hcol);
    if (strcmp(h, sharedLastHealth) != 0) {
        strncpy(sharedLastHealth, h, sizeof(sharedLastHealth) - 1);
        lv_obj_set_style_text_color(ui_healthLbl, lv_color_hex(hcol), 0);
        lv_label_set_text(ui_healthLbl, h);
    }

    // Board temperature (every status frame carries the same board reading).
    int t10 = BOARD_TEMP_UNKNOWN;
    for (uint8_t i = 0; i < MAX_FEEDERS; i++) {
        const FeederView& v = views[i];
        if (shownOnScreen(i) && feederLinked(v) && v.st.boardTempC10 != BOARD_TEMP_UNKNOWN) { t10 = v.st.boardTempC10; break; }
    }
    const int shown = (t10 == BOARD_TEMP_UNKNOWN) ? BOARD_TEMP_UNKNOWN : (t10 >= 0 ? (t10 + 5) / 10 : (t10 - 5) / 10);
    if (shown != sharedLastTemp) {
        sharedLastTemp = shown;
        if (shown == BOARD_TEMP_UNKNOWN) lv_label_set_text(ui_tempLabel, "--");
        else if (isDual()) lv_label_set_text_fmt(ui_tempLabel, LV_SYMBOL_CHARGE " %d\xC2\xB0" "C", shown);
        else               lv_label_set_text_fmt(ui_tempLabel, "%d\xC2\xB0" "C", shown);
    }

    // Driver temperature popup: the hottest driver's band; returns only hotter.
    uint8_t worst = 0;
    int who = -1;
    for (uint8_t i = 0; i < MAX_FEEDERS; i++) {
        if (shownOnScreen(i) && feederLinked(views[i]) && views[i].st.driverTemp > worst) {
            worst = views[i].st.driverTemp;
            who = i;
        }
    }
    if (worst == DRIVER_TEMP_OK) tempAckLevel = 0;
    const uint8_t want = (worst > tempAckLevel) ? worst : 0;
    if (want == shownTempLevel && (want == 0 || who == shownTempWho)) return;
    if (want) {
        static const char* const titles[] = { "", "DRIVER HOT", "DRIVER HOT", "OVERHEATING", "OVERHEATING", "DRIVER OFF" };
        static const char* const msgs[] = { "",
            "%s passed 120\xC2\xB0" "C. Still running - check the fan and airflow.",
            "%s passed 143\xC2\xB0" "C. Check the fan now - it shuts off near 150\xC2\xB0" "C.",
            "%s passed 150\xC2\xB0" "C and is about to shut itself off. Stop and let it cool.",
            "%s passed 157\xC2\xB0" "C. Stop the feeder and let it cool.",
            "%s overheated and switched itself off. Let it cool, check the fan, then resume." };
        const uint32_t accent = (want == DRIVER_TEMP_120) ? 0xFFD54F : (want == DRIVER_TEMP_143) ? 0xFFA726 : 0xFF1744;
        const uint32_t boxBg  = (want == DRIVER_TEMP_120) ? 0x2A2410 : (want == DRIVER_TEMP_143) ? 0x2A1A08 : 0x3A0A0A;
        const char* subject = !isDual() ? "Motor driver"
                            : isBullet(views[who].product) ? "Bullet motor driver" : "Case motor driver";
        char title[40], msg[140];
        if (isDual()) snprintf(title, sizeof(title), "%s %s", nameOf(views[who]), titles[want]);
        else          snprintf(title, sizeof(title), "%s  %s",
                               want == DRIVER_TEMP_SHUTDOWN ? LV_SYMBOL_POWER : LV_SYMBOL_WARNING, titles[want]);
        snprintf(msg, sizeof(msg), msgs[want], subject);
        lv_label_set_text(ui_tempTitle, title);
        lv_label_set_text(ui_tempMsg, msg);
        lv_obj_set_style_text_color(ui_tempTitle, lv_color_hex(accent), 0);
        lv_obj_set_style_border_color(ui_tempBox, lv_color_hex(accent), 0);
        lv_obj_set_style_bg_color(ui_tempBox, lv_color_hex(boxBg), 0);
        lv_obj_clear_flag(ui_tempModal, LV_OBJ_FLAG_HIDDEN);
        if (want > shownTempLevel && audioEnabled) {
            if (want >= DRIVER_TEMP_150) audio.playJamAlert(); else audio.playEmptyWarning();
        }
    } else {
        lv_obj_add_flag(ui_tempModal, LV_OBJ_FLAG_HIDDEN);
    }
    shownTempLevel = want;
    shownTempWho = who;
}

static void refreshUi() {
    if (isDual()) { refreshFeeder(0); refreshFeeder(1); }
    else          refreshFeeder(singleIdx());
    refreshShared();
}

// ---- Frames -----------------------------------------------------------------------
static void handleFrame(uint8_t command, const uint8_t* payload, uint8_t length) {
    lastValidFrameAt = millis();
    switch (command) {
        case EVT_STATUS: {
            if (length != sizeof(StatusPayload)) break;
            StatusPayload s;
            memcpy(&s, payload, sizeof(s));
            if (s.feederIndex >= MAX_FEEDERS) break;
            FeederView& v = views[s.feederIndex];
            v.st = s;
            v.product = s.productType;
            if (v.haveSeenCount && s.count != v.lastSeenCount) {
                uint16_t delta = (uint16_t)(s.count - v.lastSeenCount);
                if (delta > RATE_RING) delta = RATE_RING;
                for (uint16_t k = 0; k < delta; k++) recordFed(v, millis());
            }
            v.lastSeenCount = s.count;
            v.haveSeenCount = true;
            v.have = true;
            v.lastFrameAt = millis();
            // The layout this controller calls for; loop() rebuilds if it changed.
            wantShown   = (s.feederCount >= 2) ? MACHINE_DUAL : (uint8_t)(1U << s.feederIndex);
            wantProduct = (s.feederCount >= 2) ? PRODUCT_CASE : s.productType;
            break;
        }
        case EVT_COUNT: {
            if (length != 3 || payload[2] >= MAX_FEEDERS) break;
            FeederView& v = views[payload[2]];
            uint16_t count;
            memcpy(&count, payload, 2);
            if (!v.haveSeenCount || count != v.lastSeenCount) recordFed(v, millis());
            v.lastSeenCount = count;
            v.haveSeenCount = true;
            v.st.count = count;
            break;
        }
        case EVT_ALERT:
            // Sound only: the popups follow the status flags and states.
            if (length == 2 && audioEnabled) {
                switch (payload[0]) {
                    case ALERT_EMPTY_WARNING: audio.playEmptyWarning(); break;
                    case ALERT_IDLE_SHUTOFF:  audio.playIdleAlert();    break;
                    case ALERT_JAM_FAULT:     audio.playJamAlert();     break;
                    default: break;
                }
            }
            break;
        case RSP_NAK:
            // The controller refused a machine change (a feeder still moving).
            if (length == 1 && payload[0] == CMD_SET_MACHINE && ui_machineModal) {
                ui_machine_show("Controller refused. Stop every feeder and try again.");
                lv_obj_clear_flag(ui_machineModal, LV_OBJ_FLAG_HIDDEN);
            }
            break;
        default: break;
    }
}

static void processRxByte(uint8_t value) {
    switch (rx.stage) {
        case RxParser::WAIT_SOF: if (value == SOF) rx.stage = RxParser::READ_LEN; break;
        case RxParser::READ_LEN:
            if (value == 0 || value > MAX_PAYLOAD + 1) rx.stage = RxParser::WAIT_SOF;
            else { rx.length = value; rx.index = 0; rx.stage = RxParser::READ_BODY; }
            break;
        case RxParser::READ_BODY:
            rx.body[rx.index++] = value;
            if (rx.index >= rx.length) rx.stage = RxParser::CRC_LOW;
            break;
        case RxParser::CRC_LOW:  rx.receivedCrc = value; rx.stage = RxParser::CRC_HIGH; break;
        case RxParser::CRC_HIGH: rx.receivedCrc |= (uint16_t)value << 8; rx.stage = RxParser::WAIT_EOF; break;
        case RxParser::WAIT_EOF: {
            uint8_t crcData[MAX_PAYLOAD + 2];
            crcData[0] = rx.length;
            memcpy(&crcData[1], rx.body, rx.length);
            if (value == EOF_MARKER && crc16(crcData, rx.length + 1) == rx.receivedCrc) {
                handleFrame(rx.body[0], &rx.body[1], rx.length - 1);
            }
            rx.stage = RxParser::WAIT_SOF;
            break;
        }
    }
}

// ---- USB serial: `?` prints status --------------------------------------------------
static void printStatus() {
    Serial.println("=== Case Feeder Pro LCD ===");
    Serial.printf("  firmware:   %s\n", FW_VERSION);
    Serial.printf("  screen:     %s\n", isDual() ? "DUAL" : isBullet(uiProduct) ? "BULLET" : "CASE");
    Serial.printf("  controller: %s\n", controllerLinked() ? "linked" : "NO LINK");
    for (uint8_t i = 0; i < MAX_FEEDERS; i++) {
        const FeederView& v = views[i];
        if (!v.have) continue;
        Serial.printf("  %-6s      %s, state %u, speed %u%%, count %u\n", nameOf(v),
                      feederLinked(v) ? "linked" : "no data", v.st.state, v.st.speedPercent, v.st.count);
    }
}

static void serviceUsbSerial() {
    while (Serial.available()) {
        if (Serial.read() == '?') printStatus();
    }
}

// ---- Setup / loop -------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    uint8_t shown, product;
    {   // Restore the operator's choices and the last layout before showing anything.
        Preferences p;
        p.begin("ui", true);
        audioEnabled = p.getBool("alerts", true);
        audioVolume  = p.getUChar("vol", 2);
        lvgl_set_flipped(p.getBool("flip", SCREEN_FLIPPED != 0));
        shown   = p.getUChar("shown", MACHINE_CASE);
        product = p.getUChar("prod", PRODUCT_CASE);
        p.end();
    }
    if (shown == 0 || (shown & ~MACHINE_DUAL)) shown = MACHINE_CASE;
    views[0].product = PRODUCT_CASE;
    views[1].product = PRODUCT_BULLET;
    wantShown = shown;
    wantProduct = product;
    lvgl_driver_init();
    buildUi(shown, product);
    lv_timer_handler();
    audio.begin();
    audio.setVolume(audioVolume);
    ControllerSerial.begin(BAUD, SERIAL_8N1, PIN_CONTROLLER_RX, PIN_CONTROLLER_TX);
    sendFrame(CMD_GET_STATUS);
}

void loop() {
    while (ControllerSerial.available()) processRxByte((uint8_t)ControllerSerial.read());
    const uint32_t now = millis();
    if (now - lastPingAt >= 1000)          { lastPingAt = now;          sendFrame(CMD_PING); }
    if (now - lastStatusRequestAt >= 2000) { lastStatusRequestAt = now; sendFrame(CMD_GET_STATUS); }

    if (wantShown != uiShown || (wantShown != MACHINE_DUAL && wantProduct != uiProduct)) {
        buildUi(wantShown, wantProduct);
    }
    refreshUi();
    audio.service();
    lv_timer_handler();
    serviceUsbSerial();
}
