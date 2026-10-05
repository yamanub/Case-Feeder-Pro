/*
 * Audio Manager - I2S tone generation for the Case Feeder Pro LCD.
 * The 2.8" ESP32-S3 display board has an ES8311 codec and a speaker
 * amplifier. Uses the built-in ESP-IDF I2S driver, so no extra lib_deps.
 * Licensed under GPL-3.0.
 *
 * Onboard audio pins (fixed by the board, do not collide with TMC/beam/display):
 *   AUDIO_ENABLE = GPIO1 (amp enable, LOW = on)
 *   I2S MCLK = GPIO4, BCLK = GPIO5, WS = GPIO7, DOUT = GPIO8
 *
 * The ES8311 sits on the same I2C bus as the FT6336 touch panel; Wire.begin()
 * is already called by the touch driver during lvgl_driver_init(), so begin()
 * here must run AFTER the display/touch init.
 *
 * NON-BLOCKING: play*() only QUEUES a note sequence; service() - called every
 * loop() pass - streams samples into the I2S DMA with a zero timeout, so the
 * control loop (beam sampling, motor ramp, jam detection, UI) never stalls
 * while a tone is sounding. This matters most for the empty-feeder warning,
 * which fires while the motor is RUNNING.
 */

#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>

// ----- Onboard audio pins (LCDWiki 2.8" ESP32-S3 Display) -----
#define AUDIO_ENABLE_PIN  1     // amp enable, LOW = on
#define I2S_MCLK_PIN      4
#define I2S_BCLK_PIN      5
#define I2S_WS_PIN        7
#define I2S_DOUT_PIN      8
#define I2S_DIN_PIN      -1     // not used for playback

// ES8311 I2C address
#define ES8311_ADDR       0x18

// I2S configuration
#define I2S_PORT          I2S_NUM_0
#define I2S_SAMPLE_RATE   16000
#define I2S_BITS          I2S_BITS_PER_SAMPLE_16BIT
#define DMA_BUF_COUNT     8
#define DMA_BUF_LEN       64

class AudioManager {
private:
    struct Note { uint16_t freq; uint16_t ms; };   // freq 0 = rest (silence)

    bool  initialized = false;
    float volumeLevel = 0.20;   // see setVolume(); amplitude = 30000 * volumeLevel

    // Sequencer state. service() streams seq[] into the I2S DMA one chunk at a
    // time; seq == nullptr means idle. Sequences end with a short rest note so
    // the DMA drains to silence (no click) without any special-case code.
    const Note* seq          = nullptr;
    uint8_t     seqLen       = 0;
    uint8_t     seqIdx       = 0;
    uint32_t    noteRemaining = 0;   // sample frames left in the current note
    float       phase        = 0.0f;
    float       phaseInc     = 0.0f;
    Note        beepSeq[2]   = { {1000, 100}, {0, 20} };  // mutable slot for beep()

    bool writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(ES8311_ADDR);
        Wire.write(reg);
        Wire.write(val);
        return Wire.endTransmission() == 0;
    }

    // Minimal ES8311 setup for 16-bit I2S playback.
    void initES8311() {
        Serial.println("Configuring ES8311...");
        writeReg(0x00, 0x1F); delay(20);
        writeReg(0x00, 0x80); delay(20);   // power on
        writeReg(0x01, 0x3F);              // enable all clocks
        writeReg(0x02, 0x00);
        writeReg(0x03, 0x10);
        writeReg(0x04, 0x10);
        writeReg(0x05, 0x00);
        writeReg(0x06, 0x03);
        writeReg(0x07, 0x00);
        writeReg(0x08, 0xFF);
        writeReg(0x09, 0x0C);              // I2S 16-bit
        writeReg(0x0A, 0x0C);
        writeReg(0x0D, 0x01);              // power up DAC
        writeReg(0x0E, 0x02);
        writeReg(0x12, 0x00);
        writeReg(0x13, 0x10);
        writeReg(0x14, 0x1A);
        writeReg(0x37, 0x08);
        writeReg(0x32, 0xD0);              // DAC volume
        Serial.println("ES8311 configured");
    }

    bool initI2S() {
        i2s_config_t i2s_config = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
            .sample_rate = I2S_SAMPLE_RATE,
            .bits_per_sample = I2S_BITS,
            .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
            .communication_format = I2S_COMM_FORMAT_STAND_I2S,
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = DMA_BUF_COUNT,
            .dma_buf_len = DMA_BUF_LEN,
            .use_apll = true,
            .tx_desc_auto_clear = true,
            .fixed_mclk = I2S_SAMPLE_RATE * 384
        };
        i2s_pin_config_t pin_config = {
            .mck_io_num = I2S_MCLK_PIN,
            .bck_io_num = I2S_BCLK_PIN,
            .ws_io_num = I2S_WS_PIN,
            .data_out_num = I2S_DOUT_PIN,
            .data_in_num = I2S_DIN_PIN
        };
        if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
            Serial.println("I2S install FAILED");
            return false;
        }
        if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
            Serial.println("I2S pins FAILED");
            return false;
        }
        i2s_zero_dma_buffer(I2S_PORT);
        return true;
    }

    // Load the sequencer state for seq[seqIdx].
    void loadNote() {
        const Note& n = seq[seqIdx];
        noteRemaining = (uint32_t)I2S_SAMPLE_RATE * n.ms / 1000;
        phase         = 0.0f;
        phaseInc      = (2.0f * PI * n.freq) / I2S_SAMPLE_RATE;
    }

    // Queue a sequence for service() to stream. Replaces anything playing.
    void startSequence(const Note* s, uint8_t n) {
        if (!initialized || volumeLevel <= 0.0f || n == 0) return;
        seq    = s;
        seqLen = n;
        seqIdx = 0;
        loadNote();
    }

public:
    void begin() {
        Serial.println("\n=== Audio Init ===");
        pinMode(AUDIO_ENABLE_PIN, OUTPUT);
        digitalWrite(AUDIO_ENABLE_PIN, LOW);   // amp ON (active low)
        if (!initI2S()) {
            Serial.println("Audio FAILED - I2S error");
            return;
        }
        delay(50);
        initES8311();
        initialized = true;
        Serial.println("=== Audio Ready ===\n");
    }

    // Stream the pending sequence into the I2S DMA. Call every loop() pass.
    // Never blocks: writes with a zero timeout and stops the moment the DMA is
    // full, picking up from the same phase/sample next pass. Worst-case work per
    // call is bounded by the DMA capacity (8x64 frames = 32 ms of audio).
    void service() {
        if (!seq) return;
        int16_t buffer[DMA_BUF_LEN * 2];
        while (seq) {
            if (noteRemaining == 0) {
                if (++seqIdx >= seqLen) { seq = nullptr; break; }
                loadNote();
                continue;
            }
            uint32_t toWrite = noteRemaining < DMA_BUF_LEN ? noteRemaining
                                                           : (uint32_t)DMA_BUF_LEN;
            int16_t amplitude = (seq[seqIdx].freq == 0)
                                    ? 0 : (int16_t)(30000 * volumeLevel);
            float p = phase;
            for (uint32_t i = 0; i < toWrite; i++) {
                int16_t sample = amplitude ? (int16_t)(amplitude * sinf(p)) : 0;
                buffer[i * 2]     = sample;
                buffer[i * 2 + 1] = sample;
                p += phaseInc;
                if (p >= 2.0f * PI) p -= 2.0f * PI;
            }
            size_t bytesWritten = 0;
            i2s_write(I2S_PORT, buffer, toWrite * 4, &bytesWritten, 0);
            uint32_t accepted = bytesWritten / 4;   // stereo frames actually queued
            noteRemaining -= accepted;
            phase = fmodf(phase + phaseInc * accepted, 2.0f * PI);
            if (accepted < toWrite) break;   // DMA full - resume next pass
        }
    }

    // 0 = off (muted), 1 = low, 2 = med, 3 = high.
    void setVolume(int level) {
        if      (level <= 0) volumeLevel = 0.0f;
        else if (level == 1) volumeLevel = 0.08f;
        else if (level == 2) volumeLevel = 0.20f;
        else                 volumeLevel = 0.40f;
    }

    // Idle shutoff (feeder empty and stopped): gentle two-note descending chime.
    void playIdleAlert() {
        static const Note SEQ[] = { {784, 160}, {0, 20}, {523, 280}, {0, 30} };
        startSequence(SEQ, sizeof(SEQ) / sizeof(SEQ[0]));
    }

    // Empty-feeder warning (motor still running): a rising three-note chirp,
    // twice - distinct from the falling idle chime and the jam alarm.
    void playEmptyWarning() {
        static const Note SEQ[] = {
            {523, 110}, {659, 110}, {880, 150}, {0, 60},
            {523, 110}, {659, 110}, {880, 150}, {0, 30},
        };
        startSequence(SEQ, sizeof(SEQ) / sizeof(SEQ[0]));
    }

    // Jam fault: urgent alternating two-tone alarm.
    void playJamAlert() {
        static const Note SEQ[] = {
            {1200, 150}, {800, 150}, {1200, 150},
            {800, 150},  {1200, 150}, {800, 150}, {0, 30},
        };
        startSequence(SEQ, sizeof(SEQ) / sizeof(SEQ[0]));
    }

    // Generic test beep.
    void beep(uint16_t freq = 1000, uint16_t duration = 100) {
        beepSeq[0] = { freq, duration };
        startSequence(beepSeq, 2);
    }

    bool isPlaying()     { return seq != nullptr; }
    bool isInitialized() { return initialized; }
};

#endif // AUDIO_MANAGER_H
