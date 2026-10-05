// Case Feeder Pro - controller <-> LCD link protocol.
// Copyright (C) the Case Feeder Pro contributors. Licensed under GPL-3.0.
//
// Framed, CRC-checked messages over a 115200 8-N-1 UART:
//   SOF | LEN | CMD | payload... | CRC16 lo | CRC16 hi | EOF
// LEN counts CMD + payload. The CRC (CCITT, init 0xFFFF) covers LEN, CMD and
// the payload.
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace FeederProtocol {

constexpr uint8_t  SOF = 0xAA;
constexpr uint8_t  EOF_MARKER = 0x55;
constexpr uint8_t  MAX_PAYLOAD = 48;
constexpr uint32_t BAUD = 115200;
constexpr uint8_t  PROTOCOL_VERSION = 1;

enum Command : uint8_t {
    // LCD -> controller. Per-feeder commands end with the feeder index byte;
    // without it they apply to the controller's first feeder.
    CMD_SET_SPEED_PERCENT = 0x20,   // uint8 percent (0-100), [feeder]
    CMD_SET_RUN           = 0x21,   // uint8 1 = run, 0 = stop, [feeder]
    CMD_CLEAR_FAULT       = 0x22,   // [feeder] - resume after a fault or idle shutoff
    CMD_SET_WARN_SECONDS  = 0x24,   // uint16 empty-feeder warning time, [feeder]
    CMD_SET_MACHINE       = 0x26,   // uint8 MachineMask; refused while a feeder moves
    CMD_GET_STATUS        = 0x50,
    CMD_PING              = 0xF0,

    // controller -> LCD
    EVT_STATUS            = 0x80,   // StatusPayload, one frame per feeder
    EVT_COUNT             = 0x81,   // uint16 count, uint8 feeder - one part passed the beam
    EVT_FAULT             = 0x82,   // uint8 1, uint8 feeder
    EVT_ALERT             = 0x83,   // uint8 AlertType, uint8 feeder - the LCD sounds it
    RSP_ACK               = 0xA0,   // uint8 command
    RSP_NAK               = 0xA1,   // uint8 command
    RSP_PONG              = 0xAF,   // uint32 controller uptime, ms
};

enum State : uint8_t {
    STATE_RUNNING = 0,
    STATE_BEAM_BLOCKED,   // a part is waiting at the beam; the motor holds
    STATE_JAM_REVERSE,    // jam detected: backing up to clear it
    STATE_JAM_PAUSE,      // short pause before feeding forward again
    STATE_PAUSED,         // stopped by the operator
    STATE_FAULT,          // jammed again right after a recovery; needs the operator
    STATE_IDLE_SHUTOFF,   // nothing fed for the idle time; stopped
};

enum AlertType : uint8_t {
    ALERT_EMPTY_WARNING = 0,   // nothing fed for the warning time; still running
    ALERT_IDLE_SHUTOFF  = 1,   // stopped after staying empty
    ALERT_JAM_FAULT     = 2,   // jam it could not clear
};

enum StatusFlags : uint8_t {
    FLAG_BEAM_BLOCKED  = 1U << 0,
    FLAG_DRIVER_READY  = 1U << 1,
    FLAG_LCD_CONNECTED = 1U << 2,
    FLAG_STALL_ACTIVE  = 1U << 3,
    // The empty-feeder warning is active right now (motor still running). The
    // LCD shows its notice from this flag, so it clears itself on reload.
    FLAG_EMPTY_WARN    = 1U << 5,
};

enum ProductType : uint8_t {
    PRODUCT_CASE   = 0,
    PRODUCT_BULLET = 1,
};

// Which feeders a controller runs: one bit per feeder index. Each feeder has
// fixed ports (case: X driver + Y-STOP, bullet: E driver + Z-STOP).
enum MachineMask : uint8_t {
    MACHINE_CASE   = 1U << 0,
    MACHINE_BULLET = 1U << 1,
    MACHINE_DUAL   = MACHINE_CASE | MACHINE_BULLET,
};
constexpr uint8_t MAX_FEEDERS = 2;

// Motor driver temperature band, from the TMC2209's threshold flags.
enum DriverTemp : uint8_t {
    DRIVER_TEMP_OK       = 0,   // below 120 C
    DRIVER_TEMP_120      = 1,   // pre-warning
    DRIVER_TEMP_143      = 2,
    DRIVER_TEMP_150      = 3,
    DRIVER_TEMP_157      = 4,
    DRIVER_TEMP_SHUTDOWN = 5,   // the driver has switched its outputs off
};

constexpr int16_t BOARD_TEMP_UNKNOWN = INT16_MIN;

#pragma pack(push, 1)
struct StatusPayload {
    uint8_t  protocolVersion;
    uint8_t  state;             // State
    uint8_t  speedPercent;
    uint8_t  flags;             // StatusFlags
    uint8_t  gearboxRatio;
    uint16_t count;             // parts fed since power-up
    uint16_t stepFrequencyHz;
    uint16_t sgResult;          // StallGuard reading
    uint32_t uptimeSeconds;
    uint16_t warnSeconds;       // empty-feeder warning time (owned by the controller)
    int16_t  boardTempC10;      // controller chip temperature, tenths of a degree C
    uint8_t  driverTemp;        // DriverTemp
    uint8_t  feederIndex;       // which feeder this frame describes
    uint8_t  feederCount;       // how many feeders this controller runs
    uint8_t  productType;       // ProductType
};
#pragma pack(pop)
static_assert(sizeof(StatusPayload) <= MAX_PAYLOAD, "StatusPayload too large");

inline uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    while (length--) {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

}  // namespace FeederProtocol
