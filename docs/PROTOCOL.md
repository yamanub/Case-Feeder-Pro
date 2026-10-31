# Controller protocol

How another controller talks to the Case Feeder Pro controller (the SKR Pico)
in place of the touchscreen: start and stop each feeder, set its speed,
clear faults, choose the machine, and read every feeder's state for its own
display.

The controller owns all motor and safety behaviour. The host only sends
requests and shows what the controller reports. Protocol version: **1**
(byte 0 of every status message).

1. [Wiring](#wiring)
2. [Frame format](#frame-format)
3. [Feeders and the machine](#feeders-and-the-machine)
4. [Commands (host to controller)](#commands-host-to-controller)
5. [Messages (controller to host)](#messages-controller-to-host)
6. [Status message](#status-message)
7. [Behaviour and timing](#behaviour-and-timing)
8. [Suggested host loop](#suggested-host-loop)
9. [Example frames](#example-frames)

---

## Wiring

The host connects to the SKR Pico's **Raspberry Pi UART connector**, the
5-pin JST-XH port the touchscreen normally uses (`5V 5V GND IO1 IO0`).

| SKR Pico | Host |
|---|---|
| `IO0` (controller TX) | host RX |
| `IO1` (controller RX) | host TX |
| `GND` | GND (required) |
| `5V` | only if the host is powered from the SKR Pico |

- **UART:** 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control.
- **Logic level:** 3.3 V. An RP2040, RP2350, ESP32 or Raspberry Pi connects
  directly; a 5 V microcontroller needs a level shifter on its TX line.
- The host replaces the touchscreen; only one device can use this port.

## Frame format

Every message in both directions is one frame:

| Byte | Field | Value |
|---|---|---|
| 0 | SOF | `0xAA` |
| 1 | LEN | number of bytes in CMD + PAYLOAD (1 to 49) |
| 2 | CMD | command or message code |
| 3 … | PAYLOAD | 0 to 48 bytes |
| next | CRC low | CRC-16, low byte first |
| next | CRC high | |
| last | EOF | `0x55` |

- **CRC:** CRC-16/CCITT-FALSE (polynomial `0x1021`, initial value `0xFFFF`,
  no reflection, no final XOR), computed over **LEN, CMD and PAYLOAD** (not
  SOF, the CRC itself or EOF). Check value: the ASCII string `123456789`
  gives `0x29B1`.
- **Byte order:** all multi-byte numbers are little-endian.
- A frame with a bad CRC, a missing EOF, or LEN of 0 or above 49 is dropped
  silently. Resynchronise by scanning for the next `0xAA`.

```c
uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}
```

## Feeders and the machine

One controller runs up to two feeders. Each has a fixed **feeder index**:

| Index | Feeder | Motor | Sensor |
|---|---|---|---|
| 0 | case feeder | X | Y-STOP |
| 1 | bullet feeder | E | Z-STOP |

The **machine** setting says which of them this controller runs, as a bit
mask: `1` = case only, `2` = bullet only, `3` = dual. Status messages are
sent only for feeders in the machine, and `feederCount` in each one says how
many there are.

**Addressing:** per-feeder commands end with **one extra byte, the feeder
index**. Without it, a command applies to the first feeder in the machine.
A command addressed to a feeder that is not in the machine is refused
(`NAK`). Hosts should always send the index.

## Commands (host to controller)

| CMD | Name | Payload | Reply |
|---|---|---|---|
| `0x20` | SET_SPEED_PERCENT | `uint8 percent` (0–100), `uint8 feeder` | ACK / NAK |
| `0x21` | SET_RUN | `uint8 run` (1 = run, 0 = stop), `uint8 feeder` | ACK / NAK |
| `0x22` | CLEAR_FAULT | `uint8 feeder` | ACK / NAK |
| `0x24` | SET_WARN_SECONDS | `uint16 seconds`, `uint8 feeder` | ACK / NAK |
| `0x26` | SET_MACHINE | `uint8 mask` (1, 2 or 3) | ACK / NAK |
| `0x50` | GET_STATUS | none | one STATUS per feeder |
| `0xF0` | PING | none | PONG |

Any other command code is answered with NAK.

**SET_SPEED_PERCENT:** sets the feed speed. The controller maps 0–100 % onto
the feeder's own speed range, so **0 % is the slowest running speed, not
stop**; use SET_RUN to stop. Values above 100 are refused. The speed is saved
and restored at power-up (see [flash saves](#behaviour-and-timing)). The
touchscreen offers 20, 40, 60, 80 and 100 %; any value works.

**SET_RUN:** `1` requests running; `0` stops the feeder at once and puts it
in PAUSED from any state, including FAULT and IDLE_SHUTOFF. A run request
does not leave FAULT or IDLE_SHUTOFF on its own; send CLEAR_FAULT to resume
from those.

**CLEAR_FAULT:** acts only in FAULT or IDLE_SHUTOFF: the feeder resumes
running if a run was requested, otherwise goes to PAUSED. In other states it
does nothing. It is always acknowledged when the feeder index is valid. This
is the RESUME action after a jam fault or an empty shutoff.

**SET_WARN_SECONDS:** the empty-feeder warning time: if nothing passes the
sensor for this long while running, the controller raises the empty warning.
`0` turns the warning off. Saved. The touchscreen offers 15–90 s.

**SET_MACHINE:** switches which feeders run. Refused unless every feeder in
the current machine is stopped (PAUSED, FAULT or IDLE_SHUTOFF). On success
every feeder is stopped, the setting is saved, and status messages for the
new machine follow at once.

**GET_STATUS / PING:** not acknowledged. GET_STATUS answers with one STATUS
message per feeder; PING answers with PONG.

## Messages (controller to host)

| CMD | Name | Payload | When |
|---|---|---|---|
| `0x80` | STATUS | 23 bytes, see [Status message](#status-message) | every 500 ms per feeder, on GET_STATUS, and right after a machine change |
| `0x81` | COUNT | `uint16 count`, `uint8 feeder` | each part that passes the sensor |
| `0x82` | FAULT | `uint8 1`, `uint8 feeder` | the feeder entered FAULT (jammed again right after a recovery) |
| `0x83` | ALERT | `uint8 type`, `uint8 feeder` | see alert types below |
| `0xA0` | ACK | `uint8 cmd` | a command was accepted |
| `0xA1` | NAK | `uint8 cmd` | a command was refused (bad value, wrong length, feeder not in the machine, machine change while running) |
| `0xAF` | PONG | `uint32` controller uptime in ms | reply to PING |

**Alert types:** the touchscreen plays a sound for each; a host can do the
same or ignore them, since the STATUS state and flags carry the same
information.

| Type | Name | Meaning |
|---|---|---|
| 0 | EMPTY_WARNING | nothing fed for the warning time; still running |
| 1 | IDLE_SHUTOFF | nothing fed for 2 minutes; stopped |
| 2 | JAM_FAULT | jam that could not clear; stopped in FAULT |

## Status message

CMD `0x80`, LEN `24`, payload 23 bytes, packed, little-endian:

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | u8 | protocolVersion | `1` |
| 1 | u8 | state | see states below |
| 2 | u8 | speedPercent | current speed setting, 0–100 |
| 3 | u8 | flags | see flags below |
| 4 | u8 | gearboxRatio | `51` |
| 5 | u16 | count | parts fed since power-up (wraps at 65535) |
| 7 | u16 | stepFrequencyHz | current motor step rate; 0 when stopped |
| 9 | u16 | sgResult | StallGuard load reading (lower = more load); updated while running |
| 11 | u32 | uptimeSeconds | controller uptime |
| 15 | u16 | warnSeconds | empty-warning time in force (0 = off) |
| 17 | i16 | boardTempC10 | controller board temperature in tenths of °C; `-32768` = unknown |
| 19 | u8 | driverTemp | motor driver temperature band, see below |
| 20 | u8 | feederIndex | 0 = case, 1 = bullet |
| 21 | u8 | feederCount | feeders in this machine (1 or 2) |
| 22 | u8 | productType | 0 = case feeder, 1 = bullet feeder |

**States:**

| Value | State | Meaning |
|---|---|---|
| 0 | RUNNING | feeding |
| 1 | BEAM_BLOCKED | a part is waiting at the sensor; the motor holds until it is taken |
| 2 | JAM_REVERSE | jam detected; reversing to clear it (about 2.3 s) |
| 3 | JAM_PAUSE | short pause (0.4 s) before feeding forward again |
| 4 | PAUSED | stopped; ready |
| 5 | FAULT | jammed again within 2 s of a recovery; needs the operator, then CLEAR_FAULT |
| 6 | IDLE_SHUTOFF | nothing fed for 2 minutes; stopped; reload, then CLEAR_FAULT |

**Flags:**

| Bit | Name | Meaning |
|---|---|---|
| 0 | BEAM_BLOCKED | the sensor sees a part (after its hold delay) |
| 1 | DRIVER_READY | the motor driver is configured and answering; if clear, check 24 V |
| 2 | HOST_CONNECTED | the controller has received a valid frame from the host in the last 3 s (`FLAG_LCD_CONNECTED` in the header) |
| 3 | STALL_ACTIVE | a possible stall is being confirmed |
| 5 | EMPTY_WARN | the empty warning is active right now; clears itself when parts flow again |

Bits 4, 6 and 7 are unused and read 0.

**Driver temperature bands:** 0 = below 120 °C, 1 = 120 °C pre-warning,
2 = 143 °C, 3 = 150 °C, 4 = 157 °C, 5 = the driver has shut itself off.

## Behaviour and timing

- **The controller runs the feeders on its own.** Beam hold, jam reversing,
  the repeat-jam FAULT, the empty warning and the idle shutoff all happen
  without the host. The host decides only run or stop, speed, resume, warning
  time and machine.
- **Link loss:** if the host stops sending, the feeders keep running in their
  current state. The HOST_CONNECTED flag clears 3 s after the last valid
  frame. Send something (PING is enough) at least once a second.
- **Power-up:** every feeder starts in PAUSED. The machine, speeds, warning
  times, sensor polarity and motor direction are restored from the
  controller's saved settings.
- **Flash saves:** SET_SPEED_PERCENT, SET_WARN_SECONDS and SET_MACHINE save
  the new value when it changes. A save pauses the controller's processor for
  about 50 ms; the motors keep stepping, but a frame arriving during that
  time can be lost. If a command gets no ACK or NAK within about 200 ms, send
  it again. Avoid streaming speed changes; send a new speed only when the
  operator changes it.
- **Counting:** each part produces one COUNT message. The `count` field in
  STATUS carries the same total, so a missed COUNT message corrects itself
  within 500 ms. A parts-per-hour rate is the host's job (the touchscreen
  uses a rolling 30-second window).
- **Commands are idempotent:** sending SET_RUN 1 to a running feeder, or the
  same speed twice, is harmless.

## Suggested host loop

1. At start-up, send **GET_STATUS**. The replies give the machine
   (`feederCount`, `feederIndex`, `productType`) and each feeder's settings
   and state, so the host's screen matches the controller.
2. Send **PING** every 1 s, and **GET_STATUS** every 2 s as a backstop. The
   controller also sends STATUS every 500 ms on its own.
3. Treat a feeder as offline if no STATUS for it has arrived for 3 s.
4. **Start / stop:** SET_RUN 1 / SET_RUN 0 with the feeder index.
5. **Resume after FAULT or IDLE_SHUTOFF:** CLEAR_FAULT with the feeder index.
6. **Show** the state, speed, rate, temperatures and warnings from STATUS;
   use ALERT and FAULT for immediate notices or sounds.
7. To change the machine, stop every feeder first, then send SET_MACHINE.

## Example frames

All bytes in hex.

| Command | Frame |
|---|---|
| GET_STATUS | `AA 01 50 CB 74 55` |
| PING | `AA 01 F0 21 C1 55` |
| SET_RUN, bullet feeder, run | `AA 03 21 01 01 FA 8D 55` |
| SET_RUN, case feeder, stop | `AA 03 21 00 00 EA AE 55` |
| SET_SPEED_PERCENT, case feeder, 60 % | `AA 03 20 3C 00 22 D9 55` |
| CLEAR_FAULT, bullet feeder | `AA 02 22 01 59 D2 55` |
| SET_WARN_SECONDS, case feeder, 45 s | `AA 04 24 2D 00 00 22 A1 55` |
| SET_MACHINE, dual | `AA 02 26 03 DF 3E 55` |

A STATUS message for the case feeder, running at 60 %, driver ready, host
connected, 1234 parts, 10872 Hz, StallGuard 400, up 3600 s, warning 30 s,
board 27.3 °C, driver OK, dual machine:

```
AA 18 80 01 00 3C 06 33 D2 04 78 2A 90 01 10 0E 00 00 1E 00 11 01 00 00 02 00 3D B1 55
```

| Bytes | Field | Value |
|---|---|---|
| `AA` | SOF | |
| `18` | LEN | 24 |
| `80` | CMD | STATUS |
| `01` | protocolVersion | 1 |
| `00` | state | RUNNING |
| `3C` | speedPercent | 60 |
| `06` | flags | DRIVER_READY + HOST_CONNECTED |
| `33` | gearboxRatio | 51 |
| `D2 04` | count | 1234 |
| `78 2A` | stepFrequencyHz | 10872 |
| `90 01` | sgResult | 400 |
| `10 0E 00 00` | uptimeSeconds | 3600 |
| `1E 00` | warnSeconds | 30 |
| `11 01` | boardTempC10 | 273 (27.3 °C) |
| `00` | driverTemp | OK |
| `00` | feederIndex | case feeder |
| `02` | feederCount | 2 |
| `00` | productType | case feeder |
| `3D B1` | CRC | |
| `55` | EOF | |

The authoritative definitions are in [`src/feeder_protocol.h`](../src/feeder_protocol.h),
and the controller's handling of each command is in
[`src/controller.cpp`](../src/controller.cpp).
