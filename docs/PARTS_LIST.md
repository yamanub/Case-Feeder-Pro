# Parts list

Quantities are for one controller. "Per feeder" items are needed once for a
case or bullet feeder and twice for a dual setup.

## Controller and touchscreen

| Part | Qty | Notes | Link |
|---|---|---|---|
| BigTreeTech SKR Pico V1.0 | 1 | controller | [Amazon](https://www.amazon.com/dp/B09MVJ5XKH) |
| LCDWiki ES3C28P 2.8" ESP32-S3 display | 1 | touchscreen | [Amazon](https://www.amazon.com/dp/B0FKG7WRWV) |
| 24 V fan | 1 | driver cooling, on FAN1; required for dual | [Amazon](https://www.amazon.com/dp/B0757RPCN9) |
| WS2812 RGB LED | 1 | optional external status light | |

## Power

| Part | Qty | Notes | Link |
|---|---|---|---|
| 24 V power supply, 3 to 6 A | 1 | 3 A or more; the linked one is 3 A | [Amazon](https://www.amazon.com/dp/B078RY7BPL) |
| Power jack | 1 | | [Amazon](https://www.amazon.com/dp/B0D9B7WR23) |
| Power switch | 1 | | [Amazon](https://www.amazon.com/dp/B07S2QJKTX) |

## Feeder drive

| Part | Qty | Notes | Link |
|---|---|---|---|
| NEMA 17 stepper motor, 51:1 gearbox | per feeder | | [Amazon](https://www.amazon.com/dp/B00QEVLDVO) |
| Shaft coupler | per feeder | | [Amazon](https://www.amazon.com/dp/B08QVKMDC6) |

## Part sensor (one per feeder)

The case feeder works with either type; the bullet feeder uses the TCRT5000.

| Part | Qty | Notes | Link |
|---|---|---|---|
| TCRT5000 infrared reflective sensor | per feeder | IR sensor: one unit. Case or bullet feeder | [Amazon](https://www.amazon.com/dp/B00LZV1V10) |
| IR break-beam sensor | per case feeder | emitter + receiver pair. Case feeder only | [Amazon](https://www.amazon.com/dp/B0FPCR98F2) |

## Wiring

| Part | Qty | Notes | Link |
|---|---|---|---|
| JST-XH 2.54 mm connector kit, pre-crimped wires | 1 | the SKR Pico uses JST-XH connectors; cables need to be made (see the [build guide](BUILD_GUIDE.md#connectors-and-cables)) | [Amazon](https://www.amazon.com/dp/B0H3WK88ZL) |
| 2.54 mm female (Dupont) connectors | 4 | touchscreen cable ends at the Pi UART header | |
| Motor cable, 4-pin | per feeder | | |
| Sensor cable, 3-wire | per feeder | | |
| Touchscreen cable, 4-wire | 1 | 5V, GND, TX, RX; comes with the touchscreen (bare wires on one end) | |
| 470 Ω resistor | 2 | one in series with each touchscreen data wire | |
| Clip-on ferrite core | 1 | on the touchscreen cable | |
| USB-C cable | 1 | for flashing both boards | |

## Controller case hardware

| Part | Qty | Notes | Link |
|---|---|---|---|
| M3 x 12 mm button head screw | 4 | | |
| M3 x 4 mm button head screw | 4 | | |

## Case feeder hardware

Quantities for one case feeder.

| Part | Qty | Notes | Link |
|---|---|---|---|
| M4 x 10 mm screw | 31 | | [Amazon](https://www.amazon.com/dp/B0G19FJ4NV) (assorted lengths) |
| M4 x 16 mm screw | 6 | | [Amazon](https://www.amazon.com/dp/B0G19FJ4NV) (assorted lengths) |
| M4 nut | 24 | | [Amazon](https://www.amazon.com/dp/B0F8GGH5GX) |
| M4 heat-set insert | 11 | standard | |
| M3 x 12 mm button head screw | 6 | | |
| M3 x 6 mm button head screw | 2 | | |
| 3/4" PEX tubing | about 138 mm | feeder exit tube; cut to the length of `PEX Tube.stl` | |
| Magnet, 12 x 3 mm | 2 | feeder exit tube assembly | [Amazon](https://www.amazon.com/dp/B0CCVPVQ1L) |
| ABS sheet, 1/16" thick, 24" x 48" | 1 | outer wall of the feeder; cut from `ABS Outer v5 Flat.dxf` (932 x 250 mm) | [Amazon](https://www.amazon.com/dp/B0C7QG8463) |
| ABS sheet, 3/16" thick (optional) | 1 | to mill the one-piece feeder plate instead of printing the three-piece plate (see [printed parts](PRINTED_PARTS.md)); the reference build milled it from this sheet | |

## Bullet feeder hardware

Quantities for one bullet feeder.

| Part | Qty | Notes | Link |
|---|---|---|---|
| M4 x 10 mm screw | 7 | | [Amazon](https://www.amazon.com/dp/B0G19FJ4NV) (assorted lengths) |
| M4 x 16 mm screw | 6 | | [Amazon](https://www.amazon.com/dp/B0G19FJ4NV) (assorted lengths) |
| M4 nut | 12 | | [Amazon](https://www.amazon.com/dp/B0F8GGH5GX) |
| M3 x 12 mm button head screw | 3 | | |
| M3 x 6 mm button head screw | 3 | | |
| M3 nut | 3 | | |
| ABS sheet, 1/16" thick, 24" x 48" | 1 | outer wall of the feeder; cut from `ABS outer shell Wrap Flat.dxf` (487 x 127 mm) | [Amazon](https://www.amazon.com/dp/B0C7QG8463) |

The two outer walls together use about 932 x 250 mm plus 487 x 127 mm, so
one 24" x 48" ABS sheet covers both the case feeder and the bullet feeder,
with room to spare. A dual build needs only one sheet.
