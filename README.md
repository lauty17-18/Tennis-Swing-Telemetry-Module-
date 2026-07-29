# Tennis Swing Telemetry Module

Arduino firmware for a racket-mounted sensor that measures tennis swing mechanics in real time: racket head speed, estimated ball speed, spin rate, attack angle, and string contact quality — streamed over BLE and logged to Serial.

Built for a Yonex EZone handle mount, using a dual-IMU setup plus a piezo sensor to detect the moment of ball contact.

## Hardware

- **ICM-42688-P** IMU (SPI) — primary gyro/accel for swing detection
- **LSM6DS3** IMU (I2C) — secondary accel/gyro
- **Piezo disc** on `A0` — detects ball impact and string vibration for spin analysis
- **nRF52-based board** (uses Adafruit's `bluefruit.h` BLE stack)

## How it works

1. **Swing detection** (`swing.cpp`/`.h`) — a state machine watches gyro magnitude each 10 ms tick. A swing starts above `SWING_START_DPS` (300°/s) and ends below `SWING_END_DPS` (100°/s). At peak angular velocity it records the gyro/accel vectors and computes attack angle (negative = topspin, positive = slice) and peak racket head speed via the wrist-to-head lever arm (`LEVER_ARM_M`).
2. **Impact detection** (`piezo.cpp`/`.h`) — once a swing is underway, the piezo detector arms itself and watches `A0` for a voltage spike above `PIEZO_THRESHOLD` within `PIEZO_WINDOW_MS` of swing start, indicating ball contact.
3. **Spin analysis** — after impact, 64 samples of the piezo signal are captured at 2 kHz and analyzed for the dominant string vibration frequency (zero-crossing method), which is converted to spin RPM using main/cross string tension and attack angle.
4. **Ball speed & smash factor** — contact quality (derived from impact voltage) scales the coefficient of restitution between `COR_MIN` (1.10, poor contact) and `COR_MAX` (1.50, perfect contact), giving estimated ball speed and smash factor.
5. **Output** — each completed shot is printed to Serial and, if a BLE client is connected, sent as a 36-byte packed struct (`SwingPacket`) over a custom BLE service.

## BLE service

- Device name: `SwingTracker`
- Service UUID: `A1B2C3D4-E5F6-7890-ABCD-EF1234567890`
- Characteristic UUID: `A1B2C3D4-E5F6-7890-ABCD-EF1234567891`

`SwingPacket` layout (36 bytes, packed, little-endian floats):

| Offset | Field | Type |
|---|---|---|
| 0 | racketHeadSpeed_mph | float |
| 4 | estBallSpeed_mph | float |
| 8 | smashFactor | float |
| 12 | attackAngle_deg | float |
| 16 | contactQuality (0.0–1.0) | float |
| 20 | impactVoltage | float |
| 24 | spinRPM (+ topspin / − slice) | float |
| 28 | stringFreqHz | float |
| 32 | swingDuration_ms | uint16 |
| 34 | piezoValid | uint8 |
| 35 | reserved | uint8 |

## Serial output example

```
------------------------------
Racket Head Speed : 62.4 mph
Est. Ball Speed   : 78.1 mph
Smash Factor      : 1.32
Attack Angle      : -5.8  (topspin)
Spin Rate         : 1840 RPM
String Freq       : 168.3 Hz
Contact Quality   : 71%
Impact Voltage    : 2.14 V
Swing Duration    : 210 ms
------------------------------
```

## Setup

1. Install the Arduino libraries: `SPI`, `Wire`, `ICM42688`, `LSM6DS3` (SparkFun), and `Adafruit nRF52` board support (for `bluefruit.h`).
2. Wire the piezo disc to `A0`, ICM-42688-P to SPI with chip-select on pin `7`, and LSM6DS3 over I2C (address `0x6A`).
3. Open `arduino_swing_tracker.ino` in the Arduino IDE, select your nRF52 board, and flash.
4. Open the Serial Monitor at 115200 baud, or connect a BLE central to `SwingTracker` to receive live shot data.

## Tuning

Key constants live in `swing.h` and `piezo.h`:

- `LEVER_ARM_M` — wrist-to-racket-head distance used for head speed calculation
- `SWING_START_DPS` / `SWING_END_DPS` — gyro thresholds that bound a swing
- `SWING_MIN_MS` / `SWING_MAX_MS` — valid swing duration range
- `COR_MIN` / `COR_MAX` — coefficient of restitution range by contact quality
- `PIEZO_THRESHOLD` / `PIEZO_WINDOW_MS` — impact detection sensitivity and timing
- `MAIN_TENSION_LB` / `CROSS_TENSION_LB` (in the `.ino`) — string tension used for spin RPM calculation

## Files

| File | Purpose |
|---|---|
| `arduino_swing_tracker.ino` | Main loop: sensor reads, orchestration, Serial/BLE output |
| `swing.h` / `swing.cpp` | Swing state machine, head speed & attack angle |
| `piezo.h` / `piezo.cpp` | Impact detection & spin frequency analysis |
| `swing_ble.h` / `swing_ble.cpp` | BLE service setup and packet transmission |
