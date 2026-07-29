#pragma once
#include <bluefruit.h>
#include "swing.h"
#include "piezo.h"

#define BLE_SERVICE_UUID    "A1B2C3D4-E5F6-7890-ABCD-EF1234567890"
#define BLE_SWING_CHAR_UUID "A1B2C3D4-E5F6-7890-ABCD-EF1234567891"
#define BLE_DEVICE_NAME     "SwingTracker"

// ── BLE packet (36 bytes, packed) ─────────────────────────────────────────────
#pragma pack(1)
struct SwingPacket {
  float    racketHeadSpeed_mph;   // [0]
  float    estBallSpeed_mph;      // [4]
  float    smashFactor;           // [8]
  float    attackAngle_deg;       // [12]
  float    contactQuality;        // [16]  0.0–1.0
  float    impactVoltage;         // [20]
  float    spinRPM;               // [24]  + = topspin, - = slice
  float    stringFreqHz;          // [28]  dominant vibration frequency
  uint16_t swingDuration_ms;      // [32]
  uint8_t  piezoValid;            // [34]
  uint8_t  reserved;              // [35]  — 36 bytes total
};
#pragma pack()

class BleManager {
public:
  bool begin();
  void update();
  bool sendSwing(const SwingResult& swing, const PiezoResult& piezo,
                 float spinRPM = 0.0f, float stringFreqHz = 0.0f);
  bool isConnected() const;

private:
  SwingPacket buildPacket(const SwingResult& swing, const PiezoResult& piezo,
                          float spinRPM, float stringFreqHz);
};
