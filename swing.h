#pragma once

// ─────────────────────────────────────────────────────────────
//  swing.h  —  Swing detection & result types
//  Tennis Swing Tracker — Yonex EZone handle mount
//
//  Sensor orientation assumed:
//    X axis = along handle shaft (toward racket head)
//    Y axis = perpendicular to string face (into/out of strings)
//    Z axis = across handle width
//
//  Attack angle: angle of swing velocity vector relative to
//  horizontal at peak gyro. Negative = topspin, Positive = slice.
// ─────────────────────────────────────────────────────────────

// Tuning constants
#define LEVER_ARM_M         0.28f   // wrist-to-racket-head distance (m)
#define SWING_START_DPS     300.0f  // gyro magnitude to begin a swing (°/s)
#define SWING_END_DPS       100.0f  // gyro magnitude to end a swing (°/s)
#define SWING_MIN_MS        80      // ignore swings shorter than this (ms)
#define SWING_MAX_MS        1500    // ignore swings longer than this (ms)

// COR range — scales with contact quality (0.0–1.0)
// Poor contact: 1.10, perfect contact: 1.50
// Pro average ~1.32 corresponds to ~55% contact quality
#define COR_MIN             1.10f
#define COR_MAX             1.50f

// ─────────────────────────────────────────────────────────────
//  SwingResult  —  everything we know about one swing
// ─────────────────────────────────────────────────────────────
struct SwingResult {
  // --- Racket ---
  float peakRacketHeadSpeed_ms;   // m/s
  float peakRacketHeadSpeed_mph;  // mph

  // --- Ball (estimated without piezo) ---
  float estBallSpeed_ms;          // m/s
  float estBallSpeed_mph;         // mph

  // --- Quality ---
  float smashFactor;              // ball speed / racket head speed

  // --- Attack angle ---
  // Angle of racket head travel relative to horizontal at peak gyro
  // Negative = low-to-high (topspin), Positive = high-to-low (slice)
  // Typical values: topspin ~-6°, flat ~0°, slice ~+8°
  float attackAngle_deg;

  // --- Timing ---
  uint32_t swingDuration_ms;
  uint32_t timestamp_ms;

  // --- Placeholders (filled when piezo arrives) ---
  float spinRPM;
  float contactQuality;
  bool  piezoValid;
};

// ─────────────────────────────────────────────────────────────
//  SwingDetector  —  state machine fed raw gyro each loop tick
// ─────────────────────────────────────────────────────────────
class SwingDetector {
public:
  SwingDetector();

  // Call every loop tick with fused gyro (°/s) and accel (g)
  // Returns true when a complete swing result is ready
  bool update(float gx, float gy, float gz,
              float ax, float ay, float az,
              SwingResult& result);

  void reset();
  bool isSwinging() const { return _inSwing; }

private:
  enum State { IDLE, SWINGING };

  State    _state;
  bool     _inSwing;
  uint32_t _swingStart_ms;
  float    _peakGyroMag;
  float    _peakGx, _peakGy, _peakGz;  // gyro vector at peak
  float    _peakAx, _peakAy, _peakAz;  // accel vector at peak (for gravity ref)

  float gyroMagnitude(float gx, float gy, float gz);
  float calcAttackAngle(float gx, float gy, float gz,
                        float ax, float ay, float az);
  SwingResult buildResult(uint32_t now_ms);
};
