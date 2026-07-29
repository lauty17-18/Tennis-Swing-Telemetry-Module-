#pragma once
#include <Arduino.h>

// ── Pin & thresholds ──────────────────────────────────────────────────────────
#define PIEZO_PIN           A0
#define PIEZO_THRESHOLD     200       // ADC counts (12-bit) to register impact
#define PIEZO_WINDOW_MS     800       // ms after swing start to listen for impact
#define PIEZO_CAPTURE_MS    50        // ms to wait for peak to settle after threshold

// ── Spin FFT capture ──────────────────────────────────────────────────────────
#define SPIN_SAMPLE_COUNT   64        // number of post-impact samples
#define SPIN_SAMPLE_RATE_HZ 2000      // 2 kHz → 32ms window
#define SPIN_SAMPLE_US      500       // 1,000,000 / SPIN_SAMPLE_RATE_HZ

// ── String defaults (user can override via BLE config characteristic) ─────────
#define DEFAULT_MAIN_TENSION_LB  55.0f   // lbs, throat-to-tip strings
#define DEFAULT_CROSS_TENSION_LB 53.0f   // lbs, perpendicular strings
#define MAIN_LENGTH_M            0.320f  // ~32 cm effective vibrating length
#define CROSS_LENGTH_M           0.250f  // ~25 cm effective vibrating length

// ── Result structs ────────────────────────────────────────────────────────────
struct PiezoResult {
  bool    valid;
  int     peakADC;
  float   peakVoltage;
  float   contactQuality;   // 0.0 – 1.0
  uint32_t timestamp_ms;
};

struct SpinResult {
  float   spinRPM;          // positive = topspin, negative = backspin/slice
  float   dominantFreqHz;   // raw detected frequency
  bool    valid;
};

// ── Class ─────────────────────────────────────────────────────────────────────
class PiezoDetector {
public:
  void  begin();

  // Impact detection
  void  arm(uint32_t swingStart_ms);
  bool  update(PiezoResult& result);
  void  disarm();
  bool  isArmed() const { return _armed; }

  // Spin analysis — call immediately after update() returns valid=true
  // attackAngle_deg: negative = topspin, positive = slice
  SpinResult analyzeSpinFreq(float attackAngle_deg,
                              float mainTension_lb  = DEFAULT_MAIN_TENSION_LB,
                              float crossTension_lb = DEFAULT_CROSS_TENSION_LB);

private:
  // Impact state
  bool     _armed;
  uint32_t _armTime_ms;
  int      _peakADC;
  uint32_t _peakTime_ms;

  // Spin sample buffer
  int16_t  _spinBuf[SPIN_SAMPLE_COUNT];
  bool     _spinBufReady;

  void     _captureSpinWindow();
  float    _zeroCrossingFreq();
  float    _stringNatFreq(float tension_lb, float length_m);
  float    _freqToSpinRPM(float freqHz, float attackAngle_deg,
                          float mainTension_lb, float crossTension_lb);
};
