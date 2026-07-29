#include "piezo.h"
#include <math.h>

// ── Setup ─────────────────────────────────────────────────────────────────────
void PiezoDetector::begin() {
  analogReadResolution(12);
  _armed        = false;
  _spinBufReady = false;
  _peakADC      = 0;
  _peakTime_ms  = 0;
}

// ── Arm / disarm ──────────────────────────────────────────────────────────────
void PiezoDetector::arm(uint32_t swingStart_ms) {
  _armed        = true;
  _armTime_ms   = swingStart_ms;
  _peakADC      = 0;
  _peakTime_ms  = 0;
  _spinBufReady = false;
}

void PiezoDetector::disarm() {
  _armed = false;
}

// ── update() — call every loop tick while armed ───────────────────────────────
// Returns true when a final result (valid or invalid) is ready.
bool PiezoDetector::update(PiezoResult& result) {
  if (!_armed) return false;

  uint32_t now = millis();

  // Window expired with no hit → invalid
  if (now - _armTime_ms > PIEZO_WINDOW_MS) {
    result.valid         = false;
    result.peakADC       = 0;
    result.peakVoltage   = 0.0f;
    result.contactQuality = 0.0f;
    result.timestamp_ms  = now;
    _armed = false;
    return true;
  }

  int adc = analogRead(PIEZO_PIN);
  if (adc > _peakADC) {
    _peakADC     = adc;
    _peakTime_ms = now;
  }

  // Threshold crossed and peak has settled (5ms of no new peak)
  if (_peakADC >= PIEZO_THRESHOLD && (now - _peakTime_ms) >= 5) {

    // ── Immediately capture spin window before returning ──────────────────────
    _captureSpinWindow();

    float voltage = (_peakADC / 4095.0f) * 3.3f;

    // contactQuality: map ADC range [THRESHOLD..4095] → [0..1]
    float quality = (float)(_peakADC - PIEZO_THRESHOLD) /
                    (float)(4095 - PIEZO_THRESHOLD);
    if (quality > 1.0f) quality = 1.0f;

    result.valid          = true;
    result.peakADC        = _peakADC;
    result.peakVoltage    = voltage;
    result.contactQuality = quality;
    result.timestamp_ms   = _peakTime_ms;

    _armed = false;
    return true;
  }

  return false;
}

// ── _captureSpinWindow() ──────────────────────────────────────────────────────
// Samples the piezo at SPIN_SAMPLE_RATE_HZ for SPIN_SAMPLE_COUNT samples.
// We remove the DC offset (mean) so zero-crossings are meaningful.
void PiezoDetector::_captureSpinWindow() {
  int32_t sum = 0;
  for (int i = 0; i < SPIN_SAMPLE_COUNT; i++) {
    _spinBuf[i] = (int16_t)analogRead(PIEZO_PIN);
    sum += _spinBuf[i];
    delayMicroseconds(SPIN_SAMPLE_US);
  }
  // Remove DC offset
  int16_t mean = (int16_t)(sum / SPIN_SAMPLE_COUNT);
  for (int i = 0; i < SPIN_SAMPLE_COUNT; i++) {
    _spinBuf[i] -= mean;
  }
  _spinBufReady = true;
}

// ── _zeroCrossingFreq() ───────────────────────────────────────────────────────
// Counts zero-crossings in the buffer and converts to Hz.
float PiezoDetector::_zeroCrossingFreq() {
  if (!_spinBufReady) return 0.0f;

  int crossings = 0;
  for (int i = 1; i < SPIN_SAMPLE_COUNT; i++) {
    if ((_spinBuf[i - 1] < 0 && _spinBuf[i] >= 0) ||
        (_spinBuf[i - 1] >= 0 && _spinBuf[i] < 0)) {
      crossings++;
    }
  }
  // Each full cycle = 2 crossings
  float windowSec = (float)SPIN_SAMPLE_COUNT / (float)SPIN_SAMPLE_RATE_HZ;
  float freq = (crossings / 2.0f) / windowSec;
  return freq;
}

// ── _stringNatFreq() ─────────────────────────────────────────────────────────
// Estimates natural frequency of a string segment.
// f = (1 / 2L) * sqrt(T / μ)
// We use a fixed linear density μ for polyester string ~1.25 g/m = 0.00125 kg/m
// Tension converted from lb → N (1 lb = 4.448 N)
float PiezoDetector::_stringNatFreq(float tension_lb, float length_m) {
  const float MU_KG_M = 0.00125f;          // linear density, polyester ~1.25g/m
  float T_newtons = tension_lb * 4.448f;
  float f = (1.0f / (2.0f * length_m)) * sqrtf(T_newtons / MU_KG_M);
  return f;
}

// ── _freqToSpinRPM() ─────────────────────────────────────────────────────────
// Maps detected vibration frequency to spin RPM.
//
// Physics model:
//   - Topspin (attackAngle < 0): ball rolls up the string bed, primarily
//     exciting the mains. Detected freq closer to f_main → stronger topspin.
//   - Slice (attackAngle > 0): ball rolls across, primarily exciting crosses.
//     Detected freq closer to f_cross → stronger backspin.
//   - Flat: both excited equally, freq between the two.
//
// Empirical scaling: professional topspin ~3000 RPM at ~800 Hz main freq.
// We linearly scale: spinRPM = K * (detectedFreq / naturalFreq) * |attackAngle|
// K calibrated to ~3.75 RPM per (Hz/Hz) per degree of attack angle.
float PiezoDetector::_freqToSpinRPM(float freqHz, float attackAngle_deg,
                                     float mainTension_lb, float crossTension_lb) {
  if (freqHz < 10.0f) return 0.0f;   // no meaningful signal

  float f_main  = _stringNatFreq(mainTension_lb,  MAIN_LENGTH_M);
  float f_cross = _stringNatFreq(crossTension_lb, CROSS_LENGTH_M);

  float angle = attackAngle_deg;
  float spinRPM;

  if (angle < 0.0f) {
    // Topspin — compare to main natural frequency
    float ratio = freqHz / f_main;
    spinRPM = -1.0f * ratio * fabsf(angle) * 3.75f * 60.0f;
  } else if (angle > 0.0f) {
    // Slice / backspin — compare to cross natural frequency
    float ratio = freqHz / f_cross;
    spinRPM = +1.0f * ratio * fabsf(angle) * 3.75f * 60.0f;
  } else {
    // Flat — average of both
    float ratio = freqHz / ((f_main + f_cross) / 2.0f);
    spinRPM = ratio * 3.75f * 60.0f * 0.3f;  // low spin on flat shots
  }

  // Clamp to realistic tennis range: -6000 to +6000 RPM
  if (spinRPM >  6000.0f) spinRPM =  6000.0f;
  if (spinRPM < -6000.0f) spinRPM = -6000.0f;

  return spinRPM;
}

// ── analyzeSpinFreq() — public entry point ────────────────────────────────────
SpinResult PiezoDetector::analyzeSpinFreq(float attackAngle_deg,
                                           float mainTension_lb,
                                           float crossTension_lb) {
  SpinResult res;
  if (!_spinBufReady) {
    res.valid          = false;
    res.spinRPM        = 0.0f;
    res.dominantFreqHz = 0.0f;
    return res;
  }

  float freq = _zeroCrossingFreq();
  res.dominantFreqHz = freq;
  res.spinRPM        = _freqToSpinRPM(freq, attackAngle_deg,
                                       mainTension_lb, crossTension_lb);
  res.valid          = (freq > 10.0f);
  return res;
}
