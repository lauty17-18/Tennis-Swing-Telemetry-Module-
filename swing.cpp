// ─────────────────────────────────────────────────────────────
//  swing.cpp  —  Swing detection & result calculations
//  Tennis Swing Tracker — Yonex EZone handle mount
// ─────────────────────────────────────────────────────────────

#include <Arduino.h>
#include "swing.h"

static float mpsToMph(float mps) { return mps * 2.23694f; }

// ── Constructor ───────────────────────────────────────────────
SwingDetector::SwingDetector() {
  reset();
}

void SwingDetector::reset() {
  _state         = IDLE;
  _inSwing       = false;
  _swingStart_ms = 0;
  _peakGyroMag   = 0.0f;
  _peakGx = _peakGy = _peakGz = 0.0f;
  _peakAx = _peakAy = _peakAz = 0.0f;
}

// ── Main update ───────────────────────────────────────────────
bool SwingDetector::update(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            SwingResult& result) {
  float mag    = gyroMagnitude(gx, gy, gz);
  uint32_t now = millis();

  switch (_state) {

    case IDLE:
      if (mag >= SWING_START_DPS) {
        _state         = SWINGING;
        _inSwing       = true;
        _swingStart_ms = now;
        _peakGyroMag   = mag;
        _peakGx = gx; _peakGy = gy; _peakGz = gz;
        _peakAx = ax; _peakAy = ay; _peakAz = az;
      }
      break;

    case SWINGING: {
      // Update peak — capture gyro AND accel at that moment
      if (mag > _peakGyroMag) {
        _peakGyroMag = mag;
        _peakGx = gx; _peakGy = gy; _peakGz = gz;
        _peakAx = ax; _peakAy = ay; _peakAz = az;
      }

      uint32_t elapsed = now - _swingStart_ms;

      if (elapsed > SWING_MAX_MS) {
        reset();
        break;
      }

      if (mag < SWING_END_DPS) {
        if (elapsed >= SWING_MIN_MS) {
          result = buildResult(now);
          reset();
          return true;
        } else {
          reset();
        }
      }
      break;
    }
  }

  return false;
}

// ── Build SwingResult ─────────────────────────────────────────
SwingResult SwingDetector::buildResult(uint32_t now_ms) {
  SwingResult r;

  // Racket head speed
  float omegaRad = _peakGyroMag * (PI / 180.0f);
  r.peakRacketHeadSpeed_ms  = omegaRad * LEVER_ARM_M;
  r.peakRacketHeadSpeed_mph = mpsToMph(r.peakRacketHeadSpeed_ms);

  // Estimated ball speed — contactQuality scales the effective COR
  // contactQuality is filled in later by piezo; use placeholder 0.5 here.
  // The main sketch overwrites estBallSpeed after piezo result is known.
  float effectiveCOR    = COR_MIN + (COR_MAX - COR_MIN) * 0.5f; // placeholder
  r.estBallSpeed_ms     = r.peakRacketHeadSpeed_ms * effectiveCOR;
  r.estBallSpeed_mph    = mpsToMph(r.estBallSpeed_ms);

  // Smash factor
  r.smashFactor = (r.peakRacketHeadSpeed_ms > 0.0f)
                  ? (r.estBallSpeed_ms / r.peakRacketHeadSpeed_ms)
                  : 0.0f;

  // Attack angle
  r.attackAngle_deg = calcAttackAngle(_peakGx, _peakGy, _peakGz,
                                       _peakAx, _peakAy, _peakAz);

  // Timing
  r.swingDuration_ms = now_ms - _swingStart_ms;
  r.timestamp_ms     = now_ms;

  // Placeholders
  r.spinRPM        = 0.0f;
  r.contactQuality = 0.0f;
  r.piezoValid     = false;

  return r;
}

// ── Attack Angle Calculation ──────────────────────────────────
//
//  Strategy: use the accel vector at peak gyro as a gravity
//  reference to find "down", then project the gyro (swing
//  velocity) vector onto the vertical plane and compute the
//  angle relative to horizontal.
//
//  X = along shaft toward head
//  Y = perpendicular to string face
//  Z = across handle width
//
//  The swing velocity at the racket head is dominated by
//  rotation around Z (wrist roll) for a forehand.
//  Attack angle = atan2 of the vertical component of that
//  rotation relative to the horizontal component.
//
float SwingDetector::calcAttackAngle(float gx, float gy, float gz,
                                      float ax, float ay, float az) {
  // Normalize gravity vector from accel
  float gNorm = sqrtf(ax*ax + ay*ay + az*az);
  if (gNorm < 0.1f) return 0.0f;  // can't determine orientation
  float gravX = ax / gNorm;
  float gravY = ay / gNorm;
  float gravZ = az / gNorm;

  // Gyro vector magnitude
  float gyroMag = gyroMagnitude(gx, gy, gz);
  if (gyroMag < 1.0f) return 0.0f;

  // Normalize gyro direction (swing axis)
  float swingX = gx / gyroMag;
  float swingY = gy / gyroMag;
  float swingZ = gz / gyroMag;

  // Project swing axis onto gravity direction
  // This gives the vertical component of the swing
  float verticalComponent = swingX*gravX + swingY*gravY + swingZ*gravZ;

  // Horizontal component = sqrt(1 - vertical^2)
  float horizontalComponent = sqrtf(max(0.0f, 1.0f - verticalComponent*verticalComponent));

  // Attack angle: angle between swing direction and horizontal plane
  // atan2(vertical, horizontal) — negative = upward (topspin)
  float angle = atan2f(verticalComponent, horizontalComponent) * (180.0f / PI);

  // Negate so topspin (upward swing) = negative, slice (downward) = positive
  return -angle;
}

// ── Gyro magnitude ────────────────────────────────────────────
float SwingDetector::gyroMagnitude(float gx, float gy, float gz) {
  return sqrtf(gx*gx + gy*gy + gz*gz);
}
