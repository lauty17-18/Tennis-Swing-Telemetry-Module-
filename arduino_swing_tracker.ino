#include <SPI.h>
#include <ICM42688.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include "swing.h"
#include "piezo.h"
#include "swing_ble.h"

// ── String tension ────────────────────────────────────────────────────────────
#define MAIN_TENSION_LB   55.0f
#define CROSS_TENSION_LB  53.0f

#define ICM_CS_PIN  7
#define SAMPLE_MS   10

ICM42688      icm(SPI, ICM_CS_PIN);
LSM6DS3       lsm(I2C_MODE, 0x6A);
SwingDetector swingDetector;
PiezoDetector piezoDetector;
BleManager    bleManager;

unsigned long lastSample   = 0;
bool          pendingSwing = false;
SwingResult   pendingResult;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("=== Tennis Swing Tracker ===");

  Wire.begin();

  // ICM-42688-P
  if (icm.begin() < 0) {
    Serial.println("ERR: ICM-42688-P not found");
  } else {
    icm.setGyroFS(ICM42688::dps2000);
    icm.setAccelFS(ICM42688::gpm16);
    icm.setGyroODR(ICM42688::odr100);
    icm.setAccelODR(ICM42688::odr100);
    Serial.println("OK: ICM-42688-P");
  }

  // LSM6DS3
  if (lsm.begin() != 0) {
    Serial.println("ERR: LSM6DS3 not found");
  } else {
    Serial.println("OK: LSM6DS3");
  }

  // Piezo
  piezoDetector.begin();
  Serial.println("OK: Piezo");

  // BLE
  if (!bleManager.begin()) {
    Serial.println("ERR: BLE init failed");
  } else {
    Serial.println("OK: BLE advertising as \"SwingTracker\"");
    Serial.println("Ready — swing to record a shot.");
  }
}

void loop() {
  bleManager.update();

  unsigned long now = millis();
  if (now - lastSample < SAMPLE_MS) return;
  lastSample = now;

  // ── Read IMU ───────────────────────────────────────────────────────────────
  icm.getAGT();
  float gx = icm.gyrX(), gy = icm.gyrY(), gz = icm.gyrZ();
  float ax = icm.accX(), ay = icm.accY(), az = icm.accZ();

  // ── Swing detection ────────────────────────────────────────────────────────
  SwingResult swingResult;
  bool swingComplete = swingDetector.update(gx, gy, gz, ax, ay, az, swingResult);

  if (swingDetector.isSwinging() && !piezoDetector.isArmed() && !pendingSwing) {
    piezoDetector.arm(now);
  }

  if (swingComplete) {
    pendingSwing  = true;
    pendingResult = swingResult;
  }

  // ── Piezo update ───────────────────────────────────────────────────────────
  if (pendingSwing && piezoDetector.isArmed()) {
    PiezoResult piezoResult;
    if (piezoDetector.update(piezoResult)) {

      // ── Spin analysis ──────────────────────────────────────────────────────
      SpinResult spinResult;
      if (piezoResult.valid) {
        spinResult = piezoDetector.analyzeSpinFreq(
          pendingResult.attackAngle_deg,
          MAIN_TENSION_LB,
          CROSS_TENSION_LB
        );
      } else {
        spinResult.valid          = false;
        spinResult.spinRPM        = 0.0f;
        spinResult.dominantFreqHz = 0.0f;
      }

      // ── Merge piezo into result ────────────────────────────────────────────
      pendingResult.spinRPM        = spinResult.spinRPM;
      pendingResult.piezoValid     = piezoResult.valid;
      pendingResult.contactQuality = piezoResult.contactQuality;

      // ── Recalculate ball speed and smash factor with real contact quality ──
      if (piezoResult.valid) {
        float effectiveCOR = COR_MIN + (COR_MAX - COR_MIN) * piezoResult.contactQuality;
        pendingResult.estBallSpeed_ms  = pendingResult.peakRacketHeadSpeed_ms * effectiveCOR;
        pendingResult.estBallSpeed_mph = pendingResult.estBallSpeed_ms * 2.23694f;
        pendingResult.smashFactor      = effectiveCOR;
      }

      // ── Print to Serial ────────────────────────────────────────────────────
      Serial.println("------------------------------");
      Serial.print("Racket Head Speed : "); Serial.print(pendingResult.peakRacketHeadSpeed_mph, 1); Serial.println(" mph");
      Serial.print("Est. Ball Speed   : "); Serial.print(pendingResult.estBallSpeed_mph, 1);        Serial.println(" mph");
      Serial.print("Smash Factor      : "); Serial.println(pendingResult.smashFactor, 2);
      Serial.print("Attack Angle      : "); Serial.print(pendingResult.attackAngle_deg, 1);
      Serial.println(pendingResult.attackAngle_deg < -3 ? "  (topspin)" :
                     pendingResult.attackAngle_deg >  3 ? "  (slice)"   : "  (flat)");
      Serial.print("Spin Rate         : "); Serial.print(pendingResult.spinRPM, 0); Serial.println(" RPM");
      Serial.print("String Freq       : "); Serial.print(spinResult.dominantFreqHz, 1); Serial.println(" Hz");
      Serial.print("Contact Quality   : "); Serial.print(pendingResult.contactQuality * 100.0f, 0); Serial.println("%");
      Serial.print("Impact Voltage    : "); Serial.print(piezoResult.peakVoltage, 2); Serial.println(" V");
      Serial.print("Swing Duration    : "); Serial.print(pendingResult.swingDuration_ms); Serial.println(" ms");
      Serial.println("------------------------------");

      // ── Send via BLE ───────────────────────────────────────────────────────
      if (bleManager.isConnected()) {
        bleManager.sendSwing(pendingResult, piezoResult,
                             spinResult.spinRPM, spinResult.dominantFreqHz);
        Serial.println("BLE: shot sent.");
      }

      pendingSwing = false;
    }
  } else if (pendingSwing && !piezoDetector.isArmed()) {
    pendingSwing = false;
  }
}
