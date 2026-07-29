#include "swing_ble.h"

static BLEService        swingService(BLE_SERVICE_UUID);
static BLECharacteristic swingCharacteristic(BLE_SWING_CHAR_UUID);

static void connectCallback(uint16_t conn_handle) {
  (void)conn_handle;
  Serial.println("BLE: phone connected");
}

static void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle; (void)reason;
  Serial.println("BLE: phone disconnected");
  Bluefruit.Advertising.start(0);
}

bool BleManager::begin() {
  if (!Bluefruit.begin()) return false;

  Bluefruit.setTxPower(4);
  Bluefruit.setName(BLE_DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  swingService.begin();

  swingCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  swingCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  swingCharacteristic.setFixedLen(sizeof(SwingPacket));
  swingCharacteristic.begin();

  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(swingService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  return true;
}

void BleManager::update() {
  // Reserved for future use (connection param updates, etc.)
}

bool BleManager::isConnected() const {
  return Bluefruit.connected();
}

SwingPacket BleManager::buildPacket(const SwingResult& swing, const PiezoResult& piezo,
                                     float spinRPM, float stringFreqHz) {
  SwingPacket pkt;
  pkt.racketHeadSpeed_mph = swing.peakRacketHeadSpeed_mph;
  pkt.estBallSpeed_mph    = swing.estBallSpeed_mph;
  pkt.smashFactor         = swing.smashFactor;
  pkt.attackAngle_deg     = swing.attackAngle_deg;
  pkt.contactQuality      = piezo.contactQuality;
  pkt.impactVoltage       = piezo.peakVoltage;
  pkt.spinRPM             = spinRPM;
  pkt.stringFreqHz        = stringFreqHz;
  pkt.swingDuration_ms    = (uint16_t)swing.swingDuration_ms;
  pkt.piezoValid          = piezo.valid ? 1 : 0;
  pkt.reserved            = 0;
  return pkt;
}

bool BleManager::sendSwing(const SwingResult& swing, const PiezoResult& piezo,
                            float spinRPM, float stringFreqHz) {
  if (!Bluefruit.connected()) return false;
  SwingPacket pkt = buildPacket(swing, piezo, spinRPM, stringFreqHz);
  return swingCharacteristic.notify((uint8_t*)&pkt, sizeof(pkt));
}
