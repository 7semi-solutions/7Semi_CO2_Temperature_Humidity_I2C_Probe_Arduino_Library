#include <7Semi_CO2TH.h>

/** 7Semi CO2TH I2C Driver – Implementation
 * - Commands used:
 *   - 0x365B get_product_id          -> 18 bytes (6 words + CRCs)
 *   - 0x218B start_continuous_measurement
 *   - 0x3F86 stop_continuous_measurement (busy ~1.2 s)
 *   - 0xEC05 read_measurement        -> 12 bytes (4 words + CRCs)
 */

/* ---------- lifecycle ---------- */

/** Constructor
 * - Creates an uninitialized driver instance
 * - Call Begin(...) before any other public API
 */
CO2TH_7Semi::CO2TH_7Semi() {}

/* ---------- private helpers ---------- */

/** txRx_
 * - Sends one 16-bit big-endian command, then optionally reads rxLen bytes
 * - Adds an optional delay between write and read for command think-time
 * - Returns NO_ERROR on success or an error code on failure
 */
err_t CO2TH_7Semi::txRx_(const uint16_t tx,
                         uint8_t* rx, size_t rxLen, uint16_t delayMs) {
  if (!wire_) return ERR_STATE;

  wire_->beginTransmission(addr_);
  wire_->write((uint8_t)(tx >> 8));
  wire_->write((uint8_t)(tx & 0xFF));
  const uint8_t w = wire_->endTransmission();
  if (w != 0) return ERR_I2C;

  if (delayMs) delay(delayMs);

  if (rx && rxLen) {
    const size_t n = wire_->requestFrom((int)addr_, (int)rxLen);
    if (n != rxLen) return ERR_TIMEOUT;
    for (size_t i = 0; i < rxLen; ++i) rx[i] = wire_->read();
  }
  return NO_ERROR;
}

/** _sendCmd
 * - Sends a 16-bit big-endian command without reading a response
 * - Returns true on I2C ACK
 */
bool CO2TH_7Semi::_sendCmd(uint16_t cmd) {
  if (!wire_ || addr_ == 0) return false;
  wire_->beginTransmission(addr_);
  wire_->write((uint8_t)(cmd >> 8));
  wire_->write((uint8_t)(cmd & 0xFF));
  return (wire_->endTransmission() == 0);
}

/** crc8_
 * - Computes Sensirion CRC-8 (poly 0x31, init 0xFF) over a byte buffer
 * - Used to validate or generate CRC for each 16-bit word on the bus
 */
uint8_t CO2TH_7Semi::crc8_(const uint8_t* data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

/** be16_
 * - Loads a big-endian 16-bit value from a byte pointer
 */
uint16_t CO2TH_7Semi::be16_(const uint8_t* p) {
  return (uint16_t(p[0]) << 8) | p[1];
}

/** be32_
 * - Loads a big-endian 32-bit value from a byte pointer
 */
uint32_t CO2TH_7Semi::be32_(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}

/** be64_
 * - Loads a big-endian 64-bit value from a byte pointer
 */
uint64_t CO2TH_7Semi::be64_(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; ++i) v = (v << 8) | p[i];
  return v;
}


/** probeAt_
 * - Sets a candidate I2C address and probes by reading product ID
 * - Returns true if device responds and product ID matches CO2TH
 */
// bool CO2TH_7Semi::probeAt_(uint8_t address) {
//   addr_ = address;
//   uint32_t pid = 0;
//   uint64_t sn = 0;
//   err_t e = GetProductId(pid, sn);
//   return (e == NO_ERROR) && (pid == PRODUCT_ID);
// }

/** Begin
 * - Initializes with a Wire instance and target I2C address
 * - If i2cAddress == 0, scans 0x08..0x77 and verifies product ID
 * - Returns NO_ERROR on success; error code on failure
 */
// In your header, keep just this one declaration (remove the no-arg overload):
// err_t Begin(int sda = -1, int scl = -1, uint32_t freq = 400000, int8_t i2cPort = 0);

err_t CO2TH_7Semi::Begin(int sda, int scl, uint32_t freq, int8_t i2cPort) {
  /**
   - Initializes I2C bus (ESP32: Wire/Wire1 + optional pins; others: Wire)
   - Applies clock when supported
   - Probes PRODUCT_ID to confirm device
  */
#if defined(ARDUINO_ARCH_ESP32)
  TwoWire* bus = (i2cPort == 1) ? &Wire1 : &Wire;
  if (sda >= 0 && scl >= 0) {
    bus->begin(sda, scl, freq);
  } else {
    bus->begin();
    bus->setClock(freq);
  }
  wire_ = bus;
#else
  (void)sda; (void)scl; (void)i2cPort;
  wire_ = &Wire;
  wire_->begin();
  // If your core supports it, you can uncomment:
  // wire_->setClock(freq);
#endif

  measuring_ = false;
  addr_ = 0x64;

  uint32_t pid = 0;
  uint64_t sn  = 0;
  err_t e = GetProductId(pid, sn);
  if (e != NO_ERROR) return e;
  if (pid != PRODUCT_ID) return ERR_PARAM;

  return NO_ERROR;
}


// err_t CO2TH_7Semi::Begin() {
//   wire_ = &wire;
//   measuring_ = false;
//      wire_->begin();
//     addr_ = 0x64;
//     uint32_t pid = 0;
//     uint64_t sn = 0;
//     err_t e = GetProductId(pid, sn);
//     if (e != NO_ERROR) return e;
//     if (pid != PRODUCT_ID) return ERR_PARAM;
//     return NO_ERROR;
// }

// /**
//  - ESP32 pins/bus aware Begin
//  - On ESP32: optionally selects Wire/Wire1 and sets SDA/SCL/clock
//  - On others: uses Wire.begin() and proceeds normally
// */
// err_t CO2TH_7Semi::Begin( int sda, int scl,
//                          uint32_t freq, int8_t i2cPort) {
// #if defined(ARDUINO_ARCH_ESP32)
//   // Select bus
//   TwoWire* bus = (i2cPort == 1) ? &Wire1 : &Wire;

//   // Initialize with custom pins if provided, else defaults
//   if (sda >= 0 && scl >= 0) {
//     bus->begin(sda, scl, freq);
//   } else {
//     bus->begin();  // board defaults
//     bus->setClock(freq);
//   }
//   wire_ = bus;
// #else
//   // Non-ESP32: fall back to default Wire
//   Wire.begin();
//   wire_ = &Wire;
//   (void)sda;
//   (void)scl;
//   (void)freq;
//   (void)i2cPort;
// #endif

//   // Reuse the original logic (addressed or auto-scan)
//   measuring_ = false;

//     addr_ = 0x64;
//     uint32_t pid = 0;
//     uint64_t sn = 0;
//     err_t e = GetProductId(pid, sn);
//     if (e != NO_ERROR) return e;
//     if (pid != PRODUCT_ID) return ERR_PARAM;
//     return NO_ERROR;
// }
/** GetProductId
 * - Issues 0x365B and reads 6 words (MSB,LSB,CRC × 6)
 * - Parses 32-bit Product ID and 64-bit Serial Number
 * - Returns NO_ERROR on success; CRC/I2C/timeout on failure
 */
err_t CO2TH_7Semi::GetProductId(uint32_t& productId, uint64_t& serialNumber) {
  uint8_t buf[18]; /* 6 words: each MSB,LSB,CRC */

  err_t e = txRx_(GET_PRODUCT_ID, buf, sizeof(buf), 1);
  if (e != NO_ERROR) return e;

  uint16_t w[6];
  for (int i = 0; i < 6; ++i) {
    uint8_t* p = &buf[i * 3];
    if (crc8_(p, 2) != p[2]) return ERR_CRC;
    w[i] = be16_(p);
  }

  productId = (uint32_t(w[0]) << 16) | w[1];
  serialNumber = (uint64_t(w[2]) << 48) | (uint64_t(w[3]) << 32) | (uint64_t(w[4]) << 16) | (uint64_t(w[5]) << 0);
  return NO_ERROR;
}

/** StartContinuousMeasurement
 * - Starts continuous measurement (0x218B)
 * - Sets measuring_ true on success
 */
err_t CO2TH_7Semi::StartContinuousMeasurement() {
  err_t e = txRx_(START_CONTINUOUS_MEASUREMENT, nullptr, 0, 1);
  if (e == NO_ERROR) measuring_ = true;
  return e;
}

/** StopContinuousMeasurement
 * - Stops continuous measurement (0x3F86)
 * - Device is busy for ~1.2 s; waits then clears measuring_
 */
err_t CO2TH_7Semi::StopContinuousMeasurement() {
  err_t e = txRx_(STOP_CONTINUOUS_MEASUREMENT, nullptr, 0, 0);
  if (e != NO_ERROR) return e;
  delay(1200); /* device busy, then ready */
  measuring_ = false;
  return NO_ERROR;
}

/** ReadMeasurement
 * - Valid only when measuring_ is true
 * - Issues 0xEC05 and reads 4 words with CRC: CO2, T, RH, Status
 * - Converts T to °C and RH to %
 */
err_t CO2TH_7Semi::ReadMeasurement(int16_t& co2ppm,
                                   float& temperature,
                                   float& humidity,
                                   uint16_t& status) {
  if (!measuring_) return ERR_STATE;

  uint8_t buf[12]; /* CO2, T, RH, Status (each MSB,LSB,CRC) */

  err_t e = txRx_(READ_MEASUREMENT, buf, sizeof(buf), 1);
  if (e != NO_ERROR) return e;

  for (int i = 0; i < 4; ++i) {
    uint8_t* p = &buf[i * 3];
    if (crc8_(p, 2) != p[2]) return ERR_CRC;
  }

  const uint16_t co2u = be16_(&buf[0]);
  const uint16_t traw = be16_(&buf[3]);
  const uint16_t rhraw = be16_(&buf[6]);
  const uint16_t st = be16_(&buf[9]);

  co2ppm = (int16_t)co2u;
  temperature = -45.0f + 175.0f * (float)traw / 65535.0f;
  humidity = -6.0f + 125.0f * (float)rhraw / 65535.0f;
  status = st;

  return NO_ERROR;
}

// /** setRhtCompensationRaw
//  * - Programs absolute ambient T/RH for algorithm via 0xE000
//  * - Payload: [T_MSB T_LSB T_CRC] [RH_MSB RH_LSB RH_CRC]
//  * - Caches last written raw values for sgetRhtCompensationC
//  * - Returns true on I2C ACK
//  */
// bool CO2TH_7Semi::setRhtCompensationRaw(uint16_t temp_raw, uint16_t rh_raw) {
//   if (!wire_ || addr_ == 0) return false;

//   const uint8_t t_msb = (uint8_t)(temp_raw >> 8);
//   const uint8_t t_lsb = (uint8_t)(temp_raw & 0xFF);
//   const uint8_t r_msb = (uint8_t)(rh_raw >> 8);
//   const uint8_t r_lsb = (uint8_t)(rh_raw & 0xFF);

//   uint8_t t_buf[2] = { t_msb, t_lsb };
//   uint8_t r_buf[2] = { r_msb, r_lsb };
//   const uint8_t t_crc = crc8_(t_buf, 2);
//   const uint8_t r_crc = crc8_(r_buf, 2);

//   wire_->beginTransmission(addr_);
//   wire_->write((uint8_t)(SET_RHT_COMPENSATION >> 8));
//   wire_->write((uint8_t)(SET_RHT_COMPENSATION & 0xFF));
//   wire_->write(t_msb);
//   wire_->write(t_lsb);
//   wire_->write(t_crc);
//   wire_->write(r_msb);
//   wire_->write(r_lsb);
//   wire_->write(r_crc);

//   const bool ok = (wire_->endTransmission() == 0);
//   if (ok) {
//     comp_t_raw_ = temp_raw;
//     comp_rh_raw_ = rh_raw;
//     comp_valid_ = true;
//   }
//   return ok;
// }

// /** clearRhtCompensation
//  * - Convenience helper to program 0 values for both T and RH
//  */
// bool CO2TH_7Semi::clearRhtCompensation() {
//   return setRhtCompensationRaw(0x0000, 0x0000);
// }

// /** setRhtCompensation
//  * - Convenience helper for integer fixed-point inputs
//  * - temp_offset_c_x100 in 0.01 °C units, rh_offset_pct_x100 in 0.01 % units
//  * - Pass-through to raw writer; adjust encoding as needed per datasheet
//  */
// bool CO2TH_7Semi::setRhtCompensation(int16_t temp_offset_c_x100, int16_t rh_offset_pct_x100) {
//   return setRhtCompensationRaw((uint16_t)temp_offset_c_x100,
//                                (uint16_t)rh_offset_pct_x100);
// }

// /** setRhtCompensationC
//  * - Engineering-units wrapper for 0xE000
//  * - Encodes T[°C] and RH[%] to ticks and calls raw writer
//  */
// bool CO2TH_7Semi::setRhtCompensationC(float t_c, float rh_pct) {
//   if (!wire_ || addr_ == 0) return false;
//   const uint16_t t_raw = (uint16_t)lroundf(((t_c + 45.0f) / 175.0f) * 65535.0f);
//   const uint16_t rh_raw = (uint16_t)lroundf(((rh_pct + 6.0f) / 125.0f) * 65535.0f);
//   return setRhtCompensationRaw(t_raw, rh_raw);
// }

// /** TempCFromTicks
//  * - Converts raw ticks to °C
//  */
// static inline float TempCFromTicks(uint16_t raw) {
//   return (raw / 65535.0f) * 175.0f - 45.0f;
// }

// /** RhPctFromTicks
//  * - Converts  raw ticks to %RH
//  */
// static inline float RhPctFromTicks(uint16_t raw) {
//   return (raw / 65535.0f) * 125.0f - 6.0f;
// }

// /** sgetRhtCompensationC
//  * - Returns cached compensation as engineering units
//  * - Returns false if nothing has been written since power-up
//  */
// bool CO2TH_7Semi::getRhtCompensationC(float& t_c, float& rh_pct) {
//   if (!comp_valid_) return false;
//   t_c = TempCFromTicks(comp_t_raw_);
//   rh_pct = RhPctFromTicks(comp_rh_raw_);
//   return true;
// }

// /** setPressureCompensationRaw
//  * - Programs pressure compensation via 0xE016
//  * - Payload: [P_MSB P_LSB P_CRC]
//  * - Returns true on I2C ACK
//  */
// bool CO2TH_7Semi::setPressureCompensationRaw(uint16_t pressure_raw) {
//   if (!wire_ || addr_ == 0) return false;

//   const uint8_t msb = (uint8_t)(pressure_raw >> 8);
//   const uint8_t lsb = (uint8_t)(pressure_raw & 0xFF);

//   uint8_t buf[2] = { msb, lsb };
//   const uint8_t crc = crc8_(buf, 2);

//   wire_->beginTransmission(addr_);
//   wire_->write((uint8_t)(SET_PRESSURE_COMPENSATION >> 8));
//   wire_->write((uint8_t)(SET_PRESSURE_COMPENSATION & 0xFF));
//   wire_->write(msb);
//   wire_->write(lsb);
//   wire_->write(crc);
//   return (wire_->endTransmission() == 0);
// }

/** measureSingleShot
 * - Triggers one single-shot measurement (0x219D)
 * - Optional wait_ms for conversion time if needed by caller
 */
bool CO2TH_7Semi::measureSingleShot(uint16_t wait_ms /*=0*/) {
  if (!_sendCmd(MEASURE_SINGLE_SHOT)) return false;
  if (wait_ms) delay(wait_ms);
  return true;
}

/** enterSleepMode
 * - Puts device into low-power sleep state (0x3650)
 */
bool CO2TH_7Semi::enterSleepMode() {
  return _sendCmd(ENTER_SLEEP_MODE);
}

/** exitSleepMode
 * - Wakes device from sleep (0x0000)
 * - Optional settle delay after wake
 */
bool CO2TH_7Semi::exitSleepMode(uint16_t settle_ms /*=1*/) {
  const bool ok = _sendCmd(EXIT_SLEEP_MODE);
  if (ok && settle_ms) delay(settle_ms);
  return ok;
}

/** performConditioning
 * - Starts internal conditioning routine (0x29BC)
 * - Optional wait for routine progress
 */
bool CO2TH_7Semi::performConditioning(uint16_t wait_ms /*=0*/) {
  const bool ok = _sendCmd(PERFORM_CONDITIONING);
  if (ok && wait_ms) delay(wait_ms);
  return ok;
}

/** softReset
 * - Issues soft reset (0x0006)
 * - Optional settle delay for re-initialization
 */
bool CO2TH_7Semi::softReset(uint16_t settle_ms /*=20*/) {
  const bool ok = _sendCmd(PERFORM_SOFT_RESET);
  if (ok && settle_ms) delay(settle_ms);
  return ok;
}

/** factoryReset
 * - Restores factory defaults (0x3632)
 * - Optional longer settle delay
 */
bool CO2TH_7Semi::factoryReset(uint16_t settle_ms /*=50*/) {
  const bool ok = _sendCmd(PERFORM_FACTORY_RESET);
  if (ok && settle_ms) delay(settle_ms);
  return ok;
}

/** selfTest
 * - Runs self-test (0x278C) and reads one 16-bit status word + CRC
 * - Returns true on success and places status in status_word
 */
bool CO2TH_7Semi::selfTest(uint16_t& status_word, uint16_t wait_ms /*=1*/) {
  if (!_sendCmd(PERFORM_SELF_TEST)) return false;
  if (wait_ms) delay(wait_ms);

  wire_->requestFrom((int)addr_, 3);
  if (wire_->available() < 3) return false;

  const uint8_t msb = wire_->read();
  const uint8_t lsb = wire_->read();
  const uint8_t crc = wire_->read();

  uint8_t buf[2] = { msb, lsb };
  if (crc8_(buf, 2) != crc) return false;

  status_word = (uint16_t)((msb << 8) | lsb);
  return true;
}

/** enableTestingMode
 * - Enters vendor testing mode (0x3FBC)
 */
bool CO2TH_7Semi::enableTestingMode() {
  return _sendCmd(ENABLE_TESTING_MODE);
}

/** disableTestingMode
 * - Exits vendor testing mode (0x3F3D)
 */
bool CO2TH_7Semi::disableTestingMode() {
  return _sendCmd(DISABLE_TESTING_MODE);
}

/** performForcedRecalibration
 * - Triggers forced recalibration (0x362F)
 * - Optional wait for routine progress
 */
bool CO2TH_7Semi::performForcedRecalibration(uint16_t wait_ms /*=0*/) {
  const bool ok = _sendCmd(PERFORM_FORCED_RECALIBRATION);
  if (ok && wait_ms) delay(wait_ms);
  return ok;
}

/* ---------- error strings ---------- */

/** ErrorToString
 * - Converts an error code to a short human-readable string
 * - Writes into caller buffer and ensures null-termination
 */
void ErrorToString(err_t err, char* buf, size_t bufSize) {
  if (!buf || bufSize == 0) return;
  const char* s = "OK";
  switch (err) {
    case NO_ERROR: s = "OK"; break;
    case ERR_I2C: s = "I2C error"; break;
    case ERR_CRC: s = "CRC error"; break;
    case ERR_TIMEOUT: s = "Timeout"; break;
    case ERR_PARAM: s = "Invalid parameter"; break;
    case ERR_STATE: s = "Invalid state"; break;
    default: s = "Unknown error"; break;
  }
  strncpy(buf, s, bufSize - 1);
  buf[bufSize - 1] = '\0';
}
