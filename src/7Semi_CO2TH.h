#pragma once
#include <Arduino.h>
#include <Wire.h>

/** 7Semi CO2TH I2C Driver – Public API
 * - Supports auto I²C address detection (scan when address = 0)
 * - Verifies Product ID (0x0901018A) before reporting success
 * - Continuous and single-shot measurement helpers
 * - RHT and Pressure compensation (input/output encoding)
 * - Sleep / Reset / Conditioning / Self-test / Testing-mode helpers
 */

/** Error codes
 * - NO_ERROR      : success
 * - ERR_I2C       : I2C write/read failed (NACK or bus error)
 * - ERR_CRC       : CRC check failed on received data
 * - ERR_TIMEOUT   : fewer bytes than expected were received
 * - ERR_PARAM     : invalid parameter or unsupported configuration
 * - ERR_STATE     : API called in invalid state (e.g., not measuring)
 */
typedef enum : uint8_t {
  NO_ERROR = 0,
  ERR_I2C = 1,
  ERR_CRC = 2,
  ERR_TIMEOUT = 3,
  ERR_PARAM = 4,
  ERR_STATE = 5,
} err_t;

/** Command set
 * - Use big-endian 16-bit commands on the I²C bus
 * - CRC-8 (poly 0x31, init 0xFF) per 16-bit word on data phases
 */
#define GET_PRODUCT_ID 0x365B
#define START_CONTINUOUS_MEASUREMENT 0x218B
#define STOP_CONTINUOUS_MEASUREMENT 0x3F86
#define READ_MEASUREMENT 0xEC05
#define SET_RHT_COMPENSATION 0xE000
#define SET_PRESSURE_COMPENSATION 0xE016
#define MEASURE_SINGLE_SHOT 0x219D
#define ENTER_SLEEP_MODE 0x3650
#define EXIT_SLEEP_MODE 0x0000
#define PERFORM_CONDITIONING 0x29BC
#define PERFORM_SOFT_RESET 0x0006
#define PERFORM_FACTORY_RESET 0x3632
#define PERFORM_SELF_TEST 0x278C
#define ENABLE_TESTING_MODE 0x3FBC
#define DISABLE_TESTING_MODE 0x3F3D
#define PERFORM_FORCED_RECALIBRATION 0x362F

/** Device identity
 * - CO2TH_EXPECTED_PRODUCT_ID : value returned by GET_PRODUCT_ID for CO2TH
 */
#define PRODUCT_ID 0x0901018A

/** Error string helper
 * - Converts an error code to a short text description into caller buffer
 */
void ErrorToString(err_t err, char* buf, size_t bufSize);

/** CO2TH driver
 * - Create one instance per device
 * - Call Begin(...) before any other calls
 */
class CO2TH_7Semi {
public:
  /** Constructor
   * - Creates an uninitialized driver; no I²C is touched here
   */
  CO2TH_7Semi();

  /**
   - Begin and (on ESP32) configure the I²C pins/clock/bus
   - sda/scl: pass -1 to use board defaults
   - freq: I²C clock (Hz)
   - i2cPort: 0 -> Wire, 1 -> Wire1 (ESP32 only)
   - Works on non-ESP32 too; ignores pin args and just calls Wire.begin()
  */
err_t Begin(int sda = -1, int scl = -1, uint32_t freq = 400000, int8_t i2cPort = 0);

 

  /** GetProductId
   * - Sends GET_PRODUCT_ID (0x365B)
   * - Reads 6 words (MSB,LSB,CRC ×6)
   * - Outputs productId (u32) and serialNumber (u64)
   */
  err_t GetProductId(uint32_t& productId, uint64_t& serialNumber);

  /** StartContinuousMeasurement
   * - Sends START_CONTINUOUS_MEASUREMENT (0x218B)
   * - Sets measuring_ true on success
   */
  err_t StartContinuousMeasurement();

  /** StopContinuousMeasurement
   * - Sends STOP_CONTINUOUS_MEASUREMENT (0x3F86)
   * - Device is busy ~1.2 s; API waits then clears measuring_
   */
  err_t StopContinuousMeasurement();

  /** ReadMeasurement
   * - Valid only while measuring_
   * - Sends READ_MEASUREMENT (0xEC05)
   * - Returns CO2 [ppm] (u16), Temperature [°C], Humidity [%], Status (u16)
   */
  err_t ReadMeasurement(int16_t& co2ppm, float& temperature, float& humidity, uint16_t& status);

  // /** setRhtCompensationRaw
  //  * - Programs absolute ambient T/RH for algorithm via SET_RHT_COMPENSATION (0xE000)
  //  * - Payload on bus: [T_MSB T_LSB T_CRC] [RH_MSB RH_LSB RH_CRC]
  //  * - Values are raw words (see datasheet conversion)
  //  * - Cached internally for sgetRhtCompensationC
  //  */
  // bool setRhtCompensationRaw(uint16_t temp_raw, uint16_t rh_raw);

  // /** clearRhtCompensation
  //  * - Convenience: programs both T and RH words to zero
  //  */
  // bool clearRhtCompensation();

  // /** setRhtCompensation (fixed-point)
  //  * - Convenience for 0.01-unit fixed-point inputs
  //  * - temp_offset_c_x100 in 0.01 °C; rh_offset_pct_x100 in 0.01 %RH
  //  * - Pass-through to raw writer; adjust mapping if needed
  //  */
  // bool setRhtCompensation(int16_t temp_offset_c_x100, int16_t rh_offset_pct_x100);

  // /** setRhtCompensationC (engineering units)
  //  * - Encodes T[°C] and RH[%] to and writes via 0xE000
  //  * - Used to provide the algorithm with absolute ambient values
  //  */
  // bool setRhtCompensationC(float t_c, float rh_pct);

  // /** sgetRhtCompensationC
  //  * - Returns last-programmed compensation as °C and %RH
  //  * - CO2TH does not expose read-back; driver returns cached values
  //  */
  // bool getRhtCompensationC(float& t_c, float& rh_pct);

  // /** setPressureCompensationRaw
  //  * - Programs pressure compensation via SET_PRESSURE_COMPENSATION (0xE016)
  //  * - Payload on bus: [P_MSB P_LSB P_CRC]
  //  */
  // bool setPressureCompensationRaw(uint16_t pressure_raw);

  /** measureSingleShot
   * - Triggers a single measurement (0x219D)
   * - Caller may optionally wait for conversion time (wait_ms)
   */
  bool measureSingleShot(uint16_t wait_ms = 0);

  /** Power and procedures
   * - enterSleepMode          : ENTER_SLEEP_MODE (0x3650)
   * - exitSleepMode           : EXIT_SLEEP_MODE  (0x0000)
   * - performConditioning     : PERFORM_CONDITIONING (0x29BC)
   * - softReset               : PERFORM_SOFT_RESET   (0x0006)
   * - factoryReset            : PERFORM_FACTORY_RESET(0x3632)
   * - selfTest                : PERFORM_SELF_TEST    (0x278C), reads one word + CRC
   * - enableTestingMode       : ENABLE_TESTING_MODE  (0x3FBC)
   * - disableTestingMode      : DISABLE_TESTING_MODE (0x3F3D)
   * - performForcedRecalibration : PERFORM_FORCED_RECALIBRATION (0x362F)
   */
  bool enterSleepMode();
  bool exitSleepMode(uint16_t settle_ms = 1);
  bool performConditioning(uint16_t wait_ms = 0);
  bool softReset(uint16_t settle_ms = 20);
  bool factoryReset(uint16_t settle_ms = 50);
  bool selfTest(uint16_t& status_word, uint16_t wait_ms = 1);
  bool enableTestingMode();
  bool disableTestingMode();
  bool performForcedRecalibration(uint16_t wait_ms = 0);

private:
  /** probeAt_
   * - Sets addr_ and attempts to read Product ID for verification
   * - Returns true on matching CO2TH_EXPECTED_PRODUCT_ID
   */
  bool probeAt_(uint8_t address);

  /** txRx_
   * - Sends one 16-bit command, optional delay, optional read of rxLen bytes
   * - Returns NO_ERROR on success; error code otherwise
   */
  err_t txRx_(const uint16_t tx, uint8_t* rx, size_t rxLen, uint16_t delayMs);

  /** _sendCmd
   * - Sends one 16-bit command without reading a response
   * - Returns true on I²C ACK
   */
  bool _sendCmd(uint16_t cmd);

  /** Utilities
   * - crc8_ : Sensirion CRC-8 over a byte range (poly 0x31, init 0xFF)
   * - be16_/be32_/be64_ : big-endian loaders
   */
  uint8_t crc8_(const uint8_t* data, size_t len);
  uint16_t be16_(const uint8_t* p);
  uint32_t be32_(const uint8_t* p);
  uint64_t be64_(const uint8_t* p);

private:
  /** State
   * - wire_      : I²C bus handle
   * - addr_      : 7-bit I²C address
   * - measuring_ : true after StartContinuousMeasurement, false after Stop
   */
  TwoWire* wire_ = nullptr;
  uint8_t addr_ = 0;
  bool measuring_ = false;

  /** RHT compensation cache
   * - comp_t_raw_  : last programmed temperature word 
   * - comp_rh_raw_ : last programmed humidity word 
   * - comp_valid_  : true after a successful write via 0xE000
   */
  uint16_t comp_t_raw_ = 0;
  uint16_t comp_rh_raw_ = 0;
  bool comp_valid_ = false;
};
