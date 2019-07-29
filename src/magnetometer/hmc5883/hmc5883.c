#include "magnetometer/hmc5883/hmc5883.h"
#include "i2c1/i2c1.h"

static uint8_t m_mode;
static void HMC5883L_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
static void HMC5883L_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
static void HMC5883L_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);

/** Power on and prepare for general usage.
 * This will prepare the magnetometer with default settings, ready for single-
 * use mode (very low power requirements). Default settings include 8-sample
 * averaging, 15 Hz data output rate, normal measurement bias, a,d 1090 gain (in
 * terms of LSB/Gauss). Be sure to adjust any settings you need specifically
 * after initialization, especially the gain settings if you happen to be seeing
 * a lot of -4096 values (see the datasheet for mor information).
 */
void hmc5883l_Init() {
  // write CONFIG_A register
  uint8_t tmp = (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1))
                | (HMC5883L_RATE_75 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1))
                | (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1));
  I2C1FinishCode err = i2c1_write_buff_sync(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, &tmp, 1);
  // write CONFIG_B register
  //  hmc5883l_SetGain(HMC5883L_GAIN_1370);
  hmc5883l_SetGain(HMC5883L_GAIN_1090);
  // write MODE register
  hmc5883l_SetMode(HMC5883L_MODE_CONTINUOUS);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool hmc5883l_TestConnection() {
  uint8_t tmp[3] = { 0 };
  i2c1_read_buff_sync(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_ID_A, tmp, 3);
  return (tmp[0] == 'H' && tmp[1] == '4' && tmp[2] == '3') ? true : false;
}
// CONFIG_A register

/** Get number of samples averaged per measurement.
 * @return Current samples averaged per measurement (0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_AVERAGING_8
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
uint8_t hmc5883l_GetSampleAveraging() {
  uint8_t tmp;
  HMC5883L_ReadBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, &tmp);
  return tmp;
}

/** Set number of samples averaged per measurement.
 * @param averaging New samples averaged per measurement setting(0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
void hmc5883l_SetSampleAveraging(uint8_t averaging) {
  HMC5883L_WriteBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, averaging);
}

/** \code
 * Get data output rate value.
 * The Table below shows all selectable output rates in continuous measurement
 * mode. All three channels shall be measured within a given output rate. Other
 * output rates with maximum rate of 160 Hz can be achieved by monitoring DRDY
 * interrupt pin in single measurement mode.
 *
 * Value | Typical Data Output Rate (Hz)
 * ------+------------------------------
 * 0     | 0.75
 * 1     | 1.5
 * 2     | 3
 * 3     | 7.5
 * 4     | 15 (Default)
 * 5     | 30
 * 6     | 75
 * 7     | Not used
 * \endcode
 * @return Current rate of data output to registers
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
uint8_t hmc5883l_GetDataRate() {
  uint8_t tmp;
  HMC5883L_ReadBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, &tmp);
  return tmp;
}

/** Set data output rate value.
 * @param rate Rate of data output to registers
 * @see HMC5883L_SetDataRate()
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
void hmc5883l_SetDataRate(uint8_t rate) {
  HMC5883L_WriteBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, rate);
}

/** Get measurement bias value.
 * @return Current bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
uint8_t hmc5883l_GetMeasurementBias() {
  uint8_t tmp;
  HMC5883L_ReadBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, &tmp);
  return tmp;
}

/** Set measurement bias value.
 * @param bias New bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
void hmc5883l_SetMeasurementBias(uint8_t bias) {
  HMC5883L_WriteBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, bias);
}

// CONFIG_B register

/** \code
 * Get magnetic field gain value.
 * The table below shows nominal gain settings. Use the "Gain" column to convert
 * counts to Gauss. Choose a lower gain value (higher GN#) when total field
 * strength causes overflow in one of the data output registers (saturation).
 * The data output range for all settings is 0xF800-0x07FF (-2048 - 2047).
 *
 * Value | Field Range | Gain (LSB/Gauss)
 * ------+-------------+-----------------
 * 0     | +/- 0.88 Ga | 1370
 * 1     | +/- 1.3 Ga  | 1090 (Default)
 * 2     | +/- 1.9 Ga  | 820
 * 3     | +/- 2.5 Ga  | 660
 * 4     | +/- 4.0 Ga  | 440
 * 5     | +/- 4.7 Ga  | 390
 * 6     | +/- 5.6 Ga  | 330
 * 7     | +/- 8.1 Ga  | 230
 * \endcode
 * @return Current magnetic field gain value
 * @see HMC5883L_GAIN_1090
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
uint8_t hmc5883l_GetGain() {
  uint8_t tmp;
  HMC5883L_ReadBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_B, HMC5883L_CRB_GAIN_BIT, HMC5883L_CRB_GAIN_LENGTH, &tmp);
  return tmp;
}

float hmc5883l_GetGainF() {
  uint8_t tmp = hmc5883l_GetGain();
  switch (tmp) {
    case HMC5883L_GAIN_1370:
      return 1370.f;
    case HMC5883L_GAIN_1090:
      return 1090.f;
    case HMC5883L_GAIN_820:
      return 820.0f;
    case HMC5883L_GAIN_660:
      return 660.0f;
    case HMC5883L_GAIN_440:
      return 440.0f;
    case HMC5883L_GAIN_390:
      return 390.0f;
    case HMC5883L_GAIN_330:
      return 330.0f;
    case HMC5883L_GAIN_220:
      return 220.0f;
  }
  return 1.0f; //can't be here
}
///////////////////////////////////////////////////////

/** Set magnetic field gain value.
 * @param gain New magnetic field gain value
 * @see HMC5883L_GetGain()
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
void hmc5883l_SetGain(uint8_t gain) {
  // use this method to guarantee that bits 4-0 are set to zero, which is a
  // requirement specified in the datasheet;
  uint8_t tmp = gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1);
  i2c1_write_buff_sync(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_B, &tmp, 1);
}

// MODE register

/** Get measurement mode.
 * In continuous-measurement mode, the device continuously performs measurements
 * and places the result in the data register. RDY goes high when new data is
 * placed in all three registers. After a power-on or a write to the mode or
 * configuration register, the first measurement set is available from all three
 * data output registers after a period of 2/fDO and subsequent measurements are
 * available at a frequency of fDO, where fDO is the frequency of data output.
 *
 * When single-measurement mode (default) is selected, device performs a single
 * measurement, sets RDY high and returned to idle mode. Mode register returns
 * to idle mode bit values. The measurement remains in the data output register
 * and RDY remains high until the data output register is read or another
 * measurement is performed.
 *
 * @return Current measurement mode
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
uint8_t hmc5883l_GetMode() {
  uint8_t tmp;
  HMC5883L_ReadBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, HMC5883L_MODEREG_BIT, HMC5883L_MODEREG_LENGTH, &tmp);
  return tmp;
}

/** Set measurement mode.
 * @param newMode New measurement mode
 * @see HMC5883L_GetMode()
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
void hmc5883l_SetMode(uint8_t newMode) {
  // use this method to guarantee that bits 7-2 are set to zero, which is a
  // requirement specified in the datasheet;
  uint8_t tmp = m_mode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1);
  i2c1_write_buff_sync(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, &tmp, 1);
  m_mode = newMode; // track to tell if we have to clear bit 7 after a read
}

// DATA* registers

/** Get 3-axis heading measurements.
 * In the event the ADC reading overflows or underflows for the given channel,
 * or if there is a math overflow during the bias measurement, this data
 * register will contain the value -4096. This register value will clear when
 * after the next valid measurement is made. Note that this method automatically
 * clears the appropriate bit in the MODE register if Single mode is active.
 * @param x 16-bit signed integer container for X,Y,Z-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
void hmc5883l_HeadingSync(uint8_t *buff, uint8_t len) {
  uint8_t tmp = HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1);
  i2c1_read_buff_sync(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, buff, len);
  if (m_mode == HMC5883L_MODE_SINGLE)
    i2c1_write_buff_sync(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, &tmp, 1);
}

// STATUS register

/** Get data output register lock status.
 * This bit is set when this some but not all for of the six data output
 * registers have been read. When this bit is set, the six data output registers
 * are locked and any new data will not be placed in these register until one of
 * three conditions are met: one, all six bytes have been read or the mode
 * changed, two, the mode is changed, or three, the measurement configuration is
 * changed.
 * @return Data output register lock status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_LOCK_BIT
 */
bool hmc5883l_GetLockStatus() {
  uint8_t tmp;
  HMC5883L_ReadBit(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_STATUS, HMC5883L_STATUS_LOCK_BIT, &tmp);
  return tmp == 0x01;
}

/** Get data ready status.
 * This bit is set when data is written to all six data registers, and cleared
 * when the device initiates a write to the data output registers and after one
 * or more of the data output registers are written to. When RDY bit is clear it
 * shall remain cleared for 250 us. DRDY pin can be used as an alternative to
 * the status register for monitoring the device for measurement data.
 * @return Data ready status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_READY_BIT
 */
bool hmc5883l_GetReadyStatus() {
  uint8_t tmp;
  HMC5883L_ReadBit(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_STATUS, HMC5883L_STATUS_READY_BIT, &tmp);
  return tmp == 0x01;
}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void HMC5883L_WriteBits(uint8_t slaveAddr,
                        uint8_t regAddr,
                        uint8_t bitStart,
                        uint8_t length,
                        uint8_t data) {
  uint8_t tmp;
  i2c1_read_buff_sync(slaveAddr, regAddr, &tmp, 1);
  uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
  data <<= (bitStart - length + 1); // shift data into correct position
  data &= mask; // zero all non-important bits in data
  tmp &= ~(mask); // zero all important bits in existing byte
  tmp |= data; // combine data with existing byte
  i2c1_write_buff_sync(slaveAddr, regAddr, &tmp, 1);
}

/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void HMC5883L_WriteBit(uint8_t slaveAddr,
                       uint8_t regAddr,
                       uint8_t bitNum,
                       uint8_t data) {
  uint8_t tmp;
  i2c1_read_buff_sync(slaveAddr, regAddr, &tmp, 1);
  tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
  i2c1_write_buff_sync(slaveAddr, regAddr, &tmp, 1);
}

/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void HMC5883L_ReadBits(uint8_t slaveAddr,
                       uint8_t regAddr,
                       uint8_t bitStart,
                       uint8_t length,
                       uint8_t *data) {
  uint8_t tmp;
  I2C1FinishCode err = i2c1_read_buff_sync(slaveAddr, regAddr, &tmp, 1);
  uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
  tmp &= mask;
  tmp >>= (bitStart - length + 1);
  *data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void HMC5883L_ReadBit(uint8_t slaveAddr,
                      uint8_t regAddr,
                      uint8_t bitNum,
                      uint8_t *data) {
  uint8_t tmp;
  i2c1_read_buff_sync(slaveAddr, regAddr, &tmp, 1);
  *data = tmp & (1 << bitNum);
}
