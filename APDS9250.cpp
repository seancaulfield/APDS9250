/*
 * APDS9250.cpp
 *
 * Arduino library for the Broadcom/Avago APDS-9250 ambient light sensor, IR
 * sensor, and RGB color sensor.
 *
 * Author: Sean Caulfield <sean@yak.net>
 * License: GPLv2.0
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include <APDS9250.h>

/*
 * Constructor. Sets some default values for sanity.
 */
APDS9250::APDS9250() {
  this->mode      = APDS9250_CHAN_ALS;
  this->res       = APDS9250_RES_18BIT;
  this->meas_rate = APDS9250_RATE_100MS;
  this->gain      = APDS9250_GAIN_3X;
  this->raw_r     = 0;
  this->raw_g     = 0;
  this->raw_b     = 0;
  this->raw_als   = 0;
  this->raw_ir    = 0;
}

/*
 * Initialize and detect the sensor on the i2c bus. Returns false if it wasn't
 * able to find one.
 */
bool APDS9250::begin() {
  
  // Send i2c "ping" to see if anything is on the bus at our address
  Wire.begin();
  Wire.beginTransmission(this->addr);
  if (Wire.endTransmission() != 0) { // No reply or garbled transmission
    return false;
  }

  // Datasheet says 4.7usec min bus start/stop time, so 50usec is plenty
  delayMicroseconds(50);

  // Enable sensor
  return this->enable();

}

bool APDS9250::reset() {
  return this->write8(APDS9250_REG_MAIN_CTRL, APDS9250_CTRL_SW_RESET);
}

/*
 * Enable sensor by triggering software reset and setting light sensor as
 * active.
 */
bool APDS9250::enable() {
  return this->write8(APDS9250_REG_MAIN_CTRL, APDS9250_CTRL_LS_EN);
}

/*
 * Get current sensor mode (basically, what's in the green register).
 */
apds9250_chan_t APDS9250::getMode() {
  uint8_t temp = this->read8(APDS9250_REG_MAIN_CTRL);
  this->mode = this->_modeFromReg(temp);
  return this->mode;
}

/*
 * Set current sensor mode (basically, what's in the green register), returing
 * new mode value.
 */
apds9250_chan_t APDS9250::setMode(apds9250_chan_t newMode) {
  uint8_t temp = APDS9250_CTRL_LS_EN;
  temp |= this->_modeToReg(newMode);
  this->write8(APDS9250_REG_MAIN_CTRL, temp);
  return this->getMode();
}

/*
 * Set sensing mode to Ambient Light Sensor (ALS) + IR.
 */
void APDS9250::setModeALS() {
  this->setMode(APDS9250_CHAN_ALS);
}

/*
 * Set sensing mode to RGB+IR.
 */
void APDS9250::setModeRGB() {
  this->setMode(APDS9250_CHAN_RGB);
}

/*
 * Get ADC resolution bits.
 */
apds9250_res_t APDS9250::getResolution() {
  this->_getMeasureRateReg();
  return this->res;
}

/*
 * Set ADC resolution bits, returning new resolution value.
 */
apds9250_res_t APDS9250::setResolution(apds9250_res_t newRes) {
  this->res = newRes;
  this->_setMeasureRateReg();
  return this->getResolution();
}

/*
 * Get ADC integration time.
 */
apds9250_rate_t APDS9250::getMeasRate() {
  this->_getMeasureRateReg();
  return this->meas_rate;
}

/*
 * Set ADC integration time, returning new rate. I *think* this overrides the
 * minimum times for the higher resolutions, but the data sheet is not 100%
 * clear.
 */
apds9250_rate_t APDS9250::setMeasRate(apds9250_rate_t newRate) {
  this->meas_rate = newRate;
  this->_setMeasureRateReg();
  return this->getMeasRate();
}

/*
 * Get ADC gain
 */
apds9250_gain_t APDS9250::getGain() {
  uint8_t temp = this->read8(APDS9250_REG_LS_GAIN);
  this->gain = this->_gainFromReg(temp);
  return this->gain;
}

/*
 * Set ADC gain
 */
apds9250_gain_t APDS9250::setGain(apds9250_gain_t newGain) {
  this->write8(APDS9250_REG_LS_GAIN, this->_gainToReg(newGain));
  return this->getGain();
}

/*
 * Get raw data from the red channel. TODO autoscale somehow?
 */
uint32_t APDS9250::getRawRedData() {
  this->raw_r = this->read20(APDS9250_REG_LS_DATA_RED_0);
  return this->raw_r;
}

/*
 * Get raw data from the green channel. TODO autoscale somehow?
 */
uint32_t APDS9250::getRawGreenData() {
  this->raw_g = this->read20(APDS9250_REG_LS_DATA_GREEN_0);
  return this->raw_g;
}

/*
 * Get raw data from the blue channel. TODO autoscale somehow?
 */
uint32_t APDS9250::getRawBlueData() {
  this->raw_b = this->read20(APDS9250_REG_LS_DATA_BLUE_0);
  return this->raw_b;
}

/*
 * Get raw data from the IR channel. TODO autoscale somehow?
 */
uint32_t APDS9250::getRawIRData() {
  this->raw_ir = this->read20(APDS9250_REG_LS_DATA_IR_0);
  return this->raw_ir;
}

/*
 * Get raw ALS data (theoretically plain ol lux) TODO autoscale?
 */
uint32_t APDS9250::getRawALSData() {
  this->raw_als = this->read20(APDS9250_REG_LS_DATA_GREEN_0);
  return this->raw_als;
}

/*
 * Write a byte to a register over i2c. Returns true if successful.
 */
bool APDS9250::write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(this->addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

/*
 * Read a byte over I2C from the given register
 */
uint8_t APDS9250::read8(uint8_t reg) {
  uint8_t temp;

  Wire.beginTransmission(this->addr);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(this->addr, 1, true);
  temp = Wire.read();

  return temp;
}

/*
 * Read 20 bits bytes over I2C from the given register, and repack into a 32 bit int and return.
 */
uint32_t APDS9250::read20(uint8_t reg) {
  uint8_t lsb, isb, msb;

  Wire.beginTransmission(this->addr);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(this->addr, 3, true);
  lsb = Wire.read();
  isb = Wire.read();
  msb = Wire.read();

  return ((msb & 7) << 16) | (isb << 8) | lsb;
}

/*
 * Get LS_MEAS_RATE register values.
 */
void APDS9250::_getMeasureRateReg() {
  uint8_t temp = this->read8(APDS9250_REG_LS_MEAS_RATE);
  this->res = this->_resFromReg(temp);
  this->meas_rate = this->_measRateFromReg(temp);
}

/*
 * Set LS_MEAS_RATE register values.
 */
void APDS9250::_setMeasureRateReg() {
  uint8_t temp = 0;
  temp |= this->_resToReg(this->res) << 4;
  temp |= this->_measRateToReg(this->meas_rate);
  this->write8(APDS9250_REG_LS_MEAS_RATE, temp);
}

/*
 * Convert register data to enum type for mode.
 */
apds9250_chan_t APDS9250::_modeFromReg(uint8_t reg_value) {
  switch (reg_value & APDS9250_CTRL_CS_MASK) {
    case APDS9250_CTRL_CS_MODE_RGB:
      return APDS9250_CHAN_RGB;
    case APDS9250_CTRL_CS_MODE_ALS:
      return APDS9250_CHAN_ALS;
    default:
      return APDS9250_CHAN_ALS;
  }
}

/*
 * Convert register data to enum type for resolution.
 */
apds9250_res_t APDS9250::_resFromReg(uint8_t reg_value) {
  switch (reg_value & APDS9250_RESOLUTION_MASK) {
    case APDS9250_RESOLUTION_20BIT:
      return APDS9250_RES_20BIT;
    case APDS9250_RESOLUTION_19BIT:
      return APDS9250_RES_19BIT;
    case APDS9250_RESOLUTION_18BIT:
      return APDS9250_RES_18BIT;
    case APDS9250_RESOLUTION_17BIT:
      return APDS9250_RES_17BIT;
    case APDS9250_RESOLUTION_16BIT:
      return APDS9250_RES_16BIT;
    case APDS9250_RESOLUTION_13BIT:
      return APDS9250_RES_13BIT;
    default:
      return APDS9250_RES_18BIT;
  }
}

/*
 * Convert register data to enum type for measurement rate.
 */
apds9250_rate_t APDS9250::_measRateFromReg(uint8_t reg_value) {
  switch (reg_value & APDS9250_MEAS_RATE_MASK) {
    case APDS9250_MEAS_RATE_25MS:
      return APDS9250_RATE_25MS;
    case APDS9250_MEAS_RATE_50MS:
      return APDS9250_RATE_50MS;
    case APDS9250_MEAS_RATE_100MS:
      return APDS9250_RATE_100MS;
    case APDS9250_MEAS_RATE_200MS:
      return APDS9250_RATE_200MS;
    case APDS9250_MEAS_RATE_500MS:
      return APDS9250_RATE_500MS;
    case APDS9250_MEAS_RATE_1000MS:
      return APDS9250_RATE_1000MS;
    case APDS9250_MEAS_RATE_2000MS:
      return APDS9250_RATE_2000MS;
    case APDS9250_MEAS_RATE_DUP:
      return APDS9250_RATE_2000MS;
    default:
      return APDS9250_RATE_100MS;
  }
}

/*
 * Convert register data to enum type for gain.
 */
apds9250_gain_t APDS9250::_gainFromReg(uint8_t reg_value) {
  switch (reg_value & APDS9250_LS_GAIN_MASK) {
    case APDS9250_LS_GAIN_1X:
      return APDS9250_GAIN_1X;
    case APDS9250_LS_GAIN_3X:
      return APDS9250_GAIN_3X;
    case APDS9250_LS_GAIN_6X:
      return APDS9250_GAIN_6X;
    case APDS9250_LS_GAIN_9X:
      return APDS9250_GAIN_9X;
    case APDS9250_LS_GAIN_18X:
      return APDS9250_GAIN_18X;
    default:
      return APDS9250_GAIN_3X;
  }
}

uint8_t APDS9250::_resToReg(apds9250_res_t newRes) {
  switch (newRes) {
    case APDS9250_RES_20BIT:
      return APDS9250_RESOLUTION_20BIT;
    case APDS9250_RES_19BIT:
      return APDS9250_RESOLUTION_19BIT;
    case APDS9250_RES_18BIT:
      return APDS9250_RESOLUTION_18BIT;
    case APDS9250_RES_17BIT:
      return APDS9250_RESOLUTION_17BIT;
    case APDS9250_RES_16BIT:
      return APDS9250_RESOLUTION_16BIT;
    case APDS9250_RES_13BIT:
      return APDS9250_RESOLUTION_13BIT;
    default:
      return APDS9250_RESOLUTION_18BIT;
  }
}

uint8_t APDS9250::_measRateToReg(apds9250_rate_t newMeasRate) {
  switch (newMeasRate) {
    case APDS9250_RATE_25MS:
      return APDS9250_MEAS_RATE_25MS;
    case APDS9250_RATE_50MS:
      return APDS9250_MEAS_RATE_50MS;
    case APDS9250_RATE_100MS:
      return APDS9250_MEAS_RATE_100MS;
    case APDS9250_RATE_200MS:
      return APDS9250_MEAS_RATE_200MS;
    case APDS9250_RATE_500MS:
      return APDS9250_MEAS_RATE_500MS;
    case APDS9250_RATE_1000MS:
      return APDS9250_MEAS_RATE_1000MS;
    case APDS9250_RATE_2000MS:
      return APDS9250_MEAS_RATE_2000MS;
    default:
      return APDS9250_MEAS_RATE_100MS;
  }
}

uint8_t APDS9250::_modeToReg(apds9250_chan_t newMode) {
  switch (newMode) {
    case APDS9250_CHAN_ALS:
      return APDS9250_CTRL_CS_MODE_ALS;
    case APDS9250_CHAN_RGB:
      return APDS9250_CTRL_CS_MODE_RGB;
    default:
      return APDS9250_CTRL_CS_MODE_ALS;
  }
}

uint8_t APDS9250::_gainToReg(apds9250_gain_t newGain) {
  switch (newGain) {
    case APDS9250_GAIN_1X:
      return APDS9250_LS_GAIN_1X;
    case APDS9250_GAIN_3X:
      return APDS9250_LS_GAIN_3X;
    case APDS9250_GAIN_6X:
      return APDS9250_LS_GAIN_6X;
    case APDS9250_GAIN_9X:
      return APDS9250_LS_GAIN_9X;
    case APDS9250_GAIN_18X:
      return APDS9250_LS_GAIN_18X;
    default:
      return APDS9250_LS_GAIN_3X;
  }
}
