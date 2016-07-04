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
  Wire.begin();
  
  // Send i2c "ping" to see if anything is on the bus at our address
  Wire.beginTransmission(this->addr);
  if (Wire.endTransmission() != 0) { // No reply or garbled transmission
    return false;
  }

  // Datasheet says 4.7usec min bus start/stop time, so 50usec is plenty
  delayMicroseconds(50);

  // Enable sensor
  this->enable();

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
  uint8_t temp = 0;

  Wire.beginTransmission(this->addr);
  Wire.write(APDS9250_REG_MAIN_CTRL);
  Wire.endTransmission(false);

  Wire.requestFrom(this->addr, 1, true);
  temp = Wire.read();

  temp = this->read8(APDS9250_REG_MAIN_CTRL);

  if (temp & APDS9250_CTRL_CS_MODE_RGB) {
    this->mode = APDS9250_CHAN_RGB;
  } else {
    this->mode = APDS9250_CHAN_ALS;
  }

  return this->mode;
}

/*
 * Set current sensor mode (basically, what's in the green register), returing
 * new mode value.
 */
apds9250_chan_t APDS9250::setMode(apds9250_chan_t newMode) {
  uint8_t temp = APDS9250_CTRL_LS_EN;

  switch (newMode) {
    case APDS9250_CHAN_ALS:
      temp |= APDS9250_CTRL_CS_MODE_ALS; break;
    case APDS9250_CHAN_RGB:
      temp |= APDS9250_CTRL_CS_MODE_RGB; break;
    default:
      break;
  }

  Wire.beginTransmission(this->addr);
  Wire.write(APDS9250_REG_MAIN_CTRL);
  Wire.write(temp);
  Wire.endTransmission();

  return this->getMode();
}

/*
 * Get LS_MEAS_RATE register values.
 */
void APDS9250::_getMeasureRateReg() {
  uint8_t temp = 0;

  Wire.beginTransmission(this->addr);
  Wire.write(APDS9250_REG_LS_MEAS_RATE);
  Wire.endTransmission(false);

  Wire.requestFrom(this->addr, 1, true);
  temp = Wire.read();

  switch (temp & APDS9250_RESOLUTION_MASK) {
    case APDS9250_RESOLUTION_20BIT:
      this->res = APDS9250_RES_20BIT; break;
    case APDS9250_RESOLUTION_19BIT:
      this->res = APDS9250_RES_19BIT; break;
    case APDS9250_RESOLUTION_18BIT:
      this->res = APDS9250_RES_18BIT; break;
    case APDS9250_RESOLUTION_17BIT:
      this->res = APDS9250_RES_17BIT; break;
    case APDS9250_RESOLUTION_16BIT:
      this->res = APDS9250_RES_16BIT; break;
    case APDS9250_RESOLUTION_13BIT:
      this->res = APDS9250_RES_13BIT; break;
    default:
      break;
  }

  switch (temp & APDS9250_MEAS_RATE_MASK) {
    case APDS9250_MEAS_RATE_25MS:
      this->meas_rate = APDS9250_RATE_25MS; break;
    case APDS9250_MEAS_RATE_50MS:
      this->meas_rate = APDS9250_RATE_50MS; break;
    case APDS9250_MEAS_RATE_100MS:
      this->meas_rate = APDS9250_RATE_100MS; break;
    case APDS9250_MEAS_RATE_200MS:
      this->meas_rate = APDS9250_RATE_200MS; break;
    case APDS9250_MEAS_RATE_500MS:
      this->meas_rate = APDS9250_RATE_500MS; break;
    case APDS9250_MEAS_RATE_1000MS:
      this->meas_rate = APDS9250_RATE_1000MS; break;
    case APDS9250_MEAS_RATE_2000MS:
      this->meas_rate = APDS9250_RATE_2000MS; break;
    case APDS9250_MEAS_RATE_DUP:
      this->meas_rate = APDS9250_RATE_2000MS; break;
    default:
      break;
  }
}

/*
 * Set LS_MEAS_RATE register values.
 */
void APDS9250::_setMeasureRateReg() {
  uint8_t temp = 0;

  switch (this->res) {
    case APDS9250_RES_20BIT:
      temp |= APDS9250_RESOLUTION_20BIT; break;
    case APDS9250_RES_19BIT:
      temp |= APDS9250_RESOLUTION_19BIT; break;
    case APDS9250_RES_18BIT:
      temp |= APDS9250_RESOLUTION_18BIT; break;
    case APDS9250_RES_17BIT:
      temp |= APDS9250_RESOLUTION_17BIT; break;
    case APDS9250_RES_16BIT:
      temp |= APDS9250_RESOLUTION_16BIT; break;
    case APDS9250_RES_13BIT:
      temp |= APDS9250_RESOLUTION_13BIT; break;
    default:
      break;
  }


  switch (this->meas_rate) {
    case APDS9250_RATE_25MS:
      temp |= APDS9250_MEAS_RATE_25MS; break;
    case APDS9250_RATE_50MS:
      temp |= APDS9250_MEAS_RATE_50MS; break;
    case APDS9250_RATE_100MS:
      temp |= APDS9250_MEAS_RATE_100MS; break;
    case APDS9250_RATE_200MS:
      temp |= APDS9250_MEAS_RATE_200MS; break;
    case APDS9250_RATE_500MS:
      temp |= APDS9250_MEAS_RATE_500MS; break;
    case APDS9250_RATE_1000MS:
      temp |= APDS9250_MEAS_RATE_1000MS; break;
    case APDS9250_RATE_2000MS:
      temp |= APDS9250_MEAS_RATE_2000MS; break;
    default:
      break;
  }

  Wire.beginTransmission(this->addr);
  Wire.write(APDS9250_REG_LS_MEAS_RATE);
  Wire.write(temp);
  Wire.endTransmission();
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
  return this->gain;
}

/*
 * Set ADC gain
 */
apds9250_gain_t APDS9250::setGain(apds9250_gain_t newGain) {
  return this->getGain();
}

/*
 * Get raw data from the red channel. TODO autoscale somehow?
 */
uint32_t APDS9250::getRawRedData() {
  return this->raw_r;
}

/*
 * Get raw data from the green channel. TODO autoscale somehow?
 */
uint32_t APDS9250::getRawGreenData() {
  return this->raw_g;
}

/*
 * Get raw data from the blue channel. TODO autoscale somehow?
 */
uint32_t APDS9250::getRawBlueData() {
  return this->raw_b;
}

/*
 * Get raw data from the IR channel. TODO autoscale somehow?
 */
uint32_t APDS9250::getRawIRData() {
  return this->raw_ir;
}

/*
 * Get raw ALS data (theoretically plain ol lux) TODO autoscale?
 */
uint32_t APDS9250::getRawALSData() {
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
 * Read 3 bytes over I2C from the given register, and repack into a 32 bit int and return.
 */
uint32_t APDS9250::read24(uint8_t reg) {
  uint8_t lsb, isb, msb;

  Wire.beginTransmission(this->addr);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(this->addr, 3, true);
  lsb = Wire.read();
  isb = Wire.read();
  msb = Wire.read();

  return (msb << 16) | (isb << 8) | lsb;
}
