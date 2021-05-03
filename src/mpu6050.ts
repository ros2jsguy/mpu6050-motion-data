/* eslint-disable no-param-reassign */

import printf = require('printf');
import {I2CHelper} from './i2c-helper';
import { RpioI2CHelper } from './rpio-i2c-helper';
import { Utils } from './utils';

export type I2cOptions = {
  i2cAddress?: number;
  baudRate?: number;
}

const DEFAULT_I2C_ADDR = 0x68;
const DEFAULT_I2C_BAUDRATE = 400000;

const DEFAULT_I2OPTIONS: I2cOptions = {
  i2cAddress: DEFAULT_I2C_ADDR,
  baudRate: DEFAULT_I2C_BAUDRATE
};

export type Accel = {
  acc_x: number;
  acc_y: number;
  acc_z: number;
}

export type Gyro = {
  gyro_x: number;
  gyro_y: number;
  gyro_z: number;
}

export type MotionData = {
  accel: Accel;
  gyro: Gyro;
}

export type Vector3 = {
  x: number,
  y: number,
  z: number
}

export type Euler = {
  psi: number,
  theta: number,
  phi: number
}

export type Quaternion = {
  w: number,
  x: number,
  y: number,
  z: number
}

export type RPY = {
  roll: number,
  pitch: number,
  yaw: number
}

// export type SensorOffsets = {
//   acc_x_offset?: number;
//   acc_y_offset?: number;
//   acc_z_offset?: number;
//   gyro_x_offset?: number;
//   gyro_y_offset?: number;
//   gyro_z_offset?: number;
// }


// MPU6050 Registers
export enum Register {
  RA_XA_OFFS_H       = 0x06, // [15:0] XA_OFFS
  RA_XA_OFFS_L_TC    = 0x07,
  RA_YA_OFFS_H       = 0x08, // [15:0] YA_OFFS
  RA_YA_OFFS_L_TC    = 0x09,
  RA_ZA_OFFS_H       = 0x0A, // [15:0] ZA_OFFS
  RA_XG_OFFS_USRH    = 0x13, // [15:0] XG_OFFS_USR
  RA_XG_OFFS_USRL    = 0x14,
  RA_YG_OFFS_USRH    = 0x15, // [15:0] YG_OFFS_USR
  RA_YG_OFFS_USRL    = 0x16,
  RA_ZG_OFFS_USRH    = 0x17, // [15:0] ZG_OFFS_USR
  RA_ZG_OFFS_USRL    = 0x18,

  RA_SMPLRT_DIV      = 0x19,
  RA_CONFIG          = 0x1A,
  RA_GYRO_CONFIG     = 0x1B,
  RA_ACCEL_CONFIG    = 0x1C,
  RA_FIFO_EN         = 0x23,
  RA_INT_PIN_CFG     = 0x37,
  RA_INT_ENABLE      = 0x38,
  RA_INT_STATUS      = 0x3A,
  RA_ACCEL_XOUT_H    = 0x3B,
  RA_ACCEL_XOUT_L    = 0x3C,
  RA_ACCEL_YOUT_H    = 0x3D,
  RA_ACCEL_YOUT_L    = 0x3E,
  RA_ACCEL_ZOUT_H    = 0x3F,
  RA_ACCEL_ZOUT_L    = 0x40,
  RA_TEMP_OUT_H      = 0x41,
  RA_TEMP_OUT_L      = 0x42,
  RA_GYRO_XOUT_H     = 0x43,
  RA_GYRO_XOUT_L     = 0x44,
  RA_GYRO_YOUT_H     = 0x45,
  RA_GYRO_YOUT_L     = 0x46,
  RA_GYRO_ZOUT_H     = 0x47,
  RA_GYRO_ZOUT_L     = 0x48,
  RA_USER_CTRL       = 0x6A,
  RA_PWR_MGMT_1      = 0x6B,
  RA_BANK_SEL        = 0x6D,
  RA_MEM_START_ADDR  = 0x6E,
  RA_MEM_R_W         = 0x6F,
  RA_FIFO_COUNTH     = 0x72,
  RA_FIFO_R_W        = 0x74,
  RA_WHO_AM_I        = 0x75
}

export enum ClockSource {
  INTERNAL    = 0x00,
  PLL_XGYRO   = 0x01,
  PLL_YGYRO   = 0x02,
  PLL_ZGYRO   = 0x03,
  PLL_EXT32K  = 0x04,
  PLL_EXT19M  = 0x05,
  KEEP_RESET  = 0x07
}

export enum ClockDiv {
  DIV_348 = 0x0,
  DIV_333 = 0x1,
  DIV_320 = 0x2,
  DIV_308 = 0x3,
  DIV_296 = 0x4,
  DIV_286 = 0x5,
  DIV_276 = 0x6,
  DIV_267 = 0x7,
  DIV_258 = 0x8,
  DIV_500 = 0x9,
  DIV_471 = 0xA,
  DIV_444 = 0xB,
  DIV_421 = 0xC,
  DIV_400 = 0xD,
  DIV_381 = 0xE,
  DIV_364 = 0xF
}

export enum AccelFsRange {
  FS_2                     = 0x00,
  FS_4                     = 0x01,
  FS_8                     = 0x02,
  FS_16                    = 0x03,
}

export enum GyroFsRange {
  FS_250                    = 0x00,
  FS_500                    = 0x01,
  FS_1000                   = 0x02,
  FS_2000                   = 0x03
}

/**
 * A MPU6050 driver class based on the [I2Cdevlib library](https://www.i2cdevlib.com/devices/mpu6050), 
 * providing fused orientation (quaternion), acceleration and rotational data 
 * via the on-device digital motion processor (DMP). Includes sensor offset 
 * calibration and raw unfused sensor data. This module is implemented in 
 * TypeScript and runs on Raspberry Pi 3 & 4.
 */
export class MPU6050 {

  private i2cAddress: number;
  private i2cBaudRate: number;
  private i2cHelper: I2CHelper;
  private dmpPacketSize = 0;

  constructor(i2cOptions = DEFAULT_I2OPTIONS) {
    const options : I2cOptions = {...DEFAULT_I2OPTIONS, ...i2cOptions} ;
    this.i2cAddress = options.i2cAddress!;
    this.i2cBaudRate = options.baudRate!
    this.i2cHelper = new RpioI2CHelper(this.i2cBaudRate);
  }

  /** Power on and prepare for general usage.
   * This will activate the device and take it out of sleep mode (which must be done
   * after start-up). This function also sets both the accelerometer and the gyroscope
   * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
   * the clock source to use the X Gyro for reference, which is slightly better than
   * the default internal clock source.
   */
  initialize(): void {
    this.setClockSource(ClockSource.PLL_ZGYRO);
    this.setFullScaleGyroRange(GyroFsRange.FS_250);
    this.setFullScaleAccelRange(AccelFsRange.FS_2);
    this.setSleepEnabled(false);

    Utils.msleep(100)
  }

  shutdown(): void {
  }

  getDeviceI2CAddr(): number {
    return this.i2cAddress;
  }

  /** Get Device ID.
   * This register is used to verify the identity of the device (0b110100, 0x34).
   * @see Register.RA_WHO_AM_I
   * @see Register.WHO_AM_I_BIT
   * @see Register.WHO_AM_I_LENGTH
   * @return Devices ID (6 bits only! should be 0x34)
   */
  getDeviceID(): number {
    return this.i2cHelper.readBits(this.getDeviceI2CAddr(), Register.RA_WHO_AM_I, WHO_AM_I_BIT, WHO_AM_I_LENGTH);
  }

  /** Verify the I2C connection.
   * Make sure the device is connected and responds as expected.
   * @return True if connection is valid, false otherwise
   */
  testConnection(): boolean {
    return this.getDeviceID() === 0x34;
  }

  /** Trigger a full device reset.
   * A small delay of ~50ms may be desirable after triggering a reset.
   * @see MPU6050.RA_PWR_MGMT_1
   * @see MPU6050.PWR1_DEVICE_RESET_BIT
   */
  reset(): void {
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_PWR_MGMT_1, PWR1_DEVICE_RESET_BIT, 1);
  }

  /** Get sleep mode status.
   * Setting the SLEEP bit in the register puts the device into very low power
   * sleep mode. In this mode, only the serial interface and internal registers
   * remain active, allowing for a very low standby current. Clearing this bit
   * puts the device back into normal mode. To save power, the individual standby
   * selections for each of the gyros should be used if any gyro axis is not used
   * by the application.
   * @return Current sleep mode enabled status
   * @see MPU6050.RA_PWR_MGMT_1
   * @see MPU6050.PWR1_SLEEP_BIT
   */
  getSleepEnabled(): boolean {
    return this.i2cHelper.readBit(this.getDeviceI2CAddr(), Register.RA_PWR_MGMT_1, PWR1_SLEEP_BIT) === 1;
  }

  /** Set sleep mode status.
  * @param enabled New sleep mode enabled status
  * @see getSleepEnabled()
  * @see MPU6050.RA_PWR_MGMT_1
  * @see MPU6050.PWR1_SLEEP_BIT
  */
  setSleepEnabled(enabled: boolean): void {
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_PWR_MGMT_1, PWR1_SLEEP_BIT, enabled ? 1 : 0);
  }

  /** Get clock source setting.
   * @return Current clock source setting
   * @see MPU6050.RA_PWR_MGMT_1
   * @see MPU6050.PWR1_CLKSEL_BIT
   * @see MPU6050.PWR1_CLKSEL_LENGTH
   */
  getClockSource(): ClockSource {
    return this.i2cHelper.readBits(this.getDeviceI2CAddr(), Register.RA_PWR_MGMT_1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH);
  }

  /** Set clock source setting.
  * An internal 8MHz oscillator, gyroscope based clock, or external sources can
  * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
  * or an external source is chosen as the clock source, the MPU-60X0 can operate
  * in low power modes with the gyroscopes disabled.
  *
  * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
  * However, it is highly recommended that the device be configured to use one of
  * the gyroscopes (or an external clock source) as the clock reference for
  * improved stability. The clock source can be selected according to the following table:
  *
  * <pre>
  * CLK_SEL | Clock Source
  * --------+--------------------------------------
  * 0       | Internal oscillator
  * 1       | PLL with X Gyro reference
  * 2       | PLL with Y Gyro reference
  * 3       | PLL with Z Gyro reference
  * 4       | PLL with external 32.768kHz reference
  * 5       | PLL with external 19.2MHz reference
  * 6       | Reserved
  * 7       | Stops the clock and keeps the timing generator in reset
  * </pre>
  *
  * @param source New clock source setting
  * @see getClockSource()
  * @see MPU6050.RA_PWR_MGMT_1
  * @see MPU6050.PWR1_CLKSEL_BIT
  * @see MPU6050.PWR1_CLKSEL_LENGTH
  */
  setClockSource(source: ClockSource): void {
    this.i2cHelper.writeBits(this.getDeviceI2CAddr(), Register.RA_PWR_MGMT_1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH, source);
  }

  /** Get gyroscope output rate divider.
   * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
   * Motion detection, and Free Fall detection are all based on the Sample Rate.
   * The Sample Rate is generated by dividing the gyroscope output rate by
   * SMPLRT_DIV:
   *
   * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
   *
   * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
   * 7), and 1kHz when the DLPF is enabled (see Register 26).
   *
   * Note: The accelerometer output rate is 1kHz. This means that for a Sample
   * Rate greater than 1kHz, the same accelerometer sample may be output to the
   * FIFO, DMP, and sensor registers more than once.
   *
   * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
   * of the MPU-6000/MPU-6050 Product Specification document.
   *
   * @return Current sample rate
   * @see MPU6050.RA_SMPLRT_DIV
   */
  getRate(): number {
    return this.i2cHelper.readByte(this.getDeviceI2CAddr(), Register.RA_SMPLRT_DIV);
  }

  /** Set gyroscope sample rate divider.
  * @param rate New sample rate divider
  * @see getRate()
  * @see MPU6050.RA_SMPLRT_DIV
  */
  setRate(rate: number): void {
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), Register.RA_SMPLRT_DIV, rate);
  }

  /** Get full-scale gyroscope range.
   * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
   * as described in the table below.
   *
   * <pre>
   * 0 = +/- 250 degrees/sec
   * 1 = +/- 500 degrees/sec
   * 2 = +/- 1000 degrees/sec
   * 3 = +/- 2000 degrees/sec
   * </pre>
   *
   * @return Current full-scale gyroscope range setting
   * @see MPU6050.GYRO_FS_250
   * @see MPU6050.RA_GYRO_CONFIG
   * @see MPU6050.GCONFIG_FS_SEL_BIT
   * @see MPU6050.GCONFIG_FS_SEL_LENGTH
   */
  getFullScaleGyroRange(): number {
    return this.i2cHelper.readBits(this.getDeviceI2CAddr(), Register.RA_GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH);
  }

  /** Set full-scale gyroscope range.
  * @param range New full-scale gyroscope range value
  * @see getFullScaleRange()
  * @see MPU6050.GYRO_FS_250
  * @see MPU6050.RA_GYRO_CONFIG
  * @see MPU6050.GCONFIG_FS_SEL_BIT
  * @see MPU6050.GCONFIG_FS_SEL_LENGTH
  */
  setFullScaleGyroRange(range: number): void {
    this.i2cHelper.writeBits(this.getDeviceI2CAddr(), Register.RA_GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH, range);
  }

  /** Get full-scale accelerometer range.
   * The FS_SEL parameter allows setting the full-scale range of the accelerometer
   * sensors, as described in the table below.
   *
   * <pre>
   * 0 = +/- 2g
   * 1 = +/- 4g
   * 2 = +/- 8g
   * 3 = +/- 16g
   * </pre>
   *
   * @return Current full-scale accelerometer range setting
   * @see MPU6050.ACCEL_FS_2
   * @see MPU6050.RA_ACCEL_CONFIG
   * @see MPU6050.ACONFIG_AFS_SEL_BIT
   * @see MPU6050.ACONFIG_AFS_SEL_LENGTH
   */
  getFullScaleAccelRange(): AccelFsRange {
    return this.i2cHelper.readBits(this.getDeviceI2CAddr(), Register.RA_ACCEL_CONFIG, ACONFIG_AFS_SEL_BIT, ACONFIG_AFS_SEL_LENGTH);
  }

  /** Set full-scale accelerometer range.
  * @param range New full-scale accelerometer range setting
  * @see getFullScaleAccelRange()
  */
  setFullScaleAccelRange(range: AccelFsRange): void {
    this.i2cHelper.writeBits(this.getDeviceI2CAddr(), Register.RA_ACCEL_CONFIG, ACONFIG_AFS_SEL_BIT, ACONFIG_AFS_SEL_LENGTH, range);
  }

  /** Get accelerometer FIFO enabled value.
   * When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
   * ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
   * written into the FIFO buffer.
   * @return Current accelerometer FIFO enabled value
   * @see MPU6050.RA_FIFO_EN
   */
  getAccelFIFOEnabled(): boolean {
    return this.i2cHelper.readBit(this.getDeviceI2CAddr(), Register.RA_FIFO_EN, ACCEL_FIFO_EN_BIT) === 1;
  
  }

  /** Set accelerometer FIFO enabled value.
  * @param enabled New accelerometer FIFO enabled value
  * @see getAccelFIFOEnabled()
  * @see MPU6050.RA_FIFO_EN
  */
  setAccelFIFOEnabled(enabled: boolean): void {
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_FIFO_EN, ACCEL_FIFO_EN_BIT, enabled ? 1 : 0);
  }

  /** Get gyroscope X-axis FIFO enabled value.
  * When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
  * 68) to be written into the FIFO buffer.
  * @return Current gyroscope X-axis FIFO enabled value
  * @see MPU6050_RA_FIFO_EN
  */
  getXGyroFIFOEnabled(): boolean {
    return this.i2cHelper.readBit(this.getDeviceI2CAddr(), Register.RA_FIFO_EN, XG_FIFO_EN_BIT) === 1;
  }

  /** Set gyroscope X-axis FIFO enabled value.
  * @param enabled New gyroscope X-axis FIFO enabled value
  * @see getXGyroFIFOEnabled()
  * @see MPU6050_RA_FIFO_EN
  */
  setXGyroFIFOEnabled(enabled: boolean): void {
      this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_FIFO_EN, XG_FIFO_EN_BIT, enabled ? 1 : 0);
  }

  /** Get gyroscope Y-axis FIFO enabled value.
  * When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
  * 70) to be written into the FIFO buffer.
  * @return Current gyroscope Y-axis FIFO enabled value
  * @see MPU6050_RA_FIFO_EN
  */
  getYGyroFIFOEnabled(): boolean {
      return this.i2cHelper.readBit(this.getDeviceI2CAddr(), Register.RA_FIFO_EN, YG_FIFO_EN_BIT) === 1;
  }

  /** Set gyroscope Y-axis FIFO enabled value.
  * @param enabled New gyroscope Y-axis FIFO enabled value
  * @see getYGyroFIFOEnabled()
  * @see MPU6050_RA_FIFO_EN
  */
  setYGyroFIFOEnabled(enabled: boolean): void {
      this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_FIFO_EN, YG_FIFO_EN_BIT, enabled ? 1 : 0);
  }

  /** Get gyroscope Z-axis FIFO enabled value.
  * When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
  * 72) to be written into the FIFO buffer.
  * @return Current gyroscope Z-axis FIFO enabled value
  * @see MPU6050_RA_FIFO_EN
  */
  getZGyroFIFOEnabled(): boolean {
      return this.i2cHelper.readBit(this.getDeviceI2CAddr(), Register.RA_FIFO_EN, ZG_FIFO_EN_BIT) === 1;
  }

  /** Set gyroscope Z-axis FIFO enabled value.
  * @param enabled New gyroscope Z-axis FIFO enabled value
  * @see getZGyroFIFOEnabled()
  * @see MPU6050_RA_FIFO_EN
  */
  setZGyroFIFOEnabled(enabled: boolean): void {
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_FIFO_EN, ZG_FIFO_EN_BIT, enabled ? 1 : 0);
  }

  /** Get interrupt logic level mode.
 * Will be set 0 for active-high, 1 for active-low.
 * @return Current interrupt mode (0=active-high, 1=active-low)
 * @see MPU6050.RA_INT_PIN_CFG
 * @see MPU6050.INTCFG_INT_LEVEL_BIT
 */
  getInterruptMode(): number {
    return this.i2cHelper.readBit(this.getDeviceI2CAddr(), Register.RA_INT_PIN_CFG, INTCFG_INT_LEVEL_BIT);
  }

  /** Set interrupt logic level mode.
  * @param mode New interrupt mode (0=active-high, 1=active-low)
  * @see getInterruptMode()
  * @see MPU6050.RA_INT_PIN_CFG
  * @see MPU6050.INTCFG_INT_LEVEL_BIT
  */
  setInterruptMode(mode: boolean): void {
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_INT_PIN_CFG, INTCFG_INT_LEVEL_BIT, mode ? 1 : 0);
  }

  /** 
   * Get full interrupt enabled status.
   * Full register byte for all interrupts, for quick reading. Each bit will be
   * set 0 for disabled, 1 for enabled.
   * @return Current interrupt enabled status
   * @see MPU6050.RA_INT_ENABLE
   * @see MPU6050.INTERRUPT_FF_BIT
   */
  getIntEnabled(): number {
    return this.i2cHelper.readByte(this.getDeviceI2CAddr(), Register.RA_INT_ENABLE);
  }

  /** Set full interrupt enabled status.
  * Full register byte for all interrupts, for quick reading. Each bit should be
  * set 0 for disabled, 1 for enabled.
  * @param enabled New interrupt enabled status
  * @see getIntFreefallEnabled()
  * @see MPU6050.RA_INT_ENABLE
  * @see MPU6050.INTERRUPT_FF_BIT
  */
  setIntEnabled(enabled: number): void {
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), Register.RA_INT_ENABLE, enabled);
  }

  /** Get FIFO Buffer Overflow interrupt enabled status.
   * Will be set 0 for disabled, 1 for enabled.
   * @return Current interrupt enabled status
   * @see MPU6050.RA_INT_ENABLE
   * @see MPU6050.INTERRUPT_FIFO_OFLOW_BIT
   */
  getIntFIFOBufferOverflowEnabled(): boolean {
    return this.i2cHelper.readBit(this.getDeviceI2CAddr(), Register.RA_INT_ENABLE, INTERRUPT_FIFO_OFLOW_BIT) === 1;
  }

  /** Set FIFO Buffer Overflow interrupt enabled status.
  * @param enabled New interrupt enabled status
  * @see getIntFIFOBufferOverflowEnabled()
  * @see MPU6050.RA_INT_ENABLE
  * @see MPU6050.INTERRUPT_FIFO_OFLOW_BIT
  */
  setIntFIFOBufferOverflowEnabled(enabled: boolean): void {
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_INT_ENABLE, INTERRUPT_FIFO_OFLOW_BIT, enabled ? 1 : 0);
  }

  /** Get Data Ready interrupt enabled setting.
   * This event occurs each time a write operation to all of the sensor registers
   * has been completed. Will be set 0 for disabled, 1 for enabled.
   * @return Current interrupt enabled status
   * @see MPU6050.RA_INT_ENABLE
   * @see MPU6050.INTERRUPT_DATA_RDY_BIT
   */
  getIntDataReadyEnabled(): boolean {
    return this.i2cHelper.readBit(this.getDeviceI2CAddr(), Register.RA_INT_ENABLE, INTERRUPT_DATA_RDY_BIT) === 1;
  }

  /** Set Data Ready interrupt enabled status.
  * @param enabled New interrupt enabled status
  * @see getIntDataReadyEnabled()
  * @see MPU6050.RA_INT_CFG
  * @see MPU6050.INTERRUPT_DATA_RDY_BIT
  */
  setIntDataReadyEnabled(enabled: boolean): void {
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_INT_ENABLE, INTERRUPT_DATA_RDY_BIT, enabled ? 1 : 0);
  }

  /** Get full set of interrupt status bits.
   * These bits clear to 0 after the register has been read. Very useful
   * for getting multiple INT statuses, since each single bit read clears
   * all of them because it has to read the whole byte.
   * @return Current interrupt status
   * @see MPU6050.RA_INT_STATUS
   */
  getIntStatus(): number {
    return this.i2cHelper.readByte(this.getDeviceI2CAddr(), Register.RA_INT_STATUS);
  }

  /** Get Data Ready interrupt status.
   * This bit automatically sets to 1 when a Data Ready interrupt has been
   * generated. The bit clears to 0 after the register has been read.
   * @return Current interrupt status
   * @see MPU6050.RA_INT_STATUS
   * @see MPU6050.INTERRUPT_DATA_RDY_BIT
   */
  getIntDataReadyStatus(): boolean {
    return this.i2cHelper.readBit(this.getDeviceI2CAddr(), Register.RA_INT_STATUS, INTERRUPT_DATA_RDY_BIT) === 1;
  }

  /** Get FIFO enabled status.
   * When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
   * cannot be written to or read from while disabled. The FIFO buffer's state
   * does not change unless the MPU-60X0 is power cycled.
   * @return Current FIFO enabled status
   * @see MPU6050.RA_USER_CTRL
   * @see MPU6050.USERCTRL_FIFO_EN_BIT
   */
   getFIFOEnabled(): boolean {
    return this.i2cHelper.readBit(this.getDeviceI2CAddr(), Register.RA_USER_CTRL, USERCTRL_FIFO_EN_BIT) !== 0; 
  }

  /** Set FIFO enabled status.
  * @param enabled New FIFO enabled status
  * @see getFIFOEnabled()
  * @see MPU6050.RA_USER_CTRL
  * @see MPU6050.USERCTRL_FIFO_EN_BIT
  */
  setFIFOEnabled(enabled: boolean): void {
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_USER_CTRL, USERCTRL_FIFO_EN_BIT, enabled ? 1 : 0);
  }

  /** Reset the FIFO.
   * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
   * bit automatically clears to 0 after the reset has been triggered.
   * @see MPU6050.RA_USER_CTRL
   * @see MPU6050.USERCTRL_FIFO_RESET_BIT
   */
  resetFIFO(): void {
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_USER_CTRL, USERCTRL_FIFO_RESET_BIT, 1);
  }

  /** Get current FIFO buffer size.
   * This value indicates the number of bytes stored in the FIFO buffer. This
   * number is in turn the number of bytes that can be read from the FIFO buffer
   * and it is directly proportional to the number of samples available given the
   * set of sensor data bound to be stored in the FIFO (register 35 and 36).
   * @return Current FIFO buffer size
   */
  getFIFOCount(): number {
    const buf = this.i2cHelper.readBytes(this.getDeviceI2CAddr(), Register.RA_FIFO_COUNTH, 2);
    return buf.readUInt16BE(0);
  }

  /** Get byte from FIFO buffer.
   * This register is used to read and write data from the FIFO buffer. Data is
   * written to the FIFO in order of register number (from lowest to highest). If
   * all the FIFO enable flags (see below) are enabled and all External Sensor
   * Data registers (Registers 73 to 96) are associated with a Slave device, the
   * contents of registers 59 through 96 will be written in order at the Sample
   * Rate.
   *
   * The contents of the sensor data registers (Registers 59 to 96) are written
   * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
   * in FIFO_EN (Register 35). An additional flag for the sensor data registers
   * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
   *
   * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
   * automatically set to 1. This bit is located in INT_STATUS (Register 58).
   * When the FIFO buffer has overflowed, the oldest data will be lost and new
   * data will be written to the FIFO.
   *
   * If the FIFO buffer is empty, reading this register will return the last byte
   * that was previously read from the FIFO until new data is available. The user
   * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
   * empty.
   *
   * @return Byte from FIFO buffer
   */
  getFIFOByte(): number {
    return this.i2cHelper.readByte(this.getDeviceI2CAddr(), Register.RA_FIFO_R_W);
  }

  getFIFOBytes(length: number): Buffer {
    if (length < 1) return Buffer.alloc(0);

    return this.i2cHelper.readBytes(this.getDeviceI2CAddr(), Register.RA_FIFO_R_W, length);
  }

  /** Get raw 6-axis motion sensor readings (accel/gyro).
   * Retrieves all currently available motion sensor values.
   * @see getAcceleration()
   * @see getRotation()
   * @see MPU6050.RA_ACCEL_XOUT_H
   */
  getMotionData(): MotionData {
    const buf = this.i2cHelper.readBytes(this.getDeviceI2CAddr(), Register.RA_ACCEL_XOUT_H, 14);
    const accel: Accel = {
      acc_x: buf.readInt16BE(0),
      acc_y: buf.readInt16BE(2),
      acc_z: buf.readInt16BE(4),
    };
    const gyro: Gyro = {
      gyro_x: buf.readInt16BE(8),
      gyro_y: buf.readInt16BE(10),
      gyro_z: buf.readInt16BE(12)
    }
    return {accel, gyro};
  }

  setSensorOffsets(xAccelOffset: number, yAccelOffset: number, zAccelOffset: number, 
                   xGyroOffset: number, yGyroOffset: number, zGyroOffset: number): void {
    this.setXAccelOffset(xAccelOffset);
    this.setYAccelOffset(yAccelOffset);
    this.setZAccelOffset(zAccelOffset);
    this.setXGyroOffset(xGyroOffset);
    this.setYGyroOffset(yGyroOffset);
    this.setZGyroOffset(zGyroOffset);
  }

  /**
   * Access the X-axis acceleration offset.
   * @returns X-axis offset
   */
  getXAccelOffset(): number {
	  const SaveAddress = ((this.getDeviceID() < 0x38 )? Register.RA_XA_OFFS_H:0x77); // MPU6050,MPU9150 Vs MPU6500,MPU9250
	  const buf = this.i2cHelper.readBytes(this.getDeviceI2CAddr(), SaveAddress, 2);
    return buf.readInt16BE(0);
  }

  /**
   * Update the X-axis acceleration offset.
   * @param offset 
   */
  setXAccelOffset(offset: number): void {
    const SaveAddress = this.getDeviceID() < 0x38 ? Register.RA_XA_OFFS_H:0x77; // MPU6050,MPU9150 Vs MPU6500,MPU9250
    this.i2cHelper.writeWord(this.getDeviceI2CAddr(), SaveAddress, offset);
  }

  /**
   * Access the Y-axis acceleration offset.
   * @returns y-axis offset
   */
  getYAccelOffset(): number {
    const SaveAddress = this.getDeviceID() < 0x38 ? Register.RA_YA_OFFS_H:0x7A; // MPU6050,MPU9150 Vs MPU6500,MPU9250
    const buf = this.i2cHelper.readBytes(this.getDeviceI2CAddr(), SaveAddress, 2);
    return buf.readInt16BE(0);
  }

  /**
   * Update the Y-axis acceleration offset.
   * @param offset 
   */
  setYAccelOffset(offset: number): void {
    const SaveAddress = ((this.getDeviceID() < 0x38 )? Register.RA_YA_OFFS_H:0x77); // MPU6050,MPU9150 Vs MPU6500,MPU9250
    this.i2cHelper.writeWord(this.getDeviceI2CAddr(), SaveAddress, offset)
  }

  /**
   * Access the Z-axis acceleration offset.
   * @returns z-axis offset
   */
  getZAccelOffset(): number {
    const SaveAddress = this.getDeviceID() < 0x38 ? Register.RA_ZA_OFFS_H:0x7A; // MPU6050,MPU9150 Vs MPU6500,MPU9250
    const buf = this.i2cHelper.readBytes(this.getDeviceI2CAddr(), SaveAddress, 2);
    return buf.readInt16BE(0);
  }

  /**
   * Update the Z-axis acceleration offset.
   * @param offset 
   */
  setZAccelOffset(offset: number): void {
    const SaveAddress = this.getDeviceID() < 0x38 ? Register.RA_ZA_OFFS_H:0x77; // MPU6050,MPU9150 Vs MPU6500,MPU9250
    this.i2cHelper.writeWord(this.getDeviceI2CAddr(), SaveAddress, offset)
  }

  /**
   * Access the X-axis gyro offset.
   * @returns X-axis offset
   */
  getXGyroOffset(): number {
    return this.i2cHelper.readBytes(this.getDeviceI2CAddr(), Register.RA_XG_OFFS_USRH, 2).readUInt16BE(0);
  }

  /**
   * Update the Z-axis gyro offset.
   * @param offset 
   */
  setXGyroOffset(offset: number): void {
    this.i2cHelper.writeWord(this.getDeviceI2CAddr(), Register.RA_XG_OFFS_USRH, offset);
  }

  /**
   * Access the Y-axis gyr offset.
   * @returns y-axis offset
   */
  getYGyroOffset(): number {
    return this.i2cHelper.readBytes(this.getDeviceI2CAddr(), Register.RA_YG_OFFS_USRH, 2).readInt16BE(0);
  }

  /**
   * Update the Y-axis gyro.
   * @param offset 
   */
  setYGyroOffset(offset: number): void {
    this.i2cHelper.writeWord(this.getDeviceI2CAddr(), Register.RA_YG_OFFS_USRH, offset);
  }

  /**
   * Access the Z-axis gyro offset.
   * @returns z-axis offset
   */
  getZGyroOffset(): number {
    return this.i2cHelper.readBytes(this.getDeviceI2CAddr(), Register.RA_ZG_OFFS_USRH, 2).readInt16BE(0);
  }

  /**
   * Update the Z-axis gyro.
   * @param offset 
   */
  setZGyroOffset(offset: number): void {
    this.i2cHelper.writeWord(this.getDeviceI2CAddr(), Register.RA_ZG_OFFS_USRH, offset);
  }

  /**
   * Access the DMP enable/disable status.
   * @returns true when enabled; false otherwise.
   */
  getDMPEnabled(): boolean {
    return this.i2cHelper.readBit(this.getDeviceI2CAddr(), Register.RA_USER_CTRL, USERCTRL_DMP_EN_BIT) === 1;
  }

  /**
   * Enable or disabled the DMP.
   * @param enabled
   */
  setDMPEnabled(enabled: boolean): void {
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_USER_CTRL, USERCTRL_DMP_EN_BIT, enabled ? 1 : 0);
  }

  /**
   * Asynchronously reset DMP module. This bit auto clears after one clock cycle.
   */
  resetDMP(): void {
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), Register.RA_USER_CTRL, USERCTRL_DMP_RESET_BIT, 1);
  }

  // this is the most basic initialization I can create. with the intent that we access the register bytes as few times as needed to get the job done.
  // for detailed descriptins of all registers and there purpose google "MPU-6000/MPU-6050 Register Map and Descriptions"
  dmpInitialize(): void { // Lets get it over with fast Write everything once and set it up necely
    // Reset procedure per instructions in the "MPU-6000/MPU-6050 Register Map and Descriptions" page 41
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), 0x6B, 7, 1); // PWR_MGMT_1: reset with 100ms delay
    Utils.msleep(100);
    this.i2cHelper.writeBits(this.getDeviceI2CAddr(), 0x6A, 2, 3, 0b111); // full SIGNAL_PATH_RESET: with another 100ms delay
    Utils.msleep(100);
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), 0x6B, 0x01); // 1000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), 0x38, 0x00); // 0000 0000 INT_ENABLE: no Interrupt
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), 0x23, 0x00); // 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), 0x1C, 0x00); // 0000 0000 ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), 0x37, 0x80); // 1001 0000 INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), 0x6B, 0x01); // 0000 0001 PWR_MGMT_1: Clock Source Select PLL_X_gyro
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), 0x19, 0x04); // 0000 0100 SMPLRT_DIV: Divides the internal sample rate 400Hz ( Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), 0x1A, 0x01); // 0000 0001 CONFIG: Digital Low Pass Filter (DLPF) Configuration 188HZ  //Im betting this will be the beat
   
    this.writeProgMemoryBlock(dmpMemory); // Loads the DMP image into the MPU6050 Memory // Should Never Fail
   
    this.i2cHelper.writeWord(this.getDeviceI2CAddr(), 0x70, 0x0400); // DMP Program Start Address
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), 0x1B, 0x18); // 0001 1000 GYRO_CONFIG: 3 = +2000 Deg/sec
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), 0x6A, 0xC0); // 1100 1100 USER_CTRL: Enable Fifo and Reset Fifo
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), 0x38, 0x02); // 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
    this.i2cHelper.writeBit(this.getDeviceI2CAddr(), 0x6A, 2, 1);      // Reset FIFO one last time just for kicks. (MPUi2cWrite reads 0x6A first and only alters 1 bit and then saves the byte)

    this.setDMPEnabled(false); // disable DMP for compatibility with the MPU6050 library
  /*
      dmpPacketSize += 16;//DMP_FEATURE_6X_LP_QUAT
      dmpPacketSize += 6;//DMP_FEATURE_SEND_RAW_ACCEL
      dmpPacketSize += 6;//DMP_FEATURE_SEND_RAW_GYRO
  */
    this.dmpPacketSize = 28;
  }

  // BANK_SEL register
  // uint8_t bank, bool prefetchEnabled, bool userBank
  setMemoryBank(bank: number, prefetchEnabled = false, userBank = false): void {
    let bnk = bank & 0x1F;
    if (userBank) bnk |= 0x20;
    if (prefetchEnabled) bnk |= 0x40;
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), Register.RA_BANK_SEL, bnk);
  }

  // MEM_START_ADDR register

  setMemoryStartAddress(address: number): void {
    this.i2cHelper.writeByte(this.getDeviceI2CAddr(), Register.RA_MEM_START_ADDR, address);
  }

  // uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify
  writeProgMemoryBlock(data: Uint8Array, bank = 0, address = 0, verify = true): void {
    return this.writeMemoryBlock(data, bank, address, verify, true);
  }

  // uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem
  protected writeMemoryBlock(data: Uint8Array, bank = 0, address = 0, verify = true, useProgMem = false): void {
    this.setMemoryBank(bank);
    this.setMemoryStartAddress(address);
    const dataSize = data.length;
    let chunkSize = 0;
    let progBuffer: Buffer;
    if (useProgMem) progBuffer = Buffer.alloc(DMP_MEMORY_CHUNK_SIZE, 0);
    for (let i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;
        
        if (useProgMem) {
            // write the chunk of data as specified
            progBuffer = Buffer.alloc(chunkSize);
            for (let j = 0; j < chunkSize; j++) {
              progBuffer[j] = data[i + j]
            }
        } else {
            // write the chunk of data as specified
            progBuffer = Buffer.from(data.buffer, i);
        }

        this.i2cHelper.writeBytes(this.getDeviceI2CAddr(), Register.RA_MEM_R_W, chunkSize, progBuffer);

        // verify data if needed
        if (verify) {
            this.setMemoryBank(bank);
            this.setMemoryStartAddress(address);
            const buf = this.i2cHelper.readBytes(this.getDeviceI2CAddr(), Register.RA_MEM_R_W, chunkSize);
            if (progBuffer.compare(buf, 0, chunkSize) !== 0) {
              console.log("Block write verification error, bank: ", bank);
              return;
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;
        if (address > 255) address = 0;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address === 0) bank++;
            this.setMemoryBank(bank);
            this.setMemoryStartAddress(address);
        }
    }
  }

  dmpPacketAvailable(): boolean {
    return this.getFIFOCount() >= this.dmpGetFIFOPacketSize();
  }

  dmpGetFIFOPacketSize(): number {
    return this.dmpPacketSize;
  }

  /** Get latest byte from FIFO buffer no matter how much time has passed.
  * ===                  GetCurrentFIFOPacket                    ===
  * ================================================================
  * Returns 1) when nothing special was done
  *         2) when recovering from overflow
  *         0) when no valid data is available
  * ================================================================ */

  // I don't actually know how large this buffer is supposed to be, but
  // this seems like a good guess. This constant should properly be
  // defined elsewhere.
  // const FIFO_BUFFER_LENGTH = 32;

  dmpGetCurrentFIFOPacket(): Buffer | undefined { // overflow proof
    const FIFO_BUFFER_LENGTH = 32;
    const length = this.dmpGetFIFOPacketSize();
    let fifoC: number;
    // This section of code is for when we allowed more than 1 packet to be acquired
    const BreakTimer = Date.now();

    do {
      fifoC = this.getFIFOCount();
      if (fifoC  > length) {

        if (fifoC > 200) { // if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
          this.resetFIFO(); // Fixes any overflow corruption
          fifoC = 0;
          // eslint-disable-next-line no-cond-assign
          while (!(fifoC = this.getFIFOCount()) && ((Date.now() - BreakTimer) <= 11000)); // Get Next New Packet

        } else { // We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
          // eslint-disable-next-line no-cond-assign
          while ((fifoC = this.getFIFOCount()) > length) {  // Test each time just in case the MPU is writing to the FIFO Buffer
            fifoC -= length; // Save the last packet
            let RemoveBytes = 0;
            while (fifoC) { // fifo count will reach zero so this is safe
                RemoveBytes = Math.min(fifoC, FIFO_BUFFER_LENGTH); // Buffer Length is different than the packet length this will efficiently clear the buffer
                this.getFIFOBytes(RemoveBytes); // trash bytes
                fifoC -= RemoveBytes;
            }
          }
        }
      }
    
      if (!fifoC) return undefined; // Called too early no data or we timed out after FIFO Reset
      // We have 1 packet
      if ((Date.now() - BreakTimer) > (11000)) return undefined;
    
    } while (fifoC !== length);

    return this.getFIFOBytes(length); // Get 1 packet
  }

  dmpGetAccel(packet: Buffer): Accel {
    const accel = {
      acc_x: packet.readInt16BE(16),
      acc_y: packet.readInt16BE(18),
      acc_z: packet.readInt16BE(20),
    };
    return accel;
  }

  // uint8_t MPU6050::dmpGet6AxisQuaternion(long *data, const uint8_t* packet);
  // uint8_t MPU6050::dmpGetRelativeQuaternion(long *data, const uint8_t* packet);
  dmpGetGyro(packet: Buffer): Gyro {
    const gyro: Gyro = {
      gyro_x: packet.readInt16BE(22),
      gyro_y: packet.readInt16BE(24),
      gyro_z: packet.readInt16BE(26)
    }
    return gyro;
  }

  /** 
   * Get raw 6-axis motion sensor readings (accel/gyro).
   * Retrieves all currently available motion sensor values.
   * @see getAcceleration()
   * @see getRotation()
   * @see MPU6050.RA_ACCEL_XOUT_H
   */
  dmpGetMotionData(packet: Buffer): MotionData {
    return {
      accel: this.dmpGetAccel(packet), 
      gyro: this.dmpGetGyro(packet)
    };
  }

  dmpGetQuaternion(packet: Buffer): Quaternion {
    const quaternion = {
        w: packet.readInt32BE(0),
        x: packet.readInt32BE(4),
        y: packet.readInt32BE(8),
        z: packet.readInt32BE(12)
      };
      return quaternion;
  }

  dmpGetGravity(packet: Buffer): Vector3 {
    /* +1g corresponds to +8192, sensitivity is 2g. */
    const qI = this.dmpGetQuaternion(packet);
    const x = (Math.trunc(qI.x) * qI.z - Math.trunc(qI.w) * qI.y) / 16384;
    const y = (Math.trunc(qI.w) * qI.x + Math.trunc(qI.y) * qI.z) / 16384;
    const z = (Math.trunc(qI.w) * qI.w - Math.trunc(qI.x) * qI.x
	       - Math.trunc(qI.y) * qI.y + Math.trunc(qI.z) * qI.z) / (2 * 16384);
    return {x, y, z};
  }

  dmpGetEuler(q: Quaternion): Euler {
    const psi = Math.atan2(2 * q.x * q.y - 2 * q.w * q.z, 2 * q.w * q.w + 2 * q.x * q.x - 1);   // psi
    const theta = -Math.asin(2 * q.x * q.z + 2 * q.w * q.y);                              // theta
    const phi = Math.atan2(2 * q.y * q.z - 2 * q.w * q.x, 2 * q.w * q.w + 2 * q.z * q.z - 1);   // phi
    return {psi, theta, phi};
  }

  dmpGetYawPitchRoll(q: Quaternion, gravity: Vector3): RPY {
    // yaw: (about Z axis)
    const yaw = Math.atan2(2 * q.x * q.y - 2 * q.w * q.z,  2 * q.w * q.w + 2 * q.x * q.x - 1);

    // pitch: (nose up/down, about Y axis)
    let pitch = Math.atan2(gravity.x , Math.sqrt(gravity.y * gravity.y + gravity.z * gravity.z));

    // roll: (tilt left/right, about X axis)
    const roll = Math.atan2(gravity.y , gravity.z);

    if (gravity.z < 0) {
      if(pitch > 0) {
        pitch = Math.PI - pitch; 
      } else { 
        pitch = -Math.PI - pitch;
      }
    }

    return {
      roll,
      pitch,
      yaw
    }
  }

  dmpGetLinearAccel(accel: Accel, gravity: Vector3): Vector3 {
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
    return {
      x: accel.acc_x - gravity.x * 8192,
      y: accel.acc_y - gravity.y * 8192,
      z: accel.acc_z - gravity.z * 8192
    };
  }

  // Calibration Routines  
  /**
    @brief Fully calibrate Gyro from ZERO in about 6-7 Loops 600-700 readings
  */
  calibrateGyro(loops: number): void {
    let kP = 0.3;
    let kI = 90;
    const x = (100 - map(loops, 1, 5, 20, 0)) * .01;
    kP *= x;
    kI *= x;
    
    this.PID( 0x43,  kP, kI,  loops);
  }

  /**
    @brief      Fully calibrate Accel from ZERO in about 6-7 Loops 600-700 readings
  */
  calibrateAccel(loops: number): void {
    let kP = 0.3;
    let kI = 20;
    const x = (100 - map(loops, 1, 5, 20, 0)) * .01;
    kP *= x;
    kI *= x;
    this.PID( 0x3B, kP, kI,  loops);
  }

  // uint8_t ReadAddress, float kP,float kI, uint8_t Loops
  PID(ReadAddress: number, kP: number, kI: number, Loops: number): void {
    // eslint-disable-next-line no-nested-ternary
    const SaveAddress = ReadAddress === 0x3B ? (this.getDeviceID() < 0x38  ? 0x06 : 0x77) : 0x13;
    let Data: number; // 
    let Reading: number; // float
    const BitZero = Int16Array.from([0,0,0]); // int16
    const shift = (SaveAddress === 0x77) ? 3 : 2;  // eslint-disable-line no-nested-ternary
    let Error = 0;
    let PTerm = 0;
    const ITerm = Float32Array.from([0,0,0]); // float
    let eSample; // int16_t
    let eSum ; // uint32_t  
    process.stdout.write('>');
    for (let i = 0; i < 3; i++) {
      Data = this.i2cHelper.readWord(this.getDeviceI2CAddr(), SaveAddress + (i * shift)); // reads 1 or more 16 bit integers (Word)
      Reading = Data;
      if(SaveAddress !== 0x13){
        BitZero[i] = Data & 1; // Capture Bit Zero to properly handle Accelerometer calibration
        ITerm[i] = Reading * 8;
        } else {
        ITerm[i] = Reading * 4;
      }
    }
    for (let L = 0; L < Loops; L++) {
      eSample = 0;
      for (let c = 0; c < 100; c++) { // 100 PI Calculations
        eSum = 0;
        for (let i = 0; i < 3; i++) {
          Data = this.i2cHelper.readWord(this.getDeviceI2CAddr(), ReadAddress + (i * 2)); // reads 1 or more 16 bit integers (Word)
          Reading = Data;
          if ((ReadAddress === 0x3B)&&(i === 2)) Reading -= 16384;	// remove Gravity
          Error = -Reading;
          eSum += Math.abs(Reading);
          PTerm = kP * Error;
          ITerm[i] += (Error * 0.001) * kI;				// Integral term 1000 Calculations a second = 0.001
          if(SaveAddress !== 0x13){
            Data = Math.round((PTerm + ITerm[i] ) / 8);		// Compute PID Output
            Data = (Data & 0xFFFE) | BitZero[i];			// Insert Bit0 Saved at beginning
          } else Data = Math.round((PTerm + ITerm[i] ) / 4);	// Compute PID Output
          this.i2cHelper.writeWord(this.getDeviceI2CAddr(), SaveAddress + (i * shift), Data);
        }
        if((c === 99) && eSum > 1000){						// Error is still to great to continue 
          c = 0;
          process.stdout.write('*');
        }
        if((eSum * ((ReadAddress === 0x3B) ? 0.05 : 1)) < 5) eSample++;	// Successfully found offsets prepare to  advance
        if((eSum < 100) && (c > 10) && (eSample >= 10)) break;		// Advance to next Loop
        Utils.msleep(1);
      }
      process.stdout.write('.');
      kP *= .75;
      kI *= .75;
      for (let i = 0; i < 3; i++){
        if(SaveAddress !== 0x13) {
          Data = Math.round((ITerm[i] ) / 8);		// Compute PID Output
          Data = (Data & 0xFFFE) | BitZero[i];	// Insert Bit0 Saved at beginning
        } else Data = Math.round((ITerm[i]) / 4);
        this.i2cHelper.writeWord(this.getDeviceI2CAddr(), SaveAddress + (i * shift), Data);
      }
    }
    this.resetFIFO();
    this.resetDMP();
    console.log();
  }

  printActiveOffsets(): void {
    const AOffsetRegister = this.getDeviceID() < 0x38 ? Register.RA_XA_OFFS_H : 0x77;
    const Data = Int16Array.from([0,0,0,0,0,0]);;

    if (AOffsetRegister === 0x06)	{
      const buf = this.i2cHelper.readBytes(this.getDeviceI2CAddr(), AOffsetRegister, 6);
      Data[0] = buf.readUInt16BE(0);
      Data[1] = buf.readUInt16BE(2);
      Data[2] = buf.readUInt16BE(4);
    } else {
      Data[0] = this.i2cHelper.readWord(this.getDeviceI2CAddr(), AOffsetRegister);
      Data[1] = this.i2cHelper.readWord(this.getDeviceI2CAddr(), AOffsetRegister+3);
      Data[2] = this.i2cHelper.readWord(this.getDeviceI2CAddr(), AOffsetRegister+6);
    }
    //	A_OFFSET_H_READ_A_OFFS(Data);
    // this.i2cHelper.readWords(0x13, 3, (uint16_t *)Data);
    const buf = this.i2cHelper.readBytes(this.getDeviceI2CAddr(), 0x13, 6);
    Data[3] = buf.readUInt16BE(0);
    Data[4] = buf.readUInt16BE(2);
    Data[5] = buf.readUInt16BE(4);
    
    console.log("           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro");
    const output = printf('OFFSETS    %7d  %7d  %7d  %7d  %7d  %7d', Data[0], Data[1], Data[2], Data[3], Data[4], Data[5]);
    console.log(output);
  }

}

function map(x: number, in_min: number, in_max: number, out_min: number, out_max: number): number {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

function createMotionData(): MotionData {
  return { accel: {acc_x: 0, acc_y: 0, acc_z: 0}, gyro: {gyro_x: 0, gyro_y: 0, gyro_z: 0}};
}

// Internal Constants

// Accelerometer sensitivity - Full Scale Range (default +/- 2g)
const ACC_2G_LSB = 16384;
  
// Gyro sensitivity - Full Scale Range (default +/- 250 deg/sec)
const GYRO_250_LSB = 131;

const WHO_AM_I_BIT                   = 6;
const WHO_AM_I_LENGTH                = 6;
  
const PWR1_DEVICE_RESET_BIT          = 7;
const PWR1_SLEEP_BIT                 = 6;
const PWR1_CYCLE_BIT                 = 5;
const PWR1_TEMP_DIS_BIT              = 3;
const PWR1_CLKSEL_BIT                = 2;
const PWR1_CLKSEL_LENGTH             = 3;
  
  
const GCONFIG_FS_SEL_BIT             = 4
const GCONFIG_FS_SEL_LENGTH          = 2
  
const ACONFIG_XA_ST_BIT              = 7;
const ACONFIG_YA_ST_BIT              = 6;
const ACONFIG_ZA_ST_BIT              = 5;
const ACONFIG_AFS_SEL_BIT            = 4;
const ACONFIG_AFS_SEL_LENGTH         = 2;
const ACONFIG_ACCEL_HPF_BIT          = 2;
const ACONFIG_ACCEL_HPF_LENGTH       = 3;
  
const TEMP_FIFO_EN_BIT               = 7;
const XG_FIFO_EN_BIT                 = 6;
const YG_FIFO_EN_BIT                 = 5;
const ZG_FIFO_EN_BIT                 = 4;
const ACCEL_FIFO_EN_BIT              = 3;
const SLV2_FIFO_EN_BIT               = 2;
const SLV1_FIFO_EN_BIT               = 1;
const SLV0_FIFO_EN_BIT               = 0;
  
const INTCFG_INT_LEVEL_BIT           = 7;
const INTCFG_INT_OPEN_BIT            = 6;
const INTCFG_LATCH_INT_EN_BIT        = 5;
const INTCFG_INT_RD_CLEAR_BIT        = 4;
const INTCFG_FSYNC_INT_LEVEL_BIT     = 3;
const INTCFG_FSYNC_INT_EN_BIT        = 2;
const INTCFG_I2C_BYPASS_EN_BIT       = 1;
const INTCFG_CLKOUT_EN_BIT           = 0;
  
const INTERRUPT_FF_BIT               = 7;
const INTERRUPT_MOT_BIT              = 6;
const INTERRUPT_ZMOT_BIT             = 5;
const INTERRUPT_FIFO_OFLOW_BIT       = 4;
const INTERRUPT_I2C_MST_INT_BIT      = 3;
const INTERRUPT_PLL_RDY_INT_BIT      = 2;
const INTERRUPT_DMP_INT_BIT          = 1;
const INTERRUPT_DATA_RDY_BIT         = 0;
  
const USERCTRL_DMP_EN_BIT            = 7;
const USERCTRL_FIFO_EN_BIT           = 6;
const USERCTRL_I2C_MST_EN_BIT        = 5;
const USERCTRL_I2C_IF_DIS_BIT        = 4;
const USERCTRL_DMP_RESET_BIT         = 3;
const USERCTRL_FIFO_RESET_BIT        = 2;
const USERCTRL_I2C_MST_RESET_BIT     = 1;
const USERCTRL_SIG_COND_RESET_BIT    = 0;

const DMP_MEMORY_BANKS               = 8;
const DMP_MEMORY_BANK_SIZE           = 256;
const DMP_MEMORY_CHUNK_SIZE          = 16;

const MPU6050_DMP_CODE_SIZE = 3062;    // dmpMemory[]

/* ================================================================ *
 | Default MotionApps v6.12 28-byte FIFO packet structure:           |
 |                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  |
 |                                                                  |
 | [GYRO X][GYRO Y][GYRO Z][ACC X ][ACC Y ][ACC Z ]					|
 |  16  17  18  19  20  21  22  23  24  25  26  27					|
 * ================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)

// *** this is a capture of the DMP Firmware V6.1.2 after all the messy changes were made so we can just load it
// const  dmpMemory[MPU6050.DMP_CODE_SIZE] = [
const dmpMemory = Uint8Array.from([
  /* bank # 0 */
  0x00, 0xF8, 0xF6, 0x2A, 0x3F, 0x68, 0xF5, 0x7A, 0x00, 0x06, 0xFF, 0xFE, 0x00, 0x03, 0x00, 0x00,
  0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
  0x03, 0x0C, 0x30, 0xC3, 0x0A, 0x74, 0x56, 0x2D, 0x0D, 0x62, 0xDB, 0xC7, 0x16, 0xF4, 0xBA, 0x02,
  0x38, 0x83, 0xF8, 0x83, 0x30, 0x00, 0xF8, 0x83, 0x25, 0x8E, 0xF8, 0x83, 0x30, 0x00, 0xF8, 0x83,
  0xFF, 0xFF, 0xFF, 0xFF, 0x0C, 0xBD, 0xD8, 0x11, 0x24, 0x00, 0x04, 0x00, 0x1A, 0x82, 0x79, 0xA1,
  0x00, 0x36, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x38, 0x83, 0x6F, 0xA2,
  0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
  0x1F, 0xA4, 0xE8, 0xE4, 0xFF, 0xF5, 0xDC, 0xB9, 0x00, 0x5B, 0x79, 0xCF, 0x1F, 0x3F, 0x78, 0x76,
  0x00, 0x86, 0x7C, 0x5A, 0x00, 0x86, 0x23, 0x47, 0xFA, 0xB9, 0x86, 0x31, 0x00, 0x74, 0x87, 0x8A,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x05, 0xFF, 0xFF, 0xE9, 0xA8, 0x00, 0x00, 0x21, 0x82,
  0xFA, 0xB8, 0x4D, 0x46, 0xFF, 0xFA, 0xDF, 0x3D, 0xFF, 0xFF, 0xB2, 0xB3, 0x00, 0x00, 0x00, 0x00,
  0x3F, 0xFF, 0xBA, 0x98, 0x00, 0x5D, 0xAC, 0x08, 0x00, 0x0A, 0x63, 0x78, 0x00, 0x01, 0x46, 0x21,
  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x42, 0xB5, 0x00, 0x06, 0x00, 0x64, 0x00, 0x64, 0x00, 0x06,
  0x14, 0x06, 0x02, 0x9F, 0x0F, 0x47, 0x91, 0x32, 0xD9, 0x0E, 0x9F, 0xC9, 0x1D, 0xCF, 0x4C, 0x34,
  0x3B, 0xB6, 0x7A, 0xE8, 0x00, 0x64, 0x00, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE,
  /* bank # 1 */
  0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x07, 0x00, 0x00, 0xFF, 0xF1, 0x00, 0x00, 0xFA, 0x46, 0x00, 0x00, 0xA2, 0xB8, 0x00, 0x00,
  0x10, 0x00, 0x00, 0x00, 0x04, 0xD6, 0x00, 0x00, 0x04, 0xCC, 0x00, 0x00, 0x04, 0xCC, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x06, 0x00, 0x02, 0x00, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x05, 0x00, 0x64, 0x00, 0x20, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x32, 0xF8, 0x98, 0x00, 0x00, 0xFF, 0x65, 0x00, 0x00, 0x83, 0x0F, 0x00, 0x00,
  0x00, 0x06, 0x00, 0x00, 0xFF, 0xF1, 0x00, 0x00, 0xFA, 0x46, 0x00, 0x00, 0xA2, 0xB8, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0D, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x02, 0x00, 0x00,
  0x00, 0x01, 0xFB, 0x83, 0x00, 0x7C, 0x00, 0x00, 0xFB, 0x15, 0xFC, 0x00, 0x1F, 0xB4, 0xFF, 0x83,
  0x00, 0x00, 0x00, 0x01, 0x00, 0x65, 0x00, 0x07, 0x00, 0x64, 0x03, 0xE8, 0x00, 0x64, 0x00, 0x28,
  0x00, 0x00, 0xFF, 0xFD, 0x00, 0x00, 0x00, 0x00, 0x16, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x10, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF4, 0x00, 0x00, 0x10, 0x00,
  /* bank # 2 */
  0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x01, 0x00, 0x05, 0xBA, 0xC6, 0x00, 0x47, 0x78, 0xA2,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
  0x00, 0x00, 0x23, 0xBB, 0x00, 0x2E, 0xA2, 0x5B, 0x00, 0x00, 0x05, 0x68, 0x00, 0x0B, 0xCF, 0x49,
  0x00, 0x04, 0xFF, 0xFD, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x64, 0x00, 0x07, 0x00, 0x08, 0x00, 0x06, 0x00, 0x06, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x2E, 0xA2, 0x5B, 0x00, 0x00, 0x05, 0x68, 0x00, 0x0B, 0xCF, 0x49, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xF8, 0xF6, 0x2A, 0x3F, 0x68, 0xF5, 0x7A, 0x00, 0x04, 0xFF, 0xFD, 0x00, 0x02, 0x00, 0x00,
  0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x0E,
  0xFF, 0xFF, 0xFF, 0xCF, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xFF, 0xFF, 0xFF, 0x9C,
  0x00, 0x00, 0x43, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64,
  0xFF, 0xE5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  /* bank # 3 */
  0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x24, 0x26, 0xD3,
  0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x00, 0x96, 0x00, 0x3C,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x9E, 0x65, 0x5D,
  0x0C, 0x0A, 0x4E, 0x68, 0xCD, 0xCF, 0x77, 0x09, 0x50, 0x16, 0x67, 0x59, 0xC6, 0x19, 0xCE, 0x82,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x71, 0x1C,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xD7, 0x84, 0x00, 0x03, 0x00, 0x00, 0x00,
  0x00, 0x11, 0xDC, 0x47, 0x03, 0x00, 0x00, 0x00, 0xC7, 0x93, 0x8F, 0x9D, 0x1E, 0x1B, 0x1C, 0x19,
  0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0E, 0xDF, 0xA4, 0x38, 0x1F, 0x9E, 0x65, 0x5D,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x71, 0x1C, 0x02, 0x03, 0x18, 0x85, 0x00, 0x00, 0x40, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x3F, 0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xF4, 0xC9, 0xFF, 0xFF, 0xBC, 0xF0, 0x00, 0x01, 0x0C, 0x0F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xF5, 0xB7, 0xBA, 0xB3, 0x67, 0x7D, 0xDF, 0x7E, 0x72, 0x90, 0x2E, 0x55, 0x4C, 0xF6, 0xE6, 0x88,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  /* bank # 4 */
  0xD8, 0xDC, 0xB4, 0xB8, 0xB0, 0xD8, 0xB9, 0xAB, 0xF3, 0xF8, 0xFA, 0xB3, 0xB7, 0xBB, 0x8E, 0x9E,
  0xAE, 0xF1, 0x32, 0xF5, 0x1B, 0xF1, 0xB4, 0xB8, 0xB0, 0x80, 0x97, 0xF1, 0xA9, 0xDF, 0xDF, 0xDF,
  0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0x4C, 0xCD, 0x6C, 0xA9, 0x0C, 0xC9, 0x2C, 0x97, 0xF1, 0xA9,
  0x89, 0x26, 0x46, 0x66, 0xB2, 0x89, 0x99, 0xA9, 0x2D, 0x55, 0x7D, 0xB0, 0xB0, 0x8A, 0xA8, 0x96,
  0x36, 0x56, 0x76, 0xF1, 0xBA, 0xA3, 0xB4, 0xB2, 0x80, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB2, 0x83,
  0x98, 0xBA, 0xA3, 0xF0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xB2, 0xB9, 0xB4, 0x98, 0x83, 0xF1,
  0xA3, 0x29, 0x55, 0x7D, 0xBA, 0xB5, 0xB1, 0xA3, 0x83, 0x93, 0xF0, 0x00, 0x28, 0x50, 0xF5, 0xB2,
  0xB6, 0xAA, 0x83, 0x93, 0x28, 0x54, 0x7C, 0xF1, 0xB9, 0xA3, 0x82, 0x93, 0x61, 0xBA, 0xA2, 0xDA,
  0xDE, 0xDF, 0xDB, 0x81, 0x9A, 0xB9, 0xAE, 0xF5, 0x60, 0x68, 0x70, 0xF1, 0xDA, 0xBA, 0xA2, 0xDF,
  0xD9, 0xBA, 0xA2, 0xFA, 0xB9, 0xA3, 0x82, 0x92, 0xDB, 0x31, 0xBA, 0xA2, 0xD9, 0xBA, 0xA2, 0xF8,
  0xDF, 0x85, 0xA4, 0xD0, 0xC1, 0xBB, 0xAD, 0x83, 0xC2, 0xC5, 0xC7, 0xB8, 0xA2, 0xDF, 0xDF, 0xDF,
  0xBA, 0xA0, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xAA, 0xB3, 0x8D, 0xB4, 0x98, 0x0D, 0x35,
  0x5D, 0xB2, 0xB6, 0xBA, 0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A,
  0xB8, 0xAA, 0x87, 0x2C, 0x54, 0x7C, 0xBA, 0xA4, 0xB0, 0x8A, 0xB6, 0x91, 0x32, 0x56, 0x76, 0xB2,
  0x84, 0x94, 0xA4, 0xC8, 0x08, 0xCD, 0xD8, 0xB8, 0xB4, 0xB0, 0xF1, 0x99, 0x82, 0xA8, 0x2D, 0x55,
  0x7D, 0x98, 0xA8, 0x0E, 0x16, 0x1E, 0xA2, 0x2C, 0x54, 0x7C, 0x92, 0xA4, 0xF0, 0x2C, 0x50, 0x78,
  /* bank # 5 */
  0xF1, 0x84, 0xA8, 0x98, 0xC4, 0xCD, 0xFC, 0xD8, 0x0D, 0xDB, 0xA8, 0xFC, 0x2D, 0xF3, 0xD9, 0xBA,
  0xA6, 0xF8, 0xDA, 0xBA, 0xA6, 0xDE, 0xD8, 0xBA, 0xB2, 0xB6, 0x86, 0x96, 0xA6, 0xD0, 0xF3, 0xC8,
  0x41, 0xDA, 0xA6, 0xC8, 0xF8, 0xD8, 0xB0, 0xB4, 0xB8, 0x82, 0xA8, 0x92, 0xF5, 0x2C, 0x54, 0x88,
  0x98, 0xF1, 0x35, 0xD9, 0xF4, 0x18, 0xD8, 0xF1, 0xA2, 0xD0, 0xF8, 0xF9, 0xA8, 0x84, 0xD9, 0xC7,
  0xDF, 0xF8, 0xF8, 0x83, 0xC5, 0xDA, 0xDF, 0x69, 0xDF, 0x83, 0xC1, 0xD8, 0xF4, 0x01, 0x14, 0xF1,
  0xA8, 0x82, 0x4E, 0xA8, 0x84, 0xF3, 0x11, 0xD1, 0x82, 0xF5, 0xD9, 0x92, 0x28, 0x97, 0x88, 0xF1,
  0x09, 0xF4, 0x1C, 0x1C, 0xD8, 0x84, 0xA8, 0xF3, 0xC0, 0xF9, 0xD1, 0xD9, 0x97, 0x82, 0xF1, 0x29,
  0xF4, 0x0D, 0xD8, 0xF3, 0xF9, 0xF9, 0xD1, 0xD9, 0x82, 0xF4, 0xC2, 0x03, 0xD8, 0xDE, 0xDF, 0x1A,
  0xD8, 0xF1, 0xA2, 0xFA, 0xF9, 0xA8, 0x84, 0x98, 0xD9, 0xC7, 0xDF, 0xF8, 0xF8, 0xF8, 0x83, 0xC7,
  0xDA, 0xDF, 0x69, 0xDF, 0xF8, 0x83, 0xC3, 0xD8, 0xF4, 0x01, 0x14, 0xF1, 0x98, 0xA8, 0x82, 0x2E,
  0xA8, 0x84, 0xF3, 0x11, 0xD1, 0x82, 0xF5, 0xD9, 0x92, 0x50, 0x97, 0x88, 0xF1, 0x09, 0xF4, 0x1C,
  0xD8, 0x84, 0xA8, 0xF3, 0xC0, 0xF8, 0xF9, 0xD1, 0xD9, 0x97, 0x82, 0xF1, 0x49, 0xF4, 0x0D, 0xD8,
  0xF3, 0xF9, 0xF9, 0xD1, 0xD9, 0x82, 0xF4, 0xC4, 0x03, 0xD8, 0xDE, 0xDF, 0xD8, 0xF1, 0xAD, 0x88,
  0x98, 0xCC, 0xA8, 0x09, 0xF9, 0xD9, 0x82, 0x92, 0xA8, 0xF5, 0x7C, 0xF1, 0x88, 0x3A, 0xCF, 0x94,
  0x4A, 0x6E, 0x98, 0xDB, 0x69, 0x31, 0xDA, 0xAD, 0xF2, 0xDE, 0xF9, 0xD8, 0x87, 0x95, 0xA8, 0xF2,
  0x21, 0xD1, 0xDA, 0xA5, 0xF9, 0xF4, 0x17, 0xD9, 0xF1, 0xAE, 0x8E, 0xD0, 0xC0, 0xC3, 0xAE, 0x82,
  /* bank # 6 */
  0xC6, 0x84, 0xC3, 0xA8, 0x85, 0x95, 0xC8, 0xA5, 0x88, 0xF2, 0xC0, 0xF1, 0xF4, 0x01, 0x0E, 0xF1,
  0x8E, 0x9E, 0xA8, 0xC6, 0x3E, 0x56, 0xF5, 0x54, 0xF1, 0x88, 0x72, 0xF4, 0x01, 0x15, 0xF1, 0x98,
  0x45, 0x85, 0x6E, 0xF5, 0x8E, 0x9E, 0x04, 0x88, 0xF1, 0x42, 0x98, 0x5A, 0x8E, 0x9E, 0x06, 0x88,
  0x69, 0xF4, 0x01, 0x1C, 0xF1, 0x98, 0x1E, 0x11, 0x08, 0xD0, 0xF5, 0x04, 0xF1, 0x1E, 0x97, 0x02,
  0x02, 0x98, 0x36, 0x25, 0xDB, 0xF9, 0xD9, 0x85, 0xA5, 0xF3, 0xC1, 0xDA, 0x85, 0xA5, 0xF3, 0xDF,
  0xD8, 0x85, 0x95, 0xA8, 0xF3, 0x09, 0xDA, 0xA5, 0xFA, 0xD8, 0x82, 0x92, 0xA8, 0xF5, 0x78, 0xF1,
  0x88, 0x1A, 0x84, 0x9F, 0x26, 0x88, 0x98, 0x21, 0xDA, 0xF4, 0x1D, 0xF3, 0xD8, 0x87, 0x9F, 0x39,
  0xD1, 0xAF, 0xD9, 0xDF, 0xDF, 0xFB, 0xF9, 0xF4, 0x0C, 0xF3, 0xD8, 0xFA, 0xD0, 0xF8, 0xDA, 0xF9,
  0xF9, 0xD0, 0xDF, 0xD9, 0xF9, 0xD8, 0xF4, 0x0B, 0xD8, 0xF3, 0x87, 0x9F, 0x39, 0xD1, 0xAF, 0xD9,
  0xDF, 0xDF, 0xF4, 0x1D, 0xF3, 0xD8, 0xFA, 0xFC, 0xA8, 0x69, 0xF9, 0xF9, 0xAF, 0xD0, 0xDA, 0xDE,
  0xFA, 0xD9, 0xF8, 0x8F, 0x9F, 0xA8, 0xF1, 0xCC, 0xF3, 0x98, 0xDB, 0x45, 0xD9, 0xAF, 0xDF, 0xD0,
  0xF8, 0xD8, 0xF1, 0x8F, 0x9F, 0xA8, 0xCA, 0xF3, 0x88, 0x09, 0xDA, 0xAF, 0x8F, 0xCB, 0xF8, 0xD8,
  0xF2, 0xAD, 0x97, 0x8D, 0x0C, 0xD9, 0xA5, 0xDF, 0xF9, 0xBA, 0xA6, 0xF3, 0xFA, 0xF4, 0x12, 0xF2,
  0xD8, 0x95, 0x0D, 0xD1, 0xD9, 0xBA, 0xA6, 0xF3, 0xFA, 0xDA, 0xA5, 0xF2, 0xC1, 0xBA, 0xA6, 0xF3,
  0xDF, 0xD8, 0xF1, 0xBA, 0xB2, 0xB6, 0x86, 0x96, 0xA6, 0xD0, 0xCA, 0xF3, 0x49, 0xDA, 0xA6, 0xCB,
  0xF8, 0xD8, 0xB0, 0xB4, 0xB8, 0xD8, 0xAD, 0x84, 0xF2, 0xC0, 0xDF, 0xF1, 0x8F, 0xCB, 0xC3, 0xA8,
  /* bank # 7 */
  0xB2, 0xB6, 0x86, 0x96, 0xC8, 0xC1, 0xCB, 0xC3, 0xF3, 0xB0, 0xB4, 0x88, 0x98, 0xA8, 0x21, 0xDB,
  0x71, 0x8D, 0x9D, 0x71, 0x85, 0x95, 0x21, 0xD9, 0xAD, 0xF2, 0xFA, 0xD8, 0x85, 0x97, 0xA8, 0x28,
  0xD9, 0xF4, 0x08, 0xD8, 0xF2, 0x8D, 0x29, 0xDA, 0xF4, 0x05, 0xD9, 0xF2, 0x85, 0xA4, 0xC2, 0xF2,
  0xD8, 0xA8, 0x8D, 0x94, 0x01, 0xD1, 0xD9, 0xF4, 0x11, 0xF2, 0xD8, 0x87, 0x21, 0xD8, 0xF4, 0x0A,
  0xD8, 0xF2, 0x84, 0x98, 0xA8, 0xC8, 0x01, 0xD1, 0xD9, 0xF4, 0x11, 0xD8, 0xF3, 0xA4, 0xC8, 0xBB,
  0xAF, 0xD0, 0xF2, 0xDE, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xD8, 0xF1, 0xB8, 0xF6,
  0xB5, 0xB9, 0xB0, 0x8A, 0x95, 0xA3, 0xDE, 0x3C, 0xA3, 0xD9, 0xF8, 0xD8, 0x5C, 0xA3, 0xD9, 0xF8,
  0xD8, 0x7C, 0xA3, 0xD9, 0xF8, 0xD8, 0xF8, 0xF9, 0xD1, 0xA5, 0xD9, 0xDF, 0xDA, 0xFA, 0xD8, 0xB1,
  0x85, 0x30, 0xF7, 0xD9, 0xDE, 0xD8, 0xF8, 0x30, 0xAD, 0xDA, 0xDE, 0xD8, 0xF2, 0xB4, 0x8C, 0x99,
  0xA3, 0x2D, 0x55, 0x7D, 0xA0, 0x83, 0xDF, 0xDF, 0xDF, 0xB5, 0x91, 0xA0, 0xF6, 0x29, 0xD9, 0xFB,
  0xD8, 0xA0, 0xFC, 0x29, 0xD9, 0xFA, 0xD8, 0xA0, 0xD0, 0x51, 0xD9, 0xF8, 0xD8, 0xFC, 0x51, 0xD9,
  0xF9, 0xD8, 0x79, 0xD9, 0xFB, 0xD8, 0xA0, 0xD0, 0xFC, 0x79, 0xD9, 0xFA, 0xD8, 0xA1, 0xF9, 0xF9,
  0xF9, 0xF9, 0xF9, 0xA0, 0xDA, 0xDF, 0xDF, 0xDF, 0xD8, 0xA1, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xAC,
  0xDE, 0xF8, 0xAD, 0xDE, 0x83, 0x93, 0xAC, 0x2C, 0x54, 0x7C, 0xF1, 0xA8, 0xDF, 0xDF, 0xDF, 0xF6,
  0x9D, 0x2C, 0xDA, 0xA0, 0xDF, 0xD9, 0xFA, 0xDB, 0x2D, 0xF8, 0xD8, 0xA8, 0x50, 0xDA, 0xA0, 0xD0,
  0xDE, 0xD9, 0xD0, 0xF8, 0xF8, 0xF8, 0xDB, 0x55, 0xF8, 0xD8, 0xA8, 0x78, 0xDA, 0xA0, 0xD0, 0xDF,
  /* bank # 8 */
  0xD9, 0xD0, 0xFA, 0xF8, 0xF8, 0xF8, 0xF8, 0xDB, 0x7D, 0xF8, 0xD8, 0x9C, 0xA8, 0x8C, 0xF5, 0x30,
  0xDB, 0x38, 0xD9, 0xD0, 0xDE, 0xDF, 0xA0, 0xD0, 0xDE, 0xDF, 0xD8, 0xA8, 0x48, 0xDB, 0x58, 0xD9,
  0xDF, 0xD0, 0xDE, 0xA0, 0xDF, 0xD0, 0xDE, 0xD8, 0xA8, 0x68, 0xDB, 0x70, 0xD9, 0xDF, 0xDF, 0xA0,
  0xDF, 0xDF, 0xD8, 0xF1, 0xA8, 0x88, 0x90, 0x2C, 0x54, 0x7C, 0x98, 0xA8, 0xD0, 0x5C, 0x38, 0xD1,
  0xDA, 0xF2, 0xAE, 0x8C, 0xDF, 0xF9, 0xD8, 0xB0, 0x87, 0xA8, 0xC1, 0xC1, 0xB1, 0x88, 0xA8, 0xC6,
  0xF9, 0xF9, 0xDA, 0x36, 0xD8, 0xA8, 0xF9, 0xDA, 0x36, 0xD8, 0xA8, 0xF9, 0xDA, 0x36, 0xD8, 0xA8,
  0xF9, 0xDA, 0x36, 0xD8, 0xA8, 0xF9, 0xDA, 0x36, 0xD8, 0xF7, 0x8D, 0x9D, 0xAD, 0xF8, 0x18, 0xDA,
  0xF2, 0xAE, 0xDF, 0xD8, 0xF7, 0xAD, 0xFA, 0x30, 0xD9, 0xA4, 0xDE, 0xF9, 0xD8, 0xF2, 0xAE, 0xDE,
  0xFA, 0xF9, 0x83, 0xA7, 0xD9, 0xC3, 0xC5, 0xC7, 0xF1, 0x88, 0x9B, 0xA7, 0x7A, 0xAD, 0xF7, 0xDE,
  0xDF, 0xA4, 0xF8, 0x84, 0x94, 0x08, 0xA7, 0x97, 0xF3, 0x00, 0xAE, 0xF2, 0x98, 0x19, 0xA4, 0x88,
  0xC6, 0xA3, 0x94, 0x88, 0xF6, 0x32, 0xDF, 0xF2, 0x83, 0x93, 0xDB, 0x09, 0xD9, 0xF2, 0xAA, 0xDF,
  0xD8, 0xD8, 0xAE, 0xF8, 0xF9, 0xD1, 0xDA, 0xF3, 0xA4, 0xDE, 0xA7, 0xF1, 0x88, 0x9B, 0x7A, 0xD8,
  0xF3, 0x84, 0x94, 0xAE, 0x19, 0xF9, 0xDA, 0xAA, 0xF1, 0xDF, 0xD8, 0xA8, 0x81, 0xC0, 0xC3, 0xC5,
  0xC7, 0xA3, 0x92, 0x83, 0xF6, 0x28, 0xAD, 0xDE, 0xD9, 0xF8, 0xD8, 0xA3, 0x50, 0xAD, 0xD9, 0xF8,
  0xD8, 0xA3, 0x78, 0xAD, 0xD9, 0xF8, 0xD8, 0xF8, 0xF9, 0xD1, 0xA1, 0xDA, 0xDE, 0xC3, 0xC5, 0xC7,
  0xD8, 0xA1, 0x81, 0x94, 0xF8, 0x18, 0xF2, 0xB0, 0x89, 0xAC, 0xC3, 0xC5, 0xC7, 0xF1, 0xD8, 0xB8,
  /* bank # 9 */
  0xB4, 0xB0, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97, 0x28, 0x88, 0x9B, 0xF0,
  0x0C, 0x20, 0x14, 0x40, 0xB0, 0xB4, 0xB8, 0xF0, 0xA8, 0x8A, 0x9A, 0x28, 0x50, 0x78, 0xB7, 0x9B,
  0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xF1, 0xBB, 0xAB,
  0x88, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0xB3, 0x8B, 0xB8, 0xA8, 0x04, 0x28, 0x50, 0x78, 0xF1, 0xB0,
  0x88, 0xB4, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xBB, 0xAB, 0xB3, 0x8B, 0x02, 0x26, 0x46, 0x66, 0xB0,
  0xB8, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79, 0x8A, 0x24, 0x70, 0x59,
  0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68, 0x8A, 0x64, 0x48, 0x31,
  0x8B, 0x30, 0x49, 0x60, 0x88, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04, 0x28,
  0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66, 0xF0,
  0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xA9,
  0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60, 0x8C,
  0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76, 0x7E,
  0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0xD8, 0xB1, 0xB5, 0xB9, 0xA3, 0xDF, 0xDF, 0xDF, 0xAE, 0xD0,
  0xDF, 0xAA, 0xD0, 0xDE, 0xF2, 0xAB, 0xF8, 0xF9, 0xD9, 0xB0, 0x87, 0xC4, 0xAA, 0xF1, 0xDF, 0xDF,
  0xBB, 0xAF, 0xDF, 0xDF, 0xB9, 0xD8, 0xB1, 0xF1, 0xA3, 0x97, 0x8E, 0x60, 0xDF, 0xB0, 0x84, 0xF2,
  0xC8, 0xF8, 0xF9, 0xD9, 0xDE, 0xD8, 0x93, 0x85, 0xF1, 0x4A, 0xB1, 0x83, 0xA3, 0x08, 0xB5, 0x83,
  /* bank # 10 */
  0x9A, 0x08, 0x10, 0xB7, 0x9F, 0x10, 0xD8, 0xF1, 0xB0, 0xBA, 0xAE, 0xB0, 0x8A, 0xC2, 0xB2, 0xB6,
  0x8E, 0x9E, 0xF1, 0xFB, 0xD9, 0xF4, 0x1D, 0xD8, 0xF9, 0xD9, 0x0C, 0xF1, 0xD8, 0xF8, 0xF8, 0xAD,
  0x61, 0xD9, 0xAE, 0xFB, 0xD8, 0xF4, 0x0C, 0xF1, 0xD8, 0xF8, 0xF8, 0xAD, 0x19, 0xD9, 0xAE, 0xFB,
  0xDF, 0xD8, 0xF4, 0x16, 0xF1, 0xD8, 0xF8, 0xAD, 0x8D, 0x61, 0xD9, 0xF4, 0xF4, 0xAC, 0xF5, 0x9C,
  0x9C, 0x8D, 0xDF, 0x2B, 0xBA, 0xB6, 0xAE, 0xFA, 0xF8, 0xF4, 0x0B, 0xD8, 0xF1, 0xAE, 0xD0, 0xF8,
  0xAD, 0x51, 0xDA, 0xAE, 0xFA, 0xF8, 0xF1, 0xD8, 0xB9, 0xB1, 0xB6, 0xA3, 0x83, 0x9C, 0x08, 0xB9,
  0xB1, 0x83, 0x9A, 0xB5, 0xAA, 0xC0, 0xFD, 0x30, 0x83, 0xB7, 0x9F, 0x10, 0xB5, 0x8B, 0x93, 0xF2,
  0x02, 0x02, 0xD1, 0xAB, 0xDA, 0xDE, 0xD8, 0xF1, 0xB0, 0x80, 0xBA, 0xAB, 0xC0, 0xC3, 0xB2, 0x84,
  0xC1, 0xC3, 0xD8, 0xB1, 0xB9, 0xF3, 0x8B, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xB0,
  0x87, 0x9C, 0xB9, 0xA3, 0xDD, 0xF1, 0xB3, 0x8B, 0x8B, 0x8B, 0x8B, 0x8B, 0xB0, 0x87, 0x20, 0x28,
  0x30, 0x38, 0xB2, 0x8B, 0xB6, 0x9B, 0xF2, 0xA3, 0xC0, 0xC8, 0xC2, 0xC4, 0xCC, 0xC6, 0xA3, 0xA3,
  0xA3, 0xF1, 0xB0, 0x87, 0xB5, 0x9A, 0xD8, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC, 0xBA, 0xAC, 0xDF, 0xB9,
  0xA3, 0xFE, 0xF2, 0xAB, 0xC4, 0xAA, 0xF1, 0xDF, 0xDF, 0xBB, 0xAF, 0xDF, 0xDF, 0xA3, 0xA3, 0xA3,
  0xD8, 0xD8, 0xD8, 0xBB, 0xB3, 0xB7, 0xF1, 0xAA, 0xF9, 0xDA, 0xFF, 0xD9, 0x80, 0x9A, 0xAA, 0x28,
  0xB4, 0x80, 0x98, 0xA7, 0x20, 0xB7, 0x97, 0x87, 0xA8, 0x66, 0x88, 0xF0, 0x79, 0x51, 0xF1, 0x90,
  0x2C, 0x87, 0x0C, 0xA7, 0x81, 0x97, 0x62, 0x93, 0xF0, 0x71, 0x71, 0x60, 0x85, 0x94, 0x01, 0x29,
  /* bank # 11 */
  0x51, 0x79, 0x90, 0xA5, 0xF1, 0x28, 0x4C, 0x6C, 0x87, 0x0C, 0x95, 0x18, 0x85, 0x78, 0xA3, 0x83,
  0x90, 0x28, 0x4C, 0x6C, 0x88, 0x6C, 0xD8, 0xF3, 0xA2, 0x82, 0x00, 0xF2, 0x10, 0xA8, 0x92, 0x19,
  0x80, 0xA2, 0xF2, 0xD9, 0x26, 0xD8, 0xF1, 0x88, 0xA8, 0x4D, 0xD9, 0x48, 0xD8, 0x96, 0xA8, 0x39,
  0x80, 0xD9, 0x3C, 0xD8, 0x95, 0x80, 0xA8, 0x39, 0xA6, 0x86, 0x98, 0xD9, 0x2C, 0xDA, 0x87, 0xA7,
  0x2C, 0xD8, 0xA8, 0x89, 0x95, 0x19, 0xA9, 0x80, 0xD9, 0x38, 0xD8, 0xA8, 0x89, 0x39, 0xA9, 0x80,
  0xDA, 0x3C, 0xD8, 0xA8, 0x2E, 0xA8, 0x39, 0x90, 0xD9, 0x0C, 0xD8, 0xA8, 0x95, 0x31, 0x98, 0xD9,
  0x0C, 0xD8, 0xA8, 0x09, 0xD9, 0xFF, 0xD8, 0x01, 0xDA, 0xFF, 0xD8, 0x95, 0x39, 0xA9, 0xDA, 0x26,
  0xFF, 0xD8, 0x90, 0xA8, 0x0D, 0x89, 0x99, 0xA8, 0x10, 0x80, 0x98, 0x21, 0xDA, 0x2E, 0xD8, 0x89,
  0x99, 0xA8, 0x31, 0x80, 0xDA, 0x2E, 0xD8, 0xA8, 0x86, 0x96, 0x31, 0x80, 0xDA, 0x2E, 0xD8, 0xA8,
  0x87, 0x31, 0x80, 0xDA, 0x2E, 0xD8, 0xA8, 0x82, 0x92, 0xF3, 0x41, 0x80, 0xF1, 0xD9, 0x2E, 0xD8,
  0xA8, 0x82, 0xF3, 0x19, 0x80, 0xF1, 0xD9, 0x2E, 0xD8, 0x82, 0xAC, 0xF3, 0xC0, 0xA2, 0x80, 0x22,
  0xF1, 0xA6, 0x2E, 0xA7, 0x2E, 0xA9, 0x22, 0x98, 0xA8, 0x29, 0xDA, 0xAC, 0xDE, 0xFF, 0xD8, 0xA2,
  0xF2, 0x2A, 0xF1, 0xA9, 0x2E, 0x82, 0x92, 0xA8, 0xF2, 0x31, 0x80, 0xA6, 0x96, 0xF1, 0xD9, 0x00,
  0xAC, 0x8C, 0x9C, 0x0C, 0x30, 0xAC, 0xDE, 0xD0, 0xDE, 0xFF, 0xD8, 0x8C, 0x9C, 0xAC, 0xD0, 0x10,
  0xAC, 0xDE, 0x80, 0x92, 0xA2, 0xF2, 0x4C, 0x82, 0xA8, 0xF1, 0xCA, 0xF2, 0x35, 0xF1, 0x96, 0x88,
  0xA6, 0xD9, 0x00, 0xD8, 0xF1, 0xFF,
]);

/* eslint-enable no-param-reassign */