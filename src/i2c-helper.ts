
import {Register} from './mpu6050';

/**
 * I/O utilities for I2C communications with MPU6050 devices.
 * This interface is based on https://github.com/ElectronicCats/mpu6050/blob/master/src/I2Cdev.h
 */
export interface I2CHelper {
  
  /** Read a single bit from an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @param bitNum Bit position to read (0-7)
   * @returns Status bit value
   */
  readBit(deviceAddr: number, regAddr: Register, bitNum: number): number;

  /** Read multiple bits from an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @param bitStart First bit position to read (0-7)
   * @param length Number of bits to read (not more than 8)
   * @returns The bits read
   */
  readBits(deviceAddr: number, regAddr: Register, bitStart: number, length: number): number;

  /** Read single byte from an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @returns The byte read.
   */
  readByte(deviceAddr: number, regAddr: Register): number;

  /** Read multiple bytes from an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr First register regAddr to read from
   * @param length Number of bytes to read
   * @returns Buffer containing the read bytes
   */
  readBytes(deviceAddr: number, regAddr: Register, byteCnt: number): Buffer;

  /** Read single word from a 16-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @return The 16-bit word read.
   */
  readWord(deviceAddr: number, regAddr: Register): number;

  /** 
   * Write a single bit in an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to write to
   * @param bitNum Bit position to write (0-7)
   * @param data New bit value to write
   * @returns Status of operation (0 = success)
   */
  writeBit(deviceAddr: number, regAddr: Register, bitNum: number, data: number): number;

  /** Write multiple bits in an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to write to
   * @param bitStart First bit position to write (0-7)
   * @param length Number of bits to write (not more than 8)
   * @param data Right-aligned value to write
   * @returns Status of operation (0 = success)
   */
  writeBits(deviceAddr: number, regAddr: Register, bitStart: number, length: number, data: number): number;
 
  /** Write single byte to an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register address to write to
   * @param data New byte value to write
   * @returns Status of operation (0 = success)
   */
  writeByte(deviceAddr: number, regAddr: Register, data: number): number;

  /** Write multiple bytes to an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr First register address to write to
   * @param length Number of bytes to write
   * @param data Buffer to copy new data from
   * @returns Status of operation (0 = success)
   */
  writeBytes(deviceAddr: number, regAddr: Register, length: number, data: Buffer): number

  /** Write 2 byte number to a 16-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr First register address to write to
   * @param length Number of words to write
   * @param data Data to copy 
   * @returns Status of operation (0 = success)
   */
  writeWord(deviceAddr: number, regAddr: Register, data: number): number;
}
