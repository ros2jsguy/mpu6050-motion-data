
import rpio = require('rpio');
import { I2CHelper } from './i2c-helper';
import {Register} from './mpu6050';

const DEFAULT_REGBUF_SZ = 4;
const DEFAULT_DATABUF_SZ = 42;

// TODO - repalce this constant with actual enum type returned from
// rpio.i2cWriteXXX() once the typescript definitions have been
// udpated, https://github.com/DefinitelyTyped/DefinitelyTyped/tree/master/types/rpio
const I2C_SUCCESS = 0;

/**
 * I2C utilitis for communicating to MPU6050 devices using the [rpio](https://github.com/jperkin/node-rpio) library.
 */
export class RpioI2CHelper implements I2CHelper {

  private regBuffer: Buffer;  // data sent to device
  private dataBuffer: Buffer; // data received from device

  constructor(baudRate: number, regBufferSize = DEFAULT_REGBUF_SZ, dataBufferSize = DEFAULT_DATABUF_SZ) {
     // configure RPIO
     // TODO - i2c should be setup/shutdown externally as i2c may have multiple slave devices
     console.log('rpio:', rpio, "\n", rpio.i2cBegin);
     rpio.i2cBegin();
     rpio.i2cSetBaudRate(baudRate);

    this.regBuffer = Buffer.alloc(Math.max(regBufferSize, DEFAULT_REGBUF_SZ), 0);
    this.dataBuffer = Buffer.alloc(Math.max(dataBufferSize, DEFAULT_DATABUF_SZ), 0);
  }

  shutdown(): void {
    // TODO revise - see note above
    rpio.i2cEnd();
  }

  /** Read a single bit from an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @param bitNum Bit position to read (0-7)
   * @returns Status bit value
   */
  readBit(deviceAddr: number, regAddr: Register, bitNum: number): number {
    const b = this.readByte(deviceAddr, regAddr);
    return  b & (1 << bitNum);
  }

  /** Read multiple bits from an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @param bitStart First bit position to read (0-7)
   * @param length Number of bits to read (not more than 8)
   * @returns The bits read
   */
  readBits(deviceAddr: number, regAddr: Register, bitStart: number, length: number): number {
    let b = this.readByte(deviceAddr, regAddr);
    const mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    return b;
  }

  /** Read single byte from an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @returns The byte read.
   */
  readByte(deviceAddr: number, regAddr: Register): number {
    const status = this.readBytes(deviceAddr, regAddr, 1);
    return this.dataBuffer.readUInt8(0);
  }

  /** Read multiple bytes from an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr First register regAddr to read from
   * @param length Number of bytes to read
   * @returns Buffer containing the read bytes
   */
  readBytes(deviceAddr: number, regAddr: Register, byteCnt: number): Buffer {
    // set register to read
    this.regBuffer.writeUInt8(regAddr, 0);
    rpio.i2cSetSlaveAddress(deviceAddr);
    rpio.i2cWrite(this.regBuffer, 1);

    // read register data
    rpio.i2cRead(this.dataBuffer, byteCnt);
    return this.dataBuffer;
  }

  /** Read single word from a 16-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @return The 16-bit word read.
   */
  readWord(deviceAddr: number, regAddr: Register): number {
    this.readBytes(deviceAddr, regAddr, 2);
    return this.dataBuffer.readInt16BE(0);
  }

 /** 
   * Write a single bit in an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to write to
   * @param bitNum Bit position to write (0-7)
   * @param data New bit value to write
   * @returns Status of operation (0 = success)
   */
   writeBit(deviceAddr: number, regAddr: Register, bitNum: number, data: number): number {
    let b = this.readByte(deviceAddr, regAddr);
    b = (data !== 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    this.writeByte(deviceAddr, regAddr, b);
    return I2C_SUCCESS;
  }

  /** Write multiple bits in an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register regAddr to write to
   * @param bitStart First bit position to write (0-7)
   * @param length Number of bits to write (not more than 8)
   * @param bits Right-aligned value to write
   * @returns Status of operation (0 = success)
   */
  writeBits(deviceAddr: number, regAddr: Register, bitStart: number, length: number, bits: number): number {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    let b = this.readByte(deviceAddr, regAddr);
    const mask = ((1 << length) - 1) << (bitStart - length + 1);
    let data = bits << (bitStart - length + 1); // bits data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    this.writeByte(deviceAddr, regAddr, b);
    return I2C_SUCCESS;
  }

  /** Write single byte to an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr Register address to write to
   * @param data New byte value to write
   * @returns Status of operation (0 = success)
   */
  writeByte(deviceAddr: number, regAddr: Register, data: number): number {
    // identify register and data to write
    this.dataBuffer.writeUInt8(regAddr, 0);
    this.dataBuffer.writeUInt8(data, 1);

    // write to register
    rpio.i2cSetSlaveAddress(deviceAddr);
    rpio.i2cWrite(this.dataBuffer, 2);
    return I2C_SUCCESS;
  }

  /** Write multiple bytes to an 8-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr First register address to write to
   * @param length Number of bytes to write
   * @param data Buffer to copy new data from
   * @returns Status of operation (0 = success)
   */
   writeBytes(deviceAddr: number, regAddr: Register, length: number, data: Buffer): number {

    // identify register and data to write
    this.dataBuffer.writeUInt8(regAddr,0);

    for (let i=0; i < data.length; i++) {
      this.dataBuffer.writeUInt8(data[i], i+1);
    }

    rpio.i2cSetSlaveAddress(deviceAddr);
    rpio.i2cWrite(this.dataBuffer, data.length+1);
    return I2C_SUCCESS;
  }

  /** Write 2 byte number to a 16-bit device register.
   * @param deviceAddr I2C slave device address
   * @param regAddr First register address to write to
   * @param length Number of words to write
   * @param data Data to copy 
   * @returns Status of operation (0 = success)
   */
  writeWord(deviceAddr: number, regAddr: Register, data: number): number {
    // identify register and data to write
    this.dataBuffer.writeUInt8(regAddr, 0);
    this.dataBuffer.writeUInt16BE(data, 1);

    // write to register
    rpio.i2cSetSlaveAddress(deviceAddr);
    rpio.i2cWrite(this.dataBuffer, 3);
    return I2C_SUCCESS;
  }
}
