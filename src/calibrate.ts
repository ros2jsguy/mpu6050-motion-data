#!/usr/bin/env node

import {MPU6050} from './mpu6050';
import {Utils} from './utils';

function main() {

  const imu = new MPU6050();
  imu.initialize();
  
  console.log('MPU6050 Device')
  console.log('       connected: ', imu.testConnection());
  console.log('              id: ', imu.getDeviceID());
  console.log('  temperature(F): ', Utils.celciusToF(imu.getTemperature()).toFixed(2));
  Utils.msleep(500);

  console.log('\nComputing accelerometer and gyroscope offsets');
  imu.calibrateAccel();
  imu.calibrateGyro();
  
  console.log();
  imu.printActiveOffsets();
  console.log();

  imu.shutdown();
}

main();
