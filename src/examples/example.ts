import { MPU6050, AccelFsRange, GyroFsRange, ClockDiv, ClockSource, Gyro, Accel, Quaternion, Vector3 } from "../mpu6050";
import {Utils} from '../utils';

function main() {

  const imu = new MPU6050();
  imu.initialize();
  
  console.log('Device connected:', imu.testConnection());
  console.log('DeviceId:', imu.getDeviceID());

  imu.setRate(4); // 1khz / (1 + 4) = 200 Hz
  console.log('Rate: ', imu.getRate());

  console.log('Device Offsets')
  console.log('XAOffset:', imu.getXAccelOffset());
  console.log('YAOffset:', imu.getYAccelOffset());
  console.log('ZAOffset:', imu.getZAccelOffset());
  console.log('XGOffset:', imu.getXGyroOffset());
  console.log('YGOffset:', imu.getYGyroOffset());
  console.log('ZGOffset:', imu.getZGyroOffset());
  
  console.log();
  console.log('Sample data without calibration');
  for (let i=0; i < 3; i++) {
    console.log('motion data:', imu.getMotionData());
    Utils.msleep(500);
  }

  console.log('Initialize DMP...');
  imu.dmpInitialize();
  imu.calibrateAccel(6);
  imu.calibrateGyro(6);
  imu.printActiveOffsets();
  imu.setDMPEnabled(true);

  console.log();
  console.log('Calibrated Offsets');
  console.log('XAOffset:', imu.getXAccelOffset());
  console.log('YAOffset:', imu.getYAccelOffset());
  console.log('ZAOffset:', imu.getZAccelOffset());
  console.log('XGOffset:', imu.getXGyroOffset());
  console.log('YGOffset:', imu.getYGyroOffset());
  console.log('ZGOffset:', imu.getZGyroOffset());

  console.log();
  console.log('Sample data with dmp enabled');
  for (let i=0; i < 3; i++) {
    console.log('data:', imu.getMotionData());
    Utils.msleep(500);
  }

  console.log();
  console.log('DMP packet data');
  for (let i=0; i < 100; i++) {
    const buf = imu.dmpGetCurrentFIFOPacket();
    if (!buf) continue;

    const gyro = imu.dmpGetGyro(buf);
    const accel = imu.dmpGetAccel(buf);
    const quaternion = imu.dmpGetQuaternion(buf);
    quaternion.w /= 16384;
    const euler = imu.dmpGetEuler(quaternion);
    const gravity = imu.dmpGetGravity(buf);
    const linearAccel = imu.dmpGetLinearAccel(accel, gravity);
    const rpy = imu.dmpGetYawPitchRoll(quaternion, gravity);

    console.log('  dmpGyro:', gyro);
    console.log('  dmpAccel:', accel);
    console.log('  dmpQuaternion:', quaternion);
    console.log('  dmpEuler:', euler.phi, euler.theta, euler.psi);
    console.log('  dmpGravity:', gravity.x, gravity.y, gravity.z);
    console.log('  dmpLinearAccel:', linearAccel);
    console.log('  dmpRPY ', 
      ' roll:', rpy.roll * 180 / Math.PI, 
      ' pitch:', rpy.pitch * 180 / Math.PI, 
      ' yaw:', rpy.yaw * 180 / Math.PI);
    
    // console.log('linearAccel', accel.acc_x * 8.0 / 65536 * 9.81, accel.acc_y * 8.0 / 65536 * 9.81, accel.acc_z * 8.0 / 65536 * 9.81,)
   
    break;
  }

  console.log();
  console.log('DMP FIFO Access Test');
  let start = Date.now();
  let cnt = 1;
  const maxLoops = 1000;
  while (cnt < maxLoops) {
    const buf = imu.dmpGetCurrentFIFOPacket();
    if (!buf) continue;
    imu.dmpGetMotionData(buf);
    cnt++;
  }
  let stop = Date.now();
  console.log('Avg package access time: ', (stop - start) / cnt, 'ms');
  
  console.log();
  console.log('Sensor Access Test - no DMP');
  start = Date.now();
  cnt = 1;
  imu.setDMPEnabled(false);
  while (cnt < maxLoops) {
    imu.getMotionData();
    cnt++;
  }
  stop = Date.now();
  console.log('Avg package access time: ', (stop - start) / cnt, 'ms');
  console.log();

  imu.setDMPEnabled(true);
  while(true) { // eslint-disable-line no-constant-condition
    const buf = imu.dmpGetCurrentFIFOPacket();
    if (!buf) continue;
    const rpy = imu.dmpGetYawPitchRoll(imu.dmpGetQuaternion(buf), imu.dmpGetGravity(buf));
    console.log(
      'roll:', rad2degree(rpy.roll, 5), 
      ' pitch:', rad2degree(rpy.pitch, 5), 
      ' yaw:', rad2degree(rpy.yaw, 5) );
  }

  console.log('Complete');

  // let motionData: MotionData = {acc_x: 0, acc_y: 0, acc_z: 0, gyro_x: 0, gyro_y: 0, gyro_z: 0, pitch: 0, roll: 0};
  // for (let i=0; i < 100; i++) {
  //   imu.readMotionData(motionData);
  //   console.log('data:', motionData);

    // Full scale range +/- 250 degree/C as per sensitivity scale factor
    // let Ax = acc_x/16384.0;
    // let Ay = acc_y/16384.0;
    // let Az = acc_z/16384.0;
    
    // let Gx = gyro_x/131.0;
    // let Gy = gyro_y/131.0;
    // let Gz = gyro_z/131.0;

    // console.log('Ax:', acc_x, ' Ay:', acc_y, ' Az:', acc_z);
    // console.log('Gx:', gyro_x, ' Gy:', gyro_y, ' Gz:', gyro_z);
    // console.log();

  //   Utls.msleep(1000);
  // }

  imu.shutdown();
}

function rad2degree(rads: number, fixed = 0) {
  const deg = rads * 180 / Math.PI;
  return fixed > 0 ? deg.toFixed(fixed) : deg;
}

main();