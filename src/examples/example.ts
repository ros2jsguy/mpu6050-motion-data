import { MPU6050, AccelFsRange, GyroFsRange, ClockDiv, ClockSource, Data3D, Quaternion, Vector3 } from "../mpu6050";
import {Utils} from '../utils';

function main() {

  const imu = new MPU6050();
  imu.initialize();
  
  console.log('Device connected:', imu.testConnection());
  console.log('DeviceId:', imu.getDeviceID());

  // imu.setRate(4); // 1khz / (1 + 4) = 200 Hz
  console.log('Rate: ', imu.getRate());

  console.log('Uncalibrated Device Offsets')
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

  console.log('Setup calibration offsets and initialize DMP...');
  imu.dmpInitialize();
  imu.calibrateAccel();
  imu.calibrateGyro();
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
    console.log('  dmpEuler:', euler.x, euler.y, euler.z);
    console.log('  dmpGravity:', gravity.x, gravity.y, gravity.z);
    console.log('  dmpLinearAccel:', linearAccel);
    console.log('  dmpRPY ', 
      ' roll:', rpy.roll * 180 / Math.PI, 
      ' pitch:', rpy.pitch * 180 / Math.PI, 
      ' yaw:', rpy.yaw * 180 / Math.PI);
   
    break;
  }

  console.log();
  console.log('DMP FIFO Access Test');
  let start = Date.now();
  let cnt = 1;
  let fails = 0;
  const maxLoops = 1000;
  while (cnt < maxLoops) {
    let buf: Buffer | undefined;
    try {
      buf = imu.dmpGetCurrentFIFOPacket();
    } catch (e) {
      fails++;
    }
    if (!buf) continue;
   
    try {
      imu.dmpGetMotionData(buf);
    } catch(e) {
      fails++;
    }
    cnt++;
  }
  console.log('cnt', cnt, 'fails', fails);
  let stop = Date.now();
  console.log('Avg package access time: ', (stop - start) / cnt, 'ms', 'packets read:', cnt);
  
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
    let buf: Buffer | undefined 
    try {
      buf = imu.dmpGetCurrentFIFOPacket();
    } catch(e) {
      // do nothing
    }
    if (!buf) continue;
    const quaternion = imu.dmpGetQuaternion(buf);
    const gravity = imu.dmpGetGravity(buf);
    console.log('q:', quaternion, 'G:', gravity);
    const rpy = imu.dmpGetYawPitchRoll(quaternion, gravity);
    console.log(
      'roll:', Utils.radToDeg(rpy.roll).toFixed(5), 
      ' pitch:', Utils.radToDeg(rpy.pitch).toFixed(5), 
      ' yaw:', Utils.radToDeg(rpy.yaw).toFixed(5));
    Utils.usleep(1);
  }

  console.log('Complete');
}

main();