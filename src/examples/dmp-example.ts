import { performance } from "perf_hooks";
import { MPU6050} from "../mpu6050";
import { Utils } from "../utils";
import { InterruptMonitor } from "../interrupt-monitor";

const GPIO_MPU6050_DATA_PIN = 18;

function main() {
  let start = 0;
  let interrupts = 0;

  const imu = new MPU6050();
  imu.initialize();
  
  console.log('MPU6050 Device')
  console.log('       connected: ', imu.testConnection());
  console.log('              id: ', imu.getDeviceID());
  console.log('      clock rate: ', imu.getRate());
  console.log('  temperature(F): ', Utils.celciusToF(imu.getTemperature()).toFixed(2));
  Utils.msleep(500);

  console.log('\nDMP initialize and calibrate...');
  imu.dmpInitialize();
  imu.calibrateAccel();
  imu.calibrateGyro();
  imu.printActiveOffsets();
  imu.setInterruptLatchEnabled(true);
  imu.setInterruptDMPEnabled(true);

  const interruptMonitor = new InterruptMonitor(GPIO_MPU6050_DATA_PIN);
  interruptMonitor.on('data', () => {
      if (start === 0) start = performance.now();
      const buf = imu.dmpGetCurrentFIFOPacket();
      if (buf) {
        const data = imu.dmpGetMotionData(buf);
        // console.log(data);
      }
      interrupts++;
      if (interrupts === 1) console.log('  Receiving interrupt(s)');
  });
  interruptMonitor.on('error', (error: Error) => {
    console.log('Data error:', error.message);
  });
  interruptMonitor.start();
  
  setTimeout(()=>{
    imu.shutdown();
    interruptMonitor.shutdown();

    const stop = performance.now();
    const avg = (interrupts / (stop - start) * 1000).toFixed(2); 
    console.log('\nPerformance');
    console.log('  interrupts: ', interrupts);
    console.log('      millis: ', (stop - start).toFixed(2));
    console.log('    avg (hz): ', avg);

    process.exit(0);
  }, 10000);

  console.log('\nSampling data for 10 seconds');
  imu.setDMPEnabled(true);
  console.log('  Waiting for interrupts');
}

main();
