
import * as rpio from 'rpio';

export const Utils = {

  sleep(secs: number): void {
    rpio.sleep(secs);
  },

  msleep(mills: number): void {
    rpio.msleep(mills);
  },

  usleep(usecs: number): void {
    rpio.usleep(usecs);
  }

}