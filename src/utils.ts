import * as rpio from 'rpio';
import {MathUtils} from 'three-math-ts'

export namespace Utils {

  export function sleep(secs: number): void {
    rpio.sleep(secs);
  };

  export function msleep(mills: number): void {
    rpio.msleep(mills);
  }

  export function usleep(usecs: number): void {
    rpio.usleep(usecs);
  }

  export function clamp(value: number, min: number, max: number): number {
    return MathUtils.clamp(value, min, max);
  }

  export function toBinaryString(integer: number, withPaddingLength=8): string {
    const str = integer.toString(2);
    return str.padStart(withPaddingLength, "0");
  }

  export function toHexString(integer: number, withPaddingLength=2): string {
    const str = integer.toString(16);
    return str.padStart(withPaddingLength, "0");
  }

  export function degToRad(degrees: number): number {
    return MathUtils.degToRad(degrees);
  }

  export function radToDeg(radians: number): number {
    return MathUtils.radToDeg(radians);
  }

  export function celciusToF(celcius: number): number {
    return celcius * 1.8 + 32;
  }
}
