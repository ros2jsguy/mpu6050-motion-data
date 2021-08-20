/* eslint-disable */

import EventEmitter = require("events");

const { Gpio } = require('onoff').Gpio; // eslint-disable-line

/**
 * Monitor a GPIO pin and emit a 'data' event with the pin
 * transitions to its active state. HIGH is the default 
 * active state on the pin. If na error occurs emit an
 * 'error' event with an Error parameter.
 */
export class InterruptMonitor extends EventEmitter {

  private pin: any;

  /**
   * Create a new instance monitoring a GPIO pin. 
   * @param gpioPin - the pin to monitor
   * @param [activeHigh=true] - true sets the pin active state to HIGH; otherwise the active state is LOW.
   */
  constructor(gpioPin: number, activeHigh=true) {
    super();

    if (gpioPin < 0 || gpioPin > 40) {
      throw new Error(`Invalid gpio ${gpioPin}`);
    }

    if (!Gpio.accessible) {
      throw new Error(`gpio ${gpioPin} is not accessible`);
    }

    const edge = activeHigh ? 'rising' : 'falling';
    this.pin = new Gpio(gpioPin, 'in', edge);
  }

  /**
   * Begin monitoring the pin and emitting 'data' event when the pin is in the active state.
   */
  start(): void {
    this.pin.watch(
      (error: Error, edge: number) => {
        if (error) {
          this.emit('error', error);
        } else {
          this.emit('data');
        }
      });
  }

  /**
   * Stop monitoring the pin and discontinue emitting 'data' events.
   */
  stop(): void {
    this.pin.unwatch();
  }

  /**
   * Stop monitoring the pin and release it. 
   */
  shutdown(): void {
    this.stop();
    this.pin.unexport();
  }
}

/* eslint-enable */