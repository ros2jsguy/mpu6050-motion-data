# Changelog

# 0.2.1 - 20210824
This is a quick fix that now exports the InterruptMonitor class introduced in 0.2.0. 

# 0.2.0 - 20210820

## Added

### MPU6050
* getInterruptLatchEnabled()
* setInterruptLatchEnabled()
* getInterruptClearMode()
* setInterruptClearMode()
* shutdown()
* getDLPF()
* setDLPF()
* getTemperature()
* Use three.math Vector, Quaternion, Euler types 
### MISC
* calibrate utility
* Change log
* interrupt-monitor.ts - an event emitter for 'data' and 'error' events

## Changed
### MPU6050
* Renamed getIntEnabled() to getInterruptEnabled()
* Renamed setIntEnabled() to getInterruptEnabled()
* Renamed getIntFIFOBufferOverflowEnabled() to getInterruptFIFOBufferOverflowEnabled()
* Renamed setIntFIFOBufferOverflowEnabled() to getInterruptFIFOBufferOverflowEnabled()
* Renamed getIntDataReadyEnabled() to getInterruptDataReadyEnabled()
* Renamed setIntDataReadyEnabled() to setInterruptDataReadyEnabled()
* Renamed getIntStatus() to getInterruptStatus()
* Renamed getIntDataReadyStatus() to getInterruptDataReadyStatus()
* Reimplemented dmpInitialize() to use MPU6050 api
### MISC
* Moved i2c-bus-i2c-helper.ts and rpio-i2c-helper.ts to new src/i2c folder
* Updated README doc
* Upate fizz diagram


## Fixed
* dmp-example.ts - now uses latched interrupt mode as non-latched interrupts are signaled with a 50us pulse that was not consistently detected

