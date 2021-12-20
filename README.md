# NAU7802

This is a basic implementation of using the NAU7802 over i2c on a Raspberry Pi PICO / RP 2040 processor.

This is my 1st time doing an i2c interface, and to build this I followed along here:
https://www.youtube.com/watch?v=092xFEmAS98

This happens in a couple steps:
1.) Initalize basic ADC settings.
  The information on how to do this comes right from the datasheet for the device.
2.) repeatedly loop and echo value from scale back to STDIO.
  In this example, the RPI2040 is set up to use the default I2C interface 0 pins and 
  the default UART 0 pins.
  
The load cell that I used has 2mV/V sensitivity.  Seems to be accurate to within 1-2 grams with a 5kg load cell.

There are more professional examples of i2c interfacing in the examples folder of the SDK.
