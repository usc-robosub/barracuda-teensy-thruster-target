# barracuda-teensy-thruster-target
I2C target (on Teensy) that interfaces with the barracuda-thruster-output-server controller. Utilized by each of the two Teensys (each is responsible for generating PWM signals for 4 of the 8 ESCs). If pin 2 is connected to the ground pin, the I2C address 0x2e will be assigned to the Teensy; otherwise, 0x2d will be assigned. 

### Flashing the Teensy (With Arduino IDE)
1. Install the [Arduino IDE](https://www.arduino.cc/en/software/).
2. [Install the teensy4_i2c library](https://github.com/Richard-Gemmell/teensy4_i2c/blob/master/documentation/installation/arduino_installation.md).
3. Clone this repo onto your development machine and open the .ino file with the Arduino IDE.
4. In the "Tools" menu select "Teensy 4.0" as the board and the appropriate USB for the port.
5. Connect the Teensy to your laptop via USB and then click the "upload" button in the IDE to flash.

