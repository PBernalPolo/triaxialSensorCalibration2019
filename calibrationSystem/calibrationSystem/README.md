# calibrationSystem
These are the sources used to collect calibration data from triaxial sensors.
In particular, the code is used to:
- collect calibration data of accelerometers and gyroscopes using the Triaxial Calibration System:
  - retrieve data from IMUs (IMU_2arduino2rpi)
  - and store it in the RPi (calibrationSystemRPi)
- or collect calibration data of magnetometers by hand:
  - retrieve data (IMU_2arduino2rpi)
  - and store it in the RPi (calibrationMagnetometerRPi)
In addition, the source code is also used to:
- control the temperature of the chamber, and the angular velocity of the stepper motor (velocityAndTemperatureControllerArduino)
- control the servos of the gimbal, and manage the communication with the Arduinos (calibrationSystemRPi)

For more information see:
- Paper: https://ieeexplore.ieee.org/document/8861358
- Video: https://youtu.be/Z5JtIe8469I
- Things: https://www.thingiverse.com/thing:4063556
