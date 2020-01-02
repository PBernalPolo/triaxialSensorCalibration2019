/*
 * Copyright (C) 2019 Pablo Bernal-Polo
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#define PACKAGE_SIZE 16
#define INFORMATION_SOURCE_CLASS_ID 30  // 10: GPS,  20: Velocimeter,  30: IMU
#define WHO_AM_I 13

#include "I2Cdev.h"
#include "MPU6050.h"
#include "MessageManager.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

MessageManager MM( 2 );

// we will save measurements here
int16_t amx, amy, amz;
int16_t wmx, wmy, wmz;
int16_t temp;


void setup() {
  // we start the serial connection
  Serial.begin(115200);

  // we start the connection with the sensor
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  TWBR = 12;
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  
  // set range
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}


void loop() {
  // we take measurements
  accelgyro.getMotion6( &amx , &amy , &amz , &wmx , &wmy , &wmz );
  temp = accelgyro.getTemperature();
  
  // we send the package ( each packet is a 8-bit signed two's complement integer )
  //  first, we prepare the package who_am_i + (7 data(acceleration + angular velocity + temperature))*(2 byte/data) + 2 checksum
  const int8_t packet[PACKAGE_SIZE] = { INFORMATION_SOURCE_CLASS_ID , WHO_AM_I ,
                                        (int8_t)amx , (int8_t)(amx >> 8) ,
                                        (int8_t)amy , (int8_t)(amy >> 8) ,
                                        (int8_t)amz , (int8_t)(amz >> 8) ,
                                        (int8_t)wmx , (int8_t)(wmx >> 8) ,
                                        (int8_t)wmy , (int8_t)(wmy >> 8) ,
                                        (int8_t)wmz , (int8_t)(wmz >> 8) ,
                                        (int8_t)temp , (int8_t)(temp >> 8) };
  
  // we prepare the message
  int8_t* toWrite = MM.prepare_message( PACKAGE_SIZE , packet );
  
  // we send the communication
  for(int i=0; i<PACKAGE_SIZE+4; i++){
    Serial.write( toWrite[i] );
  }
  
}


