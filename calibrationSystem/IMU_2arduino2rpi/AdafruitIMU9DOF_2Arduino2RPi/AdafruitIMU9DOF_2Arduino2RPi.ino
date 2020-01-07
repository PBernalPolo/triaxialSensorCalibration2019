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


//#define DEBUG1
#define PACKAGE_SIZE 24
#define INFORMATION_SOURCE_CLASS_ID 30  // 10: GPS,  20: Velocimeter,  30: IMU
#define WHO_AM_I 16

//#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>  // the method Adafruit_L3GD20_Unified::get_temperature() have been introduced
//#include <Adafruit_9DOF.h>
#include "MessageManager.h"


// we create the sensor objects
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

MessageManager MM( 2 );


void setup(){

  // we start the serial communication
  Serial.begin(115200);
  
  // we test the connections to the devices
  if( !accel.begin() ){
    // there was a problem detecting the LSM303 ... check your connections
    Serial.println( "Ooops, no LSM303 detected ... Check your wiring!" );
    while( true );
  }else{
    Serial.println( "Accelerometer: OK" );
  }
  if( !mag.begin() ){
    // there was a problem detecting the LSM303 ... check your connections
    Serial.println( "Ooops, no LSM303 detected ... Check your wiring!" );
    while( true );
  }else{
    Serial.println( "Magnetometer: OK" );
  }
  if( !gyro.begin( GYRO_RANGE_2000DPS ) ){
    // there was a problem detecting the L3GD20 ... check your connections
    Serial.print( "Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!" );
    while(1);
  }else{
    Serial.println( "Gyroscope: OK" );
  }
  
  mag.setMagGain( LSM303_MAGGAIN_8_1 );
  mag.setMagRate( LSM303_MAGRATE_220 );
  mag.enable_temperatureSensor();
  
}


void loop(){
  
  // we create the sensor events
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t gyro_event;
  
  // we read the accelerometer and magnetometer
  accel.getEvent( &accel_event );
  mag.getEvent( &mag_event );
  int tempAM = mag.get_temperatureData();
  gyro.getEvent( &gyro_event );
  int tempG = gyro.get_temperatureData();
  
#if defined DEBUG1
  Serial.print( accel.raw.x );   Serial.print( " " );
  Serial.print( accel.raw.y );   Serial.print( " " );
  Serial.print( accel.raw.z );   Serial.print( " " );
  Serial.print( mag.raw.x );   Serial.print( " " );
  Serial.print( mag.raw.y );   Serial.print( " " );
  Serial.print( mag.raw.z );   Serial.print( " " );
  Serial.print( tempAM );   Serial.print( " " );
  Serial.print( gyro.raw.x );   Serial.print( " " );
  Serial.print( gyro.raw.y );   Serial.print( " " );
  Serial.print( gyro.raw.z );   Serial.print( " " );
  Serial.println( tempG );
  delay(500);
#else
  // we send the package ( each packet is a 8-bit signed two's complement integer )
  //  first, we prepare the package who_am_i + (7 data(acceleration + angular velocity + temperature))*(2 byte/data) + 2 checksum
  const int8_t packet[PACKAGE_SIZE] = { INFORMATION_SOURCE_CLASS_ID , WHO_AM_I ,
                                        (int8_t)accel.raw.x , (int8_t)(accel.raw.x >> 8) ,
                                        (int8_t)accel.raw.y , (int8_t)(accel.raw.y >> 8) ,
                                        (int8_t)accel.raw.z , (int8_t)(accel.raw.z >> 8) ,
                                        (int8_t)mag.raw.x , (int8_t)(mag.raw.x >> 8) ,
                                        (int8_t)mag.raw.y , (int8_t)(mag.raw.y >> 8) ,
                                        (int8_t)mag.raw.z , (int8_t)(mag.raw.z >> 8) ,
                                        (int8_t)tempAM , (int8_t)(tempAM >> 8) ,
                                        (int8_t)gyro.raw.x , (int8_t)(gyro.raw.x >> 8) ,
                                        (int8_t)gyro.raw.y , (int8_t)(gyro.raw.y >> 8) ,
                                        (int8_t)gyro.raw.z , (int8_t)(gyro.raw.z >> 8) ,
                                        (int8_t)tempG , (int8_t)(tempG >> 8) };
  
  // we prepare the message
  int8_t* toWrite = MM.prepare_message( PACKAGE_SIZE , packet );
  
  // we send the communication
  for(int i=0; i<PACKAGE_SIZE+4; i++){
    Serial.write( toWrite[i] );
  }
#endif
  
}




