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


void setup() {
  
  long t0 = System.nanoTime();
  calibrate_06( "data_20190530163305.dat" );
  long t1 = System.nanoTime();
  
  println( "Done." );
  println( "Total time: " + (t1-t0)*1.0e-9/60.0 + " min" );
  
  exit();
}

//void draw() {
//}


// METHODS TO CALIBRATE EACH COMBINATION OF SENSORS

private void calibrate_06( String fileName ) {
  calibrate_accelerometer10( fileName );
  calibrate_gyroscope10( fileName );
  calibrate_accelerometer16( fileName );
  calibrate_gyroscope16( fileName );
//  calibrate_magnetometer16( fileName );  magnetometer data is faulty in data sets taken with the prototype
  calibrate_accelerometer17( fileName );
  calibrate_gyroscope17( fileName );
//  calibrate_magnetometer17( fileName );  magnetometer data is faulty in data sets taken with the prototype
}


// METHODS TO CALIBRATE EACH SINGLE SENSOR

void calibrate_accelerometer10( String fileName ) {
  String path2file = sketchPath()+"/storedData/";
  TriaxialCalibrator AC = new AccelerometerCalibrator();
  AC.set_sensorID( 11 );
  AC.set_indexID( 1 );
  AC.set_indexTemperature( 8 );
  AC.set_indexAngularVelocity( 12 );
  AC.set_indexMeasurements( 2 , 3 , 4 );
  AC.calibrate_withFile( path2file , fileName );
}

void calibrate_gyroscope10( String fileName ) {
  String path2file = sketchPath()+"/storedData/";
  TriaxialCalibrator GC = new GyroscopeCalibrator();
  GC.set_sensorID( 11 );
  GC.set_indexID( 1 );
  GC.set_indexTemperature( 8 );
  GC.set_indexAngularVelocity( 12 );
  GC.set_indexMeasurements( 5 , 6 , 7 );
  GC.calibrate_withFile( path2file , fileName );
}

void calibrate_accelerometer16( String fileName ) {
  String path2file = sketchPath()+"/storedData/";
  TriaxialCalibrator AC = new AccelerometerCalibrator();
  AC.set_sensorID( 16 );
  AC.set_indexID( 1 );
  AC.set_indexTemperature( 8 );
  AC.set_indexAngularVelocity( 13 );
  AC.set_indexMeasurements( 2 , 3 , 4 );
  AC.calibrate_withFile( path2file , fileName );
}

void calibrate_gyroscope16( String fileName ) {
  String path2file = sketchPath()+"/storedData/";
  TriaxialCalibrator GC = new GyroscopeCalibrator();
  GC.set_sensorID( 16 );
  GC.set_indexID( 1 );
  GC.set_indexTemperature( 12 );
  GC.set_indexAngularVelocity( 13 );
  GC.set_indexMeasurements( 9 , 10 , 11 );
  GC.calibrate_withFile( path2file , fileName );
}

void calibrate_magnetometer16( String fileName ) {
  String path2file = sketchPath()+"/storedData/";
  TriaxialCalibrator MC = new MagnetometerCalibrator();
  MC.set_sensorID( 16 );
  MC.set_indexID( 1 );
  MC.set_indexTemperature( 8 );
  MC.set_indexAngularVelocity( 13 );
  MC.set_indexMeasurements( 5 , 6 , 7 );
  MC.calibrate_withFile( path2file , fileName );
}

void calibrate_accelerometer17( String fileName ) {
  String path2file = sketchPath()+"/storedData/";
  TriaxialCalibrator AC = new AccelerometerCalibrator();
  AC.set_sensorID( 17 );
  AC.set_indexID( 1 );
  AC.set_indexTemperature( 11 );
  AC.set_indexAngularVelocity( 12 );
  AC.set_indexMeasurements( 2 , 3 , 4 );
  AC.calibrate_withFile( path2file , fileName );
}

void calibrate_gyroscope17( String fileName ) {
  String path2file = sketchPath()+"/storedData/";
  TriaxialCalibrator GC = new GyroscopeCalibrator();
  GC.set_sensorID( 17 );
  GC.set_indexID( 1 );
  GC.set_indexTemperature( 11 );
  GC.set_indexAngularVelocity( 12 );
  GC.set_indexMeasurements( 5 , 6 , 7 );
  GC.calibrate_withFile( path2file , fileName );
}

void calibrate_magnetometer17( String fileName ) {
  String path2file = sketchPath()+"/storedData/";
  TriaxialCalibrator MC = new MagnetometerCalibrator();
  MC.set_sensorID( 17 );
  MC.set_indexID( 1 );
  MC.set_indexTemperature( 11 );
  MC.set_indexMeasurements( 8 , 9 , 10 );
  MC.calibrate_withFile( path2file , fileName );
}