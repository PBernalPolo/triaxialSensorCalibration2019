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


// VARIABLES
IM_Drawer imd;
FileDataManager theFDM;
aGUI theGUI;

IM_Sensor[] s = new IM_Sensor[0];


void setup() {
  //size(1024, 768, P3D);
  size(1152, 864, P3D);
  //fullScreen( P3D );
  //textMode( MODEL );
  textMode( SHAPE );
  textAlign( LEFT );
  
  // we set the color for each sensor
  //  calibrated sensors
  imd = new IM_Drawer();
  imd.set_colorForID( 11 , color(255,0,0) );
  imd.set_colorForID( 12 , color(0,255,0) );
  imd.set_colorForID( 13 , color(0,0,255) );
  imd.set_colorForID( 14 , color(0,255,255) );
  imd.set_colorForID( 15 , color(255,0,255) );
  imd.set_colorForID( 16 , color(180,180,180) );  // Adafruit
  imd.set_colorForID( 17 , color(255,255,0) );  // SenseHAT
  //  non-calibrated sensors
  imd.set_colorForID( 21 , color(255,0,0,100) );
  imd.set_colorForID( 22 , color(0,255,0,100) );
  imd.set_colorForID( 23 , color(0,0,255,100) );
  imd.set_colorForID( 24 , color(0,255,255,100) );
  imd.set_colorForID( 25 , color(255,0,255,100) );
  imd.set_colorForID( 26 , color(180,180,180,100) );  // Adafruit
  imd.set_colorForID( 27 , color(255,255,0,100) );  // SenseHAT
  
  // we create the FileDataManager
  theFDM = new FileDataManager( "data_20190621183329.dat" , true );  // RPi (new)
  long time = theFDM.get_time();
  
  // we add the sensors
  //  calibrated sensors
  add_sensor( new IM_IMU( 11 , time ) );
  add_sensor( new IM_IMU( 12 , time ) );
  add_sensor( new IM_IMU( 13 , time ) );
  add_sensor( new IM_IMU( 14 , time ) );
  add_sensor( new IM_IMU( 15 , time ) );
  add_sensor( new IM_IMU_Adafruit( 16 , time ) );
  add_sensor( new IM_IMU_SenseHAT( 17 , time ) );
  //  non-calibrated sensors
  add_sensor( new IM_IMU( 21 , time ) );
  add_sensor( new IM_IMU( 22 , time ) );
  add_sensor( new IM_IMU( 23 , time ) );
  add_sensor( new IM_IMU( 24 , time ) );
  add_sensor( new IM_IMU( 25 , time ) );
  add_sensor( new IM_IMU_Adafruit( 26 , time ) );
  add_sensor( new IM_IMU_SenseHAT( 27 , time ) );
  
  // we set the calibrations
  s[11].set_calibration( new String[]{ sketchPath() + "/calibrations/11a.cal" , sketchPath() + "/calibrations/11w.cal" , sketchPath() + "/calibrations/11m.cal" } );
  s[12].set_calibration( new String[]{ sketchPath() + "/calibrations/12a.cal" , sketchPath() + "/calibrations/12w.cal" } );
  s[13].set_calibration( new String[]{ sketchPath() + "/calibrations/13a.cal" , sketchPath() + "/calibrations/13w.cal" } );
  s[14].set_calibration( new String[]{ sketchPath() + "/calibrations/14a.cal" , sketchPath() + "/calibrations/14w.cal" } );
  s[15].set_calibration( new String[]{ sketchPath() + "/calibrations/15a.cal" , sketchPath() + "/calibrations/15w.cal" } );
  s[16].set_calibration( new String[]{ sketchPath() + "/calibrations/16a.cal" , sketchPath() + "/calibrations/16w.cal" , sketchPath() + "/calibrations/16m.cal" } );
  s[17].set_calibration( new String[]{ sketchPath() + "/calibrations/17a.cal" , sketchPath() + "/calibrations/17w.cal" , sketchPath() + "/calibrations/17m.cal" } );
  
  theGUI = new aGUI( this );
  
  theFDM.set_sensors( s );
  
}


void draw() {
  background( 0 );
  
  float tStroke = 0.75/theGUI.get_zoom();
  
  theGUI.begin_view();
  
  for(int i=0; i<s.length; i++){
    if( s[i] != null ){
      imd.draw( s[i] , tStroke );
    }
  }
  
  theGUI.end_view();
  
}


void mouseReleased() {
  if( theGUI.is_mouseOnScreen() ){
    println( theFDM.get_measurementsPerSecond() );
  }
}


// adds a sensor to the array
void add_sensor( IM_Sensor sensor ) {
  if( sensor.ID >= s.length ){
    IM_Sensor[] sNew = new IM_Sensor[ sensor.ID + 1 ];
    for(int i=0; i<s.length; i++){
      sNew[i] = s[i];
    }
    s = sNew;
  }
  s[ sensor.ID ] = sensor;
}


void keyPressed() {
  switch( key ){
    case 'c':
      theFDM.play_fast();
      break;
    case 'x':
      theFDM.play_normal();
      break;
    case 'z':
      theFDM.pause();
      break;
    default:
      break;
  }
}


void exit(){
  theFDM.stop();
  super.exit();
}


// SOME AUXILIARY METHODS

void line( double x1 , double y1 , double x2 , double y2 ) {
  line( (float)x1 , (float)y1 , (float)x2 , (float)y2 );
}

void line( double x1 , double y1 , double z1 , double x2 , double y2 , double z2 ) {
  line( (float)x1 , (float)y1 , (float)z1 , (float)x2 , (float)y2 , (float)z2 );
}

void vertex( double x , double y , double z ) {
  vertex( (float)x , (float)y , (float)z );
}

void vertex( double x , double y , double z , double u , double v ) {
  vertex( (float)x , (float)y , (float)z , (float)u , (float)v );
}

void text( int n , double x , double y , double z ) {
  text( n , (float)x , (float)y , (float)z );
}

void point( double x , double y , double z ) {
  point( (float)x , (float)y , (float)z );
}

void translate( double x , double y , double z ) {
  translate( (float)x , (float)y , (float)z );
}

void rotate( double a , double x , double y , double z ) {
  rotate( (float)a , (float)x , (float)y , (float)z );
}
