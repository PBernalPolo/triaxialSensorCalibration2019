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


// # TO RUN FROM THE RASPBERRY PI:
//   sudo rfcomm bind rfcomm0 98:D3:31:FD:64:09
// #   WITH DESKTOP (https://learn.adafruit.com/processing-on-the-raspberry-pi-and-pitft/processing):
//   DISPLAY=:0 processing-java --sketch=/home/pi/calibrationSystemV2/calibrationSystemRPi --present
// #   WITHOUT DESKTOP (https://github.com/processing/processing/wiki/Running-without-a-Display):
// #   sudo apt-get install xvfb libxrender1 libxtst6 libxi6
//   sudo xvfb-run processing-java --sketch=/home/pi/calibrationSystemV2/calibrationSystemRPi --run
// #     OR
//   sudo Xvfb :1 -screen 0 1024x768x24 </dev/null &
//   export DISPLAY=":1"
//   processing-java --sketch=/home/pi/calibrationSystemV2/calibrationSystemRPi --run


TCB2_Manager theTCBM;

CalibrationSystemManager theCSM;

void setup(){
  
  theCSM = new CalibrationSystemManager( this , new float[]{ 0.0 , 300 , 0.0 , -300 ,
                                                             0.0 , 400 , 0.0 , -400 ,
                                                             0.0 , 500 , 0.0 , -500 ,
                                                             0.0 , 600 , 0.0 , -600 ,
                                                             0.0 , 700 , 0.0 , -700 } , 15.0 , 45.0 , 1 );
  
}


void draw(){
  
  theCSM.notify_activity();
  theCSM.update();
  delay( 100 );
  
}


void serialEvent( Serial p ) {
  try{
    theCSM.notify_activity();
  }catch( Exception e ){
    //e.printStackTrace();
    //println( "Not able to notify the activity." );
  }
} 


void exit() {
  theCSM.stop();
  System.out.println( "exit in 5 seconds..." );
  delay( 5000 );
  super.exit();
}
