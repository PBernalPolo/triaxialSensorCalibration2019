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


import processing.io.*;  // to use the SoftwareServo library


StorageManagerM SM;


void setup(){
  SM = new StorageManagerM( this );
  SM.start_storing();
  SM.set_storeMeasurements( true );
}


void draw(){
  SM.notify_activity();
  delay( 100 );
}


void serialEvent( Serial p ) {
  try{
    SM.notify_activity();
  }catch( Exception e ){
    //e.printStackTrace();
    //println( "Not able to notify the activity." );
  }
} 


void exit() {
  SM.stop();
  super.exit();
}
