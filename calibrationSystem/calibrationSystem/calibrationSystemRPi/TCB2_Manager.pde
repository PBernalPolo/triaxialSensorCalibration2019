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


import processing.serial.*;


public class TCB2_Manager {
  
  // CONSTANTS
  private static final int waitTime = (int)(0.5*1.0e3);  // time to wait for a response (milliseconds)
  private static final int maxAttempts = 50;  // maximum number of attempts to send an order and receive a response
  
  // PRIVATE VARIABLES
  private boolean calibrate;
  private float angularVelocity;
  private Serial bluetoothPort;
  private MessageManager MM;
  
  
  // CONSTRUCTORS
  
  public TCB2_Manager( PApplet thePApplet ) {
    this.calibrate = false;
    this.angularVelocity = 0.0;
    this.bluetoothPort = new Serial( thePApplet , "/dev/rfcomm0" , 9600 );
    this.MM = new MessageManager( 2 );
  }
  
  
  // PUBLIC METHODS
  
  public void test_MM() {
    float i = 0.0;
    while( true ){
      // we receive a float
      while( this.bluetoothPort.available() > 0 ){
        byte[] theBytes = this.MM.manage_byteIn( (byte)this.bluetoothPort.read() );
        if( theBytes != null ){
          System.out.println( this.MM.get_float( 0 , theBytes ) );
        }
      }
      // we send a float
      byte[] theBytes = this.MM.get_bytes( i );
      println( theBytes );
      this.bluetoothPort.write( this.MM.prepare_message( theBytes ) );
      i += 1.0;
      try{
        Thread.sleep( 1000 );
      }catch( Exception exc ){
        System.out.println( "TCB2_Manager: test(): Not able to sleep." );
      }
    }
  }
  
  public int update() {
    while( this.bluetoothPort.available() > 0 ){
      byte newByte = (byte)this.bluetoothPort.read();
      byte[] theBytes = this.MM.manage_byteIn( newByte );
      if( theBytes != null ){
        switch( theBytes[0] ){
          case 0:  // start/stop calibration
            if( theBytes[1] > 0 ){
              this.calibrate = true;
            }else{
              this.calibrate = false;
            }
            return 0;
          case 1:  // angular velocity change
            this.angularVelocity = this.MM.get_float( 1 , theBytes );
            return 1;
          case 2:  // temperature controller change
            return theBytes[1]+3;
          default:
            return -1;
        }
      }
    }
    return -1;
  }
  
  public void reset_calibrate() {
    this.calibrate = false;
  }
  
  public boolean want_toCalibrate() {
    return this.calibrate;
  }
  
  public boolean set_angularVelocity( float av ) {
    byte[] theBytes = this.MM.get_bytes( av );
    byte[] theMessage = this.MM.prepare_message( new byte[]{ 1 , theBytes[0] , theBytes[1] , theBytes[2] , theBytes[3] } );
    for(int a=0; a<TCB2_Manager.maxAttempts; a++){
      this.bluetoothPort.write( theMessage );
      delay( TCB2_Manager.waitTime );
      while( this.bluetoothPort.available() > 0 ){
        if( this.update() == 1 ){
          return true;
        }
      }
    }
    return false;
  }
  
  public boolean warmUp() {
    byte[] theMessage = this.MM.prepare_message( new byte[]{ 2 , 1 } );
    for(int a=0; a<TCB2_Manager.maxAttempts; a++){
      this.bluetoothPort.write( theMessage );
      delay( TCB2_Manager.waitTime );
      while( this.bluetoothPort.available() > 0 ){
        if( this.update() == 4 ){
          return true;
        }
      }
    }
    return false;
  }
  
  public boolean coolDown() {
    byte[] theMessage = this.MM.prepare_message( new byte[]{ 2 , -1 } );
    for(int a=0; a<TCB2_Manager.maxAttempts; a++){
      this.bluetoothPort.write( theMessage );
      delay( TCB2_Manager.waitTime );
      while( this.bluetoothPort.available() > 0 ){
        if( this.update() == 2 ){
          return true;
        }
      }
    }
    return false;
  }
  
  public boolean disconnect_Peltier() {
    byte[] theMessage = this.MM.prepare_message( new byte[]{ 2 , 0 } );
    for(int a=0; a<TCB2_Manager.maxAttempts; a++){
      this.bluetoothPort.write( theMessage );
      delay( TCB2_Manager.waitTime );
      while( this.bluetoothPort.available() > 0 ){
        if( this.update() == 3 ){
          return true;
        }
      }
    }
    return false;
  }
  
  public double get_angularVelocity() {
    return this.angularVelocity;
  }
  
}
