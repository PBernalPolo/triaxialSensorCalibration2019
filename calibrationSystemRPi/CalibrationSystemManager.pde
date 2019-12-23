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
import java.io.*;  // to use InputStreamReader


public class CalibrationSystemManager {
  
  // PARAMETERS
  private static final int MAX_CYCLES = 1;  // maximum calibration cycles
  private static final int SERVO1_PIN = 5;  // pin for servo 1
  private static final float SERVO1_MIN_ANGLE = 12.0;  // [5,175] minimum angle for servo 1; true limits are [0,180], but we do not go to the limit to avoid knocks
  private static final float SERVO1_MAX_ANGLE = 170.0;  // [5,175] maximum angle for servo 1; true limits are [0,180], but we do not go to the limit to avoid knocks
  private static final int SERVO2_PIN = 6;  // pin for servo 2
  private static final float SERVO2_MIN_ANGLE = 5.0;  // [5,175] minimum angle for servo 2; true limits are [0,180], but we do not go to the limit to avoid knocks
  private static final float SERVO2_MAX_ANGLE = 175.0;  // [5,175] maximum angle for servo 2; true limits are [0,180], but we do not go to the limit to avoid knocks
  private static final int STEPPER_MOTOR_TIME_CHANGE = 2500;  // (ms) waiting time until the stepper motor has set an angular velocity
  private static final int TIME_TO_WAIT_BEFORE_CHANGE = 20;  // (ms) waiting time for the StorageManager to stop taking measurements
  private static final int RELAYS_TIME_OPERATION = 2000;  // (ms) waiting time until the relays transition dynamic ends
  private static final long TIME_FOR_EACH_MECHANICAL_STATE =  (long)(0.5*1.0e9);  // (ns) time that we will maintain each mechanical state (position of servos and velocity)
  
  // CONSTANTS
  private static final float MIN_RPI_TEMPERATURE = 0.0;  // minimum temperature for the RPi (0ºC)
  private static final float MAX_RPI_TEMPERATURE = 70.0;  // maximum temperature for the RPi (70ºC)
  
  // PRIVATE VARIABLES
  private long tLastChangeStateM;  // last time the stateM changed
  private final float[] targetAngularVelocity;
  private final float Tmin;
  private final float Tmax;
  private final int Ncycles;  // number of cooling/heating cycles to be performed in the calibration
  private boolean calibrating;  // true if we are calibrating; false otherwise
  private int nc;  // number of cycles performed
  private int stateM;  // state of the finite-state machine used for the TCB and servo control
  private int stateSC;  // state of the finite-state machine used for the continuous servo control (we can only use one continuous servo; otherwise we would need a variable per servo)!!!!!!
  private int stateT;  // state of the finite-state machine used for the temperature control
  private int nAV;  // index for the current angular velocity
  private TCB2_Manager TCB;
  private StorageManager SM;
  private SoftwareServoCS servo1;
  private SoftwareServoCS servo2;
  
  
  // CONSTRUCTORS
  
  public CalibrationSystemManager( PApplet thePApplet , float[] angularVelocities , float theTmin , float theTmax , int theNcycles ) {
    this.targetAngularVelocity = angularVelocities;
    this.Tmin = theTmin;
    this.Tmax = theTmax;
    this.Ncycles = theNcycles;
    
    this.servo1 = new SoftwareServoCS( thePApplet , SERVO1_PIN , SERVO1_MIN_ANGLE , SERVO1_MAX_ANGLE );
    this.servo2 = new SoftwareServoCS( thePApplet , SERVO2_PIN , SERVO2_MIN_ANGLE , SERVO2_MAX_ANGLE );
    
    // lastly, we create the TCB2_Manager and the StorageManager, so that serialEvent() does not throw exceptions
    this.TCB = new TCB2_Manager( thePApplet );
    this.SM = new StorageManager( thePApplet , this.TCB );
    this.reset();
  }
  
  
  // PUBLIC METHODS
  
  public void test() {
    int extraTime = 3000;
    this.servo1.move( SERVO1_MIN_ANGLE );
    delay( extraTime );
    this.servo1.move( SERVO1_MAX_ANGLE );
    delay( extraTime );
    this.servo1.move( 90.0 );
    delay( extraTime );
    this.servo2.move( SERVO2_MIN_ANGLE );
    delay( extraTime );
    this.servo2.move( SERVO2_MAX_ANGLE );
    delay( extraTime );
    this.servo2.move( 90.0 );
    delay( extraTime );
  }
  
  public void update() {
    // first of all, we update the TCB2_Manager
    this.TCB.update();
    // if we are calibrating,
    if( this.calibrating ){
      // first we check if we still want to calibrate
      if( !this.TCB.want_toCalibrate() ){
        // if not, we reset
        System.out.println( "Change from TCB. Stopping calibration." );
        this.reset();
        return;
      }
      // then we update the state
      this.update_mechanicalState();
      this.update_temperatureState();
    }else{  // if we are not calibrating,
      // we check if we want to calibrate
      if( this.TCB.want_toCalibrate() ){
        System.out.println( "Starting calibration..." );
        if( !this.TCB.coolDown() ){
          System.out.println( "Disconnected: not able to cool down. (0)" );
          this.reset();
          return;
        }
        this.calibrating = true;
        this.tLastChangeStateM = System.nanoTime();
        this.SM.start_storing();
        this.SM.set_storeMeasurements( true );
      }
    }
  }
  
  // to update the position of the servos and the angular velocity
  private void update_mechanicalState() {
    // we choose what to do depending on the state
    switch( this.stateM ) {
      case 0:  // the next change in the mechanical state will be a movement of servo 1
        // we change the state if we have been here for TIME_FOR_EACH_MECHANICAL_STATE nanoseconds
        if(  System.nanoTime() - this.tLastChangeStateM  >  TIME_FOR_EACH_MECHANICAL_STATE  ){
          // first, we stop taking measurements, and wait for the StorageManager to stop taking measurements
          this.SM.set_storeMeasurements( false );
          delay( TIME_TO_WAIT_BEFORE_CHANGE );
          // we change servo 1
          this.servo1.move_random();
          this.print_info();
          // we change the measurement number
          this.SM.change_measurementNumber();
          // we take note of the last time in which we changed the mechanical state
          this.tLastChangeStateM = System.nanoTime();
          // and we allow measurements again
          this.SM.set_storeMeasurements( true );
          // finally we go to state 1
          this.stateM = 1;
        }
        break;
      case 1:  // the next change in state will be a movement of servo 2
        // we change the state if we have been here for TIME_FOR_EACH_MECHANICAL_STATE nanoseconds
        if(  System.nanoTime() - this.tLastChangeStateM  >  TIME_FOR_EACH_MECHANICAL_STATE  ){
          // first, we stop taking measurements, and wait for the StorageManager to stop taking measurements
          this.SM.set_storeMeasurements( false );
          delay( TIME_TO_WAIT_BEFORE_CHANGE );
          // we change servo 2
          this.servo2.move_random();
          this.print_info();
          // we change the measurement number
          this.SM.change_measurementNumber();
          // we take note of the last time in which we changed the mechanical state
          this.tLastChangeStateM = System.nanoTime();
          // and we allow measurements again
          this.SM.set_storeMeasurements( true );
          // finally we go to state 2
          this.stateM = 2;
        }
        break;
      case 2:  // the next change in state will be a change in the angular velocity
        // we change the state if we have been here for TIME_FOR_EACH_MECHANICAL_STATE nanoseconds
        if(  System.nanoTime() - this.tLastChangeStateM  >  TIME_FOR_EACH_MECHANICAL_STATE  ){
          // first, we stop taking measurements
          this.SM.set_storeMeasurements( false );
          delay( TIME_TO_WAIT_BEFORE_CHANGE );  // we wait for the StorageManager to stop taking measurements
          // we change the angular velocity
          this.nAV++;
          if( this.nAV >= this.targetAngularVelocity.length ){
            this.nAV = 0;
          }
          if( !this.TCB.set_angularVelocity( this.targetAngularVelocity[this.nAV] ) ){
            System.out.println( "Disconnected: not able to set angular velocity." );
            this.reset();
          }
          this.print_info();
          delay( STEPPER_MOTOR_TIME_CHANGE );  // we wait for the angular velocity to be established
          // we change the measurement number
          this.SM.change_measurementNumber();
          // we take note of the last time in which we changed the mechanical state
          this.tLastChangeStateM = System.nanoTime();
          // and we allow measurements again
          this.SM.set_storeMeasurements( true );
          // finally we go to state 0
          this.stateM = 0;
        }
        break;
      default:
        this.reset();
        break;
    }  // end switch( this.stateM )
  }  // end update_mechanicalState()
  
  // to update the temperature control
  private void update_temperatureState() {
    // we choose what to do depending on the state
    switch( this.stateT ){
      case 0:  // cooling
        if(  this.SM.get_temperature() <= this.Tmin  ||  this.get_RPiTemperature() < MIN_RPI_TEMPERATURE  ){
          this.SM.set_storeMeasurements( false );
          delay( TIME_TO_WAIT_BEFORE_CHANGE );
          if( this.get_RPiTemperature() < MIN_RPI_TEMPERATURE ){
            System.out.println( "RPi below the safe temperature!" );
          }
          if( !this.TCB.warmUp() ){
            System.out.println( "Disconnected: not able to warm up." );
            this.reset();
          }
          this.print_info();
          delay( RELAYS_TIME_OPERATION );
          this.SM.set_storeMeasurements( true );
          this.stateT = 1;
        }
        break;
      case 1:  // heating
        if(  this.SM.get_temperature() >= this.Tmax  ||  this.get_RPiTemperature() > MAX_RPI_TEMPERATURE  ){
          this.SM.set_storeMeasurements( false );
          delay( TIME_TO_WAIT_BEFORE_CHANGE );
          if( this.get_RPiTemperature() > MAX_RPI_TEMPERATURE ){
            System.out.println( "RPi above the safe temperature!" );
          }
          if( !this.TCB.coolDown() ){
            System.out.println( "Disconnected: not able to cool down." );
            this.reset();
          }
          this.print_info();
          delay( RELAYS_TIME_OPERATION );
          this.SM.set_storeMeasurements( true );
          this.nc++;
          if( this.nc > MAX_CYCLES ){
            this.finish();
          }
          this.stateT = 0;
        }
        break;
      default:
        this.reset();
        break;
    }
  }
  
  public void notify_activity() {
    this.SM.notify_activity();
  }
  
  public void stop() {
    this.SM.stop();
  }
  
  public void finish() {
    this.SM.set_storeMeasurements( false );
    delay( TIME_TO_WAIT_BEFORE_CHANGE );  // we wait for the StorageManager to stop taking measurements
    this.servo1.move( 90.0 );
    this.servo2.move( 90.0 );
    this.TCB.set_angularVelocity( 0.0 );
    this.stop();
    exit();
  }
  
  // PRIVATE METHODS
  
  private void reset() {
    this.SM.set_storeMeasurements( false );
    this.SM.stop_storing();
    this.calibrating = false;
    this.nc = 0;
    this.stateM = 0;
    this.stateT = 0;
    this.nAV = 0;
    this.TCB.reset_calibrate();
  }
  
  private float get_RPiTemperature() {
    try{
      Runtime rt = Runtime.getRuntime();
      Process proc = rt.exec( "cat /sys/class/thermal/thermal_zone0/temp" );
      proc.waitFor();
      BufferedReader stdInput = new BufferedReader( new InputStreamReader( proc.getInputStream() ) );
      // read the output from the command
      return Float.parseFloat( stdInput.readLine() )/1000.0;
    }catch( Exception e ){
      e.printStackTrace();
    }
    return -1.0;
  }
  
  private void print_info() {
    System.out.print( "  time: " + String.format( "%02d" , day() ) +
                             "/" + String.format( "%02d" , hour() ) +
                             ":" + String.format( "%02d" , minute() ) +
                             ":" + String.format( "%02d" , second() ) +
                             ";" );
    System.out.print( "  cycles: " + this.nc + ";" );
    System.out.print( "  information sources: 17" + this.SM.get_informationSources() + ";" );
    if( this.stateT == 0 ) {
      System.out.print( "  stateT: cooling;  objectiveT: " + this.Tmin + ";" );
    }else{
      System.out.print( "  stateT: heating;  objectiveT: " + this.Tmax + ";" );
    }
    System.out.print( "  senseHAT temp: " + String.format( "%.2f" , this.SM.get_temperature() ) + " ºC;" );
    System.out.print( "  RPi temp: " + String.format( "%.2f" , this.get_RPiTemperature() ) + " ºC;" );
    switch( this.stateM ){
      case 0:
        System.out.print( "  servo 1 moved: phi=" + String.format("%.1f",this.SM.phi) + ", theta=" + String.format("%.1f",this.SM.theta) );
        break;
      case 1:
        System.out.print( "  servo 2 moved: phi=" + String.format("%.1f",this.SM.phi) + ", theta=" + String.format("%.1f",this.SM.theta) );
        break;
      case 2:
        System.out.print( "  angular velocity changed to: " + this.targetAngularVelocity[this.nAV] + " ~> " + String.format( "%.4f" , this.TCB.get_angularVelocity() ) + " deg/s;" );
        break;
      default:
        break;
    }
    System.out.println();
  }
  
}
