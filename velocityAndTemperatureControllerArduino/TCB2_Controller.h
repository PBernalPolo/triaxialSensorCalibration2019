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

//#define T_ANGULAR_VELOCITY 100000  // period of sending of the angular velocity (microseconds)
#define T_MAX_WAIT 20000000  // maximum time to wait for a new communication from the RPi before going to the protection state (microseconds)

// The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
#define STEPPER_IN1 3
#define STEPPER_IN2 9
#define STEPPER_IN3 10
#define STEPPER_IN4 11
#define BT_PIN_RX 8
#define BT_PIN_TX 7
#define TCB_PIN_RELAY_1 4
#define TCB_PIN_RELAY_2 5
#define TCB_PIN_PELTIER 6



#include <SoftwareSerial.h>
#include "MessageManager.h"
#include "StepperController.h"


class TCB2_Controller {
  
  private:
    // PRIVATE VARIABLES
    unsigned long tLastCommunication;  // time in which the last communication from the RPi arrived
    int state;  // state of the finite state machine
    float wb;  // angular velocity requested by bluetooth
    float wb0;  // previous angular velocity requested by bluetooth
    float w;  // angular velocity set by the Stepper Controller
    int8_t t;  // 1: heating;  -1: cooling;  0: disconnect
    SoftwareSerial btSerial; //( 4 , 5 );  // RX , TX  (in SoftwareSerial)
    MessageManager MM; //( 2 );
    StepperController SC; //( STEPPER_IN1 , STEPPER_IN2 , STEPPER_IN3 , STEPPER_IN4 );
    
    float ii;
    
  public:
    // CONSTRUCTORS
    TCB2_Controller();
    
    // PUBLIC METHODS
    void setup();
    void reset();
    void test_MM();
    void setPwmFrequency( int pin , int divisor );
    void update_bluetooth();
    void update_angularVelocity();
    void update_temperatureControl();
    void send_w();
    void send_t();
    void warmUp();
    void coolDown();
    void disconnect_Peltier();
    void update();

    int freeRam();
};



TCB2_Controller::TCB2_Controller()
                : btSerial( BT_PIN_RX , BT_PIN_TX ) , MM( 2 ) , SC( STEPPER_IN1 , STEPPER_IN2 , STEPPER_IN3 , STEPPER_IN4 ) {
}


void TCB2_Controller::setup() {
  this->SC.setup();
#if defined DEBUG1
  Serial.println( "ok1" );
#endif
  // then,
//  this->btSerial = new SoftwareSerial( BT_PIN_RX , BT_PIN_TX );  //( 4 , 5 );  // RX , TX  (in SoftwareSerial)
  this->btSerial.begin( 9600 );  // begin bluetooth
#if defined DEBUG1
  Serial.println( "ok2" );
#endif

//  this->MM = new MessageManager( 2 );
#if defined DEBUG1
  Serial.println( "ok3" );
#endif
  
#if !defined DEBUG1
  //this->SC = new StepperController( STEPPER_IN1 , STEPPER_IN2 , STEPPER_IN3 , STEPPER_IN4 );
#endif
#if defined DEBUG1
Serial.println( "ok4" );
#endif
  this->reset();
  // we set the pin outputs
#if defined DEBUG1
  
#else
  pinMode( TCB_PIN_RELAY_1 , OUTPUT );
  pinMode( TCB_PIN_RELAY_2 , OUTPUT );
  pinMode( TCB_PIN_PELTIER , OUTPUT );
#endif
#if defined DEBUG1
Serial.println( "ok5" );
#endif
  
  this->SC.update();
}


void TCB2_Controller::reset() {
  this->state = 0;
  this->wb0 = -1.0;
  this->wb = 0.0;
  this->w = 0.0;
  this->t = 0;
  this->update_angularVelocity();
  this->update_temperatureControl();
  this->SC.update();
}


void TCB2_Controller::test_MM() {
  // we receive
  while( this->btSerial.available() > 0 ){
    int8_t* theBytes = this->MM.manage_byteIn( (int8_t)this->btSerial.read() );
    if( theBytes != NULL ){
#if defined DEBUG1
      Serial.println( this->MM.get_float( 0 , theBytes ) );
#endif
    }
  }
  // we send
  int8_t* theBytes = this->MM.get_bytes( this->ii );
  int8_t* message = this->MM.prepare_message( 4 , theBytes );
  this->ii += 1.0;
  for(int i=0; i<this->MM.get_messageOutLength(); i++){
    this->btSerial.write( message[i] );
  }
  delay( 500 );
}


void TCB2_Controller::update() {
  // we choose what to do depending on the state
  switch( this->state ){
    case 0:{  // we are waiting for the first communication
      // we start the calibration
      int8_t message[] = { 0 , 1 };
      int8_t* toWrite = this->MM.prepare_message( 2 , &message[0] );
      //Serial.print("message");
      for( int i = 0;  i < this->MM.get_messageOutLength();  i++ ){
//#if defined DEBUG1
        Serial.print(" ");
        Serial.print( toWrite[i] );
//#endif
        this->btSerial.write( toWrite[i] );
      }
//#if defined DEBUG1
      Serial.println();
//#endif
      delay( 1000 );  // we wait a little
      // if there is some communication, we go to state 1
      if( this->btSerial.available() > 0 ){
//#if defined DEBUG1
        Serial.println( "Communication: established." );
//#endif
        this->tLastCommunication = micros();
        this->state = 1;
      }
      break;
      }
    case 1:  // we are receiving the data, and updating the state
      // we update the Stepper Controller
#if !defined DEBUG1
      //if( 
      this->SC.update();
      //){
#endif
        // and if we have just updated the stepper, then we update the bluetooth
      this->update_bluetooth();
#if !defined DEBUG1
      //}
#endif
      // we update the state of the controller with the information received by the bluetooth
//      this->update_bluetooth();
      if(  micros() - this->tLastCommunication > T_MAX_WAIT ){
        Serial.println( "Communication: lost. Going to terminal state." );
        this->reset();
        this->state = 2;
      }
      break;
    case 2:  // if we had some issue, or the calibration is finished, we would go to this state
      delay( 10000 );
      break;
    default:
      this->reset();
      break;
  }
  
}


void TCB2_Controller::update_bluetooth() {
  if( this->btSerial.available() > 0 ){
    int8_t* theBytes = this->MM.manage_byteIn( (int8_t)this->btSerial.read() );
    if( theBytes != NULL ){
      switch( theBytes[0] ){
        case 1:
          this->wb = this->MM.get_float( 1 , theBytes );
          this->update_angularVelocity();
          break;
        case 2:
          this->t = theBytes[1];
          this->update_temperatureControl();
          break;
        default:
          break;
      }
      // finally, we update the time in which the last communication arrived
      this->tLastCommunication = micros();
    }
  }
}


void TCB2_Controller::update_angularVelocity() {
#if defined DEBUG1
  Serial.print( this->wb );
  Serial.print( " " );
  Serial.print( this->wb0 );
#endif
  if( this->wb != this->wb0 ){
    this->w = this->SC.set_angularVelocity( this->wb );
    this->wb0 = this->wb;
  }
  // finally, we send back the corrected angular velocity
  this->send_w();
#if defined DEBUG1
  Serial.print( " " );
  Serial.print( this->w );
  Serial.print( " " );
  Serial.println( this->t );
#endif
}


void TCB2_Controller::update_temperatureControl() {
  switch( this->t ){
    case 0:  // disconnect
      this->disconnect_Peltier();
      break;
    case 1:  // warm up
      this->warmUp();
      break;
    case -1:  // cool down
      this->coolDown();
      break;
    default:
      this->t = 0;
      this->disconnect_Peltier();
      break;
  }
  // finally, we send back the temperature control received
  this->send_t();
}


void TCB2_Controller::send_w() {
  // we get the bytes from the float
  int8_t* theBytes = this->MM.get_bytes( this->w );
  // we create the message
  int8_t message[] = { 1 , theBytes[0] , theBytes[1] , theBytes[2] , theBytes[3] };
  // we prepare it
  int8_t* toWrite = this->MM.prepare_message( 5 , &message[0] );
  // and we send it
  for( int i = 0;  i < this->MM.get_messageOutLength();  i++ ){
#if defined DEBUG1
    Serial.print(" ");
    Serial.print( toWrite[i] );
#endif
    this->btSerial.write( toWrite[i] );
  }
#if defined DEBUG1
  Serial.println();
#endif
}


void TCB2_Controller::send_t() {
  // we create the message
  int8_t message[] = { 2 , this->t };
  // we prepare it
  int8_t* toWrite = this->MM.prepare_message( 2 , &message[0] );
  // and we send it
  for( int i = 0;  i < this->MM.get_messageOutLength();  i++ ){
    this->btSerial.write( toWrite[i] );
  }
}


void TCB2_Controller::warmUp() {
  digitalWrite( TCB_PIN_PELTIER , LOW );
  delayMicroseconds( 50000u );
  digitalWrite( TCB_PIN_RELAY_1 , HIGH );
  digitalWrite( TCB_PIN_RELAY_2 , LOW );
  delayMicroseconds( 50000u );
  digitalWrite( TCB_PIN_PELTIER , HIGH );
}

void TCB2_Controller::coolDown() {
  digitalWrite( TCB_PIN_PELTIER , LOW );
  delayMicroseconds( 50000u );
  digitalWrite( TCB_PIN_RELAY_1 , LOW );
  digitalWrite( TCB_PIN_RELAY_2 , HIGH );
  delayMicroseconds( 50000u );
  digitalWrite( TCB_PIN_PELTIER , HIGH );
}

void TCB2_Controller::disconnect_Peltier() {
  digitalWrite( TCB_PIN_PELTIER , LOW );
  delayMicroseconds( 50000u );
  digitalWrite( TCB_PIN_RELAY_1 , LOW );
  digitalWrite( TCB_PIN_RELAY_2 , LOW );
  delayMicroseconds( 50000u );
}

