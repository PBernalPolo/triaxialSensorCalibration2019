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

#include "StepperController.h"



StepperController::StepperController( int thePin1 , int thePin2 , int thePin3 , int thePin4 )
                  : pin1( thePin1 ) , pin2( thePin2 ) , pin3( thePin3 ) , pin4( thePin4 ) {
}


// setup. We need to do this in the setup of the sketch
void StepperController::setup() {
  // we set the pins as outputs
  pinMode( this->pin1 , OUTPUT );
  pinMode( this->pin2 , OUTPUT );
  pinMode( this->pin3 , OUTPUT );
  pinMode( this->pin4 , OUTPUT );
  // we initialize the indexes
  this->i1 = 0;
  this->i2 = 0;
  this->i3 = 0;
  this->i4 = 0;
  // and we set the pins frequency
  this->setPwmFrequency( this->pin1 );
  this->setPwmFrequency( this->pin2 );
  this->setPwmFrequency( this->pin3 );
  this->setPwmFrequency( this->pin4 );
  // we initialize the last update time, and the update time
  this->tLastUpdate = micros();
  this->dt = 0;
  // we initialize the indexes per update
  this->indexesPerUpdate = 0;
  // and we initialize the angular velocity
  this->set_angularVelocity( 0.0 );
}


// releases the motor
void StepperController::detach_motor() {
  // we set the indices
  this->i2 = this->i1;
  this->i3 = this->i1;
  this->i4 = this->i1;
}


// attaches the motor
void StepperController::attach_motor() {
  // we set the indices
  this->i2 = (uint16_t)( this->i1 + round( NSF*(2.0/4.0) ) ) & ( 0b1111111111111111 >> (16-STEPPER_POWER_2_N_FUNCTION) );
  this->i3 = (uint16_t)( this->i1 + round( NSF*(1.0/4.0) ) ) & ( 0b1111111111111111 >> (16-STEPPER_POWER_2_N_FUNCTION) );
  this->i4 = (uint16_t)( this->i1 + round( NSF*(3.0/4.0) ) ) & ( 0b1111111111111111 >> (16-STEPPER_POWER_2_N_FUNCTION) );
}


// sets the angular velocity
// w = theta/t = (ANGLE_PER_CICLE*indexesPerUpdate/NSF)/(dt*1.0e-6) = ANGLE_PER_CICLE*indexesPerUpdate/(NSF*(dt*1.0e-6))
// w_min = ANGLE_PER_CICLE/(NSF*(dt*1.0e-6))  (when indexesPerUpdate == 1)
// w_max = (NSF*1.0/4.0)*ANGLE_PER_CICLE/(NSF*(MIN_MICROSECONDS_PER_UPDATE*1.0e-6))  (maximum secure velocity; for greater angular velocities we will need smooth starting)
// w_lim = (NSF*1.0/2.0)*ANGLE_PER_CICLE/(NSF*(MIN_MICROSECONDS_PER_UPDATE*1.0e-6))  (if indexesPerUpdate > NSF/2, the motor will turn in the other direction)
float StepperController::set_angularVelocity( double w ) {
#if defined DEBUG1
  Serial.println();
  Serial.print( "w introduced: " );
  Serial.println( w );
#endif
  // first, we deactivate the motors to avoid damage
//  this->release_motor();
  float sign = ( 0 < w ) - ( w < 0 );
  // then, we reset the indixesPerUpdate
  this->indexesPerUpdate = 0;
  // we only set the angular velocity if it is different from 0.0
  if( w != 0.0 ){
    // we start from the minimum indexes per update to the maximum
    for( int ipu = 1;  ipu <= NSF*1.0/4.0;  ipu++ ){
      // we compute the update time for this indexesPerUpdate
      this->dt = (ANGLE_PER_CICLE*ipu/abs(w))*(1.0e6/NSF);
      // if the update time is too little,
      if( this->dt < MIN_MICROSECONDS_PER_UPDATE ){
        // try the next one. It will be bigger
        continue;
      }else if( this->dt < MAX_MICROSECONDS_PER_UPDATE ){  // if the update time is in the right interval,
        // we save the configuration, and we stop
        this->indexesPerUpdate = ipu*sign;
        break;
      }else{  // if it is bigger than the one we want
        // we won't get anything better. We saturate
        this->dt = MAX_MICROSECONDS_PER_UPDATE;
        // and we save the configuration because the next one will not be better
        this->indexesPerUpdate = ipu*sign;
        break;
      }
    }
    // now we compute the angular velocity for this dt
    w = ANGLE_PER_CICLE*this->indexesPerUpdate/(NSF*(this->dt*1.0e-6));
    // and we attach the motor
    this->attach_motor();
  }else{
    this->detach_motor();
  }
  
#if defined DEBUG1
  Serial.print( "w fixed: " );
  Serial.println( w );
//  Serial.println( (this->indexesPerUpdate/(MICROSECONDS_PER_UPDATE*1.0e-6))/Nsf*(4*ANGLE_PER_STEP) );
  Serial.print( "indexes per update: " );
  Serial.println( this->indexesPerUpdate );
//  Serial.print( "w min: " );
//  Serial.println( (4.0*ANGLE_PER_STEP)/(Nsf*(MICROSECONDS_PER_UPDATE*1.0e-6)) );
//  Serial.print( "w secure max: " );
//  Serial.println( (Nsf*1.0/4.0)*(4.0*ANGLE_PER_STEP)/(Nsf*(MICROSECONDS_PER_UPDATE*1.0e-6)) );
//  Serial.print( "w max: " );
//  Serial.println( (Nsf*1.0/2.0)*(4.0*ANGLE_PER_STEP)/(Nsf*(MICROSECONDS_PER_UPDATE*1.0e-6)) );
#endif
  // finally, we update the last measurement time
  this->tLastUpdate = micros();
  
  return w;
}


// updates the state of the motors
bool StepperController::update() {
  // we only update if enough time has passed
  unsigned long t = micros();
  if(  t - this->tLastUpdate  <  this->dt  ){
    return false;
  }else{
    this->tLastUpdate += this->dt;
//    this->tLastUpdate = t;
  }
  // first, we update the PWM signals
  analogWrite( this->pin1 , StepperController::stepperFunction[this->i1] );
  analogWrite( this->pin2 , StepperController::stepperFunction[this->i2] );
  analogWrite( this->pin3 , StepperController::stepperFunction[this->i3] );
  analogWrite( this->pin4 , StepperController::stepperFunction[this->i4] );
  // then, we update the indexes for the next update
  this->i1 = ( this->i1 + this->indexesPerUpdate ) & ( 0b1111111111111111 >> (16-STEPPER_POWER_2_N_FUNCTION) );
  this->i2 = ( this->i2 + this->indexesPerUpdate ) & ( 0b1111111111111111 >> (16-STEPPER_POWER_2_N_FUNCTION) );
  this->i3 = ( this->i3 + this->indexesPerUpdate ) & ( 0b1111111111111111 >> (16-STEPPER_POWER_2_N_FUNCTION) );
  this->i4 = ( this->i4 + this->indexesPerUpdate ) & ( 0b1111111111111111 >> (16-STEPPER_POWER_2_N_FUNCTION) );
  return true;
}


// updates the state of the motors (less efficient version, but more human readable)
void StepperController::update0() {
  // we only update if enough time has passed
  unsigned long t = micros();
  if(  t - this->tLastUpdate  <  this->dt  ){
    return;
  }else{
    this->tLastUpdate = t;
  }
  // first, we update the PWM signals
  analogWrite( this->pin1 , StepperController::stepperFunction[this->i1] );
  analogWrite( this->pin2 , StepperController::stepperFunction[this->i2] );
  analogWrite( this->pin3 , StepperController::stepperFunction[this->i3] );
  analogWrite( this->pin4 , StepperController::stepperFunction[this->i4] );
  // then, we update the indexes for the next update
  this->i1 += this->indexesPerUpdate;
  while( this->i1 >= NSF ) this->i1 -= NSF;
  this->i2 += this->indexesPerUpdate;
  while( this->i2 >= NSF ) this->i2 -= NSF;
  this->i3 += this->indexesPerUpdate;
  while( this->i3 >= NSF ) this->i3 -= NSF;
  this->i4 += this->indexesPerUpdate;
  while( this->i4 >= NSF ) this->i4 -= NSF;
}


// sets the PWM frequency of a pin
void StepperController::setPwmFrequency( int pin ) {
  if(  pin == 3  ||  pin == 9  ||  pin == 10  ||  pin == 11  ){
    this->setPwmFrequency( pin , 1 );
//    this->setPwmFrequency( pin , 32 );
//    this->setPwmFrequency( pin , 128 );
  }else if(  pin == 5  ||  pin == 6  ){
    this->setPwmFrequency( pin , 64 );
//    this->setPwmFrequency( pin , 256 );
  }else{
    Serial.println( "No PWM for that pin." );
  }
}


// sets the PWM frequency of a pin with a divisor of the base frequency
// h ttps://playground.arduino.cc/Code/PwmFrequency
void StepperController::setPwmFrequency( int pin , int divisor ) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

