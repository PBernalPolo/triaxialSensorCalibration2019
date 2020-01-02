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


// class that implements methods to exchange messages with checksum
class MessageManager {
  
  // CONSTANTS
  private:
    static const int8_t magicByte = '\n';  // magic byte to indicate the start of a new package
    
  // PRIVATE VARIABLES
    int Nchecksums;  // number of checksums
    int state;  // state in the management of the incoming message
    int NRB;  // Number of Received Bytes of the current incoming message
    int messageInLength;  // length of the incoming message
    int messageOutLength;  // length of the outgoing message
    int8_t* messageIn;  // pointer to array of bytes that will contain the message being received
    int8_t* checksumIn;  // checksum computed with the incoming message
    int8_t* messageOut;  // pointer to array of bytes that will contain the outgoing message with the MessageManager format
    int8_t* checksumOut;  // checksum for the outgoing message
    float value;  // to store the float of get_bytes() (so that it does not get out of scope)
    
  public:
  // CONSTRUCTORS
    MessageManager( int theNchecksums );
    
    
  // PUBLIC METHODS
    int8_t* manage_byteIn( int8_t newByte );  // manages the bytes of the incoming message. Careful: the array pointed by the return will be deleted when a new message enters
    int MessageManager::get_messageInLength();  // gets the length of the incoming message
    int8_t* prepare_message( int theMessageLength , int8_t* theMessage );  // prepares the bytes of an outgoing message. Careful: the array pointed by the return will be deleted when this method is called again
    int get_messageOutLength();  // gets the length of the prepared message
    int8_t* get_bytes( float value );  // gets the bytes that form a float
    float get_float( int i0 , int8_t* theBytes );  // gets the float from the bytes that form it
};


// CONSTRUCTORS
  
MessageManager::MessageManager( int theNchecksums ) {
  if(  theNchecksums <= 0  ||  127 < theNchecksums  ){
    //System.out.println( "Not a valid number of checksums. Number of checksums set at 1." );
    theNchecksums = 1;
  }
  this->Nchecksums = (int8_t)theNchecksums;
  this->checksumOut = new int8_t[theNchecksums];
  this->checksumIn = new int8_t[theNchecksums];
  this->messageIn = new int8_t[1];
  this->messageOut = new int8_t[1];
  this->state = 0;
  this->NRB = 0;
}


// PUBLIC METHODS
  
// manages the bytes of the incoming message. Careful: the array pointed by the return will be deleted when a new message enters
int8_t* MessageManager::manage_byteIn( int8_t newByte ) {
  int8_t* theReturn = NULL;
  // we act depending on the state of the finite-state machine,
  switch( this->state ){
    case -1:  // we have failed to receive a message
      // we wait for the magic byte,
      if(  newByte == MessageManager::magicByte  &&  random(4) != 0  ){  // if we find the magic byte, we go to state 1 with probability 0.75
        this->NRB = 0;
        this->state = 1;
      }
      break;
    case 0:  // we are not receiving a message,
      // we wait for the magic byte
      if( newByte == MessageManager::magicByte ){  // if we find the magic byte,
        this->NRB = 0;  // we initialize the number of received bytes
        this->state = 1;  // and we go to the next state
      }
      break;
    case 1:  // in this state we obtain the message length
      if( newByte > 0 ){  // the length of the message has to be positive
        // we set the message length,
        this->messageInLength = newByte;
        // create the array to store the message
        delete[] this->messageIn;
        this->messageIn = new int8_t[this->messageInLength];
        // and reset the checksums
        this->checksumIn[0] = 1;
        for(int c=1; c<this->Nchecksums; c++) this->checksumIn[c] = 0;
        // in the next state we will receive the message
        this->state = 2;
        // uncommenting these lines will allow to receive messages of zero length
//      }else if( newByte == 0 ){
//        this->messageIn = new int8_t[0];
//        this->state = 3;
      }else{
        this->state = -1;  // if newByte is negative, it can not be the length of the array; we have failed to receive the message
      }
      break;
    case 2:  // we know the size of the message, and we are receiving it
      // we store the received byte
      this->messageIn[this->NRB++] = newByte;
      // and add its contribution to the checksum
      this->checksumIn[0] += newByte;
      for(int c=1; c<this->Nchecksums; c++) this->checksumIn[c] += this->checksumIn[c-1];
      // if we complete the message,
      if( this->NRB >= this->messageInLength ){
        // then, we go to receive the checksum
        this->NRB = 0;
        this->state = 3;
      }
      break;
    case 3:  // we have finished receiving the message. Now we are receiving the checksum
      // if the checksum does not match,
      if( this->checksumIn[this->NRB++] != newByte ){
        // we have failed to receive the message
        this->state = -1;
      }else if( this->NRB >= this->Nchecksums ){
        // if we reach this line, we have received the message correctly
        theReturn = this->messageIn;
        this->state = 0;  // we go to wait for the next magic byte
      }
      break;
    default:
      // if we are in some other state, we go back to the initial one
      this->state = 0;
      break;
  }  // end switch( this.state )
  return theReturn;
}  // end manage_byteIn( int8_t newByte )


// gets the length of the incoming message
int MessageManager::get_messageInLength() {
  return this->messageInLength;
}


// prepares the bytes of an outgoing message. Careful: the array pointed by the return will be deleted when this method is called again
int8_t* MessageManager::prepare_message( int theMessageLength , int8_t* theMessage ) {
  if( theMessageLength > 127 ){
    this->messageOutLength = 0;
    return NULL;
  }
  // the prepared message will be: ( magicByte , message.length , (message) , (checksums) )
  this->messageOutLength = 2 + theMessageLength + this->Nchecksums;
  delete[] this->messageOut;
  this->messageOut = new int8_t[ this->messageOutLength ];
  this->messageOut[0] = MessageManager::magicByte;
  this->messageOut[1] = (int8_t)theMessageLength;
  // we compute the checksums
  this->checksumOut[0] = 1;
  for(int c=1; c<this->Nchecksums; c++) this->checksumOut[c] = 0;
  for(int i=0; i<theMessageLength; i++){
    this->messageOut[2+i] = theMessage[i];
    this->checksumOut[0] += theMessage[i];
    for(int c=1; c<this->Nchecksums; c++) this->checksumOut[c] += this->checksumOut[c-1];
  }
  for(int c=0; c<this->Nchecksums; c++) this->messageOut[2+theMessageLength+c] = this->checksumOut[c];
  return this->messageOut;
}  // end prepare_message( int MessageLength , int8_t* theMessage )


// gets the length of the prepared message
int MessageManager::get_messageOutLength() {
  return this->messageOutLength;
}


// gets the bytes that form a float
int8_t* MessageManager::get_bytes( float theValue ) {
  this->value = theValue;
  int8_t* packets = (int8_t*) &this->value;  // we create a int8_t pointer pointing to the address of the float
  return packets;
}


// gets the float from the bytes that form it
float MessageManager::get_float( int i0 , int8_t* theBytes ) {
//  float theValue;
//  int8_t* packets = (int8_t*) &theValue;
//  packets[0] = theBytes[i0];
//  packets[1] = theBytes[i0+1];
//  packets[2] = theBytes[i0+2];
//  packets[3] = theBytes[i0+3];
//  return theValue;
  float* theValuePointer = (float*) &theBytes[i0];  // we create a float pointer pointing to the first byte
  return *theValuePointer;  // we return the value of the float pointed by the pointer
}

