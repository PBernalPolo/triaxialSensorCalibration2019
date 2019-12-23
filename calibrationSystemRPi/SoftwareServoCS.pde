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


public class SoftwareServoCS extends SoftwareServo {
  
  // PARAMETERS
  private static final int TIME_STEP = 10;  // (ms) specifications: 0.1s/60degree => 1.7ms/degree => 10ms/degree to be conservative
  private static final int TIME_DYNAMICS = 2500;  // (ms) waiting time until the servos are detached and the transition dynamic ends
  
  // PRIVATE VARIABLES
  private final int pin;
  private final float min_angle;
  private final float max_angle;
  private float lastPosition;
  
  
  // CONSTRUCTORS
  
  public SoftwareServoCS( PApplet thePApplet , int thePin , float minimum_angle , float maximum_angle ) {
    super( thePApplet );
    this.pin = thePin;
    this.min_angle = minimum_angle;
    this.max_angle = maximum_angle;
    this.lastPosition = 90.0;
  }
  
  
  // PUBLIC METHODS
  
  public void move( float angle ) {
    try{
      // we attach the servo
      this.attach( this.pin );
      while(  !this.attached() );
      // we move the servo step by step
      int N = (int)Math.abs( angle - this.lastPosition );
      float da = Math.signum( angle - this.lastPosition );
      for( int n=0;  n<N;  n++, this.lastPosition+=da ){
        this.write( this.lastPosition );
        delay( TIME_STEP );
      }
      // and we detach the servo to avoid vibrations
      this.detach();
      // we also wait until the servos are detached and the transition dynamic ends
      delay( TIME_DYNAMICS );
    }catch( Exception e ){
      e.printStackTrace();
    }
  }
  
  public void move_random() {
    this.move( random( this.min_angle , this.max_angle ) );
  }
  
}
