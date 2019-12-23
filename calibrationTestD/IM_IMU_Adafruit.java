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


public class IM_IMU_Adafruit extends IM_IMU {
  
  // CONSTANTS
  private static final double ca = 9.8*1.0e-3;  // from datasheet (sensitivity)
  private static final double cw = Math.PI/180.0*70.0e-3;  // from datasheet (sensitivity)
  
  
  // CONSTRUCTORS
  
  public IM_IMU_Adafruit( int ID , long time ) {
    super( ID , time );
    this.aTC.set_calibration( IM_IMU_Adafruit.ca );
    this.wTC.set_calibration( IM_IMU_Adafruit.cw );
  }
  
  
  // PUBLIC METHODS
  
  public void update_measurement( long time , byte[] b ) {
    // first of all, we take the measurement that is not being used by the update method
    IM_IMU.Measurement mu = (IM_IMU.Measurement)(( this.AB )? this.mA : this.mB );
    // then, we update the measurement
    // b[0];  // INFORMATION_SOURCE_CLASS_ID
    // b[1];  // WHO_AM_I
    mu.am[0] = ( ( b[3] << 8 ) | ( b[2] & 0xFF ) );  // ax
    mu.am[1] = ( ( b[5] << 8 ) | ( b[4] & 0xFF ) );  // ay
    mu.am[2] = ( ( b[7] << 8 ) | ( b[6] & 0xFF ) );  // az
    //mu.mm[0] = ( ( b[9] << 8 ) | ( b[8] & 0xFF ) );  // mx
    //mu.mm[1] = ( ( b[11] << 8 ) | ( b[10] & 0xFF ) );  // my
    //mu.mm[2] = ( ( b[13] << 8 ) | ( b[12] & 0xFF ) );  // mz
    mu.Ta = ( ( b[15] << 8 ) | ( b[14] & 0xFF ) );  // tempAM
    mu.wm[0] = ( ( b[17] << 8 ) | ( b[16] & 0xFF ) );  // wx
    mu.wm[1] = ( ( b[19] << 8 ) | ( b[18] & 0xFF ) );  // wy
    mu.wm[2] = ( ( b[21] << 8 ) | ( b[20] & 0xFF ) );  // wz
    mu.Tw = ( ( b[23] << 8 ) | ( b[22] & 0xFF ) );  // tempW
    mu.Ta = 0.0;
    mu.Tw = 0.0;
    // finally, we make the update effective
    if(  !this.updating  &&  mu.Ta > -100.0  ){  // we filter, as this sensor produces faulty data
      this.tLastMeasurement = time;
      this.m = mu;
      this.AB = !this.AB;
      this.processedMeasurement = false;
    }
  }
  
}
