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


public class AccelerometerCalibrator extends TriaxialCalibrator {
  
  protected String get_sensorName() {
    return "accelerometer";
  }
  
  protected String get_sensorType() {
    return "a";
  }
  
  // returns true if the values are suitable for calibration
  protected boolean measurement_condition( double[] values ) {
    return (  values[this.iID] == this.sID  &&  values[this.iwm] == 0.0  &&
           (  this.sID != 16  ||  values[this.iT] > -100.0  )  &&    // the Adafruit sensor produce a lot of faulty measurements
           !( values[this.ix] == 0  &&  values[this.iy] == 0  && values[this.iz] == 0 )  );
  }
  
  // module of the y vector
  protected double get_module( double[] values ) {
    return 1.0;  // (g) gravity acceleration
  }
  
  // computes the error using the default calibration (the one presented in the datasheet)
  protected double[] compute_error0( String filePath ) {
    double s = 0.0;
    switch( this.sID ){
      case 11:
        s = 1.0/2048.0;
        break;
      case 12:
        s = 1.0/2048.0;
        break;
      case 13:
        s = 1.0/2048.0;
        break;
      case 14:
        s = 1.0/2048.0;
        break;
      case 15:
        s = 1.0/2048.0;
        break;
      case 16:
        s = 1.0e-3;
        break;
      case 17:
        s = 0.732e-3;
        break;
      default:
        break;
    }
    return this.compute_error0( filePath , s );
  }
  
}