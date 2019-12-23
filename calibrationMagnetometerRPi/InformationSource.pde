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


public class InformationSource {
  
  // CONSTANTS
  private static final double alpha = 1.0e-1;  // smoothing factor in the exponential smoothing used to compute the data period
  
  // PRIVATE VARIABLES
  private byte[] lastMeasurement;  // last measurement (format: packageLength(1byte), InformationSourceClassID(1byte) WhoAmI(1byte) dataBytes(Sensor dependent) )
  private double tLastMeasurement;  // time at which the last measurement arrived
  private double averageMeasurementPeriod;  // period at which measurements arrive
  
  
  // CONSTRUCTORS
  
  public InformationSource() {
    this.tLastMeasurement = System.nanoTime()*1.0e-9;
    this.averageMeasurementPeriod = 0.0;
  }
  
  
  // PUBLIC METHODS
  
  public void update_measurement( byte[] theMeasurement ) {
    this.lastMeasurement = theMeasurement;
    double t = System.nanoTime()*1.0e-9;
    this.averageMeasurementPeriod = (1.0-InformationSource.alpha)*this.averageMeasurementPeriod + InformationSource.alpha*(t-this.tLastMeasurement);
    this.tLastMeasurement = t;
  }
  
  public byte[] get_measurement() {
    // this is what we will return
    byte[] theReturn = this.lastMeasurement;
    // we point to null with the lastMeasurement (so we do not use the same twice)
    this.lastMeasurement = null;
    // and we return the measurement
    return theReturn;
  }
  
}
