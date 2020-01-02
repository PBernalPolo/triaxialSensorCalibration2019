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


public abstract class IM_Sensor {
  
  // PRIVATE VARIABLES
  protected final int ID;
  protected boolean updating;
  protected boolean AB;
  protected boolean processedMeasurement;
  protected long tLastMeasurement;
  protected IM_RigidBody B;
  protected Matrix Rm;
  
  
  // PUBLIC ABSTRACT METHODS
  public abstract void update_measurement( long time , byte[] b );
  public abstract void update();
  
  
  // CONSTRUCTORS
  
  protected IM_Sensor( int theID , long time ) {
    this.ID = theID;
    this.processedMeasurement = true;
    this.updating = false;
    this.tLastMeasurement = time;
    this.B = new IM_RigidBody( time );
  }
  
  // PROTECTED METHODS
  
  public void set_calibration( String[] path ) {
  }
  
}
