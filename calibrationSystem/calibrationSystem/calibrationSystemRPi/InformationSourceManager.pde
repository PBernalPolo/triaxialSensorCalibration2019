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


// class that implements the measurement management
public class InformationSourceManager {
  
  // PARAMETERS
  private static final long MAX_TIME_WITHOUT_MEASUREMENTS = (long)(1.0e9*16.0);  // (nanoseconds)
  
  // PRIVATE VARIABLES
  private boolean availableMeasurements;
  private InformationSource[] IS;  // array of InformationSource's
  private String informationSources;
  
  // CONSTRUCTORS
  
  public InformationSourceManager() {
    this.IS = new InformationSource[0];
    this.informationSources = "";
  }
  
  
  // PUBLIC METHODS
  
  public String get_informationSources() {
    return this.informationSources;
  }
  
  public void add_measurement( byte[] measurement ) {
    if( measurement == null ){
      println( "Warning: introduced null measurement." );
      return;
    }
    if( measurement.length < 2 ){  // the measurement should have the format: sensorClassID(1byte), WhoAmI(1byte), dataBytes(IS dependent)
      println( "Warning: introduced incorrect measurement." );
      return;
    }
    if( measurement[1] < 0 ){  // the WhoAmI should be an index
      println( "Warning: bad WAI value." );
      return;
    }
    if( measurement[1] >= this.IS.length ){  // if we do not have this index, we
      InformationSource[] auxIS = new InformationSource[ measurement[1]+1 ];
      for(int i=0; i<this.IS.length; i++) auxIS[i] = this.IS[i];
      this.IS = auxIS;
    }
    // if the IS does not exists,
    if( this.IS[ measurement[1] ] == null ){
      // we create it
      this.IS[ measurement[1] ] = new InformationSource();
      this.informationSources += "," + measurement[1];
    }
    // finally, we add the measurement to the InformationSource
    this.IS[ measurement[1] ].update_measurement( measurement );
    this.availableMeasurements = true;
  }  // end add_measurement( byte[] measurement )
  
  
  public byte[] get_measurement() {
    // we create the cumulative probability vector and the matching array
    double[] p = new double[this.IS.length];
    int[] ip = new int[this.IS.length];
    int nIS = 0;
    for(int i=0; i<this.IS.length; i++){
      if( this.IS[i] != null ){
        if( this.IS[i].lastMeasurement != null ){
          p[nIS] = this.IS[i].averageMeasurementPeriod;
          ip[nIS] = i;
          nIS++;
        }
      }
    }
    if( nIS < 1 ){
      return null;
    }else if( nIS == 1 ){
      this.availableMeasurements = false;
      return this.IS[ ip[0] ].get_measurement();
    }
    for(int i=1; i<nIS; i++) p[i] += p[i-1];
    // now, we choose the InformationSource depending on its average measurement period
    double theRandom = random( (float)p[nIS-1] );
    for(int i=0; i<nIS; i++){
      if( theRandom < p[i] ) return this.IS[ ip[i] ].get_measurement();
    }
    return null;  // if we reach this line, we return a null pointer
  }  // end get_measurement
  
  
  public boolean available() {
    return this.availableMeasurements;
  }
  
  public int is_everythingAlright() {
    long t = System.nanoTime();
    for(int i=0; i<this.IS.length; i++){
      if(  this.IS[i] != null  &&  t-this.IS[i].get_tLastMeasurement() > MAX_TIME_WITHOUT_MEASUREMENTS  ){
        return i;
      }
    }
    return -1;
  }
  
  public void reset_tLastMeasurement() {
    for(int i=0; i<this.IS.length; i++){
      if( this.IS[i] != null ){
        this.IS[i].reset_tLastMeasurement();
      }
    }
  }
  
}
