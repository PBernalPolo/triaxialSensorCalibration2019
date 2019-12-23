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


import java.io.*;


class TriaxialCalibration {
  
  // PRIVATE VARIABLES
  
  private int N;  // order of the polynomial
  private int N9;  // 9*N (for efficiency purposes)
  private double[] z;  // array that will contain the calibration coefficients
  
  
  // PUBLIC CONSTRUCTORS
  
  public TriaxialCalibration( int theN ) {
    this.reset( theN );
    this.z[0] = 1.0;
    this.z[2] = 1.0;
    this.z[5] = 1.0;
  }
  
  public TriaxialCalibration( String path ) {
    this.set_calibration( path );
  }
  
  
  // PUBLIC METHODS
  
  public synchronized void set_calibration( double c ) {
    for(int k=0; k<this.z.length; k++){
      this.z[k] = 0.0;
    }
    this.z[0] = c;
    this.z[2] = c;
    this.z[5] = c;
  }
  
  public synchronized void set_calibration( double cx , double cy , double cz ) {
    for(int k=0; k<this.z.length; k++){
      this.z[k] = 0.0;
    }
    this.z[0] = cx;
    this.z[2] = cy;
    this.z[5] = cz;
  }
  
  public synchronized void set_calibration( String path ) {
    // https://www.programcreek.com/2011/03/java-read-a-file-line-by-line-code-example/
    try{
      // we open the file
      FileInputStream fstream = new FileInputStream( path );
      BufferedReader br = new BufferedReader( new InputStreamReader(fstream) );
      // first, we obtain the order of the polynomial
      String strLine = br.readLine();
      String[] strValues = strLine.split(" ");
      int theN = Integer.parseInt( strValues[0] );
      this.reset( theN );
      // then, we obtain the matrix elements and the offset for each degree
      for(int n=0; n<=this.N; n++){
        strLine = br.readLine();
        strValues = strLine.split(" ");
        double[] theValues = new double[0];
        for(int i=0; i<strValues.length; i++){
          try{
            double newDouble = Double.parseDouble( strValues[i] );
            double[] newValues = new double[theValues.length+1];
            for(int j=0; j<theValues.length; j++) newValues[j] = theValues[j];
            newValues[ theValues.length ] = newDouble;
            theValues = newValues;
          }catch( Exception e ){
          }
        }
        if( theValues.length != 9 ) throw new Exception( "TriaxialCalibration: wrong calibration." );
        for(int i=0; i<9; i++) this.z[n*9+i] = theValues[i];
      }
      // we close
      br.close();
      fstream.close();
    }catch( Exception e ){
      e.printStackTrace();
    }
  }
  
  public synchronized void change_calibrationUnits( double c ) {
    for(int k=0; k<this.z.length; k++){
      this.z[k] *= c;
    }
  }
  
  public synchronized double[] get_calibratedMeasurements( double[] m , double T ) {
    double[] cm = new double[3];
    double Tn = 1.0;
    for(int n9=0; n9<=this.N9; n9+=9){
      cm[0] += ( this.z[n9+0]*m[0]                                          +  this.z[n9+6] )*Tn;
      cm[1] += ( this.z[n9+1]*m[0] + this.z[n9+2]*m[1]                      +  this.z[n9+7] )*Tn;
      cm[2] += ( this.z[n9+3]*m[0] + this.z[n9+4]*m[1] + this.z[n9+5]*m[2]  +  this.z[n9+8] )*Tn;
      Tn *= T;
    }
    return cm;
  }
  
  
  // PRIVATE METHODS
  
  private void reset( int theN ) {
    this.N = theN;
    this.N9 = 9*this.N;
    this.z = new double[9*(this.N+1)];
  }
  
}
