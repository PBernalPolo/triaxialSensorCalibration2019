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


public class IM_IMU extends IM_Sensor {
  
  // CONSTANTS
  private static final double ca = 9.8*1.0/2048.0;  // from MPU6050 datasheet (sensitivity)  //9.8*16.0/(1<<15);
  private static final double cw = Math.PI/180.0*1.0/16.4;  // from MPU6050 datasheet (sensitivity)  //2000.0*Math.PI/180.0/(1<<15);
  
  
  // MEASUREMENT DEFINITION
  protected class Measurement {
    protected double[] am;
    protected double[] wm;
    protected double Ta;  // temperature of the accelerometer (raw)
    protected double Tw;  // temperature of the gyroscope (raw)
    // CONSTRUCTOR
    private Measurement() {
      this.am = new double[3];
      this.wm = new double[3];
    }
  }
  
  
  // PRIVATE VARIABLES
  protected IM_IMU.Measurement m;
  protected IM_IMU.Measurement mA;
  protected IM_IMU.Measurement mB;
  protected TriaxialCalibration aTC;
  protected TriaxialCalibration wTC;
  
  
  // CONSTRUCTORS
  
  public IM_IMU( int ID , long time ) {
    super( ID , time );  // 6 Degrees Of Freedom: 3-orientation, 3-position
    
    this.Rm = Matrix.identity( 6 , 6 );
    this.Rm.set( 0 , 0  ,  Matrix.identity(3,3).multiply(1.0e-2)  );
    this.Rm.set( 3 , 3  ,  Matrix.identity(3,3).multiply(1.0e0)  );
    this.mA = new IM_IMU.Measurement();
    this.mB = new IM_IMU.Measurement();
    this.aTC = new TriaxialCalibration( 0 );
    this.aTC.set_calibration( IM_IMU.ca );
    this.wTC = new TriaxialCalibration( 0 );
    this.wTC.set_calibration( IM_IMU.cw );
  }
  
  
  // PUBLIC METHODS
  
  public void update_measurement( long time , byte[] b ) {
    // first of all, we take the measurement that is not being used by the update method
    IM_IMU.Measurement mu = ( this.AB )? this.mA : this.mB ;
    // then, we update the measurement
    // b[0];  // INFORMATION_SOURCE_CLASS_ID
    // b[1];  // WHO_AM_I
    mu.am[0] = ( ( b[3] << 8 ) | ( b[2] & 0xFF ) );  // ax
    mu.am[1] = ( ( b[5] << 8 ) | ( b[4] & 0xFF ) );  // ay
    mu.am[2] = ( ( b[7] << 8 ) | ( b[6] & 0xFF ) );  // az
    mu.wm[0] = ( ( b[9] << 8 ) | ( b[8] & 0xFF ) );  // wx
    mu.wm[1] = ( ( b[11] << 8 ) | ( b[10] & 0xFF ) );  // wy
    mu.wm[2] = ( ( b[13] << 8 ) | ( b[12] & 0xFF ) );  // wz
    mu.Ta = ( ( b[15] << 8 ) | ( b[14] & 0xFF ) );  // temp
    mu.Tw = mu.Ta;
    // finally, we make the update effective
    if( !this.updating ){
      this.tLastMeasurement = time;
      this.m = mu;
      this.AB = !this.AB;
      this.processedMeasurement = false;
    }
  }
  
  public void update() {
    // first, we compute the calibrated measurement
    this.processedMeasurement = true;
    if( this.m == null ) return;
    this.updating = true;
    double[] ac = this.aTC.get_calibratedMeasurements( this.m.am , this.m.Ta );
    double[] wc = this.wTC.get_calibratedMeasurements( this.m.wm , this.m.Tw );
    this.updating = false;
    this.m = null;
    Matrix m = new Matrix( 6 , 1 );
    m.set( 0 , 0  ,  ac[0]  );
    m.set( 1 , 0  ,  ac[1]  );
    m.set( 2 , 0  ,  ac[2]  );
    m.set( 3 , 0  ,  wc[0]  );
    m.set( 4 , 0  ,  wc[1]  );
    m.set( 5 , 0  ,  wc[2]  );
    // we predict the state
    this.B.predict( this.tLastMeasurement );
    // then, we compute the predicted measurement
    Matrix RTqOB = this.B.qOB.RT();
    Matrix gOB = Matrix.zeros(3,1);
    gOB.e[2][0] = -9.8;
    Matrix gBB = Matrix.product( RTqOB , gOB );
    Matrix aBB = Matrix.subtraction( this.B.aBOB , gBB );
    Matrix ap = aBB;  // if we had information about the acceleration, it would be used here to predict
    Matrix wp = this.B.wBB;
    // we compute the predicted covariance matrix of the measurement
    Matrix H = Matrix.zeros( 6 , this.B.P.Nrows );
    H.set( 0 , 0  ,  gBB.negative().cross()  );
    H.set( 3 , 3  ,  Matrix.identity(3,3)  );
    Matrix R = this.Rm.copy();
    R.add( this.B.QaBOB );
    Matrix S = Matrix.product_Cholesky( H , this.B.P );
    S.add( R ); 
    Matrix dm = new Matrix( 6 , 1 );
    dm.set( 0 , 0  ,  ap.negative()  );
    dm.set( 3 , 0  ,  wp.negative()  );
    dm.add( m );
    double sf = 1.0/S.get_maxValue();
    sf = ( sf > 0.0 )? sf : 1.0;
    S.scale( sf );
    Matrix.Cholesky( S );
    Matrix K = Matrix.product( this.B.P , H.transposed() );
    K.scale( sf );
    Matrix.solve_Cholesky( S , K );
    Matrix dx = Matrix.product( K , dm );
    Matrix IKH = Matrix.identity( this.B.P.Nrows , this.B.P.Nrows );
    IKH.subtract( Matrix.product( K , H ) );
    this.B.P = Matrix.sum( Matrix.product_Cholesky( IKH , this.B.P ) , Matrix.product_Cholesky( K , R ) );
    // and we update all the components and the covariance matrix in the new chart
    this.B.update( dx );
    // finally, we compute the acceleration measured in the vehicle
    Matrix aSOS = m.get_submatrix( 0 , 0 , 3 , 1 );
    Matrix aBOS = aSOS;
    aBOS.add( Matrix.product( this.B.qOB.RT() , gOB ) );
    this.B.aBOB = aBOS;
  }
  
  public void set_calibration( String[] path ) {
    this.aTC.set_calibration( path[0] );
    this.aTC.change_calibrationUnits( 9.8 );
    this.wTC.set_calibration( path[1] );
    this.wTC.change_calibrationUnits( Math.PI/180.0 );
  }
  
}
