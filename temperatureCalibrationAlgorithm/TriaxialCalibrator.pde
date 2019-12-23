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

// class that implements methods to calibrate a triaxial sensor using measurements (w,x,T,y), where x is the 3-vector measured by the
// triaxial sensor, T is the temperature at which the measurements were made, y is the known module of the vector measured externally,
// and w is the weight of the measurement in the cost function
public abstract class TriaxialCalibrator {
  
  // PARAMETERS
  private static final int MAX_POLYNOMIAL_ORDER = 5; // >= 0
  private static final double PROPORTION_OF_CALIBRATION_DATA = 0.75;  // proportion of calibration data (the rest is used to validate the calibration)
  private static final int MAX_CALIBRATION_ITERATIONS_WITHOUT_IMPROVEMENT = 20;  // the iterative calibration method will end if the solution does not improve after MAX_ITERATIONS_WITHOUT_IMPROVEMENT iterations
  private static final int MAX_CALIBRATION_ITERATIONS = 1000;
  
  // VARIABLES
  // order of the polynomial used for the temperature dependence
  private int N;
  private int maxN21;  // 2*N+1
  private int maxN41;  // 4*N+1
  // sum of weights
  private double W;
  // variables used to compute the variance in the measurements
  private double varX;
  private double varY;
  private double varZ;
  // tensors built with measurements (their size depend on N)
  private double[][][][][] X4;  // \sum_m w_m x_{m i} x_{m j} x_{m k} x_{m l} T_m^n  (Nx4x4x4x4)
  private double[][][] Y2;  // \sum_m w_m x_{m i} x_{m j} y_m^2 T_m^n  (Nx4x4)
  // auxiliary tensors used to build the previous ones
  private double[] x1;
  private double[][] x2;
  private double[][][] x3;
  private double[][][][] x4;
  private double[] y1;
  private double[][] y2;
  private double[] Tn;
  // current approximation to the solution
  private double[] zk;  // z_k = ( (K11,K21,K22,K31,K32,K33,c1,c2,c3)^(0) , (K11,K21,K22,K31,K32,K33,c1,c2,c3)^(1) , ... , (K11,K21,K22,K31,K32,K33,c1,c2,c3)^(N) )_k  (9*Nx1)
  // optimal approximation to the solution
  private double[] z;  // z = ( (K11,K21,K22,K31,K32,K33,c1,c2,c3)^(0) , (K11,K21,K22,K31,K32,K33,c1,c2,c3)^(1) , ... , (K11,K21,K22,K31,K32,K33,c1,c2,c3)^(N) )_optimal  (9*Nx1)
  // coefficients of the polynomial matrix A: A^(n)
  private double[][][] A;
  // calibration file parameters
  protected int sID;  // ID of the sensor that we want to calibrate
  protected int iID;  // index in the file for the sensor ID
  protected int ix;  // index in the file for the x measurements
  protected int iy;  // index in the file for the y measurements
  protected int iz;  // index in the file for the z measurements
  protected int iwm;  // index in the file for the angular velocity module
  protected int iT;  // index in the file for the temperature measurements
//  private double pCalibrationData;  // proportion of calibration data (the rest is used to test the calibration)
  private double Tmin;  // minimum temperature in file
  private double Tmax;  // maximum temperature in file
  private double[] intervalsT;  // temperature intervals for which the measurement will be taken for the test calibration
  // some other useful things
  private FileInputStream fis;
  private BufferedReader br;
  
  
  // ABSTRACT METHODS
  protected abstract String get_sensorName();
  protected abstract String get_sensorType();
  protected abstract boolean measurement_condition( double[] values );  // returns true if the values are suitable for calibration
  protected abstract double get_module( double[] values );  // gets the module of the y vector
  protected abstract double[] compute_error0( String filePath );  // computes the error using the default calibration (the one presented in the datasheet)
  
  
  // PUBLIC CONSTRUCTORS
  
  public TriaxialCalibrator() {
    this.maxN21 = 2*MAX_POLYNOMIAL_ORDER+1;
    this.maxN41 = 4*MAX_POLYNOMIAL_ORDER+1;
    // calibration tensors
    this.X4 = new double[this.maxN41][4][4][4][4];
    this.Y2 = new double[this.maxN21][4][4];
    // auxiliary tensors
    this.x1 = new double[4];
    this.x1[3] = 1.0;
    this.x2 = new double[4][4];
    this.x3 = new double[4][4][4];
    this.x4 = new double[4][4][4][4];
    this.y1 = new double[4];
    this.y2 = new double[4][4];
    this.Tn = new double[this.maxN41];
    // algorithm variables
    this.zk = new double[9*(MAX_POLYNOMIAL_ORDER+1)];
    this.z = new double[9*(MAX_POLYNOMIAL_ORDER+1)];
    this.A = new double[MAX_POLYNOMIAL_ORDER+1][3][4];
    // now we reset
    this.reset_tensors();
  }
  
  
  // PUBLIC METHODS
  
  // each sensor has an ID. This function selects the ID of the sensor to be calibrated
  public void set_sensorID( int the_sID ) {
    this.sID = the_sID;
  }
  
  // each sensor has an ID. This function selects the position of the ID in each piece of calibration data
  public void set_indexID( int the_iID ) {
    this.iID = the_iID;
  }
  
  // selects the position of the x, y, and z measurements in each piece of calibration data
  public void set_indexMeasurements( int the_ix , int the_iy , int the_iz ) {
    this.ix = the_ix;
    this.iy = the_iy;
    this.iz = the_iz;
  }
  
  // selects the position of the temperature measurement in each piece of calibration data
  public void set_indexTemperature( int the_iT ) {
    this.iT = the_iT;
  }
  
  // selects the position of the angular velocity module in each piece of calibration data
  public void set_indexAngularVelocity( int the_iwm ) {
    this.iwm = the_iwm;
  }
  
  // calibrates the file located in fPath (without extension), and stores the calibration, and the errors for each polynomial order
  public void calibrate( String fPath ) {
    System.out.println( "Computing polynomial calibration of the " + this.get_sensorName() + "..." );
    this.set_intervalsT( fPath );
    // we set the tensors
    this.reset_tensors();
    this.set_tensors( fPath );
    PrintWriter errFile = createWriter( fPath + ".err" );
    double[] ce0 = this.compute_error0( fPath );
    errFile.println( -1 + " " + ce0[0] + " " + ce0[1] + " " + ce0[2] + " " + ce0[3] );
    // and for each polynomial order,
    double minErr = Double.MAX_VALUE;
    int nMinErr = 0;
    for(int n=0; n<=MAX_POLYNOMIAL_ORDER; n++){
      System.out.println( "  Polynomial order: " + n );
      // we compute the calibration
      this.compute_calibration( n , null );
      // and we compute the error with the validation data
      double[] ce = this.compute_error( fPath );
      errFile.println( n + " " + ce[0] + " " + ce[1] + " " + ce[2] + " " + ce[3] );
      // we update the minimum error in validation data, and its corresponding polynomial order
      if( ce[2] < minErr ){
        minErr = ce[2];
        nMinErr = n;
      }
    }
    errFile.flush();
    errFile.close();
    // finally, we compute the calibration with the polynomial order that gave the minimum error
    System.out.println( " Best polynomial order: " + nMinErr );
    this.intervalsT = new double[]{ this.Tmin , this.Tmax };
    this.compute_calibration( nMinErr , fPath );
    // and we save the calibration
    this.save_calibration( fPath + ".cal" );
  }
  
  // calibrates the selected sensor using the data in fileName using the method above
  public void calibrate_withFile( String dataPath , String fileName ) {
    fileName = fileName.replace( ".dat" , "" );
    this.compute_variances( dataPath + fileName );
    String fPath = dataPath + this.sID + this.get_sensorType() + "_" + fileName;
    int Ndat = this.prepare_dataFile( dataPath+fileName , fPath );
    if( Ndat < 9 ){
      System.out.println( "Not enough data to calibrate." );
      // now we delete the old file
      File fNoData = new File( fPath + ".dat" );
      fNoData.delete();
      return;
    }
    this.calibrate( fPath );
    // and we also generate the calibrated data
    this.generate_calibratedData( fPath );
    System.out.println();
  }
  
  // calibrates the selected sensor using all calibration data stored in dataPath
  public void calibrate_withAll( String dataPath ) {
    // first, we combine the measurements
    String fPath = this.combine_files( dataPath );
    // now we calibrate normally
    this.calibrate( fPath );
    // finally, we remove the file with the combined measurements
    File theFile = new File( fPath + ".dat" );
    theFile.delete();
    System.out.println();
  }
  
  // resets the current approximation to the solution
  public void reset_zk() {
    for(int i=0; i<this.zk.length; i++){
      this.zk[i] = 0.0;
    }
    this.zk[0] = 1.0;
    this.zk[1] = 0.0;   this.zk[2] = 1.0;
    this.zk[3] = 0.0;   this.zk[4] = 0.0;   this.zk[5] = 1.0;
    this.zk[6] = 0.0;   this.zk[7] = 0.0;   this.zk[8] = 0.0;
  }
  
  // resets the tensors
  public void reset_tensors() {
    this.W = 0.0;
    // first we reset the tensors
    for(int i=0; i<4; i++){
      for(int j=0; j<4; j++){
        for(int n=0; n<this.maxN21; n++) this.Y2[n][i][j] = 0.0;
        for(int k=0; k<4; k++){
          for(int l=0; l<4; l++){
            for(int n=0; n<this.maxN41; n++) this.X4[n][i][j][k][l] = 0.0;
          }
        }
      }
    }
  }
  
  // sets the tensors using the data contained in filePath
  private void set_tensors( String filePath ) {
    System.out.print( "  Setting tensors..." );
    this.open_file( filePath + ".dat" );
    double[] theValues = this.get_nextValuesFromFile();
    while( theValues != null ){
      if( this.is_T_inside( theValues[3] ) ){
        this.include_measurement( 1.0 , theValues[0] , theValues[1] , theValues[2] , theValues[3] , theValues[4] );
      }
      theValues = this.get_nextValuesFromFile();
    }
    this.close_file();
    System.out.println( " done." );
  }
  
  // updates the tensors with a data combination
  public void include_measurement( double w , double x1 , double x2 , double x3 , double T , double y ) {
    // only if w is positive
    if( w < 0.0 ){
      println( "[TriaxialCalibrator] include_measurement: w cannot be negative." );
      return;
    }
    this.x1[0] = x1;
    this.x1[1] = x2;
    this.x1[2] = x3;
    // we define the auxiliary factor for this measurement
    double alpha = w/( this.W + w );
    // and we add the contribution to the sum of weights
    this.W += w;
    // we define the square of the module
    double y0 = y*y;
    // now we add the contribution of this measurement to the tensors
    // zeroth-order tensors
    this.Tn[0] = 1.0;
    for(int n=1; n<this.maxN41; n++) this.Tn[n] = this.Tn[n-1]*T;
    // first-order tensors
    for(int i=0; i<4; i++){
      this.y1[i] = y0*this.x1[i];
      // second-order tensors
      for(int j=0; j<4; j++){
        this.x2[i][j] = this.x1[i]*this.x1[j];
        this.y2[i][j] = this.y1[i]*this.x1[j];
        for(int n=0; n<this.maxN21; n++) this.Y2[n][i][j] = (1.0-alpha)*this.Y2[n][i][j] + alpha*this.y2[i][j]*this.Tn[n];
        // third-order tensor
        for(int k=0; k<4; k++){
          this.x3[i][j][k] = this.x2[i][j]*this.x1[k];
          // fourth-order tensor
          for(int l=0; l<4; l++){
            this.x4[i][j][k][l] = this.x3[i][j][k]*this.x1[l];
            for(int n=0; n<this.maxN41; n++) this.X4[n][i][j][k][l] = (1.0-alpha)*this.X4[n][i][j][k][l] + alpha*this.x4[i][j][k][l]*this.Tn[n];
          }  // end l
        }  // end k
      }  // end j
    }  // end i
  }  // end include_measurementCalibration( double w , double[] x , double y , double T )
  
  // computes the temperature calibration of theN order. First we need to set the tensors with the two methods above
  public void compute_calibration( int theN , String filePath ){
    this.N = theN;
    // we generate the file to print the errors
    PrintWriter lmaFile = null;
    if( filePath != null ){
      lmaFile = createWriter( filePath + ".lma" );
    }
    // we compute the solution using the Levenberg–Marquardt algorithm
    this.reset_zk();
    double minError = Double.MAX_VALUE;
    int itWithoutImprovement = 0;
    for(int k=0; k<MAX_CALIBRATION_ITERATIONS; k++){
      // first of all we update the calibration matrix
      this.update_A();
      // we compute the vector J^T*W*dy
      double[] JTWdy = this.get_JTWdy();
      // we compute the J^T*W*J matrix
      double[][] JTWJ = this.get_JTWJ();
      // we compute the next delta in the solution approximation ( delta^T*(J^T*J) = [J^T*(y-f)]^T )
      this.solve( JTWJ , JTWdy , JTWJ.length );  // now dz is stored in JTWdy
      // we compute the current error in the search for the zeros
      double err = 0.0;
      for(int i=0; i<9*(this.N+1); i++) err += JTWdy[i]*JTWdy[i];
      if( lmaFile != null ){
        lmaFile.println( k + " " + err );
      }
      if( err < minError ){
        for(int i=0; i<9*(this.N+1); i++) this.z[i] = this.zk[i];
        minError = err;
        itWithoutImprovement = 0;
      }else{
        itWithoutImprovement++;
        if( itWithoutImprovement > MAX_CALIBRATION_ITERATIONS_WITHOUT_IMPROVEMENT ) break;
      }
      // we update the solution
      for(int i=0; i<9*(this.N+1); i++) this.zk[i] += JTWdy[i];
      // and now we correct for a right-handed orientation
      if( this.zk[0] < 0.0 ) this.zk[0] = -this.zk[0];
      if( this.zk[2] < 0.0 ) this.zk[2] = -this.zk[2];
      if( this.zk[5] < 0.0 ) this.zk[5] = -this.zk[5];
    }  // end iterations
    if( lmaFile != null ){
      lmaFile.flush();
      lmaFile.close();
    }
  }  // end compute_calibration()
  
  // computes the variance in the measurements of each axis
  public void compute_variances( String filePath ) {
    System.out.print( "Computing variances..." );
    // variables used to compute the variance in the measurements
    double mNumber = -1;
    double Ex = 0.0;
    double Exx = 0.0;
    double Ey = 0.0;
    double Eyy = 0.0;
    double Ez = 0.0;
    double Ezz = 0.0;
    double Ndat = 0.0;
    double Nvar = 0.0;
    // first, we take the data of sID from the main file
    this.open_file( filePath + ".dat" );
    double[] theValues = this.get_nextValuesFromFile();
    while( theValues != null ){
      try{
        if( this.measurement_condition( theValues ) ){
          Ndat += 1.0;
          if( theValues[0] != mNumber ){
            Ex /= Ndat;
            Ey /= Ndat;
            Ez /= Ndat;
            this.varX += Exx/Ndat - Ex*Ex;
            this.varY += Eyy/Ndat - Ey*Ey;
            this.varZ += Ezz/Ndat - Ez*Ez;
            Nvar += 1.0;
            mNumber = theValues[0];
            Ex = 0.0;
            Ey = 0.0;
            Ez = 0.0;
            Exx = 0.0;
            Eyy = 0.0;
            Ezz = 0.0;
            Ndat = 0.0;
          }
          Ex += theValues[this.ix];
          Ey += theValues[this.iy];
          Ez += theValues[this.iz];
          Exx += theValues[this.ix]*theValues[this.ix];
          Eyy += theValues[this.iy]*theValues[this.iy];
          Ezz += theValues[this.iz]*theValues[this.iz];
        }
      }catch( Exception e ){
        //println( "Exception: prepare_accelerometerFile: it could be due to a line with less values than expected." );
        //println( "Error: " + e.getMessage() );
      }
      theValues = this.get_nextValuesFromFile();
    }
    this.close_file();
    this.varX /= Nvar;
    this.varY /= Nvar;
    this.varZ /= Nvar;
    System.out.println( " done." );
  }
  
  // stores the optimal approximation to the solution
  public void save_calibration( String path ){
    // we create the calibration file
    PrintWriter calibrationFile = createWriter( path );
    // first we specify the order of the polynomial used for calibration
    calibrationFile.println( this.N + " " + this.Tmin + " " + this.Tmax );
    // we store the calibration
    for(int n=0; n<=this.N; n++){
      for(int i=0; i<9; i++) calibrationFile.print( " " + this.z[n*9+i] );
      calibrationFile.println();
    }
    calibrationFile.println( "\n\n\n" );
    // we store the variance
    calibrationFile.println( this.z[0]*this.z[0]*this.varX + " " + this.z[2]*this.z[2]*this.varY + " " + this.z[5]*this.z[5]*this.varZ );
    calibrationFile.println( "\n\n\n" );
    // now we store the human readable calibration
    //  1 row
    calibrationFile.print( "K = [ " );
    calibrationFile.print( "  " + this.z[0] + " +    " );
    for(int i=0; i<2; i++) calibrationFile.print( "                      " + 0.0 );
    calibrationFile.println();
    for(int n=1; n<this.N; n++){
      calibrationFile.println( "      + " + this.z[n*9] + " +    " );
    }
    if( this.N > 0 ){
      calibrationFile.print( "      + " + this.z[this.N*9] + "      " );
    }
    calibrationFile.println( "\n" );
    //  2 row
    for(int i=1; i<3; i++) calibrationFile.print( "        " + this.z[i] + " +    " );
    calibrationFile.println( "            " + 0.0 + "              " );
    for(int n=1; n<this.N; n++){
      for(int i=1; i<3; i++) calibrationFile.print( "      + " + this.z[n*9+i] + " +    " );
      calibrationFile.println();
    }
    if( this.N > 0 ){
      for(int i=1; i<3; i++) calibrationFile.print( "      + " + this.z[this.N*9+i] + "      " );
    }
    calibrationFile.println( "\n" );
    //  3 row
    for(int i=3; i<6; i++) calibrationFile.print( "        " + this.z[i] + " +    " );
    calibrationFile.println();
    for(int n=1; n<this.N; n++){
      for(int i=3; i<6; i++) calibrationFile.print( "      + " + this.z[n*9+i] + " +    " );
      calibrationFile.println();
    }
    if( this.N > 0 ){
      for(int i=3; i<6; i++) calibrationFile.print( "      + " + this.z[this.N*9+i] + "      " );
    }
    calibrationFile.println( " ]\n\n\n" );
    // c
    calibrationFile.print( "c = [ " );
    for(int i=6; i<9; i++) calibrationFile.print( "  " + this.z[i] + " +          " );
    calibrationFile.println();
    for(int n=1; n<this.N; n++){
      for(int i=6; i<9; i++) calibrationFile.print( "      + " + this.z[n*9+i] + " +    " );
      calibrationFile.println();
    }
    if( this.N > 0 ){
      for(int i=6; i<9; i++) calibrationFile.print( "      + " + this.z[this.N*9+i] + "      " );
    }
    calibrationFile.println( " ]" );
    calibrationFile.flush();
    calibrationFile.close();
  }
  
  
  
  // PRIVATE METHODS
  
  //  PRIVATE METHODS FOR FILE MANIPULATION
  
  // https://www.programcreek.com/2011/03/java-read-a-file-line-by-line-code-example/
  private void open_file( String path ) {
    try{
      this.fis = new FileInputStream( path );
      this.br = new BufferedReader( new InputStreamReader( this.fis ) );
    }catch( Exception e ){
      println( "Error: " + e.getMessage() );
    }
  }
  
  private String get_nextStringFromFile() {
    try{
      return this.br.readLine();
    }catch( Exception e ){
      println( "Error: " + e.getMessage() );
      return null;
    }
  }
  
  private double[] get_valuesFromString( String strLine ) {
    if( strLine != null ){
      String[] strValues = strLine.split(" ");
      double[] theValues = new double[strValues.length];
      try{
        for(int i=0; i<strValues.length; i++) theValues[i] = Float.parseFloat( strValues[i] );
      }catch( Exception e ){  // if one of them can not be interpreted as a number, we return null
        return null;
      }
      return theValues;
    }else{
      return null;
    }
  }
  
  private double[] get_nextValuesFromFile() {
    return this.get_valuesFromString( this.get_nextStringFromFile() );
  }
  
  private void close_file() {
    try{
      this.br.close();
      this.fis.close();
    }catch( Exception e ){
      println( "Error: " + e.getMessage() );
    }
  }
  
  // creates a file with a convenient data layout for the selected sensor
  private int prepare_dataFile( String pathIn , String pathOut ) {
    System.out.print( "Preparing " + this.get_sensorName() + " data file..." );
    // first, we take the data of sID from the main file
    int Ndat = 0;
    PrintWriter calibrationFile = createWriter( pathOut + ".dat" );
    this.open_file( pathIn + ".dat" );
    double[] theValues = this.get_nextValuesFromFile();
    while( theValues != null ){
      try{
        if( this.measurement_condition( theValues ) ){
          calibrationFile.println( theValues[this.ix] + " " + theValues[this.iy] + " " + theValues[this.iz] +
                             " " + theValues[this.iT] +
                             " " + this.get_module( theValues ) );
          Ndat++;
        }
      }catch( Exception e ){
        //println( "Exception: prepare_accelerometerFile: it could be due to a line with less values than expected." );
        //println( "Error: " + e.getMessage() );
      }
      theValues = this.get_nextValuesFromFile();
    }
    this.close_file();
    calibrationFile.flush();
    calibrationFile.close();
    System.out.println( " done." );
    return Ndat;
  }
  
  // combines all the data files created with "prepare_dataFile" to perform the overall calibration with "calibrate_withAll"
  private String combine_files( String dataPath ) {
    // first, we get the files for this sensor
    File folder = new File( dataPath );
    File[] files = folder.listFiles();
    // then, we combine the measurements into a single file
    String sensorName = this.sID + this.get_sensorType();
    String fPath = dataPath + sensorName;
    PrintWriter calibrationFile = createWriter( fPath + ".dat" );
    for(int i=0; i<files.length; i++){
      String fileName = files[i].getName().replace( dataPath , "" );
      if(  fileName.contains( sensorName + "_" )  &&  fileName.endsWith( ".dat" )  ){
        this.open_file( dataPath + fileName );
        String nextString;
        while( ( nextString = this.get_nextStringFromFile() ) != null ){
          calibrationFile.println( nextString );
        }
        this.close_file();
      }
    }
    calibrationFile.flush();
    calibrationFile.close();
    return fPath;
  }
  
  // sets the intervals for the calibration subset and the validation subset using the PROPORTION_OF_CALIBRATION_DATA
  private void set_intervalsT( String filePath ) {
    System.out.print( "  Setting calibration data..." );
    // first, we find the maximum and minimum temperature
    this.Tmin = Double.MAX_VALUE;
    this.Tmax = -Double.MAX_VALUE;
    this.open_file( filePath + ".dat" );
    double[] theValues = this.get_nextValuesFromFile();
    while( theValues != null ){
      this.Tmin = ( this.Tmin < theValues[3] )? this.Tmin : theValues[3];
      this.Tmax = ( this.Tmax > theValues[3] )? this.Tmax : theValues[3];
      // we take the next values
      theValues = this.get_nextValuesFromFile();
    }
    //System.out.println( this.Tmin + " " + this.Tmax );
    this.close_file();
    // we compute the temperature range
    // extrapolation
    double Tmin1 = 0.5*( this.Tmin + this.Tmax ) - 0.5*PROPORTION_OF_CALIBRATION_DATA*( this.Tmax - this.Tmin );
    double Tmax1 = 0.5*( this.Tmin + this.Tmax ) + 0.5*PROPORTION_OF_CALIBRATION_DATA*( this.Tmax - this.Tmin );
    //double Tmin1 = this.Tmin + 0.5*(1.0-this.pCalibrationData)*(this.Tmax-this.Tmin);
    //double Tmax1 = this.Tmax - 0.5*(1.0-this.pCalibrationData)*(this.Tmax-this.Tmin);
    this.intervalsT = new double[]{ Tmin1 , Tmax1 };
    // interpolation
//    double Tmin1 = this.Tmin;
//    double Tmax1 = 0.5*( this.Tmin + this.Tmax ) - 0.5*(1.0-pCalibrationData)*(this.Tmax-this.Tmin);
//    double Tmin2 = 0.5*( this.Tmin + this.Tmax ) + 0.5*(1.0-pCalibrationData)*(this.Tmax-this.Tmin);
//    double Tmax2 = this.Tmax;
//    this.intervalsT = new double[]{ Tmin1 , Tmax1 , Tmin2 , Tmax2 };
    // interpolation and extrapolation
//    double Tmin1 = Tmin + 0.333*(1.0-pCalibrationData)*(Tmax-Tmin);
//    double Tmax1 = 0.5*(Tmax+Tmin) - 0.5*0.333*(1.0-pCalibrationData)*(Tmax-Tmin);
//    double Tmin2 = 0.5*(Tmax+Tmin) + 0.5*0.333*(1.0-pCalibrationData)*(Tmax-Tmin);
//    double Tmax2 = Tmax - 0.333*(1.0-pCalibrationData)*(Tmax-Tmin);
//    this.intervalsT = new double[]{ Tmin1 , Tmax1 , Tmin2 , Tmax2 };
    System.out.println( " done." );
  }
  
  // returns true if T is inside of the calibration data
  private boolean is_T_inside( double T ) {
    boolean inside = false;
    for(int k=0; k<this.intervalsT.length; k+=2){
      if( this.intervalsT[k] <= T  &&  T <= this.intervalsT[k+1] ){
        inside = true;
        break;
      }
    }
    return inside;
  }
  
  
  //  PRIVATE METHODS FOR DATA ANALYSIS
  
  // gets the matrix A
  private double[][] get_A( double T ) {
    double[][] AT = new double[3][4];
    double Tn = 1.0;
    for(int n=0; n<=this.N; n++){
      int n9 = 9*n;
      AT[0][0] += this.z[n9]*Tn;     AT[0][1] += 0.0;               AT[0][2] += 0.0;               AT[0][3] += this.z[n9+6]*Tn;
      AT[1][0] += this.z[n9+1]*Tn;   AT[1][1] += this.z[n9+2]*Tn;   AT[1][2] += 0.0;               AT[1][3] += this.z[n9+7]*Tn;
      AT[2][0] += this.z[n9+3]*Tn;   AT[2][1] += this.z[n9+4]*Tn;   AT[2][2] += this.z[n9+5]*Tn;   AT[2][3] += this.z[n9+8]*Tn;
      Tn *= T;
    }
    return AT;
  }
  
  // gets the corrected vector with the optimal approximation to the solution
  private double[] get_correctedVector( double x , double y , double z , double T ) {
    double[][] AT = this.get_A( T );
    return new double[]{ AT[0][0]*x + AT[0][1]*y + AT[0][2]*z + AT[0][3] ,
                         AT[1][0]*x + AT[1][1]*y + AT[1][2]*z + AT[1][3] ,
                         AT[2][0]*x + AT[2][1]*y + AT[2][2]*z + AT[2][3] };
  }
  
  // gets the corrected module with the optimal approximation to the solution
  private double get_correctedModule( double x , double y , double z , double T ) {
    double[] vc = this.get_correctedVector( x , y , z , T );
    return Math.sqrt( vc[0]*vc[0] + vc[1]*vc[1] + vc[2]*vc[2] );
  }
  
  // computes the MAE and the variances in the calibration data and the validation data with the default calibration (the one presented in the datasheet)
  protected double[] compute_error0( String filePath , double sensitivity ) {
    System.out.print( "  Computing default " + this.get_sensorName() + " error..." );
    this.open_file( filePath + ".dat" );
    double serrC = 0.0;
    double serr2C = 0.0;
    int nmC = 0;
    double serrV = 0.0;
    double serr2V = 0.0;
    int nmV = 0;
    double[] theValues = this.get_nextValuesFromFile();
    while( theValues != null ){
      double y = Math.abs( theValues[4] );
      double x = sensitivity*Math.sqrt( theValues[0]*theValues[0] + theValues[1]*theValues[1] + theValues[2]*theValues[2] );
      double err = Math.abs( y - x );
      if( this.is_T_inside( theValues[3] ) ){
        serrC += err;
        serr2C += err*err;
        nmC++;
      }else{
        serrV += err;
        serr2V += err*err;
        nmV++;
      }
      theValues = this.get_nextValuesFromFile();
    }
    this.close_file();
    double merrC = serrC/nmC;
    double merrV = serrV/nmV;
    double[] toReturn = new double[]{ merrC , Math.sqrt( Math.abs( serr2C/nmC - merrC*merrC ) ) , merrV , Math.sqrt( Math.abs( serr2V/nmV - merrV*merrV ) ) };  // we take absolute value to get rid of rounding errors
    System.out.println( " done." );
    return toReturn;
  }
  
  // computes the MAE and the variances in the calibration data and the validation data with the optimal approximation to the solution
  private double[] compute_error( String filePath ) {
    System.out.print( "    computing calibrated " + this.get_sensorName() + " error..." );
    this.open_file( filePath + ".dat" );
    double serrC = 0.0;
    double serr2C = 0.0;
    int nmC = 0;
    double serrV = 0.0;
    double serr2V = 0.0;
    int nmV = 0;
    double[] theValues = this.get_nextValuesFromFile();
    while( theValues != null ){
      double y = Math.abs( theValues[4] );
      double x = this.get_correctedModule( theValues[0] , theValues[1] , theValues[2] , theValues[3] );
      double err = Math.abs( y - x );
      if( this.is_T_inside( theValues[3] ) ){
        serrC += err;
        serr2C += err*err;
        nmC++;
      }else{
        serrV += err;
        serr2V += err*err;
        nmV++;
      }
      theValues = this.get_nextValuesFromFile();
    }
    this.close_file();
    double merrC = serrC/nmC;
    double merrV = serrV/nmV;
    double[] toReturn = new double[]{ merrC , Math.sqrt( Math.abs( serr2C/nmC - merrC*merrC ) ) , merrV , Math.sqrt( Math.abs( serr2V/nmV - merrV*merrV ) ) };  // we take absolute value to get rid of rounding errors
    System.out.println( " done." );
    return toReturn;
  }
  
  // stores the calibrated data with the optimal approximation to the solution
  private void generate_calibratedData( String filePath ) {
    System.out.print( " Generating calibrated data..." );
    PrintWriter newFile = createWriter( filePath + ".new" );
    this.open_file( filePath + ".dat" );
    double[] theValues = this.get_nextValuesFromFile();
    while( theValues != null ){
      double x = theValues[0];
      double y = theValues[1];
      double z = theValues[2];
      double T = theValues[3];
      double[] vc = this.get_correctedVector( x , y , z , T );
      for(int i=0; i<5; i++){
        newFile.print( theValues[i] + " " );
      }
      newFile.println( vc[0] + " " + vc[1] + " " + vc[2] );
      theValues = this.get_nextValuesFromFile();
    }
    this.close_file();
    newFile.flush();
    newFile.close();
    // now we delete the old file
    File fOld = new File( filePath + ".dat" );
    fOld.delete();
    // and we rename the new one
    File fNew = new File( filePath + ".new" );
    fNew.renameTo( fOld );
    System.out.println( " done." );
  }
  
  
  //  PRIVATE METHODS FOR CALIBRATION
  
  // updates the matrix A with the current approximation to the solution
  private void update_A(){
    for(int n=0; n<=this.N; n++){
      int n9 = 9*n;
      this.A[n][0][0] = this.zk[n9];     this.A[n][0][1] = 0.0;             this.A[n][0][2] = 0.0;             this.A[n][0][3] = this.zk[n9+6];
      this.A[n][1][0] = this.zk[n9+1];   this.A[n][1][1] = this.zk[n9+2];   this.A[n][1][2] = 0.0;             this.A[n][1][3] = this.zk[n9+7];
      this.A[n][2][0] = this.zk[n9+3];   this.A[n][2][1] = this.zk[n9+4];   this.A[n][2][2] = this.zk[n9+5];   this.A[n][2][3] = this.zk[n9+8];
    }
  }
  
  // gets a term of the matrix J^T*W*( y^2 - f )
  private double get_JTWdy( int g , int a , int b ) {
    double sum = 0.0;
    for(int n2=0; n2<=this.N; n2++){
      int nY = n2+g;
      for(int j2=0; j2<4; j2++){
        sum += this.A[n2][a][j2]*this.Y2[nY][j2][b];
      }
      for(int n=0; n<=this.N; n++){
        for(int l=0; l<=this.N; l++){
          int nX = nY+n+l;
          for(int i=0; i<3; i++){
            for(int j1=0; j1<4; j1++){
              for(int k=0; k<4; k++){
                for(int j2=0; j2<4; j2++){
                  sum -= this.A[n2][a][j2]*this.A[n][i][j1]*this.A[l][i][k]*this.X4[nX][j2][b][j1][k];
                }  // j2
              }  // k
            }  // j
          }  // i
        }  // l
      }  // n
    }  // n2
    return 2.0*sum;
  }
  
  // gets the matrix J^T*W*( y^2 - f )
  private double[] get_JTWdy(){
    // now we compute the matrix J^T*W*dy
    double[] JTWdy = new double[9*(this.N+1)];
    int iJ = 0;
    for(int g=0; g<=this.N; g++){
      // K part
      for(int a=0; a<3; a++){
        for(int b=0; b<=a; b++){
          JTWdy[ iJ++ ] = this.get_JTWdy( g , a , b );
        }  // end b
      }  // end a
      // c part
      for(int a=0; a<3; a++){
        JTWdy[ iJ++ ] = this.get_JTWdy( g , a , 3 );
      }  // end a
    }  // end g
    return JTWdy;
  }  // get_JTWdy()
  
  
  // gets a term of the matrix J^T*W*J
  private double get_JTWJ( int g1 , int a1 , int b1 , int g2 , int a2 , int b2 ) {
    double sum = 0.0;
    for(int n1=0; n1<=this.N; n1++){
      for(int n2=0; n2<=this.N; n2++){
        int nX = n1+g1+n2+g2;
        for(int j1=0; j1<4; j1++){
          for(int j2=0; j2<4; j2++){
            sum += this.A[n1][a1][j1]*this.A[n2][a2][j2]*this.X4[nX][j1][b1][j2][b2];
          }  // j2
        }  // j
      }  // n2
    }  // n
    return 4.0*sum;
  }
  
  // gets the matrix J^T*W*J
  private double[][] get_JTWJ(){
    // now we compute the matrix J^T*W*J
    double[][] JTWJ = new double[9*(this.N+1)][9*(this.N+1)];
    int iJ1 = 0;
    for(int g=0; g<=this.N; g++){
      // K# part
      for(int a=0; a<3; a++){
        for(int b=0; b<=a; b++){
          int iJ2 = 0;
          for(int g2=0; g2<=this.N; g2++){
            // K part
            for(int a2=0; a2<3; a2++){
              for(int b2=0; b2<=a2; b2++){
                JTWJ[iJ1][iJ2] = this.get_JTWJ( g , a , b , g2 , a2 , b2 );
                iJ2++;
              }
            }
            // c part
            for(int a2=0; a2<3; a2++){
              JTWJ[iJ1][iJ2] = this.get_JTWJ( g , a , b , g2 , a2 , 3 );
              iJ2++;
            }
          }
          iJ1++;
        }
      }
      // c# part
      for(int a=0; a<3; a++){
        int iJ2 = 0;
        for(int g2=0; g2<=this.N; g2++){
          // K part
          for(int a2=0; a2<3; a2++){
            for(int b2=0; b2<=a2; b2++){
              JTWJ[iJ1][iJ2] = this.get_JTWJ( g , a , 3 , g2 , a2 , b2 );
              iJ2++;
            }
          }
          // c part
          for(int a2=0; a2<3; a2++){
            JTWJ[iJ1][iJ2] = this.get_JTWJ( g , a , 3 , g2 , a2 , 3 );
            iJ2++;
          }
        }
        iJ1++;
      }
    }
    return JTWJ;
  }  // end get_JTWJ()
  
  
  // Method: Cholesky
  // performs the Cholesky decomposition of a positive definite matrix ( S = L*L' )
  // inputs:
  //  S: NxN positive definite matrix to be decomposed (must be stored by columns)
  // outputs:
  //  S: the lower triangular matrix L (6x6) is overwritten in S (is stored by columns)
  private void Cholesky( double[][] S , int n ){
    // for each column
    for(int j=0; j<n; j++){
      double sum = 0.0;  //sum for the diagonal term
      // we first fill with 0.0 until diagonal
      for(int i=0; i<j; i++){
        S[i][j] = 0.0;
        //we can compute this sum at the same time
        sum += S[j][i]*S[j][i];
      }
      // now we compute the diagonal term
      S[j][j] = Math.sqrt( S[j][j] - sum );
      // finally we compute the terms below the diagonal
      for(int i=j+1; i<n; i++){
        //first the sum
        sum = 0.0;
        for(int k=0; k<j; k++){
          sum += S[i][k]*S[j][k];
        }
        //after the non-diagonal term
        S[i][j] = ( S[i][j] - sum )/S[j][j];
      }
    }//end j
    
    return;
  }
  
  
  // Method: solve
  // solves the system of linear equations  K*S = M  for K
  // inputs:
  //  S: nxn positive definite matrix
  //  M: 1xn matrix stored by rows
  // outputs:
  //  M: K (1xn) is stored in the M memory space
  private void solve( double[][] S , double[] M , int n ){
    // we first compute the Cholesky decomposition for transform the system from  K*S = M  into K*L*L' = M
    this.Cholesky( S , n );
    
    // first we solve (y*L' = M)
    for(int j=0; j<n; j++){
      double sum = M[j];
      for(int k=0; k<j; k++){
        sum -= M[k]*S[j][k];
      }
      M[j] = sum/S[j][j];
    }
    // now we solve (Ki*L = y)
    for(int j=n-1; j>-1; j--){
      double sum = M[j];
      for(int k=j+1; k<n; k++){
        sum -= M[k]*S[k][j];
      }
      M[j] = sum/S[j][j];
    }
    
    return;
  }
  
}