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


public class Matrix {
  
  // CONSTANTS
  private static final double MACHINE_EPSILON = 3.0e-16;
  
  // PRIVATE VARIABLES
  protected final int Nrows;
  protected final int Ncols;
  protected double[][] e;
  
  
  // CONSTRUCTORS
  
  public Matrix( int Nr , int Nc ) {
    this.Nrows = Nr;
    this.Ncols = Nc;
    this.e = new double[Nr][Nc];
  }
  
  
  // PUBLIC METHODS
  
  public String size() {
    return ( this.Nrows + "x" + this.Ncols );
  }
  
  public String toString() {
    String output = "";
    for(int i=0; i<this.Nrows; i++){
      for(int j=0; j<this.Ncols; j++){
        output += String.format( "%16.3e" , this.e[i][j] );
      }
      output += "\n";
    }
    output += "\n";
    return output;
  }
  
  public Matrix print() {
    System.out.print( this.toString() );
    return this;
  }
  
  public double get( int i , int j ) {
    return this.e[i][j];
  }
  
  public void set( int i , int j , double v ) {
    this.e[i][j] = v;
  }
  
  public void set( int i , int j , Matrix M ) {
    for(int k=0; k<M.Nrows; k++){
      for(int l=0; l<M.Ncols; l++) this.e[i+k][j+l] = M.e[k][l];
    }
  }
  
  public Matrix get_submatrix( int i0 , int j0 , int Nr , int Nc ) {
    Matrix m = new Matrix( Nr , Nc );
    for(int i=0; i<Nr; i++){
      for(int j=0; j<Nc; j++) m.e[i][j] = this.e[i0+i][j0+j];
    }
    return m;
  }

  public Matrix get_submatrixRows( int i0 , int Nr ) {
    Matrix m = new Matrix( Nr , this.Ncols );
    for(int i=0; i<Nr; i++){
      for(int j=0; j<this.Ncols; j++){
        m.e[i][j] = this.e[i0+i][j];
      }
    }
    return m;
  }
  
  public Matrix get_submatrixColumns( int j0 , int Nc ) {
    Matrix m = new Matrix( this.Nrows , Nc );
    for(int i=0; i<this.Nrows; i++){
      for(int j=0; j<Nc; j++){
        m.e[i][j] = this.e[i][j0+j];
      }
    }
    return m;
  }
  
  public double get_minValue() {
    double min = this.e[0][0];
    for(int i=0; i<this.Nrows; i++){
      for(int j=0; j<this.Ncols; j++){
        min = ( this.e[i][j] < min )? this.e[i][j] : min;
      }
    }
    return min;
  }
  
  public double get_maxValue() {
    double max = this.e[0][0];
    for(int i=0; i<this.Nrows; i++){
      for(int j=0; j<this.Ncols; j++){
        max = ( this.e[i][j] > max )? this.e[i][j] : max;
      }
    }
    return max;
  }
  
  public Matrix copy() {
    Matrix C = new Matrix( this.Nrows , this.Ncols );
    for(int i=0; i<this.Nrows; i++){
      for(int j=0; j<this.Ncols; j++) C.e[i][j] = this.e[i][j];
    }
    return C;
  }
  
  public Matrix transposed() {
    Matrix R = new Matrix( this.Ncols , this.Nrows );
    for(int i=0; i<this.Ncols; i++){
      for(int j=0; j<this.Nrows; j++) R.e[i][j] = this.e[j][i];
    }
    return R;
  }
  
  public void add( Matrix M ) {
    for(int i=0; i<M.Nrows; i++){
      for(int j=0; j<M.Ncols; j++) this.e[i][j] += M.e[i][j];
    }
  }
  
  public void subtract( Matrix M ) {
    for(int i=0; i<M.Nrows; i++){
      for(int j=0; j<M.Ncols; j++) this.e[i][j] -= M.e[i][j];
    }
  }
  
  public Matrix negative() {
    Matrix M = new Matrix( this.Nrows , this.Ncols );
    for(int i=0; i<this.Nrows; i++){
      for(int j=0; j<this.Ncols; j++) M.e[i][j] = -this.e[i][j];
    }
    return M;
  }
  
  public Matrix abs() {
    Matrix M = new Matrix( this.Nrows , this.Ncols );
    for(int i=0; i<this.Nrows; i++){
      for(int j=0; j<this.Ncols; j++){
        M.e[i][j] = ( this.e[i][j] >= 0.0 )? this.e[i][j] : -this.e[i][j];
      }
    }
    return M;
  }
  
  public void scale( double alpha ) {
    for(int i=0; i<this.Nrows; i++){
      for(int j=0; j<this.Ncols; j++){
        this.e[i][j] *= alpha;
      }
    }
  }
  
  public Matrix multiply( double alpha ) {
    Matrix M = new Matrix( this.Nrows , this.Ncols );
    for(int i=0; i<this.Nrows; i++){
      for(int j=0; j<this.Ncols; j++){
        M.e[i][j] = alpha*this.e[i][j];
      }
    }
    return M;
  }
  
  public boolean is_positiveDefinite() {
    Matrix M = this.copy();
    Matrix.Cholesky( M );
    for(int i=this.Nrows-1; i>=0; i--){
      for(int j=i; j>=0; j--){
        if( M.e[i][j] != M.e[i][j] ){
          return false;
        }
      }
    }
    return true;
  }
  
  public boolean is_NaN() {
    for(int i=this.Nrows-1; i>=0; i--){
      for(int j=this.Ncols-1; j>=0; j--){
        if( this.e[i][j] != this.e[i][j] ){
          return true;
        }
      }
    }
    return false;
  }
  
  public void lowPass( double alpha , Matrix M ) {
    double beta = 1.0-alpha;
    for(int i=0; i<M.Nrows; i++){
      for(int j=0; j<M.Ncols; j++) this.e[i][j] = beta*this.e[i][j] + alpha*M.e[i][j];
    }
  }
  
  public UnitQuaternion dw( double dt ) {
    UnitQuaternion dw = UnitQuaternion.identity();
    double nw = Math.sqrt( this.e[0][0]*this.e[0][0] + this.e[1][0]*this.e[1][0] + this.e[2][0]*this.e[2][0] );
    if( nw > 0.0 ){
      double arg = 0.5*nw*dt;
      double sn = Math.sin( arg )/nw;
      dw.q0 = Math.cos( arg );
      dw.q1 = this.e[0][0]*sn;
      dw.q2 = this.e[1][0]*sn;
      dw.q3 = this.e[2][0]*sn;
    }
    return dw;
  }
  
  public Matrix cross() {
    Matrix X = new Matrix( 3 , 3 );
    X.e[0][0] = 0.0;             X.e[0][1] = -this.e[2][0];   X.e[0][2] = this.e[1][0];
    X.e[1][0] = this.e[2][0];    X.e[1][1] = 0.0;             X.e[1][2] = -this.e[0][0];
    X.e[2][0] = -this.e[1][0];   X.e[2][1] = this.e[0][0];    X.e[2][2] = 0.0;
    return X;
  }
  
  public Matrix diagExp() {
    Matrix E = new Matrix( this.Nrows , this.Ncols );
    for(int i=0; i<this.Nrows; i++){
      E.e[i][i] = Math.exp( this.e[i][i] );
    }
    return E;
  }
  
  public Matrix diagInverse() {
    Matrix I = new Matrix( this.Nrows , this.Ncols );
    for(int i=0; i<this.Nrows; i++){
      I.e[i][i] = 1.0/this.e[i][i];
    }
    return I;
  }
  
  public double trace() {
    int min = ( this.Nrows < this.Ncols )? this.Nrows : this.Ncols;
    double sum = 0.0;
    for(int i=0; i<min; i++){
      sum += this.e[i][i];
    }
    return sum;
  }
  
  public double norm() {
    return Math.sqrt( this.e[0][0]*this.e[0][0] + this.e[1][0]*this.e[1][0] + this.e[2][0]*this.e[2][0] );
  }
  
  public Matrix normalized() {
    double norm = this.norm();
    if( norm > 0.0 ){
      return this.multiply( 1.0/norm );
    }else{
      return Matrix.zeros( 3 , 1 );
    }
  }
  
  public void normalize() {
    double norm = this.norm();
    if( norm > 0.0 ){
      this.scale( 1.0/norm );
    }
  }
  
  public double distance( Matrix M ) {
    double sum = 0.0;
    for(int i=0; i<this.Nrows; i++){
      for(int j=0; j<this.Ncols; j++){
        double dif = this.e[i][j] - M.e[i][j];
        sum += dif*dif;
      }
    }
    return Math.sqrt( sum );
  }
  
  
  // STATIC METHODS
  
  public static Matrix zeros( int n , int m ) {
    return new Matrix( n , m );
  }
  
  public static Matrix identity( int n , int m ) {
    Matrix I = new Matrix( n , m );
    if( n < m ){
      for(int i=0; i<n; i++) I.e[i][i] = 1.0;
    }else{
      for(int i=0; i<m; i++) I.e[i][i] = 1.0;
    }
    return I;
  }
  
  public static Matrix random( int n , int m ) {
    Matrix R = new Matrix( n , m );
    for(int i=0; i<n; i++){
      for(int j=0; j<m; j++) R.e[i][j] = Math.random();
    }
    return R;
  }
  
  public static Matrix vector2( double u , double v ) {
    Matrix vec = new Matrix( 2 , 1 );
    vec.e[0][0] = u;
    vec.e[1][0] = v;
    return vec;
  }
  
  public static Matrix vector3( double x , double y , double z ) {
    Matrix vec = new Matrix( 3 , 1 );
    vec.e[0][0] = x;
    vec.e[1][0] = y;
    vec.e[2][0] = z;
    return vec;
  }
  
  public static Matrix sum( Matrix A , Matrix B ) {
    Matrix R = new Matrix( A.Nrows , B.Ncols );
    for(int i=0; i<A.Nrows; i++){
      for(int j=0; j<B.Ncols; j++) R.e[i][j] = A.e[i][j] + B.e[i][j];
    }
    return R;
  }
  
  public static Matrix subtraction( Matrix A , Matrix B ) {
    Matrix R = new Matrix( A.Nrows , B.Ncols );
    for(int i=0; i<A.Nrows; i++){
      for(int j=0; j<B.Ncols; j++) R.e[i][j] = A.e[i][j] - B.e[i][j];
    }
    return R;
  }
  
  public static Matrix product( Matrix A , Matrix B ) {
    Matrix R = new Matrix( A.Nrows , B.Ncols );
    for(int i=0; i<A.Nrows; i++){
      for(int j=0; j<B.Ncols; j++){
        double sum = 0.0;
        for(int k=0; k<B.Nrows; k++) sum += A.e[i][k]*B.e[k][j];
        R.e[i][j] = sum;
      }
    }
    return R;
  }
  
  public static Matrix product_Cholesky( Matrix H , Matrix P ) {
    Matrix Pc = P.copy();
    Matrix.Cholesky( Pc );
    Matrix HL = new Matrix( H.Nrows , P.Ncols );
    for(int i=0; i<H.Nrows; i++){
      for(int j=0; j<P.Ncols; j++){
        double sum = 0.0;
        for(int k=j; k<P.Nrows; k++) sum += H.e[i][k]*Pc.e[k][j];
        HL.e[i][j] = sum;
      }
    }
    Matrix HPHT = new Matrix( H.Nrows , H.Nrows );
    for(int i=0; i<H.Nrows; i++){
      for(int j=i; j<H.Nrows; j++){
        double sum = 0.0;
        for(int k=0; k<P.Nrows; k++) sum += HL.e[i][k]*HL.e[j][k];
        HPHT.e[i][j] = sum;
        HPHT.e[j][i] = sum;
      }
    }
    return HPHT;
  }
  
  public static Matrix product_LDLT( Matrix H , Matrix P ) {
    Matrix Pc = P.copy();
    Matrix.LDLT( Pc );
    Matrix HL = new Matrix( H.Nrows , P.Ncols );
    for(int i=0; i<H.Nrows; i++){
      for(int j=0; j<P.Ncols; j++){
        double sum = H.e[i][j];
        for(int k=j+1; k<P.Nrows; k++) sum += H.e[i][k]*Pc.e[k][j];
        HL.e[i][j] = sum;
      }
    }
    Matrix HPHT = new Matrix( H.Nrows , H.Nrows );
    for(int i=0; i<H.Nrows; i++){
      for(int j=i; j<H.Nrows; j++){
        double sum = 0.0;
        for(int k=0; k<P.Nrows; k++) sum += HL.e[i][k]*Pc.e[k][k]*HL.e[j][k];
        HPHT.e[i][j] = sum;
        HPHT.e[j][i] = sum;
      }
    }
    return HPHT;
  }
  
  public static Matrix cross( Matrix u , Matrix v ) {
    Matrix c = new Matrix( 3 , 1 );
    c.e[0][0] = u.e[1][0]*v.e[2][0] - u.e[2][0]*v.e[1][0];
    c.e[1][0] = u.e[2][0]*v.e[0][0] - u.e[0][0]*v.e[2][0];
    c.e[2][0] = u.e[0][0]*v.e[1][0] - u.e[1][0]*v.e[0][0];
    return c;
  }
  
  // Method: Cholesky
  // Caution: this method modifies the value of S
  // performs the Cholesky decomposition of a positive definite matrix ( S = L*L' )
  // inputs:
  //  S: positive definite matrix to be decomposed
  // outputs:
  //  S: the lower triangular matrix L is overwritten in S
  protected static void Cholesky( Matrix S ) {
    // for each column
    for(int j=0; j<S.Ncols; j++){
      double sumD = 0.0;  // sum for the diagonal term
      // we first fill with 0.0 until diagonal
      for(int i=0; i<j; i++){
        S.e[i][j] = 0.0;
        // we can compute this sum at the same time
        sumD -= S.e[j][i]*S.e[j][i];
      }
      sumD += S.e[j][j];
      // now we compute the diagonal term
      sumD = ( sumD > 0.0 )? sumD : -sumD + MACHINE_EPSILON ;  // with this line we get rid of round-off errors; if sumD is not cero, sumD; else MACHINE_EPSILON
      S.e[j][j] = Math.sqrt( sumD );
      // we compute the terms below the diagonal
      for(int i=j+1; i<S.Ncols; i++){
        // first the sum
        double sumL = 0.0;
        for(int k=0; k<j; k++){
          sumL -= S.e[j][k]*S.e[i][k];
        }
        sumL += S.e[i][j];
        // then the division
        S.e[i][j] = sumL/S.e[j][j];
      }
    }  //end for(int j=0; j<S.Ncols; j++)
  }
  
  // Method: LDLT
  // Caution: this method modifies the value of S
  // performs the LDLT decomposition of a positive definite matrix ( S = L*D*L' )
  // inputs:
  //  S: positive definite matrix to be decomposed
  // outputs:
  //  S: L is overwritten in the lower triangular part of S; D is overwritten in the diagonal terms of S
  protected static void LDLT( Matrix S ) {
    // for each column
    for(int j=0; j<S.Ncols; j++){
      double sumD = S.e[j][j];  // sum for the diagonal term
      // we first fill with 0.0 until diagonal
      for(int i=0; i<j; i++){
        S.e[i][j] = 0.0;
        // we can compute this sum at the same time
        sumD -= S.e[j][i]*S.e[j][i]*S.e[i][i];
      }
      // now we set the diagonal term
      S.e[j][j] = sumD;
      // depending on the value of the diagonal term
      if( Math.abs( S.e[j][j] ) > MACHINE_EPSILON ){  // if we know with certainty that S.e[j][j] is not zero
        // we compute the terms below the diagonal
        for(int i=j+1; i<S.Ncols; i++){
          // first the sum
          double sumL = S.e[i][j];
          for(int k=0; k<j; k++){
            sumL -= S.e[j][k]*S.e[i][k]*S.e[k][k];
          }
          // then the division
          S.e[i][j] = sumL/S.e[j][j];
        }
      }else{
        // or we set them to zero
        for(int i=j+1; i<S.Ncols; i++){
          S.e[i][j] = 0.0;
        }
      }  // end if( S.e[j][j] > MACHINE_EPSILON )
    }  // end for(int j=0; j<S.Ncols; j++)
  }
  
  // Method: solve_Cholesky
  // solves the system of linear equations  K*S = M  for K
  // inputs:
  //  L: Cholesky decomposition of the positive definite matrix S
  //  M: matrix
  // outputs:
  //  M: K is stored in the memory space of M
  protected static void solve_Cholesky( Matrix L , Matrix M ) {
    // we need the Cholesky decomposition to transform the system from  K*S = M  into K*L*L' = M
    // Matrix.Cholesky( S ); before!!!!
    
    // then we take each pair of rows of K and M independently
    for(int i=0; i<M.Nrows; i++){
      // first we solve (y*L' = M)
      for(int j=0; j<M.Ncols; j++){
        double sum = M.e[i][j];
        for(int k=0; k<j; k++){
          sum -= M.e[i][k]*L.e[j][k];
        }
        M.e[i][j] = sum/L.e[j][j];
      }
      // now we solve (Ki*L = y)
      for(int j=M.Ncols-1; j>-1; j--){
        double sum = M.e[i][j];
        for(int k=j+1; k<M.Ncols; k++){
          sum -= M.e[i][k]*L.e[k][j];
        }
        M.e[i][j] = sum/L.e[j][j];
      }
    }
  }
  
  // Method: solve_LDLT
  // solves the system of linear equations  K*S = M  for K
  // inputs:
  //  LD: LDLT decomposition of the positive definite matrix S ( the result of LDLT(S) )
  //  M: matrix
  // outputs:
  //  M: K is stored in the memory space of M
  protected static void solve_LDLT( Matrix LD , Matrix M ) {
    // we need the LDLT decomposition to transform the system from  K*S = M  into K*L*D*L' = M
    // Matrix.LDLT( S ); before!!!!
    
    // then, we take each pair of rows of K and M independently
    for(int i=0; i<M.Nrows; i++){
      // first we solve (y*D*L' = M)
      for(int j=0; j<M.Ncols; j++){
        double sum = M.e[i][j];
        for(int k=0; k<j; k++){
          sum -= M.e[i][k]*LD.e[k][k]*LD.e[j][k];
        }
        M.e[i][j] = sum/LD.e[j][j];
      }
      // now we solve (Ki*L = y)
      for(int j=M.Ncols-1; j>-1; j--){
        double sum = M.e[i][j];
        for(int k=j+1; k<M.Ncols; k++){
          sum -= M.e[i][k]*LD.e[k][j];
        }
        M.e[i][j] = sum;
      }
    }
  }
  
  // obtains the inverse of an invertible matrix M (slower and less accurate than inverse_CholeskyProduct)
  protected static void inverse_CholeskySolve( Matrix M ) {
    Matrix.Cholesky( M );
    Matrix I = Matrix.identity( M.Nrows , M.Ncols );
    Matrix.solve_Cholesky( M , I );
    M.e = I.e;
  }
  
  // computes the inverse of a lower triangular matrix L (obtained from a Cholesky factorization)
  protected static void invert_L( Matrix L ) {
    for(int i=0; i<L.Nrows; i++){
      double iLii = 1.0/L.e[i][i];
      for(int j=0; j<i; j++){
        double sum = 0.0;
        for(int k=0; k<i; k++){
          sum -= L.e[i][k]*L.e[k][j];
        }
        L.e[i][j] = sum*iLii;
      }
      L.e[i][i] = iLii;
    }
  }
  
  // obtains the inverse of an invertible matrix M (faster and more accurate than inverse_CholeskySolve)
  // M = L*L'  =>  M^{-1} = (L*L')^{-1} = (L')^{-1}*L^{-1} = (L^{-1})'*L^{-1}
  protected static void inverse_CholeskyProduct( Matrix M ) {
    Matrix.Cholesky( M );
    Matrix.invert_L( M );
    //M.e = Matrix.product( M.transposed() , M ).e;
    double[][] newe = new double[M.Nrows][M.Nrows];
    for(int i=0; i<M.Nrows; i++){
      for(int j=i; j<M.Nrows; j++){
        double sum = 0.0;
        for(int k=0; k<M.Nrows; k++) sum += M.e[k][i]*M.e[k][j];
        newe[i][j] = sum;
        newe[j][i] = sum;
      }
    }
    M.e = newe;
  }
  
  
  // Method: compute_MahalanobisDistance2
  // computes the squared Mahalanobis distance using the Cholesky decomposition of S ( d^2 = y^T*y ,  with y = L^(-1)*x  =>  L*y = x )
  // inputs:
  //  L: Cholesky decomposition of the positive definite matrix S (S=L*L^T)
  //  x: x Matrix
  // outputs:
  //  x: y Matrix is stored in the x memory space
  protected static double compute_MahalanobisDistance2( Matrix L , Matrix x ) {
    // we need the Cholesky decomposition to transform the distance from  x^T*S^(-1)*x  into  [L^(-1)*x]^T*L^(-1)*x
    // Matrix.Cholesky( S ); before!!!!
    double Md = 0.0;
    for(int i=0; i<L.Nrows; i++){
      double sum = x.e[i][0];
      for(int j=0; j<i; j++) sum -= L.e[i][j]*x.e[j][0];
      x.e[i][0] = sum/L.e[i][i];
      Md += x.e[i][0]*x.e[i][0];
    }
    return Md;
  }
  
  
  // Method: compute_determinant
  // computes the squareroot of the determinant of a positive definite matrix using the Cholesky decomposition of S ( S = L*L^T  =>  det(S) = det(L)*det(L^T) )
  protected static double compute_sqrtDeterminant( Matrix L ) {
    double det = 1.0;
    for(int i=0; i<L.Nrows; i++) det *= L.e[i][i];
    return det;
  }
  
}
