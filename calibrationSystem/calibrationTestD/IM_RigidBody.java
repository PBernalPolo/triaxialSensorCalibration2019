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


import java.util.*;


public class IM_RigidBody {
  
  // PRIVATE VARIABLES
  protected final int DOF;
  protected final Matrix F;
  protected final Matrix Q;
  protected chart c;
  
  protected long tLastUpdate;
  protected UnitQuaternion qOB;
  protected Matrix wBB;
  protected Matrix xOOB;
  protected Matrix vOOB;
  protected Matrix tBB;
  protected Matrix aBOB;
  protected Matrix QtBB;
  protected Matrix QaBOB;
  protected Matrix P;
  
  protected final int Nxmax = 100;
  protected int Nxinc;
  protected int cx;
  protected Matrix[] xOOBs;
  
  
  // CONSTRUCTORS
  
  protected IM_RigidBody( long time ) {
    this.DOF = 12;  // 12 Degrees Of Freedom: 3-orientation, 3-angular velocity, 3-position, 3-velocity
    this.F = Matrix.identity( this.DOF , this.DOF );
    this.Q = Matrix.zeros( this.DOF , this.DOF );
    this.c = new cRP();
    
    this.tLastUpdate = time;
    this.QtBB = Matrix.identity( 3 , 3 ).multiply(1.0e1);
    this.QaBOB = Matrix.identity( 3 , 3 ).multiply(1.0e-1);
    this.P = this.get_P0();
    this.reset();
    
    this.Nxinc = 0;
    this.cx = 0;
    this.xOOBs = new Matrix[this.Nxmax];
    for(int i=0; i<this.Nxmax; i++){
      this.xOOBs[i] = new Matrix(3,1);
    }
  }
  
  
  // PROTECTED METHODS
  
  public void reset() {
    this.qOB = UnitQuaternion.identity();
    this.wBB = Matrix.zeros( 3 , 1 );
    this.xOOB = Matrix.zeros( 3 , 1 );
    this.vOOB = Matrix.zeros( 3 , 1 );
    this.tBB = Matrix.zeros( 3 , 1 );
    this.aBOB = Matrix.zeros( 3 , 1 );
  }
  
  public void set( IM_RigidBody rb ) {
    this.tLastUpdate = rb.tLastUpdate;
    this.qOB.set( rb.qOB );
    this.wBB.set( 0 , 0  ,  rb.wBB  );
    this.xOOB.set( 0 , 0  ,  rb.xOOB  );
    this.vOOB.set( 0 , 0  ,  rb.vOOB  );
    this.tBB.set( 0 , 0  ,  rb.tBB  );
    this.aBOB.set( 0 , 0  ,  rb.aBOB  );
    this.QtBB.set( 0 , 0  ,  rb.QtBB  );
    this.QaBOB.set( 0 , 0  ,  rb.QaBOB  );
  }
  
  public void print() {
    System.out.print( "qOB: " );  this.qOB.print();
    System.out.print( "wBB: " );  this.wBB.transposed().print();
    System.out.print( "xOOB: " );  this.xOOB.transposed().print();
    System.out.print( "vOOB: " );  this.vOOB.transposed().print();
    System.out.print( "aBOB: " );  this.aBOB.transposed().print();
    System.out.println();
  }
  
  public String toString() {
    String q = this.qOB.toString();
    String w = this.wBB.transposed().toString();
    String x = this.xOOB.transposed().toString();
    String v = this.vOOB.transposed().toString();
    return ( q + " " + w + " " + x + " " + v );
  }
  
  protected Matrix get_P0() {
    Matrix P0 = Matrix.zeros( this.DOF , this.DOF );
    P0.set( 0 , 0  ,  Matrix.identity( 3 , 3 ).multiply( 1.0e20 )  );
    P0.set( 6 , 6  ,  Matrix.identity( 3 , 3 ).multiply( 1.0e-10 )  );
    return P0;
  }
  
  protected boolean predict( long time ) {
    // first, we compute the time step
    double dt = ( time - this.tLastUpdate )*1.0e-9;
    if( dt < 0.0 ){
      return false;
    }
    this.tLastUpdate = time;
    // then, we compute the rotation quaternion and rotation matrix from the mean angular velocity of the RigidBody
    // now we update the expected values of the state
    //  first the quaternion
    UnitQuaternion dw = this.wBB.dw( dt );
    this.qOB.multiply( dw );
    Matrix RqOB = this.qOB.R();
    Matrix aOOB = Matrix.product( RqOB , this.aBOB );
    //  then, the angular velocity
    this.wBB.add( this.tBB.multiply( dt ) );
    //  then, the velocity
    this.vOOB.add( aOOB.multiply( dt ) );
    //  then, the position
    this.xOOB.add( this.vOOB.multiply( dt ) );
    this.xOOB.add( aOOB.multiply( 0.5*dt*dt ) );
    //  and finally, we reset
    this.tBB.scale( 0.0 );
    this.aBOB.scale( 0.0 );
    // now we can compute the matrices
    //  first F
    Matrix Idt = Matrix.identity( 3 , 3 ).multiply( dt );
    this.F.set( 0 , 0  ,  dw.RT()  );
    this.F.set( 0 , 3  ,  Idt  );
    this.F.set( 6 , 9  ,  Idt  );
    //  then Q
    this.Q.set( 0 , 0  ,  this.QtBB.multiply(dt*dt*dt/3.0)  );
    this.Q.set( 3 , 0  ,  this.QtBB.multiply(-dt*dt/2.0)  );
    this.Q.set( 0 , 3  ,  this.QtBB.multiply(-dt*dt/2.0)  );
    this.Q.set( 3 , 3  ,  this.QtBB.multiply(dt)  );
    Matrix QaOOB = Matrix.product( RqOB , Matrix.product( this.QaBOB , RqOB.transposed() ) );
    this.Q.set( 6 , 6  ,  QaOOB.multiply(dt*dt*dt/3.0)  );
    this.Q.set( 9 , 6  ,  QaOOB.multiply(-dt*dt/2.0)  );
    this.Q.set( 6 , 9  ,  QaOOB.multiply(-dt*dt/2.0)  );
    this.Q.set( 9 , 9  ,  QaOOB.multiply(dt)  );
    // and we update the covariance matrix
    this.P.add( Q );
    this.P = Matrix.product_Cholesky( F , this.P );
    return true;
  }
  
  public void update( Matrix dx ) {
    UnitQuaternion delta = this.c.fC2M( dx.e[0][0] , dx.e[1][0] , dx.e[2][0] );
    this.qOB.multiply( delta );
    this.qOB.normalize();
    this.wBB.e[0][0] += dx.e[3][0];
    this.wBB.e[1][0] += dx.e[4][0];
    this.wBB.e[2][0] += dx.e[5][0];
    this.xOOB.e[0][0] += dx.e[6][0];
    this.xOOB.e[1][0] += dx.e[7][0];
    this.xOOB.e[2][0] += dx.e[8][0];
    this.vOOB.e[0][0] += dx.e[9][0];
    this.vOOB.e[1][0] += dx.e[10][0];
    this.vOOB.e[2][0] += dx.e[11][0];
    Matrix T = Matrix.identity( 12 , 12 );
    T.set( 0 , 0  ,  this.c.chartUpdateMatrix( delta )  );
    this.P = Matrix.product_Cholesky( T , this.P );
  }
  
}
