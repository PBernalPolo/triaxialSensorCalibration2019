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


public class UnitQuaternion {
  
  // PRIVATE VARIABLES
  protected double q0;
  protected double q1;
  protected double q2;
  protected double q3;
  
  
  // CONSTRUCTORS
  
  public UnitQuaternion( double p0 , double p1 , double p2 , double p3 ) {
    this.q0 = p0;
    this.q1 = p1;
    this.q2 = p2;
    this.q3 = p3;
  }
  
  
  // PUBLIC METHODS
  
  public String toString() {
    return ( this.q0 + " " + this.q1 + " " + this.q2 + " " + this.q3 );
  }
  
  public UnitQuaternion print() {
    System.out.println( "uq: " + this.toString() );
    return this;
  }
  
  public UnitQuaternion copy() {
    return new UnitQuaternion( this.q0 , this.q1 , this.q2 , this.q3 );
  }
  
  public void set( UnitQuaternion p ) {
    this.q0 = p.q0;
    this.q1 = p.q1;
    this.q2 = p.q2;
    this.q3 = p.q3;
  }
  
  public Matrix get_vector() {
    Matrix qv = new Matrix(3,1);
    qv.e[0][0] = this.q1;
    qv.e[1][0] = this.q2;
    qv.e[2][0] = this.q3;
    return qv;
  }
  
  public double norm() {
    return Math.sqrt( this.q0*this.q0 + this.q1*this.q1 + this.q2*this.q2 + this.q3*this.q3 );
  }
  
  public void normalize() {
    double inorm = 1.0/this.norm();
    this.q0 *= inorm;
    this.q1 *= inorm;
    this.q2 *= inorm;
    this.q3 *= inorm;
  }
  
  public void multiply( UnitQuaternion p ) {
    double qp0 = this.q0*p.q0  - this.q1*p.q1 - this.q2*p.q2 - this.q3*p.q3;
    double qp1 = this.q0*p.q1  + p.q0*this.q1  + this.q2*p.q3 - this.q3*p.q2;
    double qp2 = this.q0*p.q2  + p.q0*this.q2  + this.q3*p.q1 - this.q1*p.q3;
    double qp3 = this.q0*p.q3  + p.q0*this.q3  + this.q1*p.q2 - this.q2*p.q1;
    this.q0 = qp0;
    this.q1 = qp1;
    this.q2 = qp2;
    this.q3 = qp3;
  }
  
  public UnitQuaternion inverse() {
    return new UnitQuaternion( this.q0 , -this.q1 , -this.q2 , -this.q3 );
  }
  
  public Matrix R() {
    Matrix R = new Matrix( 3 , 3 );
    R.e[0][0] = -this.q2*this.q2-this.q3*this.q3;   R.e[0][1] = this.q1*this.q2-this.q3*this.q0;    R.e[0][2] = this.q1*this.q3+this.q2*this.q0;
    R.e[1][0] = this.q1*this.q2+this.q3*this.q0;    R.e[1][1] = -this.q1*this.q1-this.q3*this.q3;   R.e[1][2] = this.q2*this.q3-this.q1*this.q0;
    R.e[2][0] = this.q1*this.q3-this.q2*this.q0;    R.e[2][1] = this.q2*this.q3+this.q1*this.q0;    R.e[2][2] = -this.q1*this.q1-this.q2*this.q2;
    R.e[0][0] += R.e[0][0] + 1.0;   R.e[0][1] += R.e[0][1];         R.e[0][2] += R.e[0][2];
    R.e[1][0] += R.e[1][0];         R.e[1][1] += R.e[1][1] + 1.0;   R.e[1][2] += R.e[1][2];
    R.e[2][0] += R.e[2][0];         R.e[2][1] += R.e[2][1];         R.e[2][2] += R.e[2][2] + 1.0;
    return R;
  }
  
  public Matrix RT() {
    Matrix RT = new Matrix( 3 , 3 );
    RT.e[0][0] = -this.q2*this.q2-this.q3*this.q3;   RT.e[0][1] = this.q1*this.q2+this.q3*this.q0;    RT.e[0][2] = this.q1*this.q3-this.q2*this.q0;
    RT.e[1][0] = this.q1*this.q2-this.q3*this.q0;    RT.e[1][1] = -this.q1*this.q1-this.q3*this.q3;   RT.e[1][2] = this.q2*this.q3+this.q1*this.q0;
    RT.e[2][0] = this.q1*this.q3+this.q2*this.q0;    RT.e[2][1] = this.q2*this.q3-this.q1*this.q0;    RT.e[2][2] = -this.q1*this.q1-this.q2*this.q2;
    RT.e[0][0] += RT.e[0][0] + 1.0;   RT.e[0][1] += RT.e[0][1];         RT.e[0][2] += RT.e[0][2];
    RT.e[1][0] += RT.e[1][0];         RT.e[1][1] += RT.e[1][1] + 1.0;   RT.e[1][2] += RT.e[1][2];
    RT.e[2][0] += RT.e[2][0];         RT.e[2][1] += RT.e[2][1];         RT.e[2][2] += RT.e[2][2] + 1.0;
    return RT;
  }
  
  public double[] get_angleAxis(){
    double[] angAxis = new double[]{ 0.0 , 1.0 , 0.0 , 0.0 };
    double norm = Math.sqrt( this.q1*this.q1 + this.q2*this.q2 + this.q3*this.q3 );
    if( norm > 0.0 ){
      angAxis[0] = 2.0*Math.atan2( norm , this.q0 );
      double inorm = 1.0/norm;
      angAxis[1] = this.q1*inorm;
      angAxis[2] = this.q2*inorm;
      angAxis[3] = this.q3*inorm;
    }
    return angAxis;
  }
  
  
  // STATIC METHODS
  
  public static UnitQuaternion identity() {
    return new UnitQuaternion( 1.0 , 0.0 , 0.0 , 0.0 );
  }
  
  public static UnitQuaternion random() {
    double s1 = Math.sqrt( -2.0*Math.log( 1.0 - Math.random() ) );  // Math.random() returns values in [0,1)  =>  1-Math.random() is in (0,1]
    double a1 = 2.0*Math.PI*Math.random();
    double s2 = Math.sqrt( -2.0*Math.log( 1.0 - Math.random() ) );  // Math.random() returns values in [0,1)  =>  1-Math.random() is in (0,1]
    double a2 = 2.0*Math.PI*Math.random();
    UnitQuaternion q = new UnitQuaternion( s1*Math.cos(a1) , s1*Math.sin(a1) , s2*Math.cos(a2) , s2*Math.sin(a2) );
    q.normalize();
    return q;
  }
  
  public static UnitQuaternion product( UnitQuaternion p , UnitQuaternion q ) {
    return new UnitQuaternion( p.q0*q.q0  - p.q1*q.q1 - p.q2*q.q2 - p.q3*q.q3 ,
                               p.q0*q.q1  + q.q0*p.q1  + p.q2*q.q3 - p.q3*q.q2 ,
                               p.q0*q.q2  + q.q0*p.q2  + p.q3*q.q1 - p.q1*q.q3 ,
                               p.q0*q.q3  + q.q0*p.q3  + p.q1*q.q2 - p.q2*q.q1 );
  }
  
  /*public static void rotate( Vector r , Quaternion q , Vector u ) {
    double[] qu = new double[]{ q.q[0]*u.v[0] + q.q[2]*u.v[2] - q.q[3]*u.v[1] ,
                                q.q[0]*u.v[1] + q.q[3]*u.v[0] - q.q[1]*u.v[2] ,
                                q.q[0]*u.v[2] + q.q[1]*u.v[1] - q.q[2]*u.v[0] };
    double[] s = new double[]{ q.q[2]*qu[2] - q.q[3]*qu[1] ,
                               q.q[3]*qu[0] - q.q[1]*qu[2] ,
                               q.q[1]*qu[1] - q.q[2]*qu[0] };
    r.v[0] = u.v[0] + s[0] + s[0];
    r.v[1] = u.v[1] + s[1] + s[1];
    r.v[2] = u.v[2] + s[2] + s[2];
  }*/
  
}
