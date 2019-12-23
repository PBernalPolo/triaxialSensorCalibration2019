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


public class cRV extends chart {
  
  // PROTECTED METHODS
  
  // Method: fC2M
  // defines the map from the chart points, to the manifold points (through the delta quaternion)
  // inputs:
  //  e: point of the Euclidean space that we want to map to a unit quaternion
  // outputs:
  //  delta: quaternion mapped with the e point
  public UnitQuaternion fC2M( double e1 , double e2 , double e3 ){
    // delta from the chart definition: Rotation Vector
    double enorm = Math.sqrt( e1*e1 + e2*e2 + e3*e3 );
    if( enorm > Math.PI ){
      double aux = Math.PI/enorm;
      e1 *= aux;
      e2 *= aux;
      e3 *= aux;
      enorm = Math.PI;
    }
    UnitQuaternion delta = UnitQuaternion.identity();
    if( enorm != 0.0 ){
      double aux = Math.sin(0.5*enorm)/enorm;
      delta.q0 = Math.cos(0.5*enorm);
      delta.q1 = e1*aux;
      delta.q2 = e2*aux;
      delta.q3 = e3*aux;
    }
    return delta;
  }
  
  // Method: chartUpdateMatrix
  // this function defines the transformation on the covariance matrix
  // when it is redefined from the chart centered in q quaternion, to the
  // chart centered in p quaternion, being them related by  p = q * delta
  // inputs:
  //  delta: quaternion used to update the quaternion estimation
  // outputs:
  //  G: transformation matrix to update the covariance matrix
  public Matrix chartUpdateMatrix( UnitQuaternion delta ){
    double dnorm = Math.sqrt( delta.q1*delta.q1 + delta.q2*delta.q2 + delta.q3*delta.q3 );
    Matrix G;
    if( dnorm != 0.0 ){
      double[] udelta = new double[3];
      double idnorm = 1.0/dnorm;
      udelta[0] = delta.q1*idnorm;
      udelta[1] = delta.q2*idnorm;
      udelta[2] = delta.q3*idnorm;
      double dnasindn = dnorm/Math.asin(dnorm);
      // we will not use delta again in this update, so we transform it to save computations
      delta.q0 *= dnasindn;
      delta.q1 *= dnasindn;
      delta.q2 *= dnasindn;
      delta.q3 *= dnasindn;
      G = Matrix.zeros( 3 , 3 );
      G.e[0][0] = delta.q0;     G.e[0][1] = delta.q3;     G.e[0][2] = -delta.q2;
      G.e[1][0] = -delta.q3;    G.e[1][1] = delta.q0;     G.e[1][2] = delta.q1;
      G.e[2][0] = delta.q2;     G.e[2][1] = -delta.q1;    G.e[2][2] = delta.q0;
      for(int i=0; i<3; i++){
        for(int j=0; j<3; j++) G.e[i][j] += (1.0-delta.q0)*udelta[i]*udelta[j];
      }
    }else{
      G = Matrix.identity( 3 , 3 );
    }
    return G;
  }
  
}
