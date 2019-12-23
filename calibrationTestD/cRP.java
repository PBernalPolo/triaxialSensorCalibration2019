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


public class cRP extends chart {
  
  // PROTECTED METHODS
  
  // Method: fC2M
  // defines the map from the chart points, to the manifold points (through the delta quaternion)
  // inputs:
  //  e: point of the Euclidean space that we want to map to a unit quaternion
  // outputs:
  //  delta: quaternion mapped with the e point
  public UnitQuaternion fC2M( double e1 , double e2 , double e3 ){
    // delta from the chart definition: Rodrigues Parameters
    double aux = 1.0/Math.sqrt( 4.0 + e1*e1 + e2*e2 + e3*e3 );
    return new UnitQuaternion( 2.0*aux , e1*aux , e2*aux , e3*aux );
  }
  
  // Method: chartUpdateMatrix
  // Caution: this method modifies the value of q
  // this function defines the transformation on the covariance matrix when it is 
  // redefined from the chart centered in q quaternion, to the chart centered in
  // p quaternion, being them related by  p = q * delta
  // inputs:
  //  delta: quaternion used to update the quaternion estimation
  // outputs:
  //  G: transformation matrix to update the covariance matrix
  public Matrix chartUpdateMatrix( UnitQuaternion delta ){
    // we will not use delta again in this update, so we transform it to save computations
    delta.q1 *= delta.q0;
    delta.q2 *= delta.q0;
    delta.q3 *= delta.q0;
    delta.q0 *= delta.q0;
    // now we build the transformation matrix
    Matrix G = Matrix.zeros( 3 , 3 );
    G.e[0][0] = delta.q0;     G.e[0][1] = delta.q3;     G.e[0][2] = -delta.q2;
    G.e[1][0] = -delta.q3;    G.e[1][1] = delta.q0;     G.e[1][2] = delta.q1;
    G.e[2][0] = delta.q2;     G.e[2][1] = -delta.q1;    G.e[2][2] = delta.q0;
    return G;
  }
  
}
