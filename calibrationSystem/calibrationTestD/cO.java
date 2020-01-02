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


public class cO extends chart {
  
  private static final double EPSILON = 1.0e-2;
  
  // PROTECTED METHODS
  
  // Method: fC2M
  // defines the map from the chart points, to the manifold points (through the delta quaternion)
  // inputs:
  //  e: point of the Euclidean space that we want to map to a unit quaternion
  // outputs:
  //  delta: quaternion mapped with the e point
  public UnitQuaternion fC2M( double e1 , double e2 , double e3 ){
    // delta from the chart definition: Orthographic
    double enorm = Math.sqrt( e1*e1 + e2*e2 + e3*e3 );
    if( enorm > 2.0-cO.EPSILON ){
      double aux = (2.0-cO.EPSILON)/enorm;
      e1 *= aux;
      e2 *= aux;
      e3 *= aux;
      enorm = 2.0-cO.EPSILON;
    }
    return new UnitQuaternion( Math.sqrt(1.0-0.25*enorm*enorm) , 0.5*e1 , 0.5*e2 , 0.5*e3 );
  }
  
  // Method: chartUpdateMatrix
  // this function defines the transformation on the covariance matrix when it is 
  // redefined from the chart centered in q quaternion, to the chart centered in
  // p quaternion, being them related by  p = q * delta
  // inputs:
  //  delta: quaternion used to update the quaternion estimation
  // outputs:
  //  G: transformation matrix to update the covariance matrix
  public Matrix chartUpdateMatrix( UnitQuaternion delta ){
    Matrix G = Matrix.zeros( 3 , 3 );
    G.e[0][0] = delta.q0;     G.e[0][1] = delta.q3;     G.e[0][2] = -delta.q2;
    G.e[1][0] = -delta.q3;    G.e[1][1] = delta.q0;     G.e[1][2] = delta.q1;
    G.e[2][0] = delta.q2;     G.e[2][1] = -delta.q1;    G.e[2][2] = delta.q0;
    
    Matrix deltav = delta.get_vector();
    G.add( Matrix.product( deltav , deltav.transposed() ).multiply( 1.0/delta.q0 ) );
    
    return G;
  }
  
}
