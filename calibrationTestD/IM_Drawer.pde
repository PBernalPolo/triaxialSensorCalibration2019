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


public class IM_Drawer {
  
  // VARIABLES
  private Map<Integer,Integer> mc;  // ID -> color
  private PShape rfShape;
  
  
  // CONSTRUCTORS
  
  public IM_Drawer() {
    this.mc = new HashMap<Integer,Integer>();
    this.rfShape = this.generate_shape();
  }
  
  
  // PUBLIC METHODS
  
  public void set_colorForID( int ID , color c ) {
    this.mc.put( ID , c );
  }
  
  public void draw( IM_RigidBody rb , float traceWeight ) {
    // first of all, we copy the orientation and position of the RB; otherwise we would see something like a glitch
    UnitQuaternion qOB = rb.qOB.copy();
    Matrix xOOB = rb.xOOB.copy();
    // we draw the rigid body
    // qRB = qRO * qOB = qOR^* * qOB
    // xOOR + R(qOR) xRRB = xOOB  =>  xRRB = R^T(qOR) ( xOOB - xOOR )
    pushMatrix();
    // finally, we want the reference frame to be right-handed
    scale( 1.0 , -1.0 , 1.0 );
    // START REFERENCE FRAME O
    // we draw the trayectory of the vehicle
    rb.xOOBs[rb.cx].set( 0 , 0  ,  rb.xOOB  );
    rb.cx++;
    rb.cx = ( rb.cx < rb.Nxmax )? rb.cx : 0;
    rb.Nxinc++;
    rb.Nxinc = ( rb.Nxinc < rb.Nxmax )? rb.Nxinc : rb.Nxmax;
    strokeWeight( traceWeight );
    noFill();
    beginShape();
    //stroke( 204 , 255 , 204 );
    for(int i=0, c=rb.cx; i<rb.Nxinc; i++){
      c--;
      c = ( c >= 0 )? c : rb.Nxmax-1;
      //stroke( 128 , 65 , 0 , 255-255.0*i/this.Nxmax );
      vertex( rb.xOOBs[c].e[0][0] , rb.xOOBs[c].e[1][0] , rb.xOOBs[c].e[2][0] );
    }
    endShape();
    // END REFERENCE FRAME O
    // second, we translate xOOB
    translate( xOOB.e[0][0] , xOOB.e[1][0] , xOOB.e[2][0] );
    // first, we rotate qOB
    double[] angleAxis = qOB.get_angleAxis();
    rotate( angleAxis[0] , angleAxis[1] , angleAxis[2] , angleAxis[3] );
    // START REFERENCE FRAME B
    // we draw some vectorial information here
    strokeCap(ROUND);
    strokeWeight(0.002);
    //  acceleration
    stroke( 0 , 0 , 255 );
    line( 0.0 , 0.0 , 0.0 , rb.aBOB.e[0][0] , rb.aBOB.e[1][0] , rb.aBOB.e[2][0] );
    //  angular velocity
    stroke( 0 , 255 , 0 );
    line( 0.0 , 0.0 , 0.0 , rb.wBB.e[0][0] , rb.wBB.e[1][0] , rb.wBB.e[2][0] );
    // we draw the IM_RigidBody
    shape( this.rfShape );
    // END REFERENCE FRAME B
    popMatrix();
  }
  
  public void draw( IM_Sensor s , float traceWeight ) {
    // we set the fill
    this.rfShape.setFill( this.mc.get( s.ID ) );  // exception? you probably have not defined the color for this sensor
    stroke( this.mc.get( s.ID ) );
    // and we draw the reference frame
    this.draw( s.B , traceWeight );
  }
  
  
  // PRIVATE METHODS
  
  private PShape generate_shape() {
    float scale = 0.1; //1.0;
    float theLength = 1.0*scale;
    float theWidth = 0.1*scale;
    // we create the axes independently
    PShape box = createShape( BOX , theWidth ,  theWidth , theWidth );
    PShape x = generate_rodShape( 4 , theLength , theWidth );
    x.rotateY( HALF_PI );
    PShape y = generate_rodShape( 3 , theLength , theWidth );
    y.rotateX( -HALF_PI );
    PShape z = generate_rodShape( 10 , theLength , theWidth );
    // we merge the axes to form the reference frame
    PShape rf = createShape( GROUP );
    rf.addChild( x );
    rf.addChild( y );
    rf.addChild( z );
    rf.addChild( box );
    return rf;
  }
  
  private PShape generate_rodShape( int N , float theLength , float theWidth ) {
    PShape top = createShape();
    PShape wall = createShape();
    top.beginShape();
    wall.beginShape( QUAD_STRIP );
    top.stroke( 0 );
    if( N < 5 ){
      wall.stroke( 0 );
    }else{
      wall.noStroke();
    }
    for(int n=0; n<=N; n++){
      float theta = (TWO_PI*n)/N;
      top.vertex( 0.5*theWidth*cos( theta ) , 0.5*theWidth*sin( theta ) , theLength );
      wall.vertex( 0.5*theWidth*cos( theta ) , 0.5*theWidth*sin( theta ) , theLength );
      wall.vertex( 0.5*theWidth*cos( theta ) , 0.5*theWidth*sin( theta ) , 0.5*theWidth );
    }
    top.endShape( CLOSE );
    wall.endShape();
    // we merge both shapes
    PShape rod = createShape( GROUP );
    rod.addChild( top );
    rod.addChild( wall );
    return rod;
  }
  
}
