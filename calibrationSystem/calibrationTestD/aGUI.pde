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


import controlP5.*;  // the wonderful GUI library for processing


// this class contains all GUI elements and implements the methods to link them with external parameters
public class aGUI{
  
  // CONSTANTS
  private final float xiScreen;
  private final float xfScreen;
  private final float yiScreen;
  private final float yfScreen;
  
  // VARIABLES
  // the ControlP5 object
  private ControlP5 cp5;
  
  // state variables
  private float elevation;
  private float azimuth;
  private float zoom;
  
  // GUI elements
  private Slider elevationSlider;
  private Slider azimuthSlider;
  private Slider zoomSlider;
  
  
  // CONSTRUCTORS
  
  public aGUI( PApplet thePApplet ){
    // GUI elements parameters
    float alphaGUI = 0.5*( width + height );
    float elementsSeparation = alphaGUI/30.0;  // distance from GUI elements to borders
    float cameraSlidersWidth = alphaGUI/50;
    float zoomSliderWidth = alphaGUI/30.0;
    
    // we set the constants
    this.xiScreen = 2.0*elementsSeparation + zoomSliderWidth;
    this.xfScreen = width - 2.0*elementsSeparation - cameraSlidersWidth;
    this.yiScreen = 2.0*elementsSeparation + cameraSlidersWidth;
    this.yfScreen = height - elementsSeparation;
    
    // we create the ControlP5 object
    this.cp5 = new ControlP5( thePApplet );
    
    this.elevation = HALF_PI; //0.32; //HALF_PI; //0.0; //HALF_PI;
    this.azimuth = PI; //0.0;
    this.zoom = 380; //200.0; //380.0; //20.0;
    
    // we create the sliders
    this.elevationSlider = this.cp5.addSlider( "elevationSlider" )
                               .setBroadcast(false)
                               .setLabel( "elevation" )
                               .setPosition( width-elementsSeparation-cameraSlidersWidth , 2.0*elementsSeparation+cameraSlidersWidth )
                               .setSize( (int)(cameraSlidersWidth) , (int)(0.5*height-3.0*elementsSeparation-cameraSlidersWidth) )
                               .setRange( -HALF_PI , HALF_PI )
                               .setValue( this.elevation )
                               .setLock( false )
                               .plugTo( this , "set_elevation" )
                               .setBroadcast(true)
                               ;
    this.elevationSlider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    
    this.azimuthSlider = cp5.addSlider( "azimuthSlider" )
                            .setBroadcast(false)
                            .setLabel("azimuth")
                            .setPosition( 2.0*elementsSeparation+zoomSliderWidth , elementsSeparation )
                            .setSize( (int)(width-4.0*elementsSeparation-zoomSliderWidth-cameraSlidersWidth) , (int)(cameraSlidersWidth) )
                            .setRange( 0.0 , TWO_PI )
                            .setValue( this.azimuth )
                            .plugTo( this , "set_azimuth" )
                            .setBroadcast(true)
                            ;
    this.azimuthSlider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    
    this.zoomSlider = cp5.addSlider( "zoomSlider" )
                         .setBroadcast(false)
                         .setLabel("zoom")
                         .setPosition( elementsSeparation , 2.0*elementsSeparation+cameraSlidersWidth )
                         .setSize( (int)(zoomSliderWidth) , (int)(0.5*height-3.0*elementsSeparation-cameraSlidersWidth) )
                         .setRange( 1/*280.0*/ , 1000.0 )
                         .setValue( this.zoom )
                         .plugTo( this , "set_zoom" )
                         .setBroadcast(true)
                         ;
    this.zoomSlider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    
  }
  
  
  // PUBLIC METHODS
  
  public void set_elevation( float theValue ) {
    this.elevation = theValue;
  }
  
  public void set_azimuth( float theValue ) {
    this.azimuth = theValue;
  }
  
  public void set_zoom( float theValue ) {
    this.zoom = theValue;
  }
  
  public float get_zoom() {
    return this.zoom;
  }
  
  public void begin_view() {
    //pointLight(255, 255, 255, 0.5*width, 0.5*width, 1000);
    pushMatrix();
    translate( 0.5*width , 0.5*height , 0.0 );
    scale( zoom );
    rotateX( HALF_PI - this.elevation );
    rotateZ( -this.azimuth );
    //lights();
    
  }
  
  public void end_view() {
    popMatrix();
  }
  
  public boolean is_mouseOnScreen() {
    return (  this.xiScreen < mouseX  &&  mouseX < this.xfScreen  &&  this.yiScreen < mouseY  &&  mouseY < this.yfScreen  );
  }
  
}
