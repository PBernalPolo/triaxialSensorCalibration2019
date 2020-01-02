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


// PARAMETERS
long RESET_TIME = (long)(1.0e9*10.0);  // (nanoseconds) time to wait before each reset (zero position; zero velocity)


// class that manages the data obtained from the file
public class FileDataManager implements Runnable {
  
  // PRIVATE VARIABLES
  private int nd;  // index for the data
  private byte data[];  // data taken from the file
  private long t0f;  // initial reference time of the file
  private long t0;  // initial reference time of the program
  private long tLastMeasurement;  // time of the last measurement to compute the time we need to sleep
  private long tLastReset;  // last time we reset the position of the sensors
  private int trajectoryNumber;  // number of generated trajectories
  private int nMeasurements;  // number of measurements, to compute the measurements per second
  private long tMS;  // time from last update, to compute the measurements per second
  private boolean normalPlay;  // true when we are playing at normal speed
  private boolean running;  // true while the thread is running (to manage the run method)
  private boolean wait;  // true if we want the thread to wait (to manage the run method)
  private boolean waiting;  // true if the thread is waiting (to manage the run method)
  private final MessageManager MM;  // used to manage the data reception through the network
  private IM_Sensor[] s;  // sensors to update
  private boolean createOutputFiles;  // true if we want to create the output files
  private PrintWriter[] dFiles;  // here we will store the distances in human-readable format
  
  
  // CONSTRUCTORS
  
  public FileDataManager( String theFile , boolean doICreateOutputFiles ){
    this.nd = 0;
    this.data = loadBytes( sketchPath() + "/storedData/" + theFile );
    this.nMeasurements = 0;
    this.tMS = System.nanoTime();
    this.normalPlay = true;
    this.running = true;
    this.wait = false;
    this.waiting = false;
    this.MM = new MessageManager( 1 );
    // we take the first measurement to know the first time
    // FIRST MEASUREMENTS ARE TAKEN FAR APPART FROM EACH OTHER IN TIME. THIS CAUSES A DEVICE IN THE FIRST FEW SECONDS
    // I tried rejecting the first 10 measurements to get rid of that device, but that messed up the experiment timing
    byte[] m = get_nextData();
    this.tLastReset = this.tLastMeasurement;
    this.trajectoryNumber = 1;
    
    this.createOutputFiles = doICreateOutputFiles;
  }
  
  
  // PUBLIC METHODS
  
  public long get_time() {
    return this.tLastMeasurement;
  }
  
  public void set_sensors( IM_Sensor[] theSensors ) {
    this.s = theSensors;
    // we create a file for each sensor
    if( this.createOutputFiles ){
      this.dFiles = new PrintWriter[ this.s.length ];
      this.create_files();
    }
    this.play_normal();
    
    // once we have set the sensors, we can start the thread
    ( new Thread( this ) ).start();
  }
  
  public void notify_activity() {
    this.wait = false;
    if( this.waiting ){
      synchronized( this ){
        this.notify();
      }
    }
  }
  
  public synchronized void run() {
    while(  this.running  &&  this.nd < this.data.length  ){
      long ts = System.nanoTime();
      byte[] m = get_nextData();
      // and we add the measurement
      if(  m[1] < this.s.length  &&  this.s[ m[1] ] != null  ){
        this.s[ m[1] ].update_measurement( this.tLastMeasurement , m );
        this.s[ m[1] ].update();
        if( this.createOutputFiles ){
          this.dFiles[ m[1] ].println( (this.tLastMeasurement-this.tLastReset) + " " + this.s[ m[1] ].B.vOOB.norm() + " " + this.s[ m[1] ].B.xOOB.norm() );
        }
      }
      if(  m[1]+10 < this.s.length  &&  this.s[ m[1]+10 ] != null ){
        this.s[ m[1]+10 ].update_measurement( this.tLastMeasurement , m );
        this.s[ m[1]+10 ].update();
        if( this.createOutputFiles ){
          this.dFiles[ m[1]+10 ].println( (this.tLastMeasurement-this.tLastReset) + " " + this.s[ m[1]+10 ].B.vOOB.norm() + " " + this.s[ m[1]+10 ].B.xOOB.norm() );
        }
      }
      // and we reset if we have to
      if( this.tLastMeasurement - this.tLastReset > RESET_TIME ){
        this.reset_sensors();
      }
      //
      long te = System.nanoTime();
      long dt = this.tLastMeasurement-this.t0f - (System.nanoTime()-this.t0) - (te-ts);
      if(  this.normalPlay  &&  dt > 0  ){
        long dt1 = dt/1000000;
        int dt2 = (int)(dt - dt1*1000000);
        try{
          Thread.sleep( dt1 );
        }catch( Exception e ){
          System.out.println( "Not sleeping." + dt1 + " " + dt2 );
        }
      }
      //
      if( this.wait ){
        synchronized( this ){
          this.waiting = true;
          try{
            this.wait();
          }catch( Exception e ){
            e.printStackTrace();
          }
          this.waiting = false;
        }
      }
    }  // end while( this.running )
    println( "OUT" );
    // finally, we stop storing data
    this.stop();
  }  // end public synchronized void run()
  
  // resets the position and velocity of each sensor
  void reset_sensors() {
    this.tLastReset = this.tLastMeasurement;
    // we close the previous file if we have to
    if( this.createOutputFiles ){
      this.close_files();
    }
    // we reset the positions
    for(int i=0; i<s.length; i++){
      if( s[i] != null ){
        s[i].B.xOOB.set( 0 , 0  ,  Matrix.zeros(3,1)  );
        s[i].B.vOOB.set( 0 , 0  ,  Matrix.zeros(3,1)  );
      }
    }
    // we create the new files if we have to
    if( this.createOutputFiles ){
      this.create_files();
    }
  }
  
  void stop(){
    this.running = false;
    if( this.createOutputFiles ){
      this.close_files();
    }
  }
  
  public void play_fast() {
    this.wait = false;
    this.normalPlay = false;
    this.notify_activity();
    System.out.println( "play fast." );
  }
  
  public void play_normal() {
    this.wait = false;
    this.normalPlay = true;
    this.t0f = this.tLastMeasurement;
    this.t0 = System.nanoTime();
    this.notify_activity();
    System.out.println( "play normal." );
  }
  
  public void pause() {
    this.wait = true;
    System.out.println( "paused." );
  }
  
  public double get_measurementsPerSecond() {
    long t = System.nanoTime();
    double mps = this.nMeasurements/( (t-this.tMS)*1.0e-9 );
    this.nMeasurements = 0;
    this.tMS = t;
    return mps;
  }
  
  
  // PRIVATE METHODS
  
  private void create_files() {
    for(int i=0; i<this.s.length; i++){
      if( this.s[i] != null ){
        this.dFiles[i] = createWriter( sketchPath() + "/storedData/t" + this.trajectoryNumber + "_" + i + ".dat" );
      }
    }
  }
  
  private void close_files() {
    for(int i=0; i<this.s.length; i++){
      if( this.s[i] != null ){
        this.dFiles[i].flush();
        this.dFiles[i].close();
      }
    }
    this.trajectoryNumber++;
  }
  
  private byte[] get_nextData() {
    int theLength = this.MM.get_int( this.nd , this.data );
    this.nd += 4;
    byte[] m = new byte[ theLength ];
    System.arraycopy(  this.data , this.nd  ,  m , 0 ,  theLength  );
    this.nd += theLength;
    this.tLastMeasurement = this.get_time( m );
    this.nMeasurements++;
    return m;
  }
  
  private long get_time( byte[] m ) {
    return this.MM.get_long( m.length-8 , m );
  }
  
  
}
