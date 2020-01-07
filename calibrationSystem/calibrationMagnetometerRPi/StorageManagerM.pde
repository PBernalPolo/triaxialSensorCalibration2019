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


// class that manages connected serial devices
public class StorageManagerM implements Runnable {
  
  // PRIVATE VARIABLES
  private PApplet thePApplet;
  private boolean running;  // true while the thread is running
  private boolean wait;  // true if we want to pause the thread
  private boolean waiting;  // true if we are waiting for activity
  private boolean storeMeasurements;
  private int NSPM;  // Number of Serial Port Managers in the last update (in the list; not all will be available)
  private int NASPM;  // Number of Available Serial Port Managers in the last update (only those that can be opened; those that do not throw an exception when you try to open them)
  private int Nm;  // number of the measurement (to distinguish between different positions)
  private float temp;  // temperature of the senseHAT
  private SerialPortManager[] SPM;
  private InformationSourceManager ISM;
  private LSM9DS1_AG accelGyro;
  private LSM9DS1_M magnetometer;
  private PrintWriter PW;  // used to store all data in a file
  
  
  // CONSTRUCTORS
  
  public StorageManagerM( PApplet aPApplet ) {
    this.thePApplet = aPApplet;
    this.running = true;
    this.wait = false;
    this.waiting = false;
    this.storeMeasurements = false;
    
    this.temp = 25.0;
    this.NSPM = 0;
    this.NASPM = 0;
    this.ISM = new InformationSourceManager();
    
    this.senseHatSetup();
    
    ( new Thread( this ) ).start();
    this.fastUpdate_serialPortManagers();
  }
  
  
  // PUBLIC METHODS
  
  public void start_storing() {
    String fileName = "/storedData/data_" + String.format( "%04d" , year() ) + String.format( "%02d" , month() ) + String.format( "%02d" , day() ) + String.format( "%02d" , hour() ) + String.format( "%02d" , minute() ) + String.format( "%02d" , second() ) + ".dat";
    this.PW = createWriter( sketchPath() + fileName );
  }
  
  public void stop_storing() {
    if( this.PW != null ){
      this.PW.flush();
      this.PW.close();
    }
  }
  
  public void fastUpdate_serialPortManagers() {
    // first, we check if the number of serial ports has changed
    if( this.NSPM != Serial.list().length ){
      // we wait a bit for the system to update the serial ports
      delay(1000);
      this.update_serialPortManagers();
    }
  }
  
  public synchronized void update_serialPortManagers() {
    // if the number has changed, we redefine the serial ports
    int newNSPM = Serial.list().length;
    SerialPortManager[] auxSPM = new SerialPortManager[newNSPM];
    int newNASPM = 0;
    for(int j=0; j<newNSPM; j++){
      String theName = Serial.list()[j];
      // we check that the serial port is not prohibited (we do not want to mess with the bluetooth)
      if( theName.equals( "/dev/rfcomm0" ) || theName.equals( "/dev/ttyAMA0" ) || theName.equals( "/dev/serial1" ) ) continue;
      // we check if the j-th serial port is already opened
      boolean opened = false;
      for(int i=0; i<this.NASPM; i++){
        if( this.SPM[i].is_thisSerialPort( theName ) ){  // if its name is in the list of available serial ports, then it is opened
          auxSPM[newNASPM] = this.SPM[i];
          newNASPM++;
          opened = true;
          break;
        }
      }
      // if it has not been opened, we try to open it (here is where the objects are created)
      if( !opened ){
        try{
          auxSPM[newNASPM] = new SerialPortManager( this.thePApplet , theName );
          newNASPM++;
        }catch( Exception exc ){
        }
      }
    }
    // then, we close the non-used serial ports
    for(int i=0; i<this.NASPM; i++){
      boolean used = false;
      for(int j=0; j<newNASPM; j++){
        if( this.SPM[i].is_thisSerialPort( auxSPM[j].serialPortName ) ){
          used = true;
          break;
        }
      }
      if( !used ){
        this.SPM[i].stop();
      }
    }
    // finally, we perform the redefinition
    this.SPM = new SerialPortManager[newNASPM];
    for(int i=0; i<newNASPM; i++){
      this.SPM[i] = auxSPM[i];
      System.out.println( this.SPM[i].serialPortName );
    }
    System.out.println();
    this.NASPM = newNASPM;
    this.NSPM = newNSPM;
  }  // end update_serialPorts
  
  public synchronized void run() {
    while( this.running ){
      this.wait = true;
      // we manage the serial ports
      this.manage_serial();
      // and store data
      this.store_data();
      //
      try{
        while( this.wait ){
          this.waiting = true;
          this.wait();
        }
      }catch( Exception exc ){
        System.out.println( "StorageManager: not able to wait." );
      }finally{
        this.waiting = false;
      }
    }
  }
  
  public void notify_activity() {
    this.wait = false;
    if( this.waiting ){
      synchronized( this ){
        this.notify();
      }
    }
  }
  
  void stop(){
    this.running = false;
    this.notify_activity();
    this.stop_storing();
  }
  
  public void set_storeMeasurements( boolean sm ) {
    this.storeMeasurements = sm;
  }
  
  public void change_measurementNumber() {
    this.Nm++;
  }
  
  public float get_temperature() {
    return this.temp;
  }
  
  public String get_informationSources() {
    return this.ISM.get_informationSources();
  }
  
  
  // PRIVATE METHODS
  
  private void manage_serial() {
    for(int i=0; i<this.NASPM; i++){
      while( this.SPM[i].available() > 0 ){
        byte[] data = this.SPM[i].read();
        if(  data != null  &&  this.storeMeasurements  ) this.ISM.add_measurement( data );
      }
    }
  }
  
  private short[] get_senseHatMeasurement() {
    short[] accelData = accelGyro.get_accelData();
    short[] gyroData = accelGyro.get_gyroData();
    short[] magData = magnetometer.get_magnetometerData();
    short temp = accelGyro.get_temperatureData();
    this.temp = temp/16.0 + 25.0;
    return new short[]{ accelData[0] , accelData[1] , accelData[2] , gyroData[0] , gyroData[1] , gyroData[2] , magData[0] , magData[1] , magData[2] , temp };
  }
  
  private void store_data() {
    // while there is data to write,
    while( this.ISM.available() ){
      // we take the next measurement,
      int[] m = this.get_measurement( this.ISM.get_measurement() );
      // and we write it to the file only if it is not null
      if(  m != null  &&  this.storeMeasurements  ){
        this.PW.print( this.Nm );
        for(int i=0; i<m.length; i++){
          this.PW.print( " " + m[i] );
        }
        this.PW.println( /*" " + this.TCB.get_angularVelocity()*/ );
      }
    }  // end while( this.ISM.available() )
    // now we store a senseHat measurement
    if( this.storeMeasurements ){
      short[] m = this.get_senseHatMeasurement();
      this.PW.print( this.Nm + " " + 17 + " " );  // WHO_AM_I
      for(int i=0; i<m.length; i++){
        this.PW.print( m[i] + " " );
      }
      this.PW.println( /*this.TCB.get_angularVelocity()*/ );
    }
  }
  
  private int[] get_measurement( byte[] theBytes ) {
    int[] measurement = null;
    if( theBytes[0] == 30 ){  // 30 is IMU
      switch( theBytes[1] ){
        case 11:
          if( theBytes.length == 22 ){
            measurement = this.get_measurement_1( theBytes );
          }else{
            measurement = this.get_measurement_12345( theBytes );
          }
          break;
        case 12:
          measurement = this.get_measurement_12345( theBytes );
          break;
        case 13:
          measurement = this.get_measurement_12345( theBytes );
          break;
        case 14:
          measurement = this.get_measurement_12345( theBytes );
          break;
        case 15:
          measurement = this.get_measurement_12345( theBytes );
          break;
        case 16:
          measurement = this.get_measurement_6( theBytes );
          break;
        default:
          break;
      }
    }
    return measurement;
  }
  
  // GY-88
  private int[] get_measurement_1( byte[] theBytes ) {
    int[] m = new int[11];
    //theBytes[0];  // INFORMATION_SOURCE_CLASS_ID
    m[0] = theBytes[1];  // WHO_AM_I
    m[1] = ( ( theBytes[3] << 8 ) | ( theBytes[2] & 0xFF ) );  // ax
    m[2] = ( ( theBytes[5] << 8 ) | ( theBytes[4] & 0xFF ) );  // ay
    m[3] = ( ( theBytes[7] << 8 ) | ( theBytes[6] & 0xFF ) );  // az
    m[4] = ( ( theBytes[9] << 8 ) | ( theBytes[8] & 0xFF ) );  // wx
    m[5] = ( ( theBytes[11] << 8 ) | ( theBytes[10] & 0xFF ) );  // wy
    m[6] = ( ( theBytes[13] << 8 ) | ( theBytes[12] & 0xFF ) );  // wz
    m[7] = ( ( theBytes[15] << 8 ) | ( theBytes[14] & 0xFF ) );  // tempAW
    m[8] = ( ( theBytes[17] << 8 ) | ( theBytes[16] & 0xFF ) );  // mx
    m[9] = ( ( theBytes[19] << 8 ) | ( theBytes[18] & 0xFF ) );  // my
    m[10] = ( ( theBytes[21] << 8 ) | ( theBytes[20] & 0xFF ) );  // mz
    //this.temp = measurement[7]/340.0 + 521.0/340.0 + 35.0;  // this is according to the datasheet of the MPU6050
    return m;
  }
  
  // pressure sensor
  private long[] get_measurementP( byte[] theBytes ) {
    long[] m = new long[3];
    //theBytes[0];  // INFORMATION_SOURCE_CLASS_ID
    m[0] = theBytes[1];  // WHO_AM_I
    m[1] = ( ( (theBytes[3] & 0xFF) << 8 ) | ( theBytes[2] & 0xFF ) );  // tempP
    m[2] = ( ( (theBytes[6] & 0xFF) << 16 ) | ( (theBytes[5] & 0xFF) << 8 ) & 0xFF | ( theBytes[4] & 0xFF ) );  // mp
    return m;
  }
  
  // MPU
  private int[] get_measurement_12345( byte[] theBytes ) {
    int[] m = new int[8];
    //theBytes[0];  // INFORMATION_SOURCE_CLASS_ID
    m[0] = theBytes[1];  // WHO_AM_I
    m[1] = ( ( theBytes[3] << 8 ) | ( theBytes[2] & 0xFF ) );  // ax
    m[2] = ( ( theBytes[5] << 8 ) | ( theBytes[4] & 0xFF ) );  // ay
    m[3] = ( ( theBytes[7] << 8 ) | ( theBytes[6] & 0xFF ) );  // az
    m[4] = ( ( theBytes[9] << 8 ) | ( theBytes[8] & 0xFF ) );  // wx
    m[5] = ( ( theBytes[11] << 8 ) | ( theBytes[10] & 0xFF ) );  // wy
    m[6] = ( ( theBytes[13] << 8 ) | ( theBytes[12] & 0xFF ) );  // wz
    m[7] = ( ( theBytes[15] << 8 ) | ( theBytes[14] & 0xFF ) );  // temp
    //this.temp = measurement[7]/340.0 + 521.0/340.0 + 35.0;  // this is according to the datasheet of the MPU6050
    return m;
  }
  
  // Adafruit sensor
  private int[] get_measurement_6( byte[] theBytes ) {
    int[] m = new int[12];
    //theBytes[0];  // INFORMATION_SOURCE_CLASS_ID
    m[0] = theBytes[1];  // WHO_AM_I
    m[1] = ( ( theBytes[3] << 8 ) | ( theBytes[2] & 0xFF ) );  // ax
    m[2] = ( ( theBytes[5] << 8 ) | ( theBytes[4] & 0xFF ) );  // ay
    m[3] = ( ( theBytes[7] << 8 ) | ( theBytes[6] & 0xFF ) );  // az
    m[4] = ( ( theBytes[9] << 8 ) | ( theBytes[8] & 0xFF ) );  // mx
    m[5] = ( ( theBytes[11] << 8 ) | ( theBytes[10] & 0xFF ) );  // my
    m[6] = ( ( theBytes[13] << 8 ) | ( theBytes[12] & 0xFF ) );  // mz
    m[7] = ( ( theBytes[15] << 8 ) | ( theBytes[14] & 0xFF ) );  // tempAM
    m[8] = ( ( theBytes[17] << 8 ) | ( theBytes[16] & 0xFF ) );  // wx
    m[9] = ( ( theBytes[19] << 8 ) | ( theBytes[18] & 0xFF ) );  // wy
    m[10] = ( ( theBytes[21] << 8 ) | ( theBytes[20] & 0xFF ) );  // wz
    m[11] = ( ( theBytes[23] << 8 ) | ( theBytes[22] & 0xFF ) );  // tempG
    return m;
  }
  
  private void senseHatSetup() {
    // ACCELEROMETER AND GYROSCOPE
    // object creation:
    // if you do not know the device direction, simply use
    //  accelGyro = new LSM9DS1_AG();
    // however, if you know the device direction it will be faster (and cleaner if you look at the console) to use
    this.accelGyro = new LSM9DS1_AG( 0x6A );
    
    // some possible operations:
    this.accelGyro.reset();  // to reset the device using software
    this.accelGyro.reboot();  // to reboot the memory of the device
    
    // settings:
    // measurement range settings
    this.accelGyro.set_accelMeasurementRange( 3 );  // ( mode=0 -> 2g;  mode=1 -> 4g;  mode=2 -> 8g;  mode=3 -> 16g )
    this.accelGyro.set_gyroMeasurementRange( 2 );  // ( mode=0 -> 245dps;  mode=1 -> 500dps;  mode=2 -> 2000dps )
    // data rate settings
    this.accelGyro.set_accelDataRate( 4 );  // ( mode=0 -> 0Hz;  mode=1 -> 10Hz;  mode=2 -> 50Hz;  mode=3 -> 119Hz;  mode=4 -> 238Hz;  mode=5 -> 476Hz;  mode=6 -> 952Hz )
    this.accelGyro.set_gyroDataRate( 4 );  // ( mode=0 -> 0Hz;  mode=1 -> 14.9Hz;  mode=2 -> 59.5Hz;  mode=3 -> 119Hz;  mode=4 -> 238Hz;  mode=5 -> 476Hz;  mode=6 -> 952Hz )
    // update operation
    this.accelGyro.set_nonContinuousDataUpdate();  // (this feature prevents the reading of LSB and MSB related to different samples)
    //  accelGyro.set_continuousDataUpdate();  // if you do not mind if you are reading the LSB and the MSB changes
    
    // MAGNETOMETER
    // object creation:
    // if you do not know the device direction, simply use
    //  magnetometer = new LSM9DS1_M();
    // however, if you know the device direction it will be faster (and cleaner if you look at the console) to use
    this.magnetometer = new LSM9DS1_M( 0x1C );
    
    // some possible operations:
    this.magnetometer.reset();  // to reset the device using software
    this.magnetometer.reboot();  // to reboot the memory of the device
    // power-off the device
    this.magnetometer.powerOff();
    // power-on the device
    this.magnetometer.powerOn();
    
    // settings:
    // measurement range settings
    this.magnetometer.set_magnetometerMeasurementRange( 2 );  // ( mode=0 -> 4gauss;  mode=1 -> 8gauss;  mode=2 -> 12gauss;  mode=3 -> 16gauss )
    // data rate settings
    this.magnetometer.set_magnetometerDataRate( 7 );  // ( mode=0 -> 0.625Hz;  mode=1 -> 1.25Hz;  mode=2 -> 2.5Hz;  mode=3 -> 5Hz;  mode=4 -> 10Hz;  mode=5 -> 20Hz;  mode=6 -> 40Hz;  mode=7 -> 80Hz )
    // update operation
    this.magnetometer.set_nonContinuousDataUpdate();  // (this feature prevents the reading of LSB and MSB related to different samples)
    //  magnetometer.set_continuousDataUpdate();  // if you do not mind if you are reading the LSB and the MSB changes
  }
  
}
