#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS.h>
#include <DS3231.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_BMP085.h>
#include <math.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
// #define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//define the pins used by the transceiver module
#define ss 53
#define rst 11
#define dio0 3
// -1.091182,Longitude: 37.023082

//long   lat,lon; // create variable for latitude and longitude object



// ===================================================================================================================
                                          // MPU VARIABLES
// ====================================================================================================================

// #define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
// #define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
// #define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 7 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

# define DEVICE_ID "KDMD_001"

bool blinkState = false;
int ledStatus = 6;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

long prevTimeTask1 = millis();
long intervalTask1 = 700;

long prevTimeTask2 = millis();
long intervalTask2 = 1000;

long prevTimeTask3 = millis();
long intervalTask3 = 1000;
long currentTime;
// ====================================================================================================================


float lat=-1.091182;
float lon=37.023082;


Adafruit_BMP085 bmp;

float T0 = 0;
float P0 = 0;
float t0 = 0;
float p0 = 0;
float ph = 0;
float h = 0;
float alt;
int bmpCounter = 0;

 float tempVal;
 float pressureVal;

float y;
float p;
float r;
uint32_t unixtimestamp =0;
String t_and_date;

TinyGPS gps; // create gps object
MPU6050 mpu(0x69); // create mpu object
DS3231  rtc(20, 21); // initialize RTC

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void bmpSetUp(){
  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}  
  }
    
  for(int i=0;i<500;i++){
    p0 = p0 + bmp.readPressure();
    t0 = t0 + bmp.readTemperature();  
  }
  T0 = (t0/500)-273.15;
  P0 = (p0/500);
}

void setup(){
Serial.begin(115200); // connect serial
pinMode(ledStatus, OUTPUT); 
digitalWrite(ledStatus, HIGH);
Serial.println("The GPS Received Signal:");
// Serial1.begin(9600); // connect gps sensor
LoRa.begin(866E6);
delay(5000);
// rtc.begin(); 
mpuSetUp();
loraSetup();
bmpSetUp();
digitalWrite(ledStatus, LOW);
digitalWrite(ledStatus, HIGH);
digitalWrite(ledStatus, LOW);
}
 
void loop(){
  
  currentTime = millis();
 
  getMpuData();
  
  getBmpValues();
 
  if(currentTime - prevTimeTask1 >= intervalTask1){
  loraSendData(); 
  prevTimeTask1 = currentTime;  
  } 

if(currentTime - prevTimeTask2 >= intervalTask2){
  getGPSInfo(); 
  prevTimeTask2 = currentTime;  
  }   
  delay(100);
  Serial.flush();
} 

void getGPSInfo(){
while(Serial1.available()){ // check for gps data
    if(gps.encode(Serial1.read()))// encode gps data
    { 
    gps.f_get_position(&lat,&lon); // get latitude and longitude
    alt=gps.f_altitude();    

    Serial.print("Position: ");
    
    //Latitude
    Serial.print("Latitude: ");
    Serial.print(lat,6);
    
    Serial.print(",");
    
    //Longitude
    Serial.print("Longitude: ");
    Serial.println(lon,6); 
    
   }
  }  
}

void loraSetup(){

Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(866E6)) {
    Serial.println(".");
    delay(500);
  }
  // LoRa.begin(866E6);
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF6);
  Serial.println("LoRa Initializing OK!");  
}

void loraSendData(){
  digitalWrite(ledStatus, HIGH);
  Serial.print("Sending packet: ");
  // Serial.println(counter);

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print(String(lat,6));
  LoRa.print(",");
  LoRa.print(String(lon,6));
  LoRa.print(",");
  LoRa.print(alt);
  LoRa.print(",");
  LoRa.print(y);
  LoRa.print(",");
  LoRa.print(p);
  LoRa.print(",");
  LoRa.print(r);
  LoRa.print(",");
  LoRa.print(h);
  LoRa.print(",");
  LoRa.print(DEVICE_ID);
  LoRa.endPacket();
  Serial.println("sent!");  
  delay(100);  
  digitalWrite(ledStatus, LOW);
}

void mpuSetUp(){

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(200000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(200, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);  
}

void getMpuData(){
 // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);

        #endif

        // #ifdef OUTPUT_READABLE_EULER
        //     // display Euler angles in degrees
        //     mpu.dmpGetQuaternion(&q, fifoBuffer);
        //     mpu.dmpGetEuler(euler, &q);
            // Serial.print("euler\t");
            // Serial.print(euler[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(euler[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(euler[2] * 180/M_PI);
        // #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            y = ypr[0] * 180/M_PI;
            p = ypr[1] * 180/M_PI;
            r = ypr[2] * 180/M_PI;
            Serial.print("ypr\t");
            Serial.print(y);
            Serial.print("\t");
            Serial.print(p);
            Serial.print("\t");
            Serial.println(r);
        #endif

      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
           

        
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
   }  
}

void getTimeAndDate(){
  // t_and_date = String(rtc.getDOWStr())+" -- "+String(rtc.getDateStr())+"  "+String(rtc.getTimeStr());
  // Wait one second before repeating :)
  // Serial.println(t_and_date);
  unixtimestamp = rtc.getUnixTime(rtc.getTime());
  Serial.println(unixtimestamp);
}

void getBmpValues(){
  alt = bmp.readAltitude();
  tempVal= tempVal+ bmp.readTemperature();
  pressureVal = pressureVal+ bmp.readPressure();
  if( bmpCounter >=300){
      tempVal = tempVal/300;
      pressureVal  = pressureVal/300;
      // Calculate altitude assuming 'standard' barometric
      // pressure of 1013.25 millibar = 101325 Pascal
      Serial.print("Altitude = ");
      Serial.print(alt);
      Serial.println(" meters");

      Serial.print("Pressure at sealevel (calculated) = ");
      Serial.print(bmp.readSealevelPressure());
      Serial.println(" Pa");
      Serial.print("Height Above Ground = ");
      
      h = ((98.57*T0*log(P0/pressureVal))*0.3048)-13;
      Serial.print(h);
      Serial.println(" meters");
      Serial.println(); 
      bmpCounter = 0;  
      }
      bmpCounter++;
}


