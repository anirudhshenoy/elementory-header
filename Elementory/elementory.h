//#include <dht.h>
//#include <VirtualWire.h>
#include <Wire.h>
//#include <Adafruit_BMP085.h>
#include <LiquidCrystal.h>
//#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"


//Adafruit_BMP085 bmp;

//dht DHT;

//MPU6050 mpu;


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
/*bool dmpReady = false;  // set true if DMP init was successful
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



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



void calculateAngles(){
   // reset interrupt flag and get INT_STATUS byte
   mpuInterrupt = false;
   mpuIntStatus = mpu.getIntStatus();

   // get current FIFO count
   fifoCount = mpu.getFIFOCount();

   // check for overflow (this should never happen unless our code is too inefficient)
   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
       // reset so we can continue cleanly
       mpu.resetFIFO();
       Serial.println(F("FIFO overflow!"));

   // otherwise, check for DMP data ready interrupt (this should happen frequently)
   } else if (mpuIntStatus & 0x02) {
       // wait for correct available data length, should be a VERY short wait
       while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

       // read a packet from FIFO
       mpu.getFIFOBytes(fifoBuffer, packetSize);

       // track FIFO count here in case there is > 1 packet available
       // (this lets us immediately read more without waiting for an interrupt)
       fifoCount -= packetSize;


       mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        for ( int i =0; i <3; i++){
          ypr[i]=ypr[i]* 180/M_PI;
        }
  }
}

float getRollAngle(){
  return ypr[2];
}

float getPitchAngle(){
  return ypr[1];
}

float getYawAngle(){
  return ypr[0];
}


void MPU_start(){
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

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
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
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
}
*/


const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
/*
float readHumidity(int humidity_pin){
  DHT.read11(humidity_pin);
  return DHT.humidity;
}
*/

/*void transmit_data (String data){

  char *msg = data.c_str();
  vw_send(msg, strlen(msg));
  vw_wait_tx(); // Wait until the whole message is gone
}

void tx_begin(int tx_pin){
  vw_set_tx_pin(tx_pin);
  vw_set_ptt_inverted(true);
  vw_setup(4500);
}
*/
float readTemperature(int temperature_pin) {
  int totalTemp=0;
  for(int i=0 ; i<100; i++) {
    totalTemp+=analogRead(temperature_pin);
  }
  return (totalTemp/100.00)*(500.00/1023.00);
}

/*float readTemperature() {
  int temp=0;
  temp=bmp.readTemperature();
  return temp;
}

float readPressure() {
  int pres = 0;
  pres = bmp.readPressure();
  return pres;
} */

double calculateDewPoint(float humidity, float temperature){
  double gamma = log(humidity / 100.00) + ((17.62 * temperature) / (243.5 + temperature));
  double dp = 243.5 * gamma / (17.62 - gamma);
  return dp;
}


int readCO2 (int co2pin){

int co2_0 =55  ;
int co2_now[10];                               //int array for co2 readings
int co2_raw = 0;                               //int for raw value of co2
int co2_comp = 0;                              //int for compensated co2
int co2_ppm = 0;                               //int for calculated ppm
int co2_avg = 0;                               //int for averaging

for (int x = 0; x < 10; x++)                    //samplpe co2 10x over 0.5 seconds
{
  co2_now[x] = analogRead(co2pin);
  delay(50);
}

for (int x = 0; x < 10; x++)
{ //add samples together
  co2_avg = co2_avg + co2_now[x];

}

co2_raw = co2_avg / 10;                         //divide samples by 10
co2_comp = co2_raw - co2_0;                     //get compensated value

return co2_comp;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
