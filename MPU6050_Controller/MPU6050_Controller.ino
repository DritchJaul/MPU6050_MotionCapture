
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68); // <-- use for AD0 high
MPU6050 mpu2(0x69);

#define POT_PIN A3
#define BUT_PIN 7
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


bool dmpReady2 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus2;   // holds actual interrupt status byte from MPU
uint8_t devStatus2;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer2[64]; // FIFO storage buffer


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties


    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mpu2.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 1 connection successful") : F("MPU6050 1 connection failed"));
    Serial.println(mpu2.testConnection() ? F("MPU6050 2 connection successful") : F("MPU6050 2 connection failed"));
    
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    devStatus2 = mpu2.dmpInitialize();
    
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    
    mpu2.setXGyroOffset(220);
    mpu2.setYGyroOffset(76);
    mpu2.setZGyroOffset(-85);
    mpu2.setZAccelOffset(1788); // 1688 factory default for my test chip


    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
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

    if (devStatus2 == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu2.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        mpuIntStatus2 = mpu2.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady2 = true;
        packetSize2 = mpu2.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP 2 Initialization failed (code "));
        Serial.print(devStatus2);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(POT_PIN, INPUT);
    pinMode(BUT_PIN, INPUT);

    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    if (!dmpReady2) return;

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    fifoCount2 = mpu2.getFIFOCount();
    
    if (fifoCount >= 1024) {
        mpu.resetFIFO();
        mpu2.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        while (fifoCount2 < packetSize2) fifoCount2 = mpu2.getFIFOCount();

        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu2.getFIFOBytes(fifoBuffer2, packetSize2);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        fifoCount2 -= packetSize2;

        // Pot & But = 48
        // Upper MPU = 49
        // Lower MPU = 50

        sendQuat((byte)49, fifoBuffer);
        sendQuat((byte)50, fifoBuffer2);
        sendPotBut();
        Serial.flush();

    }
    
}

void sendPotBut(){
  short pot = analogRead(POT_PIN);
  
  byte low = (byte)(pot & 0x00FF);
  byte high = (byte)((pot >> 8) & 0x00FF);

  byte button = (byte)(digitalRead(BUT_PIN));

  high |= (byte)(button << 2);

  byte bytes[6] = { 48, (high >> 4) | 64, (high & 15) | 64 , (low >> 4) | 64, (low & 15) | 64 ,  10};
  
  Serial.write(bytes,6);
}



void sendQuat(byte header, uint8_t* fifo){
  mpu.dmpGetQuaternion(&q, fifo);
  float quat[4] = {q.w, q.x, q.y, q.z};
  byte bytes[34];
  bytes[ 0] = header;
  for (int i = 0; i < 4; i++){
    byte * b = (byte *) &quat[i];
    for (int j = 0; j < 4; j++){
      int k = (((i << 2) + j) << 1) + 1;
      bytes[k]     = (b[j] & 15) | 64;
      bytes[k + 1] = (b[j] >> 4) | 64;
    }
  }
  bytes[33] = 10;
  Serial.write(bytes,34);
}

