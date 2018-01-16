// MPU6050 pin      Arduino pin
// -----------------------------
//     vcc              3.3V
//     gnd              GND
//     sda              A4
//     scl              A5
//     int               2

//must download the library from here ==> https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"        
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//#define DEBUG      //uncomment to show the data in serial monitor
#define SAMPLETIME 500  
#define KP 4.35      //tune properly to get the best result
#define KI 1.10      //tune properly to get the best result(response time)
#define KD 169       //tune properly to get the best result
#define MAX 250

MPU6050 mpu;

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

float error,prevError,totalError;
int output;
unsigned long lastTime = 0;

const int ENA = 3;             //connect L293D EN1 pin
const int ENB = 5;             //connect L293D EN2 pin
const int DIR_A1 = 8;          //connect L293D IN_2 pin
const int DIR_A2 = 9;          //connect L293D IN_1 pin
const int DIR_B1 = 10;         //connect L293D IN_3 pin
const int DIR_B2 = 11;         //connect L293D IN_4 pin

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    #ifdef DEBUG
      Serial.begin(115200);
    #endif
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
    if (devStatus == 0) 
    {
      mpu.setDMPEnabled(true);
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    
    initMotors();
    delay(2500);
}

void loop()
{
  while (!mpuInterrupt && fifoCount < packetSize);
  getYPR();
  calculatePID();
  updateMotors(output);
}

void calculatePID(void)
{
  unsigned long now = micros();
  unsigned long timeChange = now - lastTime;
  if(timeChange >= SAMPLETIME)        //refresh pid calculation every 500ms
  {
    error = ypr[1] - 4;          //try to set the gyro parallel to the actuator of the motor accurately
                                 //i can't set so accurately,so 4 is the right angle for me 
    totalError += KI*error;
    if(totalError > MAX) totalError = MAX;
    else if(totalError < -MAX) totalError = -MAX;
    output =  KP*error + totalError + (KD*(error-prevError));
    if(output > MAX) output = MAX;            //to keep the pwm in range
    else if(output < -MAX) output = -MAX;     //to keep the pwm in range
    
    #ifdef DEBUG
      Serial.println(output);
    #endif
 }
  
  prevError = error;
  lastTime = now;
}

void updateMotors(int signal)
{
  if(signal > 0)
  {
//    PORTB = B00000101;
    digitalWrite(DIR_A1,HIGH);
    digitalWrite(DIR_A2,LOW);
    digitalWrite(DIR_B1,HIGH);
    digitalWrite(DIR_B2,LOW);
  }
  else if(signal < 0)
  {
//    PORTB = B00001010;
    digitalWrite(DIR_A1,LOW);
    digitalWrite(DIR_A2,HIGH);
    digitalWrite(DIR_B1,LOW);
    digitalWrite(DIR_B2,HIGH);
  }
  analogWrite(ENA,abs(signal));
  analogWrite(ENB,abs(signal));
}

void initMotors(void)
{
//  DDRB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3);
//  DDRD |= (1<<DDD4) | (1<<DDD5);
//  PORTB &= (~(1<<PORTB0)) & (~(1<<PORTB1)) & (~(1<<PORTB2)) & (~(1<<PORTB3));
//  PORTD &= (~(1<<PORTD4)) & (~(1<<PORTD5));

  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(DIR_A1,OUTPUT);
  pinMode(DIR_A2,OUTPUT);
  pinMode(DIR_B1,OUTPUT);
  pinMode(DIR_B2,OUTPUT);
  
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  digitalWrite(DIR_A1,LOW);
  digitalWrite(DIR_A2,LOW);
  digitalWrite(DIR_B1,LOW);
  digitalWrite(DIR_B2,LOW);
  
}

void getYPR()
{
    if (!dmpReady) return;
//    while (!mpuInterrupt && fifoCount < packetSize);
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024){
        mpu.resetFIFO();
    } 
    else if (mpuIntStatus & 0x02) 
    {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      ypr[1] = ypr[1] * 180/M_PI;
      
      #ifdef DEBUG
        Serial.print("ypr\t");
        Serial.print(ypr[1]);
        Serial.print("\t");
      #endif
    }
}
