/*
 * Dronix.kr
 */

#ifndef Project D
#define Project D

#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wireless_PID_Tuning.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#define DEBUG


/*  Arduino Pin configuration
 *  
 */

#define ESC_A 9
#define ESC_B 6
#define ESC_C 5
#define ESC_D 3

#define RC_1 13
#define RC_2 12 
#define RC_3 11
#define RC_4 10
#define RC_5 8
#define RC_PWR A0


/* ESC configuration
 *
 */

#define ESC_MIN 700
#define ESC_MAX 2000
#define ESC_TAKEOFF_OFFSET 760
#define ESC_ARM_DELAY 5000

/* RC configuration
 * 
 */

#define RC_HIGH_CH1 1000
#define RC_LOW_CH1 2000
#define RC_HIGH_CH2 1000
#define RC_LOW_CH2 2000
#define RC_HIGH_CH3 1000
#define RC_LOW_CH3 2000
#define RC_HIGH_CH4 1000
#define RC_LOW_CH4 2000
#define RC_HIGH_CH5 1000
#define RC_LOW_CH5 2000
#define RC_ROUNDING_BASE 50

/*  PID configuration
 *  
 */

float PITCH_P_VAL = 0;
float PITCH_I_VAL = 0;
float PITCH_D_VAL = 0;
float* addr_PP = &PITCH_P_VAL;
float* addr_PI = &PITCH_I_VAL;
float* addr_PD = &PITCH_D_VAL;

float ROLL_P_VAL = 0;
float ROLL_I_VAL = 0;
float ROLL_D_VAL = 0;
float* addr_RP = &ROLL_P_VAL;
float* addr_RI = &ROLL_I_VAL;
float* addr_RD = &ROLL_D_VAL;

float YAW_P_VAL = 0;
float YAW_I_VAL = 0;
float YAW_D_VAL = 0;
float* addr_YP = &YAW_P_VAL;
float* addr_YI = &YAW_I_VAL;
float* addr_YD = &YAW_D_VAL;

/* Flight parameters
 *
 */

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 40
#define PID_ROLL_INFLUENCE 40
#define PID_YAW_INFLUENCE 10


/*  MPU variables
 *
 */

MPU6050 mpu;                           // mpu interface object


uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};

volatile bool mpuInterrupt = false;    //interrupt flag

/* Interrupt lock
 *
 */
 
boolean interruptLock = false;

/*  RC variables
 *
 */

float ch1, ch2, ch3, ch4, ch5;         // RC channel inputs

unsigned long rcLastChange1 = micros();
unsigned long rcLastChange2 = micros();
unsigned long rcLastChange3 = micros();
unsigned long rcLastChange4 = micros();
unsigned long rcLastChange5 = micros();

/*  Motor controll variables
 *
 */

int velocity;                          // global velocity

float bal_ac, bal_bd;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd

int va, vb, vc, vd;                    //velocities
int v_ac, v_bd;                        // velocity of axes

Servo a,b,c,d;




/*  Filter variables
 *
 */
 
float ch1Last, ch2Last, ch4Last, velocityLast;

/*  Setup function
 *
 */

void setup(){
  
  initRC();                            // Self explaining
  initMPU();
  initESCs();
  initBalancing();
  initRegulators();
  
  #ifdef DEBUG                        // Device tests go here
  
  Serial.begin(9600);                 // Serial only necessary if in DEBUG mode
  Serial.flush();
  
  #endif
}

/* loop function
 *
 */

void loop(){
  
  while(!mpuInterrupt && fifoCount < packetSize){
     
    /* Do nothing while MPU is not working
     * This should be a VERY short period
     */
  }
  
  if(Serial.available()) WirelessPIDtuning(); // Execute just when PID gain updated
  getYPR();                          
  computePID();
  calculateVelocities();
  updateMotors();

}


/* PID Tuning fucntion
 *  
 * update PID gains via Xbee Pro1
 * 
 * Warning! : Use this function just as you equip all requirements
 * please refer my blog, http://dronix.kr/
 */

 void WirelessPIDtuning(){
 
 int cnt = 0;
 int i = 1;

 char message[10] = {0,0,0,0,0,0,0,0,0,0};
 
 char chr_pp_gain[7] = {0,0,0,0,0,0,0};
 char chr_pi_gain[7] = {0,0,0,0,0,0,0};
 char chr_pd_gain[7] = {0,0,0,0,0,0,0};

 char chr_rp_gain[7] = {0,0,0,0,0,0,0};
 char chr_ri_gain[7] = {0,0,0,0,0,0,0};
 char chr_rd_gain[7] = {0,0,0,0,0,0,0};

 char chr_yp_gain[7] = {0,0,0,0,0,0,0};
 char chr_yi_gain[7] = {0,0,0,0,0,0,0};
 char chr_yd_gain[7] = {0,0,0,0,0,0,0};

 
while(cnt < 10){
    if(Serial.available()) {
      message[cnt] = Serial.read();
      //Serial.print(message[cnt]);
      if(message[cnt] == 'e') break;
      cnt = cnt + 1;

      
    }
    }

if(message[0] == 'P'){
  if(message[1] == 'P'){
    while(1){
      if(message[i+1] != 'e') {
        chr_pp_gain[i-1] = message[i+1];
        i = i+1;
     }
        else break;
      }
  float pp_gain = atof(chr_pp_gain);
  *addr_PP = pp_gain;

  Serial.print("PITCH P GAIN Updated! : ");
  Serial.println(pp_gain, 5);
}

else if(message[1] == 'I'){
  while(1){
    if(message[i+1] != 'e') {
      chr_pi_gain[i-1] = message[i+1];
      i = i+1;
    }
      else break;
  }
  float pi_gain = atof(chr_pi_gain);
  *addr_PI = pi_gain;

  Serial.print("PITCH I GAIN Updated! : ");
  Serial.println(pi_gain, 5);
}

else if(message[1] == 'D'){
  while(1){
    if(message[i+1] != 'e') {
      chr_pd_gain[i-1] = message[i+1];
      i = i+1;
    }
       else break;
  }
  float pd_gain = atof(chr_pd_gain);
  *addr_PD = pd_gain;

  Serial.print("PITCH D GAIN Updated! : ");
  Serial.println(pd_gain, 5);
}
}

else if(message[0] == 'R'){
  if(message[1] == 'P'){
    while(1){
      if(message[i+1] != 'e') {
        chr_rp_gain[i-1] = message[i+1];
        i = i+1;
     }
        else break;
      }
  float rp_gain = atof(chr_rp_gain);
  *addr_RP = rp_gain;

  Serial.print("ROLL P GAIN Updated! : ");
  Serial.println(rp_gain, 5);
}

else if(message[1] == 'I'){
  while(1){
    if(message[i+1] != 'e') {
      chr_ri_gain[i-1] = message[i+1];
      i = i+1;
    }
      else break;
  }
  float ri_gain = atof(chr_ri_gain);
  *addr_RI = ri_gain;

  Serial.print("ROLL I GAIN Updated! : ");
  Serial.println(ri_gain, 5);
}

else if(message[1] == 'D'){
  while(1){
    if(message[i+1] != 'e') {
      chr_rd_gain[i-1] = message[i+1];
      i = i+1;
    }
       else break;
  }
  float rd_gain = atof(chr_rd_gain);
  *addr_RD = rd_gain;

  Serial.print("ROLL D GAIN Updated! : ");
  Serial.println(rd_gain, 5);
}
}

else if(message[0] == 'Y'){
  if(message[1] == 'P'){
    while(1){
      if(message[i+1] != 'e') {
        chr_yp_gain[i-1] = message[i+1];
        i = i+1;
     }
        else break;
      }
  float yp_gain = atof(chr_yp_gain);
  *addr_YP = yp_gain;

  Serial.print("YAW P GAIN Updated! : ");
  Serial.println(yp_gain, 5);
}

else if(message[1] == 'I'){
  while(1){
    if(message[i+1] != 'e') {
      chr_yi_gain[i-1] = message[i+1];
      i = i+1;
    }
      else break;
  }
  float yi_gain = atof(chr_yi_gain);
  *addr_YI = yi_gain;

  Serial.print("YAW I GAIN Updated! : ");
  Serial.println(yi_gain, 5);
}

else if(message[1] == 'D'){
  while(1){
    if(message[i+1] != 'e') {
      chr_yd_gain[i-1] = message[i+1];
      i = i+1;
    }
       else break;
  }
  float yd_gain = atof(chr_yd_gain);
  *addr_YD = yd_gain;

  Serial.print("YAW D GAIN Updated! : ");
  Serial.println(yd_gain, 5);
}
}
 }

 /*  PID variables
 *
 */

PID pitchReg(&ypr[1], &bal_bd, &ch2, &PITCH_P_VAL, &PITCH_I_VAL, &PITCH_D_VAL, DIRECT);
PID rollReg(&ypr[2], &bal_ac, &ch1, &ROLL_P_VAL, &ROLL_I_VAL, &ROLL_D_VAL, DIRECT);
PID yawReg(&ypr[0], &bal_axes, &ch4, &YAW_P_VAL, &YAW_I_VAL, &YAW_D_VAL, DIRECT);
 

/*  computePID function
 *
 *  formats data for use with PIDLib
 *  and computes PID output
 */

void computePID(){

  acquireLock();
  
  ch1 = floor(ch1/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch2 = floor(ch2/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch4 = floor(ch4/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;

  ch2 = map(ch2, RC_LOW_CH2, RC_HIGH_CH2, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW_CH4, RC_HIGH_CH4, YAW_MIN, YAW_MAX);
  
  if((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;
  
  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;
  
  ypr[0] = ypr[0] * 180/M_PI;
  ypr[1] = ypr[1] * 180/M_PI;
  ypr[2] = ypr[2] * 180/M_PI;
  
  if(abs(ypr[0]-yprLast[0])>30) ypr[0] = yprLast[0];
  if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
  if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];
  
  yprLast[0] = ypr[0];
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];

  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();
  
  releaseLock();

}

/*  getYPR function
 *
 *  gets data from MPU and
 *  computes pitch, roll, yaw on the MPU's DMP
 */

void getYPR(){
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount >= 1024){ 
      
      mpu.resetFIFO(); 
    
    }else if(mpuIntStatus & 0x02){
    
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      fifoCount -= packetSize;
    
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    }

}

/*  calculateVelocities function
 *  
 *  calculates the velocities of every motor
 *  using the PID output
 */

void calculateVelocities(){

  acquireLock();

  ch3 = floor(ch3/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  velocity = map(ch3, RC_LOW_CH3, RC_HIGH_CH3, ESC_MIN, ESC_MAX);
  
  releaseLock();

  if((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;
  
  velocityLast = velocity;
  
  v_ac = (abs(-100+bal_axes)/100)*velocity;
  v_bd = ((100+bal_axes)/100)*velocity;
  
  va = ((100+bal_ac)/100)*v_ac;
  vb = ((100+bal_bd)/100)*v_bd;
  
  vc = (abs((-100+bal_ac)/100))*v_ac;
  vd = (abs((-100+bal_bd)/100))*v_bd;
  /*
  Serial.print("ROLL_P_VAL :");
  Serial.print(*addr_RP, 5);
  Serial.print("va :");
  Serial.print(va);
  Serial.print("vc :");
  Serial.println(vc);
  */
  if(velocity < ESC_TAKEOFF_OFFSET){
  
    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;
  
  }
  
}

inline void updateMotors(){

  a.writeMicroseconds(va);
  c.writeMicroseconds(vc);
  b.writeMicroseconds(vb);
  d.writeMicroseconds(vd);

}

inline void arm(){

  a.writeMicroseconds(ESC_MIN);
  b.writeMicroseconds(ESC_MIN);
  c.writeMicroseconds(ESC_MIN);
  d.writeMicroseconds(ESC_MIN);
  
  delay(ESC_ARM_DELAY);

}

inline void dmpDataReady() {
    mpuInterrupt = true;
}

inline void initRC(){
  pinMode(RC_PWR, OUTPUT);
  digitalWrite(RC_PWR, HIGH);
  
  // FIVE FUCKING INTERRUPTS !!!
  PCintPort::attachInterrupt(RC_1, rcInterrupt1, CHANGE);
  PCintPort::attachInterrupt(RC_2, rcInterrupt2, CHANGE);
  PCintPort::attachInterrupt(RC_3, rcInterrupt3, CHANGE);
  PCintPort::attachInterrupt(RC_4, rcInterrupt4, CHANGE);
  PCintPort::attachInterrupt(RC_5, rcInterrupt5, CHANGE);
  
}

void initMPU(){
  
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  }
}

inline void initESCs(){

  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);
  
  delay(100);
  
  arm();

}

inline void initBalancing(){

  bal_axes = 0;
  bal_ac = 0;
  bal_bd = 0;

}

inline void initRegulators(){

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  
  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

}

inline void rcInterrupt1(){
   if(!interruptLock) ch1 = micros() - rcLastChange1;
   rcLastChange1 = micros(); 
}

inline void rcInterrupt2(){
  if(!interruptLock) ch2 = micros() - rcLastChange2;
  rcLastChange2 = micros();
}

inline void rcInterrupt3(){
  if(!interruptLock) ch3 = micros() - rcLastChange3;
  rcLastChange3 = micros();
}

inline void rcInterrupt4(){
  if(!interruptLock) ch4 = micros() - rcLastChange4;
  rcLastChange4 = micros();
}

inline void rcInterrupt5(){
  if(!interruptLock) ch5 = micros() - rcLastChange5;
  rcLastChange5 = micros();
}

inline void acquireLock(){
  interruptLock = true; 
}

inline void releaseLock(){
  interruptLock = false;
}

#endif


