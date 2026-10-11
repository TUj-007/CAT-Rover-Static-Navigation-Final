#include <Arduino.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <math.h>
#include "HardwareConfig.h"
#include "CommandParser.h"
#include "DirectionSafety.h"

void earlyWatchdogOff() __attribute__((naked,section(".init3")));
void earlyWatchdogOff(){MCUSR=0;wdt_disable();}
volatile uint32_t ticks[4]={};
uint32_t previous[4]={},stallSince[4]={};
float integral[4]={}; int applied[4]={}; DirectionSafety direction[4];
uint8_t mode=0,fault=0; uint32_t epoch=1,lastControl=0,lastTelemetry=0;
CommandGate gate;
char rx[100],tx[180]; uint8_t rxLen=0; bool overflow=false;
uint16_t txLen=0,txPos=0; bool queryPending=false;

void countWheel(uint8_t i){ticks[i]+=(uint32_t)(int32_t)((digitalRead(ENC_B[i])?1:-1)*ENC_SIGN[i]);}
void enc0(){countWheel(0);} void enc1(){countWheel(1);}
void enc2(){countWheel(2);} void enc3(){countWheel(3);}

void rawMotor(uint8_t i,int value){
  value=constrain(value,-MAX_PWM,MAX_PWM)*MOTOR_SIGN[i];
  if(i==0||i==2){ // PHYSICAL FRONT: MD10 PWM/DIR
    uint8_t pwm=i==0?FL_PWM:FR_PWM,dir=i==0?FL_DIR:FR_DIR;
    analogWrite(pwm,0);digitalWrite(dir,value>=0?HIGH:LOW);analogWrite(pwm,abs(value));
  }else{ // PHYSICAL REAR: BTS7960
    uint8_t rp=i==1?RL_RPWM:RR_RPWM,lp=i==1?RL_LPWM:RR_LPWM;
    uint8_t re=i==1?RL_REN:RR_REN,le=i==1?RL_LEN:RR_LEN;
    analogWrite(rp,0);analogWrite(lp,0);
    digitalWrite(re,value!=0);digitalWrite(le,value!=0);
    if(value>0)analogWrite(rp,value);if(value<0)analogWrite(lp,-value);
  }
}
void stopMotors(){for(uint8_t i=0;i<4;i++){rawMotor(i,0);applied[i]=0;integral[i]=0;stallSince[i]=0;}}
void resetSession(uint8_t reason){stopMotors();gate.reset();epoch++;mode=0;fault=reason;}
void handleLine(char* line,uint32_t now){
  char copy[100];strncpy(copy,line,sizeof(copy));copy[sizeof(copy)-1]=0;
  if(wireBody(copy)&&!strcmp(copy,"Q")){queryPending=true;return;}
  if(!HARDWARE_CONFIRMED||!CALIBRATION_CONFIRMED||(fault&17))return;
  if(gate.accept(line,now,epoch)){
    mode=2;fault=0;
    if(!gate.left&&!gate.right)stopMotors();
  }
}
void pollUSB(uint32_t now){
  for(uint8_t budget=0;budget<64&&Serial.available();budget++){
    char c=Serial.read();
    if(c=='\n'){if(!overflow){rx[rxLen]=0;handleLine(rx,now);}rxLen=0;overflow=false;}
    else if(c!='\r'){if(rxLen<sizeof(rx)-1&&!overflow)rx[rxLen++]=c;else overflow=true;}
  }
}
void control(uint32_t now){
  uint32_t elapsed=now-lastControl;if(elapsed<20)return;lastControl=now;
  uint32_t counts[4];ATOMIC_BLOCK(ATOMIC_RESTORESTATE){for(uint8_t i=0;i<4;i++)counts[i]=ticks[i];}
  if(elapsed>100){for(uint8_t i=0;i<4;i++)previous[i]=counts[i];resetSession(8);return;}
  float dt=elapsed/1000.f;
  for(uint8_t i=0;i<4;i++){
    int32_t delta=(int32_t)(counts[i]-previous[i]);previous[i]=counts[i];
    float speed=delta*(PI*WHEEL_DIAMETER_M[i]/COUNTS_PER_REV[i])/dt;
    float desired=constrain((i<2?gate.left:gate.right)/1000.f,-MAX_WHEEL_MPS,MAX_WHEEL_MPS);
    if(mode!=2||!gate.zeroSeen||fabs(desired)<.005f){rawMotor(i,0);applied[i]=0;integral[i]=0;stallSince[i]=0;continue;}
    if(fabs(speed)>1.f){resetSession(16);return;}
    int requestedSign=(desired>0)-(desired<0);
    int permission=direction[i].check(requestedSign,speed,now,REVERSAL_PAUSE_MS);
    if(permission<0){resetSession(16);return;}
    if(permission>0){
      // Brake the previous direction before applying the next. A coasting wheel
      // during reversal is not itself an encoder polarity fault.
      rawMotor(i,0);applied[i]=0;integral[i]=0;stallSince[i]=0;continue;
    }
    float error=desired-speed;
    integral[i]=constrain(integral[i]+error*dt,-.5f,.5f);
    float effort=(desired>0?MIN_PWM[i]:-MIN_PWM[i])+KF[i]*desired+KP[i]*error+KI[i]*integral[i];
    int output=desired>0?constrain(effort,0.f,(float)MAX_PWM):constrain(effort,-(float)MAX_PWM,0.f);
    int sign=(output>0)-(output<0);
    if(output>0)output=min(output,max(0,applied[i])+PWM_RISE_PER_20MS);
    if(output<0)output=max(output,min(0,applied[i])-PWM_RISE_PER_20MS);
    if(abs(output)>100&&fabs(speed)<.005f){
      if(!stallSince[i])stallSince[i]=now;
      if(now-stallSince[i]>1200){resetSession(16);return;}
    }else stallSince[i]=0;
    applied[i]=output;rawMotor(i,output);
  }
}
void emitTelemetry(uint32_t now){
  if(txPos<txLen){int n=min((int)(txLen-txPos),Serial.availableForWrite());
    if(n>0){Serial.write((uint8_t*)tx+txPos,n);txPos+=n;}return;}
  if(queryPending){strcpy(tx,"V,2,1,1");queryPending=false;}
  else{
    if(now-lastTelemetry<50)return;lastTelemetry=now;
    uint32_t c[4];ATOMIC_BLOCK(ATOMIC_RESTORESTATE){for(uint8_t i=0;i<4;i++)c[i]=ticks[i];}
    snprintf(tx,sizeof(tx),"T,%lu,%lu,%u,%u,%lu,%lu,%lu,%lu,1500,1500,2000,2000",
      (unsigned long)now,(unsigned long)epoch,mode,fault,(unsigned long)c[0],(unsigned long)c[1],(unsigned long)c[2],(unsigned long)c[3]);
  }
  uint16_t crc=wireCRC(tx),len=strlen(tx);snprintf(tx+len,sizeof(tx)-len,"*%04X\n",crc);txLen=strlen(tx);txPos=0;
}
void setup(){
  const uint8_t pins[]={FL_PWM,FL_DIR,FR_PWM,FR_DIR,RL_RPWM,RL_LPWM,RR_RPWM,RR_LPWM,RL_REN,RL_LEN,RR_REN,RR_LEN};
  for(uint8_t p:pins){digitalWrite(p,LOW);pinMode(p,OUTPUT);}stopMotors();
  pinMode(ESTOP_PIN,INPUT_PULLUP);
  for(uint8_t i=0;i<4;i++){pinMode(ENC_A[i],INPUT_PULLUP);pinMode(ENC_B[i],INPUT_PULLUP);}
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]),enc0,RISING);attachInterrupt(digitalPinToInterrupt(ENC_A[1]),enc1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]),enc2,RISING);attachInterrupt(digitalPinToInterrupt(ENC_A[3]),enc3,RISING);
  Serial.begin(115200);lastControl=millis();wdt_enable(WDTO_250MS);
}
void loop(){
  wdt_reset();uint32_t now=millis();
  if(ESTOP_REQUIRED&&digitalRead(ESTOP_PIN)!=LOW){if(fault!=1)resetSession(1);stopMotors();}
  else if(fault==1){resetSession(0);}
  if(!HARDWARE_CONFIRMED||!CALIBRATION_CONFIRMED){mode=0;fault=4;stopMotors();}
  if(gate.hasSequence&&now-gate.acceptedAt>COMMAND_TIMEOUT_MS)resetSession(8);
  pollUSB(now);control(now);emitTelemetry(now);
}
