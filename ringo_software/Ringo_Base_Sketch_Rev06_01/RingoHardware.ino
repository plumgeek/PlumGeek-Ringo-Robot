/*

Ringo Robot:  RingoHardware  Rev06.01  12/2015

Significant portions of this code written by
Dustin Soodak for Plum Geek LLC. Some portions
contributed by Kevin King.
Portions from other open source projects where noted.
This code is licensed under:
Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
https://creativecommons.org/licenses/by-sa/2.0/

Visit http://www.plumgeek.com for Ringo information.
Visit http://www.arduino.cc to learn about the Arduino.

*/

#include "RingoHardware.h"
#include "Navigation.h"
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

// ***************************************************
// Ringo Hardware
// ***************************************************


void HardwareBegin(void){//Ver. 1.0, Dustin Soodak
  I2CBegin();                               // startup I2C
  GyroWriteRegister(GYR_CTRL_REG3,0x10);    // turn off gyro open drain (drains current via int line from accel if not set)
  GyroWriteRegister(GYR_CTRL_REG1,0x0f);    // place gyro in power down mode (saves 6mA of current)
                                            // call NavigationBegin(); to wake up and configure Gyro and Accelerometer  
  MotorsBegin();
  pinMode(Edge_Lights,OUTPUT);
  digitalWrite(Edge_Lights,0);
  pinMode(Source_Select,OUTPUT);
  digitalWrite(Source_Select,0);
  
  pinMode(Light_Bus_BTN1,INPUT_PULLUP);
  Serial.begin(SERIAL_SPEED);//Serial.begin(9600);
  
  pinMode(_38kHz_Rx,INPUT_PULLUP);
  
  pinMode(IR_Send,OUTPUT);
  digitalWrite(IR_Send,0);
  pinMode(IR_Enable_Front,OUTPUT);
  digitalWrite(IR_Enable_Front,1);
  pinMode(IR_Enable_RearLeft,OUTPUT);
  digitalWrite(IR_Enable_RearLeft,1);
  pinMode(IR_Enable_RearRight,OUTPUT);
  digitalWrite(IR_Enable_RearRight,1);
  GetGyroCalibrationMultiplier();  
}


char Mode_ButtonOrPixels=99;
void SwitchButtonToPixels(void){//Ver. 1.0, Dustin Soodak
  //if(!(Mode_ButtonOrPixels==1)){
    pinMode(Light_Bus_BTN1,OUTPUT);
    digitalWrite(Light_Bus_BTN1, LOW);
    //pixels.begin();      //pixels.begin() does exactly what above two lines of code do, so not required to call
    Mode_ButtonOrPixels=1;
  //}
}
void SwitchPixelsToButton(void){//Ver. 1.0, Dustin Soodak
  //if(!(Mode_ButtonOrPixels==0)){
    pinMode(Light_Bus_BTN1,INPUT_PULLUP); 
    delay(1);
    Mode_ButtonOrPixels=0;
  //}
}
char Mode_SerialOrMotors=99;
void SwitchSerialToMotors(void){//Ver. 1.0, Dustin Soodak
  //if(!(Mode_SerialOrMotors==1)){
    Serial.end();
    pinMode(MotorDirection_Left, OUTPUT);
    pinMode(MotorDirection_Right, OUTPUT);
    Mode_SerialOrMotors=1;
  //}
}
void SwitchMotorsToSerial(void){//Ver. 1.0, Dustin Soodak
  //if(!(Mode_SerialOrMotors==0)){
    pinMode(MotorDirection_Left, INPUT);
    Serial.begin(SERIAL_SPEED);//Serial.begin(9600);
    Mode_SerialOrMotors=0;
  //}
}

char ButtonPressed(void){//Ver. 1.0, Dustin Soodak
  SwitchPixelsToButton();
  return (digitalRead(Light_Bus_BTN1)==0);
}


int LeftMotor;
int RightMotor;
void MotorsBegin(void){//Ver. 1.0, Dustin Soodak
  LeftMotor=0;
  RightMotor=0;
  pinMode(MotorDirection_Left, OUTPUT);
  pinMode(MotorDirection_Right, OUTPUT);
  analogWrite(MotorDrive_Left,0);
  analogWrite(MotorDrive_Right,0);  
}


void Motors(int LeftMotorSpeed, int RightMotorSpeed){//Ver. 1.0, Dustin Soodak
  if(!(Mode_SerialOrMotors==1)){
     SwitchSerialToMotors();
  }
  if(LeftMotorSpeed>MOTOR_MAX)
    LeftMotorSpeed=MOTOR_MAX;
  if(LeftMotorSpeed<-MOTOR_MAX)
    LeftMotorSpeed=-MOTOR_MAX;
  if(RightMotorSpeed>MOTOR_MAX)
    RightMotorSpeed=MOTOR_MAX;
  if(RightMotorSpeed<-MOTOR_MAX)
    RightMotorSpeed=-MOTOR_MAX;
  LeftMotor=LeftMotorSpeed;
  RightMotor=RightMotorSpeed;
  if(LeftMotor<0){
    digitalWrite(MotorDirection_Left,0);
    //Serial.print("left - ");
  }
  else
    digitalWrite(MotorDirection_Left,1);
  if(RightMotor<0){
    digitalWrite(MotorDirection_Right,0);
    //Serial.print("right - ");
  }
  else
    digitalWrite(MotorDirection_Right,1);
    
  analogWrite(MotorDrive_Left,abs(LeftMotor));
  analogWrite(MotorDrive_Right,abs(RightMotor));
}

void motors(int LeftMotorSpeed, int RightMotorSpeed){//Ver. 1.0, Dustin Soodak
  Motors(LeftMotorSpeed,RightMotorSpeed);
}

void EdgeLightsOn(void){//Ver. 1.0, Dustin Soodak
  digitalWrite(Edge_Lights, HIGH);
}

void EdgeLightsOff(void){//Ver. 1.0, Dustin Soodak
  digitalWrite(Edge_Lights, LOW);
}

void SwitchAmbientToEdge(void){//Ver. 1.0, Dustin Soodak
  digitalWrite(Source_Select, LOW);
}

void SwitchEdgeToAmbient(void){//Ver. 1.0, Dustin Soodak
  digitalWrite(Source_Select, HIGH);
}

int ReadLeftLightSensor(void){//Ver. 1.0, Dustin Soodak
  return analogRead(LightSense_Left);
}

int ReadRightLightSensor(void){//Ver. 1.0, Dustin Soodak
  return analogRead(LightSense_Right);
}

int ReadRearLightSensor(void){//Ver. 1.0, Dustin Soodak
  return analogRead(LightSense_Rear);
}

extern int LeftEdgeSensorValue,RightEdgeSensorValue,RearEdgeSensorValue;//defined below
extern uint32_t LookAtEdgePrevTimeUs;//defined below
void ReadEdgeLightSensors(char Averages){//Ver. 1.2, Dustin Soodak
int i;
  int templeft,tempright,temprear;
  uint32_t us;  
  us=micros();
  if(us-LookAtEdgePrevTimeUs<LIGHT_SENSOR_STABLIZATION_TIME){
    delayMicroseconds(LIGHT_SENSOR_STABLIZATION_TIME-(us-LookAtEdgePrevTimeUs));
  }
  SwitchAmbientToEdge();  
  LeftEdgeSensorValue=0;
  RightEdgeSensorValue=0;
  RearEdgeSensorValue=0;
  for(i=0;i<Averages;i++){
    templeft=ReadLeftLightSensor();
    tempright=ReadRightLightSensor();
    temprear=ReadRearLightSensor();    
    EdgeLightsOn();
    delayMicroseconds(LIGHT_SENSOR_STABLIZATION_TIME);
    LeftEdgeSensorValue+=ReadLeftLightSensor()-templeft-LEFT_ZERO;
    RightEdgeSensorValue+=ReadRightLightSensor()-tempright-RIGHT_ZERO;
    RearEdgeSensorValue+=ReadRearLightSensor()-temprear-REAR_ZERO;
    EdgeLightsOff();  
  }
  LookAtEdgePrevTimeUs=micros();
  if(LeftEdgeSensorValue<0) LeftEdgeSensorValue=0; else LeftEdgeSensorValue/=Averages;
  if(RightEdgeSensorValue<0) RightEdgeSensorValue=0; else RightEdgeSensorValue/=Averages;
  if(RearEdgeSensorValue<0) RearEdgeSensorValue=0; else RearEdgeSensorValue/=Averages;
}

#ifdef RUNNING_AVERAGE
int LeftEdgeArray[8];
int RightEdgeArray[8];
int RearEdgeArray[8];
char EdgeArrayPos=0;
int LeftEdgeSensorAverageTimes8=0,RightEdgeSensorAverageTimes8=0,RearEdgeSensorAverageTimes8=0;
#endif
#ifdef TIME_ADJUSTED_AVERAGE
float LookAtEdgeStabilizationTime=STABILIZATION_TIME_DEFAULT;
int LeftEdgeSensorValuePrevious,RightEdgeSensorValuePrevious,RearEdgeSensorValuePrevious;
int LeftEdgeSensorAverageTimes32=0,RightEdgeSensorAverageTimes32=0,RearEdgeSensorAverageTimes32=0;
#endif

int LeftEdgeSensorAverage=0,RightEdgeSensorAverage=0,RearEdgeSensorAverage=0;
int LeftEdgeSensorValue=0,RightEdgeSensorValue=0,RearEdgeSensorValue=0;
uint32_t LookAtEdgePrevTimeUs=0;//so LookAtEdge() can be paused until sensors have stabilized from previous time
float LookAtEdgeTimeBetweenReadings=1.0;//can be used to check how long since the last time LookAtEdge() was called.
void ResetLookAtEdge(void){//Ver. 1.1, Dustin Soodak
  char i;
  #ifdef RUNNING_AVERAGE
  for(i=0;i<8;i++){
    LeftEdgeArray[i]=0;
    RightEdgeArray[i]=0;
    RearEdgeArray[i]=0;
  }
  EdgeArrayPos=0;  
  LeftEdgeSensorAverageTimes8=0;  
  RearEdgeSensorAverageTimes8=0;
  RightEdgeSensorAverageTimes8=0;  
  LeftEdgeSensorAverage=0;
  RightEdgeSensorAverage=0;
  RearEdgeSensorAverage=0;  
  #endif//end #ifdef RUNNING_AVERAGE
  #ifdef TIME_ADJUSTED_AVERAGE
  ReadEdgeLightSensors(1);
  LeftEdgeSensorValuePrevious=LeftEdgeSensorValue;
  RearEdgeSensorValuePrevious=RearEdgeSensorValue;
  RightEdgeSensorValuePrevious=RightEdgeSensorValue;
  LeftEdgeSensorAverageTimes32=LeftEdgeSensorValue<<5;
  RearEdgeSensorAverageTimes32=RearEdgeSensorValue<<5;
  RightEdgeSensorAverageTimes32=RightEdgeSensorValue<<5;
  #endif
  LookAtEdgePrevTimeUs=micros();
  for(i=0;i<8;i++){
    //LookAtEdge(); 
  }
}



void LookAtEdge(void){//Ver. 1.2, Dustin Soodak
  //note: ir remote control signals don't usually get into the edge sensors
  uint32_t us;
  #ifdef TIME_ADJUSTED_AVERAGE
  float r1,r2;
  LeftEdgeSensorValuePrevious=LeftEdgeSensorValue;
  RearEdgeSensorValuePrevious=RearEdgeSensorValue;
  RightEdgeSensorValuePrevious=RightEdgeSensorValue;
  #endif
  us=micros();
  if(us-LookAtEdgePrevTimeUs<LIGHT_SENSOR_STABLIZATION_TIME){
    delayMicroseconds(LIGHT_SENSOR_STABLIZATION_TIME-(us-LookAtEdgePrevTimeUs));
  }    
  
  //Serial.println(micros()-us);//remove!!!
  SwitchAmbientToEdge();  
  //Measure
  LeftEdgeSensorValue=ReadLeftLightSensor();
  RightEdgeSensorValue=ReadRightLightSensor();   
  RearEdgeSensorValue=ReadRearLightSensor(); 
  EdgeLightsOn();
  delayMicroseconds(LIGHT_SENSOR_STABLIZATION_TIME);  //originally 5us
  LeftEdgeSensorValue=ReadLeftLightSensor()-LeftEdgeSensorValue-LEFT_ZERO;
  RightEdgeSensorValue=ReadRightLightSensor()-RightEdgeSensorValue-RIGHT_ZERO;
  RearEdgeSensorValue=ReadRearLightSensor()-RearEdgeSensorValue-REAR_ZERO;
  EdgeLightsOff();  
  LookAtEdgeTimeBetweenReadings=((float)us-LookAtEdgePrevTimeUs);
  LookAtEdgePrevTimeUs=micros();
  if(LookAtEdgeTimeBetweenReadings<(LIGHT_SENSOR_STABLIZATION_TIME))
    LookAtEdgeTimeBetweenReadings=(LIGHT_SENSOR_STABLIZATION_TIME);
  LookAtEdgeTimeBetweenReadings*=0.001;
  //Process data
  if(LeftEdgeSensorValue<0)//so can always use "/2" -> ">>1" trick so no divisions have to be done.
    LeftEdgeSensorValue=0;
  if(RightEdgeSensorValue<0){//so can always use "/2" -> ">>1" trick so no divisions have to be done.
    RightEdgeSensorValue=0; 
  }
  if(RearEdgeSensorValue<0){//so can always use "/2" -> ">>1" trick so no divisions have to be done.
    RearEdgeSensorValue=0; 
  }
  #ifdef RUNNING_AVERAGE
  EdgeArrayPos=(EdgeArrayPos+1)&7;// pos=(pos+1) mod 8
  LeftEdgeSensorAverageTimes8-=LeftEdgeArray[EdgeArrayPos];
  RightEdgeSensorAverageTimes8-=RightEdgeArray[EdgeArrayPos];
  RearEdgeSensorAverageTimes8-=RearEdgeArray[EdgeArrayPos];
  LeftEdgeArray[EdgeArrayPos]=LeftEdgeSensorValue;
  RightEdgeArray[EdgeArrayPos]=RightEdgeSensorValue;
  RearEdgeArray[EdgeArrayPos]=RearEdgeSensorValue;
  LeftEdgeSensorAverageTimes8+=LeftEdgeSensorValue;
  RightEdgeSensorAverageTimes8+=RightEdgeSensorValue;
  RearEdgeSensorAverageTimes8+=RearEdgeSensorValue;
  LeftEdgeSensorAverage=LeftEdgeSensorAverageTimes8>>3;
  RightEdgeSensorAverage=RightEdgeSensorAverageTimes8>>3;
  RearEdgeSensorAverage=RearEdgeSensorAverageTimes8>>3;
  #endif
  #ifdef TIME_ADJUSTED_AVERAGE
  //TimeAdjustedAverage=NewValue*(dt/totalT)+AverageValue*((totalT-dt)/totalT)
  //                   =(1/totalT)*(NewValue*dt+AverageValue*(totalT-dt))
  //                   dt=LookAtEdgeTimeBetweenReadings
  //                   totalT=LookAtEdgeStabilizationTime
  //                   AverageValue=, for example, LeftEdgeSensorAverage
  //                   NewValue=, for example, LeftEdgeSensorValue
  //                   generally make sure totalT/dt>=8 so average isn't changed too much by new value
  //                   
  r2=1/LookAtEdgeStabilizationTime;
  r1=LookAtEdgeTimeBetweenReadings*r2;
  if(r1>.125){
    r1=.125;
    r2=.875;
  }
  else
    r2=(LookAtEdgeStabilizationTime-LookAtEdgeTimeBetweenReadings)*r2;
  r1*=32;//since using "Times32" averages
  LeftEdgeSensorAverageTimes32=LeftEdgeSensorValue*r1+LeftEdgeSensorAverageTimes32*r2;
  RearEdgeSensorAverageTimes32=RearEdgeSensorValue*r1+RearEdgeSensorAverageTimes32*r2;
  RightEdgeSensorAverageTimes32=RightEdgeSensorValue*r1+RightEdgeSensorAverageTimes32*r2;
  LeftEdgeSensorAverage=LeftEdgeSensorAverageTimes32>>5;
  RearEdgeSensorAverage=RearEdgeSensorAverageTimes32>>5;
  RightEdgeSensorAverage=RightEdgeSensorAverageTimes32>>5;
  #endif
 
}


char IsOverEdge(void){//Ver 1.0, Dustin Soodak
  char edge=0;
  int Dark1,Dark2,Bright1,Bright2;
  //Absolute endpoint tests:
  #ifdef DARK_MIN
  edge|=LeftEdgeSensorValue<DARK_MIN?LEFT_DARK:0;
  edge|=RightEdgeSensorValue<DARK_MIN?RIGHT_DARK:0;
  edge|=RearEdgeSensorValue<DARK_MIN?REAR_DARK:0;
  if(edge!=0)
    edge|=EXTRA_DARK;
  #endif
  #ifdef LIGHT_MAX
  edge|=LeftEdgeSensorValue>LIGHT_MAX?LEFT_BRIGHT:0;
  edge|=RearEdgeSensorValue>LIGHT_MAX?REAR_BRIGHT:0;
  edge|=RightEdgeSensorValue>LIGHT_MAX?RIGHT_BRIGHT:0;
  #endif  

  //More sensitive tests:
  #ifdef CHECK_EDGE_TWICE
  if(!LeftDarkDetected(edge)){    
    edge|=((LeftEdgeSensorValue<DarkEdgeMult2(LeftEdgeSensorAverage) && LeftEdgeSensorValuePrev()<DarkEdgeMult1(LeftEdgeSensorAverage))?LEFT_DARK:0);
    if(!LeftDarkDetected(edge))
      edge|=((LeftEdgeSensorValue>BrightEdgeMult2(LeftEdgeSensorAverage) && LeftEdgeSensorValuePrev()>BrightEdgeMult1(LeftEdgeSensorAverage))?LEFT_BRIGHT:0);
  }
  if(!RearDarkDetected(edge)){
    edge|=((RearEdgeSensorValue<DarkEdgeMult2(RearEdgeSensorAverage) && RearEdgeSensorValuePrev()<DarkEdgeMult1(RearEdgeSensorAverage))?REAR_DARK:0);
    if(!RearDarkDetected(edge))
      edge|=((RearEdgeSensorValue>BrightEdgeMult2(RearEdgeSensorAverage) && RearEdgeSensorValuePrev()>BrightEdgeMult1(RearEdgeSensorAverage))?REAR_BRIGHT:0);
  }
  if(!RightDarkDetected(edge)){
    edge|=((RightEdgeSensorValue<DarkEdgeMult2(RightEdgeSensorAverage) && RightEdgeSensorValuePrev()<DarkEdgeMult1(RightEdgeSensorAverage))?RIGHT_DARK:0);
    if(!RightDarkDetected(edge))
      edge|=((RightEdgeSensorValue>BrightEdgeMult2(RightEdgeSensorAverage) && RightEdgeSensorValuePrev()>BrightEdgeMult1(RightEdgeSensorAverage))?RIGHT_BRIGHT:0);
  }
  #endif //end check twice
  #ifndef CHECK_EDGE_TWICE  
  if(!LeftDarkDetected(edge)){
    edge|=((LeftEdgeSensorValue<DarkEdgeMult1(LeftEdgeSensorAverage))?LEFT_DARK:0);
    if(!LeftDarkDetected(edge))
      edge|=((LeftEdgeSensorValue>BrightEdgeMult1(LeftEdgeSensorAverage))?LEFT_BRIGHT:0);
  }
  if(!RearDarkDetected(edge)){
    edge|=((RearEdgeSensorValue<DarkEdgeMult1(RearEdgeSensorAverage))?REAR_DARK:0);
    if(!RearDarkDetected(edge))
      edge|=((RearEdgeSensorValue>BrightEdgeMult1(RearEdgeSensorAverage))?REAR_BRIGHT:0);
  }
  if(!RightDarkDetected(edge)){
    edge|=((RightEdgeSensorValue<DarkEdgeMult1(RightEdgeSensorAverage)))?BRIGHT_DARK:0);
    if(!RightDarkDetected(edge))
      edge|=((RightEdgeSensorValue>BrightEdgeMult1(RightEdgeSensorAverage))?BRIGHT_BRIGHT:0);
  }
  #endif// end check once
  return edge;
}//end IsOverEdge


// calls LookAtEdge(), and uses running average to detect edges or white tape.
// Output bit num: 0: right dark edge (0x01)(0b000001)  1: right bright edge (0x02)(0b000010)   
//                 2: rear dark edge (0x04)(0b000100)   3: rear bright edge (0x08)(0b001000)  
//                 4: left dark edge (0x10)(0b010000)   5: left bright edge (0x20)(0b100000)
char LookForEdge(void){//Ver. 1.1, Dustin Soodak
  //Ver. 1.1:
  LookAtEdge();
  return IsOverEdge();
  
}



int RightLightLevel,LeftLightLevel,RearLightLevel;
int RightLightLevelPrev,LeftLightLevelPrev,RearLightLevelPrev;
float RightLightLevelAverage=0,LeftLightLevelAverage=0,RearLightLevelAverage=0;//float instead of int so enough precision for 
                                                                               //the averaging algorithm to still work with 
                                                                               //extra long stabilization times.
int RearAmbientLightLevel,RightAmbientLightLevel,LeftAmbientLightLevel;
uint32_t ReadSideSensorsPrevTimeUs=0;//note: in future, maybe make this static inside ReadSideSensors (unless we want
                                     //      a "ResetReadSideSensors()" function).
void ReadSideSensors(void){//Ver. 1.1, Dustin Soodak
  uint32_t dt=micros()-ReadSideSensorsPrevTimeUs;
  float a1,b1,a2,b2;
  if(dt<LIGHT_SENSOR_STABLIZATION_TIME){
    delayMicroseconds(LIGHT_SENSOR_STABLIZATION_TIME-dt);
  }   
  digitalWrite(IR_Send,0);
  digitalWrite(IR_Enable_Front,1);
  digitalWrite(IR_Enable_RearLeft,1);
  digitalWrite(IR_Enable_RearRight,1);
  //ReadEdgeLightSensors();
  LeftLightLevelPrev=LeftLightLevel;
  RearLightLevelPrev=RearLightLevel;
  RightLightLevelPrev=RightLightLevel;
  SwitchEdgeToAmbient();
  LeftAmbientLightLevel=ReadLeftLightSensor();
  RightAmbientLightLevel=ReadRightLightSensor();
  RearAmbientLightLevel=ReadRearLightSensor();
  digitalWrite(IR_Send,1); 
  delayMicroseconds(LIGHT_SENSOR_STABLIZATION_TIME);
  LeftLightLevel=ReadLeftLightSensor()-LeftAmbientLightLevel;
  RightLightLevel=ReadRightLightSensor()-RightAmbientLightLevel;
  RearLightLevel=ReadRightLightSensor()-RearAmbientLightLevel;
  digitalWrite(IR_Send,0);
  if(LeftLightLevel<0) LeftLightLevel=0;
  if(RightLightLevel<0) RightLightLevel=0;  
  if(RearLightLevel<0) RearLightLevel=0;  
  //  
  //TimeAdjustedAverage=NewValue*(dt/totalT)+AverageValue*((totalT-dt)/totalT)
  //totalT=SIDE_SENSOR_AVER_RISE_TIME us or SIDE_SENSOR_AVER_FALL_TIME us.
  if(dt>SIDE_SENSOR_AVER_RISE_TIME){
    a1=1;
    b1=0;
  }
  else{
    a1=((float)dt)/SIDE_SENSOR_AVER_RISE_TIME;
    b1=((float)SIDE_SENSOR_AVER_RISE_TIME-dt)/SIDE_SENSOR_AVER_RISE_TIME;
  }
  if(dt>SIDE_SENSOR_AVER_FALL_TIME){
    a2=1;
    b2=0;
  }
  else{
    a2=((float)dt)/SIDE_SENSOR_AVER_FALL_TIME;
    b2=((float)SIDE_SENSOR_AVER_FALL_TIME-dt)/SIDE_SENSOR_AVER_FALL_TIME;
  }
  if(LeftLightLevel>LeftLightLevelAverage)
    LeftLightLevelAverage=a1*LeftLightLevel+b1*LeftLightLevelAverage;
  else
    LeftLightLevelAverage=a2*LeftLightLevel+b2*LeftLightLevelAverage;
  if(RightLightLevel>RightLightLevelAverage)
    RightLightLevelAverage=a1*RightLightLevel+b1*RightLightLevelAverage;
  else
    RightLightLevelAverage=a2*RightLightLevel+b2*RightLightLevelAverage;
  if(RearLightLevel>RearLightLevelAverage)
    RearLightLevelAverage=a1*RearLightLevel+b1*RearLightLevelAverage;
  else
    RearLightLevelAverage=a2*RearLightLevel+b2*RearLightLevelAverage;
  ReadSideSensorsPrevTimeUs=micros();
}
//Some test code for ReadSideSensors():
//HardwareBegin();
//RestartTimer();
//while(!ButtonPressed()){
//  if(GetTime()>200){
//    ReadSideSensors();
//    //Try printing something like this:
//    Serial.print(LeftLightLevel,DEC);Serial.print(" ");Serial.print(RightLightLevel,DEC);Serial.print(" Amb: ");
//    Serial.print(LeftAmbientLightLevel,DEC);Serial.print(" ");Serial.print(RightAmbientLightLevel,DEC);
//    //or this (if these values > 50, it usually means a barrier is coming up):
//    //Serial.print(LeftLightLevel-LeftLightLevelAverage,DEC);Serial.print(" ");Serial.print(RightLightLevel-RightLightLevelAverage,DEC);
//    Serial.println();
//    RestartTimer();
//  }
//}

void TxIR(unsigned char *Data, int Length){//Ver. 1.2, Dustin Soodak
    int i;
    char j;
    const uint16_t Freq=38000,UsOn=5; //For R2=2k pull up, 8 us delay before pin falls. Inputs (28000,5) give a decent square wave in this case. 
    RxIRStop();
    EnableIROutputs(1);
    ModulateIR(Freq,UsOn);
    delayMicroseconds(9000);
    EnableIROutputs(0);               //EnableIROutputs(0) turns off IR 38khz out but this makes receiver voltage go high.
    delayMicroseconds(4500);
    EnableIROutputs(1);
    delayMicroseconds(520);
    for(i=0;i<Length;i++){      
      for(j=0;j<8;j++){
        EnableIROutputs(0);
        if((Data[i])&(1<<j))
          delayMicroseconds(1610);
        else
          delayMicroseconds(580);
        EnableIROutputs(1);
        delayMicroseconds(520);
      }      
    }//end for(i=0;i<Length;i++)    
    ModulateIR(38000, 0);
    EnableIROutputs(1); 
}//end TxIR()

void TxIRKey(byte key){//Ver. 1.0, Kevin King
  if(key<1){
    // do nothing
  }
  else if(key>21){
    // do nothing
  }
  else{     // actually send IR key if it was within correct range
  key-=1;   //subract 1 from key number so it matches array IRRemoteButtons
  irData[0]=0x00;irData[1]=0xFF;     // all remote keys begin with 0x00, 0xFF
  irData[2]=IRRemoteButtons[key][0]; // add 3rd value
  irData[3]=IRRemoteButtons[key][1]; // add 4th value
  TxIR(irData,4);                    // actually transmit via any enabled IR sources
  }
}//end TxIRKey()

int IRTransitionCount=0;
char IRBitNum=0,IRByte=0,IRNumOfBytes=0;
unsigned char IRBytes[20];
uint16_t IRPrevTime=0,IRTime;
uint32_t MsAtLastIR=0;//used for end of communication timeout
volatile char IRReceiving=0;
char IRActive=0;
char IRMaxByteCount=4;


void RxIRRestart(char BytesToLookFor){//Ver. 1.2, Dustin Soodak
  int i;
  detachInterrupt(1);//interrupt 1 is I/O 3 which is _38kHz_R
  TCCR1A = 0x00;          // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1 -- PWM11=0,PWM10=0 => PWM Operation disabled
  // ICNC1=0 => Capture Noise Canceler disabled -- ICES1=0 => Input Capture Edge Select (not used) -- CTC1=0 => Clear Timer/Counter 1 on Compare/Match
  // CS12=0 CS11=1 CS10=1 => Set prescaler to clock/64
  TCCR1B = 0x03;          // 8MHz clock with prescaler 0x03 means TCNT1 increments every 8uS
  // ICIE1=0 => Timer/Counter 1, Input Capture Interrupt Enable -- OCIE1A=0 => Output Compare A Match Interrupt Enable -- OCIE1B=0 => Output Compare B Match Interrupt Enable
  // TOIE1=0 => Timer 1 Overflow Interrupt Enable
  TIMSK1 = 0x00;          
  pinMode(_38kHz_Rx, INPUT_PULLUP);  
  IRReceiving=0;  
  IRTransitionCount=0;
  IRPrevTime=0;
  MsAtLastIR=0;IRBitNum=0;IRByte=0;IRNumOfBytes=0;
  IRActive=1;
  IRMaxByteCount=BytesToLookFor;
  attachInterrupt(1, IRHandler, CHANGE);//interrupt 1 is I/O 3 which is _38kHz_Rx  
}
void RxIRStop(void){//Ver. 1.2, Dustin Soodak
  detachInterrupt(1);//interrupt 1 is I/O 3 which is _38kHz_Rx    
  TCCR1B=0;//turn off timer        
  pinMode(_38kHz_Rx, INPUT_PULLUP);  
  
  MsAtLastIR=0;//so IRHandler() recognizes it as first falling edge of next transition
  IRReceiving=0;
  IRActive=0;
  IRTransitionCount=0;//so IsIRDone() does not expect anything just because RxIRStop() called.
}

// Infrared transmit and recieve functions are based on:
//http://playground.arduino.cc/Code/InfraredReceivers by Paul Malmsten 
//https://github.com/z3t0/Arduino-IRremote by Ken Shirriff

void IRHandler(void){//Ver. 2.0, Dustin Soodak, Kevin King
  //using interrupt 1 is I/O 3 which is _38kHz_Rx  
  int16_t dTime;
  char Level;
  noInterrupts();
  IRTime=TCNT1;
   
  Level=digitalRead(_38kHz_Rx);  

  if(!Level){//note; 38khz IR signal makes level go low
    IRReceiving=1;
  }
  if((millis()-MsAtLastIR>15) && (IRNumOfBytes<IRMaxByteCount)) {//should never be more than 9 inside valid signal
    TCNT1=0;
    IRPrevTime=0;
    IRTransitionCount=0;
    IRBitNum=0;IRNumOfBytes=0;IRByte=0;  
    //IRReceiving=1;  
  }  
  else{  
    if(IRTime>IRPrevTime)
      dTime=IRTime-IRPrevTime;
    else
      dTime=0xFFFF-IRPrevTime+1+IRTime;   
    IRPrevTime=IRTime;    
    dTime=dTime<<3;
    if(IRTransitionCount>=3 && (IRTransitionCount&1)){//should be high
      if(dTime>1000){
        IRByte|=(1<<IRBitNum); 
      }
      if(dTime<300){//error
         IRNumOfBytes=0;
         IRReceiving=0;
      }
      IRBitNum++;
      if(IRBitNum>7){
        if(IRNumOfBytes<IRMaxByteCount){
          IRBytes[IRNumOfBytes]=IRByte;
          IRNumOfBytes++;
        }        
        else{
           IRReceiving=0;           
        }
        IRBitNum=0; 
        IRByte=0;               
      }      
    }
    IRTransitionCount++;
  }
  MsAtLastIR=millis();
  interrupts();
}

char IsIRDone(void){//Ver. 1.0, Dustin Soodak
  return ((millis()-MsAtLastIR>40 || IRNumOfBytes==IRMaxByteCount) && IRTransitionCount);  
}

//Note: first and second bytes are always 0x00 and 0xFF. IRRemoteButtons[] contains the third and fourth bytes.
const byte IRRemoteButtons[][2]={
  {0x0C,0xF3},//"1" key: 1
  {0x18,0xE7},//"2" key: 2
  {0x5E,0xA1},//"3" key: 3
  {0x08,0xF7},//"4" key: 4
  {0x1C,0xE3},//"5" key: 5
  {0x5A,0xA5},//"6" key: 6
  {0x42,0xBD},//"7" key: 7
  {0x52,0xAD},//"8" key: 8
  {0x4A,0xB5},//"9" key: 9
  {0x16,0xE9},//"0" key: 10
  {0x40,0xBF},//"FORWARD" key: 11
  {0x07,0xF8},//"LEFT" key: 12
  {0x09,0xF6},//"RIGHT" key: 13
  {0x19,0xE6},//"BACKWARD" key: 14
  {0x45,0xBA},//"POWER" key: 15
  {0x46,0xB9},//"PLUM LOGO" key: 16
  {0x47,0xB8},//"MENU" key: 17
  {0x44,0xBB},//"A" key: 18
  {0x43,0xBC},//"B" key: 19
  {0x15,0xEA},//"PLAY" key: 20
  {0x0D,0xF2}//"X" key: 21
};
byte GetIRButton(void){//Ver. 2.0, Dustin Soodak, Kevin King
  byte ButtonNumber=0,i;

    for(i=0;i<sizeof(IRRemoteButtons)/2;i++){
      if(IRBytes[2]==IRRemoteButtons[i][0] && IRBytes[3]==IRRemoteButtons[i][1]){
        ButtonNumber=i+1;
        break; 
      }
    } 

    return ButtonNumber;       //return the button number that was pressed
    
}// end GetIRButton()


// ***************************************************
// end Ringo Hardware
// ***************************************************

// ***************************************************
// Pixels
// ***************************************************

// These functions use the Adafruit NeePixel Libraray https://github.com/adafruit/Adafruit_NeoPixel

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS, Light_Bus_BTN1, NEO_GRB + NEO_KHZ800);
void SetPixelRGB(int Pixel, int Red, int Green, int Blue){//Ver. 1.0, Dustin Soodak
  SwitchButtonToPixels();  //make sure pixel line is selected
  if(Pixel>6)
    Pixel=6;
  if(Pixel<0)
    Pixel=0;
  pixels.setPixelColor(Pixel, pixels.Color(Red,Green,Blue));
  pixels.show();//Remove this line and use RefreshPixels() below 
  //              if you want to set several at once for high-speed patterns.
}
void SetAllPixelsRGB(int Red, int Green, int Blue){//Ver. 1.1, Dustin Soodak
  SwitchButtonToPixels();
  char i;
  for(i=0;i<NUM_PIXELS;i++){
    pixels.setPixelColor(i, pixels.Color(Red,Green,Blue)); 
  }
  pixels.show();//added Ver.1.1
}
void RefreshPixels(void){//Ver. 1.0, Dustin Soodak
  pixels.show();  
}
// ***************************************************
// end Pixels
// ***************************************************


// ***************************************************
// MovementFunctions
// ***************************************************

//
//MaintainHeading:
//
//This is a simplified PID (proportiona-Integral-Derivative) function for
//maintaining your present heading. A complete example can be found below
//the MaintainHeading(degrees,speed,wiggle) function.
//
int MaintTainHeadingOffsetDir=1;
int MaintainHeadingIntegral=0;
uint32_t MaintainHeadingPrevTimeUs;

void MaintainHeadingReset(){//Ver. 1.0, Dustin Soodak
  MaintainHeadingIntegral=0;
  MaintainHeadingPrevTimeUs=micros();
}

char MaintainHeading(int Heading, int Speed, int Wiggle){//Ver. 1.0, Dustin Soodak
  int Input,Output,Proportional,left,right;
  char ret=0;
  float dt;
  if(Wiggle>0){
    if(MaintTainHeadingOffsetDir>0){
      if(GetDegrees()>=Heading+Wiggle)
        MaintTainHeadingOffsetDir=-1;    
    }
    else{
      if(GetDegrees()<=Heading-Wiggle)
        MaintTainHeadingOffsetDir=1;
    }
  }  
  Input=GetDegrees()+GetDegreesToStop();
  Proportional=(Heading+MaintTainHeadingOffsetDir*Wiggle-Input);//make it try to turn to the wiggle value
  //Note: "MaintainHeadingAverageDerivative" no longer needs to be needed. Code left here in case necessary for modded bot.
  //TimeAdjustedAverage=NewValue*(dt/totalT)+AverageValue*((totalT-dt)/totalT)
  //In this case, we are doing a 1/10th second (100000us) time average.
  //if(dt>100000)
  //  MaintainHeadingAverageDerivative=GetDegreesPerSecond();
  //else
  //  MaintainHeadingAverageDerivative=(((float)dt)*.00001)*GetDegreesPerSecond()+((100000-(float)dt)*.00001)*MaintainHeadingAverageDerivative;
  dt=micros()-MaintainHeadingPrevTimeUs;
  MaintainHeadingPrevTimeUs=MaintainHeadingPrevTimeUs+dt;
  if(dt>100000)
    dt=100000;  
    MaintainHeadingIntegral+=(((float)dt)*.0001)*Proportional;
  if(MaintainHeadingIntegral>20*MAINTAIN_HEADING_MAX_INTEGRAL_TERM)
    MaintainHeadingIntegral=20*MAINTAIN_HEADING_MAX_INTEGRAL_TERM;
  if(MaintainHeadingIntegral<-20*MAINTAIN_HEADING_MAX_INTEGRAL_TERM)
    MaintainHeadingIntegral=-20*MAINTAIN_HEADING_MAX_INTEGRAL_TERM;   
  Output=Proportional+MaintainHeadingIntegral/20;
  //Use this version in case MaintainHeadingAverageDerivative has to brought back:
  //Output=Proportional+MaintainHeadingIntegral/20-(Wiggle==0?1*MaintainHeadingAverageDerivative/4:0*MaintainHeadingAverageDerivative/4);
  if(Output>150)
    Output=150;
  if(Output<-150)
    Output=-150;
  left=Speed+Output;
  right=Speed-Output;
  //Note: only include this section if you know what your robots min motor speed is (below which it doesn't move at all):
  if(abs(left)<MIN_MOTOR_SPEED){
    left=(left>0?MIN_MOTOR_SPEED:left<0?-MIN_MOTOR_SPEED:0);
  }
  if(abs(right)<MIN_MOTOR_SPEED){
    right=(right>0?MIN_MOTOR_SPEED:right<0?-MIN_MOTOR_SPEED:0);
  }
  Motors(left,right);
  //a debug/test section:
  /*RecordedDataRow.Prop=Proportional;
  RecordedDataRow.Int=MaintainHeadingIntegral;
  RecordedDataRow.Der=MaintainHeadingAverageDerivative;
  RecordedDataRefresh();*/
  return ret;
}
//A complete example: (CTRL-/ to un-comment)
//#include "RingoHardware.h"
//int Heading;
//int Speed=100;
//int Wiggle=0;
//void setup(){
//  HardwareBegin();
//  PlayStartChirp(); 
//  while(!ButtonPressed());
//  delay(1000);
//  NavigationBegin();
//  ResumeNavigation();
//  Heading=PresentHeading();
//  MaintainHeadingReset();
//}
//void loop(){
//  SimpleGyroNavigation();//or SimpleNavigation(), or NavigationXY()
//  MaintainHeading(Heading,Speed,Wiggle);
//  if(PresentHeading()>Heading)
//    SetPixelRGB(BODY_TOP,0,0,10);
//  else if(PresentHeading()==Heading)
//    SetPixelRGB(BODY_TOP,0,10,0);
//  else
//    SetPixelRGB(BODY_TOP,10,0,0);
//}




void DriveArc(int TurnDegrees, int left, int right, int MaxExpectedTurnTime, int MaxExpectedSkidTime){//Ver. 1.0, Dustin Soodak
  uint32_t timeout;
  int degr,DegrPredict,DegrInit;
  SwitchSerialToMotors();
  if(NavigationOn){
    CalibrateNavigationSensors();
    ResumeNavigation();
  }
  else
    NavigationBegin();
  
  DegrInit=GetDegrees();     
  Motors(left,right);
  timeout=millis()+MaxExpectedTurnTime;
  RestartTimer();
  while(millis()<timeout){
    SimpleGyroNavigation();
    degr=GetDegrees()-DegrInit;
    DegrPredict=degr+GetDegreesToStop(); //DegrPredict=degr+GyroDegreesToStopFromRaw(rate);//(((float)(-rate))*2000/32768)*0.1029-(24);        
    if(TurnDegrees>=0?(DegrPredict>=TurnDegrees):(DegrPredict<=TurnDegrees)){ 
      break; 
    }
  }
  Motors(0,0);     
  timeout=millis()+MaxExpectedSkidTime;
  while(millis()<timeout){
    SimpleNavigation();
  }
  PauseNavigation();
}

char RotateAccurate(int Heading, int MaxExpectedTurnTime){//Ver. 1.0, Dustin Soodak
  uint32_t timeout,timestart;
  int degr,skid,motor,degrprev;
  char res=0;
  char reverses=0;
  //RecordedDataReset(100000);
  if(NavigationOn){
    CalibrateNavigationSensors();
    ResumeNavigation();
  }
  else
    NavigationBegin();    
  
  timestart=millis();
  timeout=timestart+MaxExpectedTurnTime;
  degrprev=GetDegrees()-Heading;
  SwitchSerialToMotors();
  while(1){
    SimpleGyroNavigation();//NavigationXY(100,800);//SimpleGyroNavigation();
    //Data.degr=GetDegrees();
    //Data.ms=RecordedDataTime();
    //if(!RecordedDataFull())
    //  RecordedDataRefresh();
    degr=GetDegrees()-Heading;
    if((degr>0)!=(degrprev>0) && millis()>timestart+300)
      reverses++;
    degrprev=degr;
    skid=GetDegreesToStop();    
    if(abs(degr)<=1){
      Motors(0,0);
      if(GetDegreesPerSecond()==0){        
        timeout=millis()+50;
        while(millis()<timeout){
          SimpleGyroNavigation();//NavigationXY(100,800);//SimpleGyroNavigation();
        }
        degr=GetDegrees()-Heading;
        if(abs(degr)<=1){
          res=1;
          break;
        }
      }
    }
    else{
      motor=MIN_MOTOR_SPEED+abs(degr+skid);
      if(motor>200)
        motor=200;      
      if(degr+skid>0)
        Motors(-motor,motor); 
      else
        Motors(motor,-motor); 
    }
    if(millis()>timeout || reverses>3){      
      res=0;
      Motors(0,0);
      while(GetDegreesPerSecond()!=0 && millis()<timeout+500){
        SimpleGyroNavigation();//NavigationXY(100,800);//SimpleGyroNavigation(); 
      }
      break;
    }
  }//end while(1)
  return res;
}//end Rotate()



void MoveWithOptions(int Heading, int Distance, int Speed, int MaxExpectedRunTime, int MaxExpectedSkidTime, void (*EdgeFunction)(char), char Wiggle){//Ver. 1.0, Dustin Soodak
  
  uint32_t timeout;
  int xrelative,yrelative,xrelativeinit,yrelativeinit,X,Y;
  int Input,Output,Proportional,Integral=0,Derivative;
  int MinLeftEdge,MinRightEdge;
  char left,right,edge;
  signed char BackAway=0;
  signed char OffsetDir=1;
  char StartedInRightDirection=0;
  char DirectionCount=0;
  float theta;
  
  if(!NavigationOn){//put this back (so doesn't reset navigation each time) when it works with accurate rotation function.
    NavigationBegin();
    PauseNavigation();
  }
  ResetLookAtEdge();    
  
  theta=((float)(90-Heading))*3.14159/180;
  X=cos(theta)*Distance+GetPositionX();
  Y=sin(theta)*Distance+GetPositionY();
  
    
  xrelative=GetPositionX()-X;
  xrelativeinit=xrelative;
  yrelative=GetPositionY()-Y; 
  yrelativeinit=yrelative;
    
  ResumeNavigation(); 
  SwitchSerialToMotors();
  Motors(Speed,Speed);
  timeout=millis()+MaxExpectedRunTime;
  Input=GetDegrees()+GetDegreesToStop();
 
  RestartTimer();
  while(millis()<timeout){
    NavigationXY(100,1000);  
    
    xrelative=GetPositionX()-X;
    yrelative=GetPositionY()-Y;
    if(abs(GetDegrees())-Heading>30 && Wiggle==0){
      Heading=90-atan2(-yrelative,-xrelative)*180/3.14159;
      xrelativeinit=xrelative;//finish line is in new direction
      yrelativeinit=yrelative;
    }

    MaintainHeading(Heading,Speed,Wiggle);

    
    /*
    if(OffsetDir>0){
      if(GetDegrees()>Heading+Wiggle)
        OffsetDir=-1;    
    }
    else{
      if(GetDegrees()<Heading-Wiggle)
        OffsetDir=1;
    }
    Input=GetDegrees()+GetDegreesToStop();
    Proportional=(Heading+OffsetDir*Wiggle-Input);//make it try to turn to the wiggle value
    Derivative=GetDegreesPerSecond();
    Integral+=Proportional;
    if(Integral/20>100)
      Integral=20*100;
    if(Integral/20<-100)
      Integral=-20*100; 
    Output=Proportional+Integral/20-(Wiggle==0?3*Derivative/4:Derivative/4);
    if(Output>100)
      Output=100;
    if(Output<-100)
      Output=-100; 
    Motors(Speed+Output,Speed-Output);  
    */
    //If dot product of initial and current relative positions is negative, then their orientations differ by more than 90 degrees.
    //This is how we determine if we have passed the imaginary "finish line".
    if(((int32_t)xrelative)*xrelativeinit+((int32_t)yrelative)*yrelativeinit<0){
      
      break;
    }

    if(EdgeFunction){
      edge=LookForEdge();
      if(edge){
        EdgeFunction(edge);        
        break;//exit
      }
    }
  }//end while(millis()<timeout)
  Motors(0,0);     
  //SwitchButtonToPixels();SetPixelRGB(5,H_Bright,H_Bright/3,H_Bright/3);SetPixelRGB(6,H_Bright,H_Bright/3,H_Bright/3);RefreshPixels();//for behavior 030
  timeout=millis()+MaxExpectedSkidTime;
  while(millis()<timeout){
    NavigationXY(80,800);//lower values to be sure it really is stationary
    if(IsStationary)
      break;
  }
  
}


void MoveXYWithOptions(int X, int Y, int Speed, int MaxExpectedRunTime, int MaxExpectedSkidTime, void (*EdgeFunction)(char), char Wiggle){//Ver. 1.0, Dustin Soodak
  int32_t xrelative=GetPositionX()-X;
  int32_t yrelative=GetPositionY()-Y;
  int Heading=90-atan2(-yrelative,-xrelative)*180/3.14159;
  int Distance=sqrt(((int32_t)xrelative)*xrelative+((int32_t)yrelative)*yrelative);
  Heading=MinTurn(Heading-GetDegrees())+GetDegrees();
  //SwitchMotorsToSerial();Serial.print("heading ");Serial.print(Heading);Serial.print(" Distance ");Serial.println(Distance);
  if(Distance>0)
    MoveWithOptions(Heading,Distance,Speed,MaxExpectedRunTime,MaxExpectedSkidTime,EdgeFunction,Wiggle);
}



// ***************************************************
// end MovementFunctions
// ***************************************************


// ***************************************************
// Timer
// ***************************************************

int32_t Timer_InitTime=0,Timer_StoppedTime=0;
char Timer_Running=0;
int32_t GetTime(void){//Ver. 1.0, Dustin Soodak
  if(Timer_Running){
    return millis()-(uint32_t)Timer_InitTime;
  }
  else
    return Timer_StoppedTime;
}
void RestartTimer(void){//Ver. 1.0, Dustin Soodak
  Timer_InitTime=millis();
  Timer_Running=1;
}
void StopTimer(void){//Ver. 1.0, Dustin Soodak
  if(Timer_Running){
    Timer_StoppedTime=millis()-(uint32_t)Timer_InitTime;
    Timer_Running=0;
  }
}

// ***************************************************
// end Timer
// ***************************************************


// ***************************************************
// IR Data Sending
// ***************************************************

void EnableIROutputs(char Level){//Ver. 1.0, Dustin Soodak
  if(Level){
    digitalWrite(IR_Enable_Front,1);
    digitalWrite(IR_Enable_RearLeft,1);
    digitalWrite(IR_Enable_RearRight,1);
  }
  else{
    digitalWrite(IR_Enable_Front,0);
    digitalWrite(IR_Enable_RearLeft,0);
    digitalWrite(IR_Enable_RearRight,0);
  }
}

void ModulateIR(unsigned int Frequency, unsigned int OnTime){//Ver. 1.0, Dustin Soodak 
  //ModulateIR(38000,6) seems to produce best square wave for 38kHz.
  //Frequency is in Hz
  //OnTime is in units of UsOn
  uint16_t temp;
  uint16_t period,dutycycle;
  uint8_t prescalerbits;
  if(OnTime>100)
    OnTime=100;
  if (F_CPU/Frequency/2>=0x10000){
    if(F_CPU/Frequency/2>=0x10000*8){
        prescalerbits=0b011;//prescaler 64
        period=F_CPU/Frequency/(2*16);
        dutycycle=F_CPU/1000000*OnTime/2/64;               
    }
    else{
      prescalerbits = 0b010;// prescaler 8
      period=F_CPU/Frequency/(2*8);
      dutycycle=F_CPU/1000000*OnTime/2/8;
    }
  }
  else{
    prescalerbits = 0b001;  //on but no prescaling
    period=F_CPU/Frequency/(2*1);
    dutycycle=F_CPU/1000000*OnTime/2/1;
  }
  if(OnTime==0){
    TCCR1A=0;
    TCCR1B=0;
    pinMode(Chirp,INPUT);
    
    //Serial.println("off");
  }
  else{
    TCCR1B&=~0b00000111;//turn off timer    
    ICR1=period;
    OCR1B=dutycycle;
    TCCR1A = (0b10<<4) | 0b10;//COM1B1 COM1B0, and WGM11 WGM10
    TCCR1B = (0b10<<3) | prescalerbits;//WGM13 WGM12, and off/(on with prescaler)    
  }
}

void PlayChirpIR(unsigned int Frequency, unsigned int OnTime){//Ver. 1.0, Dustin Soodak
  // ModulateIR used to be called PlayChirpIR, left in for backward compatibility.
   ModulateIR(Frequency,OnTime); 
}

char CheckMenuButton(void){
  byte button;
  if(IsIRDone()){              //wait for an IR remote control command to be received
      button = GetIRButton();  // read which button was pressed, store in "button" variable
      RxIRRestart(4);          // restart looking for another buttom press
      if(button == 17){         // button 17 is the "MENU" button on the IR remote
      return 1;
      }
      else{
      return 0;  
      }
  }
  return 0;
}

// ***************************************************
// end IR Data Sending
// ***************************************************


// ***************************************************
// Recorded Data
// ***************************************************

  //Ver. 1.0, Dustin Soodak
  //Global variables:
  RecordedDataStruct RecordedDataRow;
  RecordedDataStruct RecordedDataArray[RECORDED_DATA_ARRAY_LENGTH];
  unsigned char RecordedDataLength=0;
  unsigned char RecordedDataPosition=0;
  int RecordedDataN;
  uint32_t RecordedDataStart,RecordedDataPrev;
  uint16_t RecordedDataMinDelay;
      
  //RecordedDataRow.ms=RecordedDataTime()/1000;RecordedDataRow.degr=GetDegrees();RecordedDataRefresh();
  
// ***************************************************
// end Recorded Data
// ***************************************************




