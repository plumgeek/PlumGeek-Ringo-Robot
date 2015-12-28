/*

Ringo Robot:  Navigation  Rev01.02  12/2015

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

#include "Navigation.h"

//Ver. 1.0, Dustin Soodak

// ***************************************************
// Ringo I2C
// ***************************************************

#include <RingoWire.h> 
//put "RingoWireLibrary\RingoWire" into "C:\Program Files\Arduino\libraries"

//Accelerometer:
//http://cache.freescale.com/files/sensors/doc/data_sheet/MMA8451Q.pdf
//p.18
//
//Gyroscope:
//http://www.st.com/web/en/resource/technical/document/datasheet/DM00036465.pdf
//p.23-24
//
//Gyro: "Use same format for read/write single/multiple bytes"
//Accel: "The MMA8451Q automatically increments the received register address commands after a write command is received"


void I2CBegin(void){//Ver. 1.0, Dustin Soodak
  Wire.begin();
}

uint8_t I2CReadRegs(uint8_t Device, uint8_t Reg, uint8_t *RxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  uint8_t i;
  Wire.beginTransmission(Device); // transmit to device (note: actually just starts filling of buffer)
  Wire.write(Reg);
  Wire.endTransmission(0);//send data without stop at end
  Wire.requestFrom(Device, Length);
  i=0;
  while(Wire.available()){
    RxBuffer[i]=Wire.read(); 
    i++;
  }
  return i;  
}

uint8_t I2CReadReg(uint8_t Device, uint8_t Reg){//Ver. 1.0, Dustin Soodak
  uint8_t dat=0;
  I2CReadRegs(Device, Reg, &dat, 1);
  return dat;
}

void I2CWriteRegs(uint8_t Device, uint8_t Reg, uint8_t *TxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  char i;
  Wire.beginTransmission(Device); // transmit to device (note: actually just starts filling of buffer)
  Wire.write(Reg);              // sends one byte 
  for(i=0;i<Length;i++){
    Wire.write(TxBuffer[i]); 
  }
  Wire.endTransmission();    //send data with stop at end 
}

void I2CWriteReg(uint8_t Device, uint8_t Reg, uint8_t TxData){//Ver. 1.0, Dustin Soodak
  I2CWriteRegs( Device, Reg, &TxData, 1);
}

// ***************************************************
// end Ringo I2C
// ***************************************************




// ***************************************************
// Navigation
// ***************************************************

float GyroscopeCalibrationMultiplier=GYROSCOPE_CALIBRATION_MULTIPLIER_DEFAULT;

char IsStationary=1;//used in NavigationXY()
int NonStationaryValue=0;
char NonStationaryAxis=0;
char IsStationaryCount=0;


char XYMode=0;//added to tell GetAccelerationX(), etc. which navigation function was called: SimpleNavigation() or NavigationXY().

char GyroFifoOverflow=0;
int GyroZeroes[3]={0,0,0};
char AccelFifoOverflow=0;
int AccelZeroes[3]={0,0,0};
char NavigationOutOfRange=0;

char PauseNavigationInterrupt=1;
char NavigationOn=0;
int32_t count;

extern volatile char IRReceiving;
int nav_data[3];//made global so compiler doesn't optimize it out of code
int nav_accel[3];
int nav_data3[3];

signed char GyroEdgeDetection=0;
int GyroEdgeDetectionLevel=100;
char GyroEdgeDetectionRepeats=10;
char GyroEdgeDetected;


void ZeroGyroEdgeDetection(void){//Ver. 1.0, Dustin Soodak
  //works but already too late by the time edge is detected
  GyroEdgeDetection=0;
}

void UpdateGyroEdgeDetection(int GyroYAxisRawZeroed){//Ver. 1.0, Dustin Soodak
  //works but already too late by the time edge is detected
  if(!GyroEdgeDetected){
    if(GyroYAxisRawZeroed>10){
      if(GyroEdgeDetection>=0) 
        GyroEdgeDetection=-1;
      else
        GyroEdgeDetection--;
      if(GyroEdgeDetection<-GyroEdgeDetectionLevel){
        GyroEdgeDetection=0;//so ready for next time when GyroEgeDetected set to 0
        GyroEdgeDetected=2;//bit 1: left
      }
    }
    else if(GyroYAxisRawZeroed<-10){
      if(GyroEdgeDetection<=0) 
        GyroEdgeDetection=1;
      else
        GyroEdgeDetection++;
      if(GyroEdgeDetection>GyroEdgeDetectionLevel){
        GyroEdgeDetection=0;//so ready for next time when GyroEgeDetected set to 0
        GyroEdgeDetected=1;//bit 0: right
      }
    }
  }//end if(!GyroEdgeDetected)
}//end UpdateGyroEdgeDetection

void SimpleGyroNavigation(void){//Ver. 1.0, Dustin Soodak
  char n;
  //int nav_data[3];//Compiler error:  when nav_data[] local, it was not actually reserving space for this since it isn't explicitly set (is set by pointers)
  char i,j;
  ConvertNavigationCoordinates(0);
  if(IRReceiving){
    if(IsIRDone())
     IRReceiving=0; 
  }
  if(!PauseNavigationInterrupt && !IRReceiving){
    
    //Get Gyroscope data
    n=GyroBufferSize();
    //if(n>0)
    //digitalWrite(Light_Bus_BTN1,0);
    for(i=0;i<n;i++){
      GyroGetAxes(nav_data);
      //digitalWrite(Light_Bus_BTN1,0);
      for(j=1;j<3;j++){//just y & z axis      
        GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
        GyroPosition[j]+=(GyroVelocity[j]);
      }    
      //UpdateGyroEdgeDetection(GyroVelocity[1]);//works but already too late by the time edge is detected

    }//end for(i=0;i<n;i++)
    //get current rotational velocity for x & y axes
    for(j=0;j<2;j++){
      GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
    }  
    if(n>=31)
      GyroFifoOverflow=1;    
  } 
}
int32_t AverGyroVelocity;

int AccelPositionXOffset=0,AccelPositionYOffset;
//Can remove this feature by commenting out body of
//this function(replace with "XYMode=NewXYMode;")
//and removing AccelPositionYOffset from GetPositionY().
void ConvertNavigationCoordinates(char NewXYMode){//Ver. 1.0, Dustin Soodak
  float Theta,conversion;
  if(XYMode==(!NewXYMode)){
    Theta=((float)90-GetDegrees())*3.14159/180;//(((float)GyroPosition[2])*((0.0000355*2000))*3.14159/180);
    conversion=(-((float)32768)*380*380/(2*9800));//since GetPositionX() in 
    //XYMode is ((float)AccelPosition[0])*(-(float)(2*9800)/32768/380/380)+AccelPositionXOffset;
    //GetPositionY() is same in XYMode but is ((float)AccelPosition[1])*(-(float)(2*9800)/32768/400) 
    //when using SimpleNagivation()
    AccelPositionXOffset=cos(Theta)*ACCEL_DIST;
    AccelPositionYOffset=sin(Theta)*ACCEL_DIST;
    if(NewXYMode){
      AccelPosition[1]=((float)AccelPosition[1])*(((float)380)*380/400);//scale to XYMode
      AccelPosition[1]+=((float)AccelPositionYOffset)*conversion;//add offset    
    }
    else{
      AccelPosition[1]-=((float)AccelPositionYOffset)*conversion;//get rid of offset
      AccelPosition[1]=((float)AccelPosition[1])*(((float)400)/380/380);//scale to Simple mode
    }
    XYMode=NewXYMode;
  }//if XY mode changed
  
}


void SimpleNavigation(void){//Ver. 1.0, Dustin Soodak
  //can't be put in interrupt since I2C functions use an interrupt(update: new I2C functions don't use an interrupt)
  char i,j,n,gn,an;
  //int nav_data[3];//Compiler error:  when nav_data[] local, it was not actually reserving space for this since it isn't explicitly set (is set by pointers)
  int32_t AccelPositionNewDat[3]={0,0,0};
  int32_t AccelVelocityNewDat[3]={0,0,0};
  int32_t GyroPositionNewDat[3]={0,0,0};
  AverGyroVelocity=0;
  count++;
  ConvertNavigationCoordinates(0);
    
  
  if(IRReceiving){
    if(IsIRDone())
     IRReceiving=0; 
  }
  if(!PauseNavigationInterrupt && !IRReceiving){
        
    //Get Gyroscope data
    gn=GyroBufferSize();
    
    for(i=0;i<gn;i++){
      GyroGetAxes(nav_data);
      
      for(j=1;j<3;j++){//just y,z axis      
        GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
        GyroPosition[j]+=(GyroVelocity[j]);
      }
      //UpdateGyroEdgeDetection(GyroVelocity[1]);//works but already too late by the time edge is detected
      
      AverGyroVelocity+=abs(GyroVelocity[2]);
            
    }
    //get current rotational velocity for x & y axes
    for(j=0;j<2;j++){
      GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
    }
    AverGyroVelocity/=gn;
    if(gn>=31)
      GyroFifoOverflow=1;
    
    //Get Accelerometer data
    an=AccelBufferSize();
    for(i=0;i<an;i++){
      AccelGetAxes(nav_accel);
      //digitalWrite(Light_Bus_BTN1,0);
      /*
      //less efficient but easier to understand:
      for(j=0;j<2;j++){   //just x and y axis
        AccelAcceleration[j]=((int32_t)nav_accel[j])-AccelZeroes[j];  
        AccelVelocity[j]+=AccelAcceleration[j];        
        AccelPosition[j]+=AccelVelocity[j]/400;//so in range and same as velocity
      }      
      */
      //more efficient:
      for(j=0;j<3;j++)//get rav values for all 3 in case another function wants to reference them
        AccelAcceleration[j]=((int32_t)nav_accel[j])-AccelZeroes[j];
      for(j=1;j<2;j++){   //just y axis
        //AccelAcceleration[1]+=(10*(AverGyroVelocity))/8;//assuming that it always rotates about the same point (around where wheels are), experimentally putting this in
        AccelVelocity[j]+=AccelAcceleration[j];        
        AccelPositionNewDat[j]+=AccelVelocity[j];//so in range and same as velocity
      }
      if(i==an-1){
        for(j=1;j<2;j++){//just y axis
          AccelPosition[j]+=AccelPositionNewDat[j]/400;
        }
      }      
    }//end for(i=0;i<n;i++
     //get current acceleration for x,y, and z axes
    if(an>0){
      for(j=0;j<3;j++){
        AccelAcceleration[j]=((int32_t)nav_accel[j])-AccelZeroes[j];
      }
    }
    if(an>=31)
      AccelFifoOverflow=1;
  }
  
  
}//end NavigationHandler()


#ifdef INT_VER
const int SinFunctionTable[]={0,286,572,857,1143,1428,1713,1997,2280,2563,2845,3126,3406,3686,3964,4240,4516,4790,5063,5334,5604,5872,6138,6402,6664,6924,7182,7438,7692,7943,8192,8438,8682,8923,9162,9397,9630,9860,10087,10311,10531,10749,10963,11174,11381,11585,11786,11982,12176,12365,12551,12733,12911,13085,13255,13421,13583,13741,13894,14044,14189,14330,14466,14598,14726,14849,14968,15082,15191,15296,15396,15491,15582,15668,15749,15826,15897,15964,16026,16083,16135,16182,16225,16262,16294,16322,16344,16362,16374,16382,16384};
int SineFunction(int degr){//Ver. 1.0, Dustin Soodak
  int sign=1;
  if(degr<0){
    sign=-1;
    degr=-degr;
  }
  if(degr>360)
    degr%=360;
  if(degr<=90)
    return sign*SinFunctionTable[degr];
  else if(degr<=180)
    return sign*SinFunctionTable[180-degr];
  else if(degr<=270)
    return -sign*SinFunctionTable[degr-180];
  else
    return -sign*SinFunctionTable[270-degr];
}
int CosineFunction(int degr){//Ver. 1.0, Dustin Soodak
   if(degr<0)
     degr=-degr;
   if(degr>360)
     degr%=360;
   if(degr<=90)
     return SinFunctionTable[90-degr];
   else if(degr<=180)
     return -SinFunctionTable[degr-90];
   else if(degr<=270)
     return -SinFunctionTable[270-degr];
   else
     return SinFunctionTable[360-degr];
}
#endif //end #ifdef INT_VER

extern int GyroVelocityZPrev;//(defined below)
//int32_t accelxraw,accelyraw;
//int CosDegr,SinDegr;
int32_t N_accelx,N_accely;//made global so compiler doesn't "optimize" them out.


//Notes on NavigationXY()
//
//This assumes flat level surface and no tilting forwards & backwards.
//Recently added code to account (in GetPositionX/Y() functions) for 
//fact that accelerometer is actually 3cm in front of the rotation point
//of the robot. 
//
//Possible improvements:
//
//I created a couple of versions of NavigationXYTilt() which were supposed to
//take forward/backward tilt into account (since gravity is a much stronger force
//than the motors can produce, even a degree or 2 tilt can make a huge difference)
//but the drift of x-axis of the gyroscope ended up being too big to simply plug
//its value into the "degrees of tilt" variable. Since the accelerometer is in 
//front it might also experience negative acceleration from the robot vibrating
//along its x-axis (though I haven't measured this directly so don't know if
//it is actually significant).
//
//Another possible improvement is to take into account that this is a wheeled
//vehicle and not a hovercraft. If the robot is swerving slightly to the 
//right & left as it travels, then it should be possible to correct for
//drift in calculated velocity since the sideways force exerted as it turns
//should be proportional to its forward velocity. Of course, the first step
//must be to calculate what the sideways acceleration is at a point 30mm 
//behind the accelerometer(I currently only have POSITION corrected for in
//this way...acceleration might be more complicated and has to be done real-time). 
//
//If you do plan on making functions that directly access gyroscope and
//accelerometer functions, keep in mind that the vibrations from movement
//are much larger than the general linear or rotational acceleration of the
//robot as a whole. Also: the functions in the "Recorded Data" section of
//RingoHardware.h might be useful.
//Dustin Soodak
//

void NavigationXY(int GyroSensitivity, int AccelSensitivity){  //Ver. 1.0, Dustin Soodak
  //Note: (50,800) are good input values
  signed char i,j,n,gn,an,m;
  int32_t N_accelxraw,N_accelyraw;//32 bit since zeroes being subtracted might make a negative value out of range
  static int32_t N_accelxrawprev,N_accelyrawprev;//,N_gyrorawprev;
    
  #ifdef FLOAT_VER
  float Theta,CosTheta,SinTheta;
  #endif
  #ifdef INT_VER
  //int N_accelxdif,N_accelydif,N_gyrodif;
  //int32_t N_accelx,N_accely;
  int degr;
  int CosDegr,SinDegr;
  #endif
  ConvertNavigationCoordinates(1);
  count++;
  if(IRReceiving){
    if(IsIRDone())
     IRReceiving=0; 
  }
  if(!PauseNavigationInterrupt && !IRReceiving){
        
    gn=GyroBufferSize();              //Reads how many buffer positions are full in the Gyro (the Gyro has 32 buffer 
                                      //positions/readings and must be read prior to reaching 32 to avoid inaccurate calculation)
    an=AccelBufferSize();             //Reads how many buffer positions are full in the Accelerometer (the Accel has 32 buffer
                                      //positions/readings and must be read prior to reaching 32 to avoid inaccurate calculation)
    
    if(an>gn)                         //Because the accel reads 400 Hz and gryo reads 380 Hz, this section keeps them in sync by
      m=gn/(an-gn);                   //occasionally averaging 2 consecutive readings from the accel
    else
      m=99;  
    //digitalWrite(Light_Bus_BTN1,0);                               //<-- uncomment to spit values out the serial debug
    for(i=0;i<gn;i++){      
      GyroGetAxes(nav_data);            //380 hz       
      if(an)//generally, an>=gn, unless only one reading has been received
        AccelGetAxes(nav_accel);          //400 hz
      //otherwise, use last reading
      //Serial.print(i);Serial.print(" ");Serial.print(gn);Serial.print(an);Serial.println(m,DEC);    //<-- uncomment to spit values out the serial debug
      if(((i+1)%m)==0){                 //to keep buffers in sync
         AccelGetAxes(nav_data3);
         for(j=0;j<3;j++){
            nav_accel[j]=(((int32_t)nav_accel[j])+nav_data3[j])/2;
         }
      }
      
      
      N_accelxrawprev=N_accelxraw;//Used in code that makes sure sensor drift or small 
      N_accelyrawprev=N_accelyraw;//changes in orientation don't make it think its moving.
      N_accelxraw=((int32_t)nav_accel[0])-AccelZeroes[0];//non-rotated value
      N_accelyraw=((int32_t)nav_accel[1])-AccelZeroes[1];//non-rotated value
      
      GyroVelocity[2]=((nav_data[2]-GyroZeroes[2])); 
      
      //UpdateGyroEdgeDetection(nav_data[1]-GyroZeroes[1]);//works but already too late by the time edge is detected
      #ifdef INT_VER                                               //INT_VER should always be used.  It takes about 300 uS to run as it does tricks
                                                                   //to do the math using integer values rather than float values.  The float version
                                                                   //takes about 1000 uS to run.
      //see N_accelxYGData tab of Ringo spreadsheet
      //average .7276ms with all integer operations except for theta/sin/cos
      //average .3272ms with lookup array trig functions SinFunction() and CosFunction()
      //RestartTimer();for(i=0;i<100;i++){                         //<-- uncomment this line and "Serial.println(GetTime(),DEC);" below to measure time to run this function
       
      GyroPosition[2]+=GyroVelocity[2];
      GyroAccelerationZ=GyroVelocity[2]-GyroVelocityZPrev;
      GyroVelocityZPrev=GyroVelocity[2];
      degr=(((float)GyroPosition[2])*(0.0000355*2000/380));//proper direction but pi/2 offset
      SinDegr=SineFunction(degr);//integer trig functions scaled by 0x4000
      CosDegr=CosineFunction(degr);//integer trig functions scaled by 0x4000
      
      N_accelx=(N_accelxraw*CosDegr-N_accelyraw*SinDegr)/0x4000;//rotated value
      N_accely=(N_accelyraw*CosDegr+N_accelxraw*SinDegr)/0x4000;//rotated value
      
      //AccelDistance+=sqrt(((AccelVelocity[0]/380*AccelVelocity[0]/380)+(AccelVelocity[1]/380*AccelVelocity[1]/380))/380);
      //Serial.print(nav_accel[0]);Serial.print("\t");Serial.print(nav_accel[1]);Serial.print("\t");Serial.println(nav_data[2]);//remember: motors must be switched to serial to use this
      //}  Serial.println(GetTime(),DEC);  //<-- uncomment this line and "RestartTimer();for(i=0;i<100;i++){" above to measure time to run function & echo out serial
      #endif//end #ifdef INT_VER 
      //
      //
      #ifdef FLOAT_VER  //kep around for reference so INT_VER is easier to understand
      //average: .973ms
      //RestartTimer();for(i=0;i<100;i++){     
         
      GyroPosition[2]+=((float)(GyroVelocity[2]))/380;//GyroRawToDegreesMult
      Theta=GyroPosition[2]*((0.0000355*2000)*3.14159/180);//see N_accelxYGData tab of Ringo spreadsheet
      SinTheta=sin(Theta);
      CosTheta=cos(Theta);      
      
      N_accelx=(N_accelxraw*CosTheta-N_accelyraw*SinTheta);//rotated value
      N_accely=(N_accelyraw*CosTheta+N_accelxraw*SinTheta);//rotated value
      
      //}Serial.println(GetTime(),DEC);
      #endif//end #ifdef FLOAT_VER
      
      if(abs(N_accelxraw)>AccelSensitivity || abs(N_accelyraw)>AccelSensitivity || abs(GyroVelocity[2])>GyroSensitivity){//is moving (should catch this immediately)      
        if(abs(N_accelxrawprev-N_accelxraw)>AccelSensitivity || abs(N_accelyrawprev-N_accelyraw)>AccelSensitivity){
          if(IsStationary){     
            NonStationaryValue=(abs(N_accelxraw)>AccelSensitivity?N_accelxraw:abs(N_accelyraw)>AccelSensitivity?N_accelyraw:GyroVelocity[2]);
            NonStationaryAxis=(abs(N_accelxraw)>AccelSensitivity?0x1:abs(N_accelyraw)>AccelSensitivity?0x2:0x4);
          }//what if gyro but not accel?
          IsStationaryCount=0;
          IsStationary=0;  
      }
      }
      else{//doesn't appear to be changing its velocity or orientation very much
        if(!IsStationary){
          if(IsStationaryCount<10){
            IsStationaryCount++;
          }
          else{//definitely not moving if still of 10 consecutive readings
            IsStationary=1;
            AccelVelocity[0]=0;
            AccelVelocity[1]=0; 
          }
        }
      }
      if(IsStationary==0){
        AccelVelocity[0]+=N_accelx;
        AccelVelocity[1]+=N_accely; 
        AccelPosition[0]+=AccelVelocity[0];            
        AccelPosition[1]+=AccelVelocity[1];
        //Note: theta has a Pi/2 offset
        #ifdef INT_VER
        AccelPositionXOffset=-((int32_t)ACCEL_DIST)*SinDegr/0x4000;
        AccelPositionYOffset=((int32_t)ACCEL_DIST)*CosDegr/0x4000;
        #endif
        #ifdef FLOAT_VER
        AccelPositionXOffset=-SinTheta*ACCEL_DIST;
        AccelPositionYOffset=CosTheta*ACCEL_DIST;
        #endif
      }
      
    }//end for(i=0;i<gn;i++)
   
    if(an>0){ 
      #ifdef INT_VER     
      AccelAcceleration[0]=N_accelx;      
      AccelAcceleration[1]=N_accely;   
      #endif
      AccelAcceleration[2]=((int32_t)nav_accel[2])-AccelZeroes[2];
    }
    
    //digitalWrite(Light_Bus_BTN1,1);
    if(an>=31)
      AccelFifoOverflow=1;
    if(gn>=31)
      GyroFifoOverflow=1;  
      
  }//end if(!PauseNavigationInterrupt && !IRReceiving)
  
}//end NavigationXY()

// calibrates out stationary drift in sensors. MUST BE RUN WHEN RINGO IS PERFECTLY STILL!!
void CalibrateNavigationSensors(void){//Ver. 1.0, Dustin Soodak
  char i,j,n,prev;
  int32_t totals[3];
  //int nav_data[3];//Compiler error:  when nav_data[] local, it was not actually reserving space for this since it isn't explicitly set (is set by pointers)
  prev=PauseNavigationInterrupt;
  PauseNavigationInterrupt=1;//PauseNavigationInterrupt=1;//noInterrupts();
  //Gyro average to find zero value (assuming not currently moving)
  for(i=0;i<3;i++)
    totals[i]=0;
  i=1000;
  while(GyroBufferSize()<20 && i)i--; 
  for(i=0;i<20;i++){
    GyroGetAxes(nav_data);
    for(j=0;j<3;j++){
      totals[j]+=nav_data[j];
    }    
  }
  for(i=0;i<3;i++){
    GyroZeroes[i]=totals[i]/20;
  }
  //Accel average to find zero value (assuming not currently moving)
  for(i=0;i<3;i++)
    totals[i]=0;
  i=1000;
  while(AccelBufferSize()<20 && i)i--; 
  for(i=0;i<20;i++){
    AccelGetAxes(nav_accel);
    for(j=0;j<3;j++){
      totals[j]+=nav_accel[j];
    }    
  }
  for(i=0;i<3;i++){
    AccelZeroes[i]=totals[i]/20;
    AccelVelocity[i]=0;//assumes it isn't moving when this function called
  }  
  //Clear buffers
  n=AccelBufferSize();
  for(i=0;i<n;i++){
    AccelGetAxes(nav_data);
  }
  n=GyroBufferSize();
  for(i=0;i<n;i++){
    GyroGetAxes(nav_data);
  }
  AccelFifoOverflow=0; 
  PauseNavigationInterrupt=prev;//interrupts();//PauseNavigationInterrupt=0;
  for(i=0;i<3;i++){
    AccelVelocity[i]=0;
  }   
  
  //Zero sensors with arcsin:
  //9800*sin(GyroXRaw*((0.0000355*2000/380)*3.14159/180))=AccelYRaw*(-(float)(2*9800)/32768
  //9800*sin(GyroPosition[0]*((0.0000355*2000/380)*3.14159/180))=AccelZeroes[1]*(-(float)(2*9800)/32768
  //GyroPosition[0]=-arcsin(AccelZeroes[1]*((float)(2)/32768))/((0.0000355*2000/380)*3.14159/180)
  //Zero sensors with arctan:
  //atan2(x,y) (instead of y,x) so theta of 90 degrees -> 0 degrees.
  //GyroPosition[0]=-atan2(AccelZeroes[1],AccelZeroes[2])/((0.0000355*2000/380)*3.14159/180);
  //Calculate new:
  //Theta=GyroPosition[0]*((0.0000355*2000/380)*3.14159/180);
  //AccelAcceleration[1]-=sin(Theta)*16384;//raw to mm/s^2 is ((float)(2*9800)/32768);
  //Since sin(theta)=theta for small theta, just need one float and no trig:
  //AccelAcceleration[1]-=GyroPosition[0]*(((0.0000355*2000/380)*3.14159/180)*16384)

  //For gyroX correction to accelY:
  GyroPosition[0]=-asin(AccelZeroes[1]*((float)(2)/32768))/((0.0000355*2000/380)*3.14159/180);
  //Serial.print(AccelZeroes[1],DEC);Serial.print(" ");Serial.println(AccelZeroes[2]);
  //Serial.println(atan2(AccelZeroes[1],AccelZeroes[2]));
  //Serial.println(atan2(AccelZeroes[1],AccelZeroes[2])*180/3.14159);
  
  //9800*sin(GyroPosition[0]*((0.0000355*2000/380)*3.14159/180))
  
  //Serial.print("init gyro raw after zeroed ");
  //Serial.println(GyroPosition[0],DEC);
  //Serial.print(" to degr ");
  //Serial.println(GyroRawToDegrees(GyroPosition[0]),DEC);
  
  IsStationary=1;IsStationaryCount=0;//IsMovingCount=0;
  GyroVelocityZPrev=0;
  NavigationOutOfRange=0;
  //ZeroGyroEdgeDetection();//works, but already too late by the time an edge is detected 
  //The following initializes accel in case gyro
  //reading but not accel reading so accel has an initial
  //default value for NavigationXY()
  nav_accel[0]=AccelZeroes[0];
  nav_accel[1]=AccelZeroes[1];
  //Though already set above, put here explicitly in case
  //other code changed for an unrelated reason.
  
}//end CalibrateNavigationSensors()

//re-sets Ringo's X, Y, and Heading coordinates to zeros
void ZeroNavigation(void){//Ver. 1.0, Dustin Soodak
  char i;
   //Zero total changes in position, velocity, and orientation
  for(i=0;i<3;i++){
    AccelPosition[i]=0;
    AccelVelocity[i]=0;
    if(i>0)//for part in CalibrateNavigationSensors() which 
      GyroPosition[i]=0;
  }
  XYMode=0;
  AccelDistance=0;  
  IsStationary=1;IsStationaryCount=0;//IsMovingCount=0;
  AccelFifoOverflow=0; 
  GyroFifoOverflow=0; 
  NavigationOutOfRange=0;
  AccelPositionYOffset=30;
  AccelPositionXOffset=0;
}


void PauseNavigation(void){//Ver. 1.0, Dustin Soodak
  char i;
  PauseNavigationInterrupt=1; 
  for(i=0;i<3;i++){
    AccelVelocity[i]=0;
  }
}

void ResumeNavigation(void){//Ver. 1.0, Dustin Soodak
  char i,n;
  //clear gyro buffer and save last values
  n=GyroBufferSize();
  for(i=0;i<n;i++){
    GyroGetAxes(GyroVelocity);
  }
  //subtract gyro zeroes
  for(i=0;i<3;i++){
   GyroVelocity[i]-=GyroZeroes[i];
  }
  //clear accel buffer and save last values
  n=AccelBufferSize();
  for(i=0;i<n;i++){
   AccelGetAxes(AccelAcceleration);
  }
  //subtract accel zeroes
  for(i=0;i<3;i++){
   AccelAcceleration[i]-=AccelZeroes[i];
  }
  for(i=0;i<2;i++){
   AccelVelocity[i]=0;
  }
  PauseNavigationInterrupt=0;
  IsStationary=1;IsStationaryCount=0;//IsMovingCount=0;
  GyroVelocityZPrev=0;
}

char NavigationPaused(void){//Ver. 1.0, Dustin Soodak
  return PauseNavigationInterrupt;
}

void NavigationBegin(void){ //Ver. 1.0, Dustin Soodak   
  char i;
  I2CBegin();//needs to be called before writing Gryro or Accel registers
  PauseNavigationInterrupt=1;
  //
  //GyroWriteRegister(GYR_CTRL_REG3,0x34);//open drain (bit4), watermark interrupt on DRDY/INT2 (bit2). bit5 is INT1 high, but seems to work for INT2 as well
  GyroWriteRegister(GYR_CTRL_REG3,0x10);  //0x10 eliminates excess current drain from Accel via the shared interrupt line
  GyroWriteRegister(GYR_FIFO_CTRL_REG,0x54);//watermark FIFO threshold of 20 and fifo stream mode (bits7:6)
  GyroWriteRegister(GYR_CTRL_REG1,0x0f);//all axes on(0:2) and auto-power off disabled
  GyroWriteRegister(GYR_CTRL_REG5,0x40);//fifo enable
  //
  AccelWriteRegister(ACC_CTRL_REG1,0x0);// 0x2a inactive but can set registers (bit1)
  AccelWriteRegister(ACC_CTRL_REG3,0x3);// 0x2c interrupt is OD(bit0), active high(bit1)
  AccelWriteRegister(ACC_CTRL_REG4,0x40);// 0x2d fifo interrupt enable
  //AccelWriteRegister(ACC_CTRL_REG4,0x00);
  AccelWriteRegister(ACC_CTRL_REG5,0x40);// 0x2e fifo interrupt on INT1 pin
  AccelWriteRegister(ACC_F_SETUP,0x14);// 0x9 watermark at 20 measurements
  AccelWriteRegister(ACC_CTRL_REG1,0x9);// 0x2a 400 Hz and active (bit1)
  AccelWriteRegister(ACC_F_SETUP,0x54);// 0x09 circular buffer mode (now at 0x54=01010100)
  //
  
  GyroSetFrequency(380);
  GyroSetRange(2000);

  delay(200);                         // let nav sensors buffer fill some before calibrating
  CalibrateNavigationSensors();
  ZeroNavigation();
  
  pinMode(Accel_Interrupt, INPUT_PULLUP);
  
  NavigationOn=1;
  PauseNavigationInterrupt=0;  
  
}//end void NavigationBegin(void)


int PresentHeading(void){//Ver. 1.0, Kevin King
  return -GyroRawToDegrees(GyroPosition[2]);// z axis
}
int GetDegrees(void){//Ver. 1.0, Dustin Soodak
  return -GyroRawToDegrees(GyroPosition[2]);// z axis 
}
int GetDegreesX(void){//Ver. 1.0, Dustin Soodak
  return GyroRawToDegrees(GyroPosition[0]);// x axis
}
int GetDegreesPerSecond(void){//Ver. 1.0, Dustin Soodak
  return -GyroRawToDegreesPerSec(GyroVelocity[2]);// z axis
}
int GetDegreesPerSecondX(void){//Ver. 1.0, Dustin Soodak
  return GyroRawToDegreesPerSec(GyroVelocity[0]);
}
int GetDegreesPerSecondY(void){//Ver. 1.0, Dustin Soodak
  return GyroRawToDegreesPerSec(GyroVelocity[1]);
}
int GetDegreesToStop(void){//Ver. 1.0, Dustin Soodak
  return -GyroDegreesToStopFromRaw(GyroVelocity[2]);
}

int GetAccelerationX(void){//Ver. 1.0, Dustin Soodak
  return ((float)AccelAcceleration[0])*(-(float)(2*9800)/32768);
}
int GetAccelerationY(void){//Ver. 1.0, Dustin Soodak
  return ((float)AccelAcceleration[1])*(-(float)(2*9800)/32768);
}
int GetAccelerationYUnZeroed(void){//Ver. 1.0, Dustin Soodak
  //for front/back tilt analysis
  return ((float)AccelAcceleration[1]+AccelZeroes[1])*(-(float)(2*9800)/32768);
}
int GetAccelerationZ(void){//Ver. 1.0, Dustin Soodak
  return ((float)AccelAcceleration[2])*(-(float)(2*9800)/32768);
}
//The following 4 functions have different cases depending on if 
//NavigationXY() or one of the simpler functions are used.
int GetVelocityX(void){//Ver. 1.0, Dustin Soodak
  if(XYMode)
    return ((float)AccelVelocity[0])*(-(float)(2*9800)/32768/380);
  else
    return 0;
}
int GetVelocityY(void){//Ver. 1.0, Dustin Soodak
  if(XYMode)
    return ((float)AccelVelocity[1])*(-(float)(2*9800)/32768/380);  
  else
    return ((float)AccelVelocity[1])*(-(float)(2*9800)/32768/400);  
}
int GetPositionX(void){//Ver. 1.0, Dustin Soodak
  //SimpleNavigation() and SimpleGyroNavigation() leave X unchanged, so decided
  //not to have an XYMode==0 option.
  //if(XYMode)
    return ((float)AccelPosition[0])*(-(float)(2*9800)/32768/380/380)-AccelPositionXOffset;
  //else
    //return ((float)AccelPosition[0])*(-(float)(2*9800)/32768/380/380);
}
int GetPositionY(void){//Ver. 1.0, Dustin Soodak
  if(XYMode)  //put these sections back to use auto offset
    return ((float)AccelPosition[1])*(-(float)(2*9800)/32768/380/380)-AccelPositionYOffset;
  else
    return ((float)AccelPosition[1])*(-(float)(2*9800)/32768/400);//note: already an extra "/400" in handler to keep in range (400 for 400hz)

}
         
void DelayWithSimpleNavigation(int ms){//Ver. 1.0, Dustin Soodak
  int32_t total=millis()+ms;
  while(millis()<total){
    SimpleNavigation();
  }
}

void DelayWithNavigation(int ms){//Ver. 1.0, Dustin Soodak
  int32_t total=millis()+ms;
  while(millis()<total){
    NavigationXY(80,800);
  }
}

// ***************************************************
// end Navigation
// ***************************************************

// ***************************************************
// Accelerometer
// ***************************************************

//Accelerometer code based on code by Jim Lindblom
//https://github.com/sparkfun/MMA8452_Accelerometer/tree/master/Firmware

#define AccelAddr 0x1C

int32_t AccelPosition[3]={0,0,0};//running total
int32_t AccelDistance=0;
int32_t AccelVelocity[3]={0,0,0};//running total
int AccelAcceleration[3]={0,0,0};

uint8_t AccelReadRegisters(uint8_t Reg, uint8_t *RxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  return I2CReadRegs(AccelAddr,0x80|Reg,RxBuffer,Length);//"In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddress field"
}
uint8_t AccelReadRegister(uint8_t Reg){//Ver. 1.0, Dustin Soodak
  return I2CReadReg(AccelAddr,Reg);
}
void AccelWriteRegisters(uint8_t Reg, uint8_t *TxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
   I2CWriteRegs(AccelAddr, Reg, TxBuffer, Length);
}
void AccelWriteRegister(uint8_t Reg, uint8_t TxData){//Ver. 1.0, Dustin Soodak
  I2CWriteReg(AccelAddr, Reg, TxData);
}
//Put in standby to change register settings
void AccelStandby(void){//Ver. 1.0, Dustin Soodak
  byte c = AccelReadRegister(ACC_CTRL_REG1);
  AccelWriteRegister(ACC_CTRL_REG1, c & ~(0x01));
}
//Needs to be in this mode to output data
void AccelActive(void){//Ver. 1.0, Dustin Soodak
  byte c = AccelReadRegister(ACC_CTRL_REG1);
  AccelWriteRegister(ACC_CTRL_REG1, c | 0x01);
}

uint8_t AccelBufferSize(void){//Ver. 1.0, Dustin Soodak
  return AccelReadRegister(ACC_F_STATUS)&0x3F;
}
int AccelGetAxis(char Axis){//Ver. 1.0, Dustin Soodak
  //Axis=0,1,2 (see p.22)
  uint8_t ar[2];
  int16_t val;//OUT_X_MSB, etc.
  if(Axis>2)
    Axis=2;
  AccelReadRegisters((ACC_OUT_X_MSB+2*Axis),ar,2);
  val=(ar[0]>>2)+(((uint8_t)(ar[1]&0x7F))<<6)+(((uint8_t)(ar[1]&0x80))<<8);
  return val;
}
void AccelGetAxes(int *Axes){//Ver. 1.0, Dustin Soodak
  //Axis=0,1,2 (see p.22) [int is 16 bit in adruino]
  uint8_t ar[6];
  int i;
  AccelReadRegisters((ACC_OUT_X_MSB),ar,6);
  for(i=0;i<6;i+=2){
     //Axes[i>>1]=(int)(ar[i]>>2)+(((uint8_t)(ar[i+1]&0x7F))<<6)+(((uint8_t)(ar[i+1]&0x80))<<8);
     Axes[i>>1]=(((signed short)ar[i])<<8)+ar[i+1];//((((unsigned int)ar[i])&0x10)<<8)|((((unsigned int)ar[i])&0x7F)<<6);
  }
}

//Serial.print(" v= ");Serial.print(-(GetVelocityRaw(1))*(2*9800)/(32768)/400,DEC);  // debugging code
//Serial.print(" p= ");Serial.println(-(GetPositionRaw(1))*(2*9800)/(32768)/400/* /400 in handler instead*/,DEC);  // debugging code

// ***************************************************
// end Accelerometer
// ***************************************************

// ***************************************************
// Gyro
// ***************************************************

//Note: referenced code from "L3G4200D 3-axis gyro example code" by Jim Lindblom at
//https://www.sparkfun.com/products/10612

int32_t GyroPosition[3]={0,0,0};//running total
int GyroVelocity[3]={0,0,0};
int GyroAccelerationZ=0;//rate of change in angular speed (degrees pre second)
int GyroVelocityZPrev=0;//used in NavigationXY() to calculate GyroAccelerationZ

#define GyroAddr 0x6B
#define GR_250dps 0x00
#define GR_500dps 0x10
#define GR_2000dps 0x20
int GyroRange=250;//default value for chip
#define GF_95Hz  0b00000000  //wg 20 0f
#define GF_190Hz 0b01000000  //wg 20 4f
#define GF_380Hz 0b10000000  //wg 20 8f
#define GF_760Hz 0b11000000  //wg 20 cf
int GyroFrequency=95;
float GyroRawToDegreesMult=1;
float GyroDegreesToRawMult=1;
float GyroRawToDegreesPerSecMult=1;
float GyroDegreesPerSecToRawMult=1;
float GyroRawToSkidMult=0;

uint8_t GyroReadRegisters(uint8_t Reg, uint8_t *RxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  return I2CReadRegs(GyroAddr,Reg,RxBuffer,Length);
}
uint8_t GyroReadRegister(uint8_t Reg){//Ver. 1.0, Dustin Soodak
  return I2CReadReg(GyroAddr,Reg);
}
void GyroWriteRegisters(uint8_t Reg, uint8_t *TxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
   I2CWriteRegs(GyroAddr, Reg, TxBuffer, Length);
}
void GyroWriteRegister(uint8_t Reg, uint8_t TxData){//Ver. 1.0, Dustin Soodak
  I2CWriteReg(GyroAddr, Reg, TxData);
}

uint8_t GyroBufferSize(void){//Ver. 1.0, Dustin Soodak
  return GyroReadRegister(GYR_FIFO_SRC_REG)&0x1F;
}

int16_t GyroGetAxis(char Axis){//Ver. 1.0, Dustin Soodak
  //Axis=0,1,2 (p.36)
  int16_t val;//OUT_X_L=0x28, OUT_X_H=0x29 (y and z follow)
  if(Axis>2)
    Axis=2;
  GyroReadRegisters((GYR_OUT_X_L+2*Axis)|0x80,(uint8_t*)val,2);//"In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddressfield."
  return val;
}
void GyroGetAxes(int *Axes){//Ver. 1.0, Dustin Soodak
//Axis=0,1,2
  //OUT_X_L=0x28, OUT_X_H=0x29 (y and z follow)
  GyroReadRegisters((GYR_OUT_X_L)|0x80,(uint8_t*)Axes,6);//"In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddressfield."
}

//gyro:
//range	calc dps/dig	"typ" in datasheet	to get to "typ"		
//250	0.007629395	0.00875           	1.14687993         
//500	0.015258789	0.0175           	1.146880005
//2000	0.061035156	0.070            	1.146880005

void UpdateGyroConversionVars(void){//Ver. 1.0, Dustin Soodak
  GyroRawToDegreesPerSecMult=((float)GyroRange)*0.0000355/GyroscopeCalibrationMultiplier;// 1/2^15=1/32768=0.000030517578125      // 1/2^15*1.14688=exactly .000035
  GyroRawToDegreesMult=GyroRawToDegreesPerSecMult/GyroFrequency;
  GyroDegreesPerSecToRawMult=((float)28169)/GyroRange*GyroscopeCalibrationMultiplier; //2^15=32768                             //1/0.000035=28571.42857
  GyroDegreesToRawMult=GyroDegreesPerSecToRawMult*GyroFrequency;    
  GyroRawToSkidMult=GyroRawToDegreesPerSecMult*0.1029;//used in GyroDegreesToStopFromRaw()
}

float EEPROM_CalMultiplier;
void GetGyroCalibrationMultiplier(void){//Ver. 1.0, Kevin King   //Gets gyro cal mult from EEPROM if present
  EEPROM_readAnything(GYRO_CAL_MULT_ADDRESS, EEPROM_CalMultiplier);
  if((EEPROM_CalMultiplier >= 0.85) && (EEPROM_CalMultiplier <= 1.15)){
  GyroscopeCalibrationMultiplier=EEPROM_CalMultiplier; //use calibration value if within bounds
  }
}

void GyroSetRange(int Range){//Ver. 1.0, Dustin Soodak
  char RangeByte;
  if(Range==2000)
    RangeByte=GR_2000dps;
  else if(Range==500)
    RangeByte=GR_500dps;
  else
    RangeByte=GR_250dps;
  GyroWriteRegister(GYR_CTRL_REG4,RangeByte);
  //FS=250dps: 8.75 mdps/digit
  //   500:    17.5
  //   2000:   70 
  GyroRange=Range;
  UpdateGyroConversionVars();    
}

int GyroGetRangeFromChip(void){//Ver. 1.0, Dustin Soodak
  char d=GyroReadRegister(GYR_CTRL_REG4)&0x30;
  if(d==GR_250dps)
    return 250;
  else if(d==GR_500dps)
    return 500;
  else if(d==GR_2000dps)
    return 2000;
  else 
    return 250;
}

int GyroGetFrequencyFromChip(void){//Ver. 1.0, Dustin Soodak
  char d=GyroReadRegister(GYR_CTRL_REG1)&0xC0;
  if(d==GF_95Hz)
    return 95;
  else if(d==GF_190Hz)
    return 190;
  else if(d==GF_380Hz)
    return 380;
  else if(d==GF_760Hz)
    return 760;  
  else
    return 95;
}

//GyroSetFrequencyByte(GyroReadRegister(GYR_CTRL_REG1)&0xC0);
//GyroSetRangeByte(GyroReadRegister(GYR_CTRL_REG4)&0x30); 

void GyroSetFrequency(int Frequency){//Ver. 1.0, Dustin Soodak
  char FrequencyByte;
  char r=GyroReadRegister(GYR_CTRL_REG1);  
  if(Frequency==190)
    FrequencyByte=GF_190Hz;
  else if(Frequency==380)
    FrequencyByte=GF_380Hz;
  else if(Frequency==760)
    FrequencyByte=GF_760Hz;
  else if(Frequency==95)
    FrequencyByte=GF_95Hz;
  GyroWriteRegister(GYR_CTRL_REG1,FrequencyByte|(r&~0b11000000));
  GyroFrequency=Frequency;
  UpdateGyroConversionVars();
}


int32_t GyroDegreesToRaw(int Degrees){//Ver. 1.0, Dustin Soodak
  int32_t raw=1;
  return GyroDegreesToRawMult*Degrees;
  /*if(GyroRangeByte==GR_250dps){
    raw=-Degrees*100000/875;
    //Serial.print("_250_");
  }
  else if(GyroRangeByte==GR_500dps){
    //Serial.print("_500_");
    raw=-Degrees*10000/175;
  }
  else if(GyroRangeByte==GR_2000dps){
    raw=-Degrees*1000/70;
    //Serial.print("_2000_");
  }
  if(GyroFrequencyByte==GF_95Hz)
    return raw*95;
  else if(GyroFrequencyByte==GF_190Hz)
    return raw*190;
  else if(GyroFrequencyByte==GF_380Hz)
    return raw*380;
  else if(GyroFrequencyByte==GF_760Hz){
    //Serial.print("_760_");
    return raw*760;
  }
  else
    return 0;
  */
}
int GyroDegreesPerSecToRaw(int Degrees){//Ver. 1.0, Dustin Soodak
  return Degrees*GyroDegreesPerSecToRawMult;  
}
int GyroRawToDegrees(int32_t Raw){//Ver. 1.0, Dustin Soodak
  int32_t deg=1;
  return Raw*GyroRawToDegreesMult;
  /*
  if(GyroRangeByte==GR_250dps)
    deg=-Raw*875/100000;
  else if(GyroRangeByte==GR_500dps)
    deg=-Raw*175/10000;
  else if(GyroRangeByte==GR_2000dps){
    deg=-Raw*70/1000;
  }
  if(GyroFrequencyByte==GF_95Hz)
    return deg/95;
  else if(GyroFrequencyByte==GF_190Hz)
    return deg/190;
  else if(GyroFrequencyByte==GF_380Hz)
    return deg/380;
  else if(GyroFrequencyByte==GF_760Hz)
    return deg/760;
  else
    return 0;
  */
}

int GyroRawToDegreesPerSec(int Raw){//Ver. 1.0, Dustin Soodak
  return Raw*GyroRawToDegreesPerSecMult;  
}

int GyroDegreesToStopFromRaw(int DegreesPerSecondRaw){//Ver. 1.0, Dustin Soodak
 int mx=((float)DegreesPerSecondRaw)*GyroRawToSkidMult;
 if(-24<=mx && mx<=24)
   return 0;
 else if(mx>24)
   return mx-24;
 else //mx<24
   return mx+24;
}

int GyroDegreesToStop(int DegreesPerSecond){//Ver. 1.0, Dustin Soodak
  int mx=((float)DegreesPerSecond)*0.1029;
  if(-24<=mx && mx<=24)
   return 0;
 else if(mx>24)
   return mx-24;
 else //mx<24
   return mx+24; 
}

// ***************************************************
// end Gyro
// ***************************************************

// ***************************************************
// Computation
// ***************************************************

int MinTurn(int ChangeInDegrees){//Ver. 1.0, Dustin Soodak
  int degr=ChangeInDegrees;
  char IsNegative=0;
  if(degr<0){
    IsNegative=1;
    degr=-degr;
  }
  if(degr>360)
      degr=degr%360;
  if(degr<=180)
    degr;
  else
    -(360-degr);
  if(IsNegative)
    degr=-degr;
  return degr;
}
int MinTurnToHeading(int DesiredHeading){//Ver. 1.0, Dustin Soodak
  return MinTurn(DesiredHeading-GetDegrees())+GetDegrees();
}

int VectorToDegrees(int x,int y){//Ver. 1.0, Dustin Soodak
  return 90-(int)(atan2(y,x)*180/3.14159265359);
}

// ***************************************************
// end Computation
// ***************************************************










