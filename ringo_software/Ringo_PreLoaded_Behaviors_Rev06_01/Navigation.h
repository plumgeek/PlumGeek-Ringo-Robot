/*

Ringo Robot:  Navigation.h  Rev01.02  12/2015

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

#include "Arduino.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

//Un-comment one for NavigationXY():
#define INT_VER // 1/3 ms run time for calculation section
//#define FLOAT_VER // 1 ms run time but slightly more accurate. mainly here to show what INT_VER is doing

#define ACCEL_DIST 30  //30 mm forward from center between wheels
#define GYROSCOPE_CALIBRATION_MULTIPLIER_DEFAULT 1.0
#define GYRO_CAL_MULT_ADDRESS 1020  //EEPROM address where Gyroscope_Calibration_Multipler is located

// ***************************************************
// Navigation
// ***************************************************
//Note: this section contains most of the navigation
//functions that will be commonly called directly by the user.

extern char AccelFifoOverflow;
extern char GyroFifoOverflow;
extern float GyroscopeCalibrationMultiplier;    //this value used to fine tune gyroscope
extern void GetGyroCalibrationMultiplier(void); //checks EEPROM for GyroCalibrationMuliplier at default address

extern void NavigationBegin(void);
extern char NavigationOn;
extern void SimpleGyroNavigation(void);
extern void SimpleNavigation(void);
extern void NavigationXY(int GyroSensitivity, int AccelSensitivity);//50,800 are good input values
//would put in interrupt, but I2C library can't be used in an interrupt and also takes more than 
//1ms so would interfere with millis() function.

extern void CalibrateNavigationSensors(void); // NavigationBegin() does this 
                                              // calibrates out stationary drift in sensors. 
                                              // MUST BE RUN WHEN RINGO IS PERFECTLY STILL!!
extern void ZeroNavigation(void); //NavigationBegin() does this
                                  //re-sets Ringo's X, Y, and Heading coordinates to zeros
extern void PauseNavigation(void);//call after NavigationBegin() if NavigationHandlerSimple() not being called immediately after
extern void ResumeNavigation(void);//clears buffers
extern char NavigationPaused(void);//lets you know if navigation is paused
extern void DelayWithNavigation(int ms);//like delay(ms) but keeps track of how XY position and orientation changes
extern void DelayWithSimpleNavigation(int ms);//like delay(ms) but keeps track of how Y position and orientation changes


extern int PresentHeading(void);  // returns present heading in positive or negative degress, does not loop at 360
extern int GetDegrees(void);      // the same as PresentHeading(), left in for backward compatibililty.
extern int GetDegreesX(void);
#define GetDegreesPerSecondZ GetDegreesPerSecond
extern int GetDegreesPerSecond(void);
extern int GetDegreesPerSecondX(void);
extern int GetDegreesPerSecondY(void);
extern int GetDegreesToStop(void);

extern int GetAccelerationX(void);// mm/sec^2
extern int GetAccelerationY(void);// mm/sec^2
extern int GetAccelerationYUnZeroed(void);//for front/back tilt analysis
extern int GetAccelerationZ(void);// mm/sec^2
extern int GetVelocityX(void);// mm/sec
extern int GetVelocityY(void);// mm/sec
extern int GetPositionX(void);// mm
extern int GetPositionY(void);// mm
extern char IsStationary;//set by NavigationXY(). 
                        //IsStationary is tripped with even one vibration (so accurate 
                        //position can be calculated), but takes several low readings to go back to 1. 
extern int NonStationaryValue;//value of xaccel,yaccel, or gyro rotation which resulted in IsStationary==0
extern char NonStationaryAxis;//bit indicating which one's value went into NonStationaryValue
//
extern int GyroEdgeDetectionLevel;//Default: 100
extern char GyroEdgeDetectionRepeats;//Set from 1 to 127. Default: 10. 
extern char GyroEdgeDetected;//bit 1: left, bit 0: right. Reset to zero after you handle the event.
extern int AccelPositionXOffset,AccelPositionYOffset;
// ***************************************************
// end Navigation
// ***************************************************


// ***************************************************
// Computation (couple of useful functions)
// ***************************************************

extern int MinTurn(int ChangeInDegrees);//ex: enter a value of 300 degrees and it will return -60
extern int MinTurnToHeading(int DesiredHeading);//ex: if PresentHeading()=180 degrees, then MinTurnToHeading(-60)=300
extern int VectorToDegrees(int x,int y);

// ***************************************************
// end Computation
// ***************************************************








// ***************************************************
// Accel
// ***************************************************

//Accelerometer read/write registers functions on code by Jim Lindblom
//https://github.com/sparkfun/MMA8452_Accelerometer/tree/master/Firmware

extern int AccelZeroes[3];
extern int32_t AccelPosition[3];//running total
extern int32_t AccelVelocity[3];//running total
extern int AccelAcceleration[3];
extern int32_t AccelDistance;

extern uint8_t AccelReadRegisters(uint8_t Reg, uint8_t *RxBuffer, uint8_t Length);
extern uint8_t AccelReadRegister(uint8_t Reg);
extern void AccelWriteRegisters(uint8_t Reg, uint8_t *TxBuffer, uint8_t Length);
extern void AccelWriteRegister(uint8_t Reg, uint8_t TxData);
//Put in standby to change register settings
extern void AccelStandby(void);
//Needs to be in this mode to output data
extern void AccelActive(void);

extern uint8_t AccelBufferSize(void);
extern int AccelGetAxis(char Axis);//Axis=0,1,2 
extern void AccelGetAxes(int *Axes);//Axis=0,1,2  [int is 16 bit in adruino]

// ***************************************************
// end Accel
// ***************************************************


// ***************************************************
// Gyro
// ***************************************************
//Gyroscope read/write register functions badsed on
//"L3G4200D 3-axis gyro example code" by Jim Lindblom at 
//https://www.sparkfun.com/products/10612
extern int GyroZeroes[3];
extern int32_t GyroPosition[3];//running total
extern int GyroVelocity[3];//raw values go here
extern int GyroAccelerationZ;//derivative of GyroVelocity[2] (z-axis)

extern int GyroRange;//default value for chip
extern int GyroFrequency;

extern uint8_t GyroBufferSize(void);
extern int16_t GyroGetAxis(char Axis);//Axis=0,1,2 (p.36)
extern void GyroGetAxes(int *Axes);//Axis=0,1,2
//
extern void GyroSetRange(int Range);
#define GyroGetRange() GyroRange
extern int GyroGetRangeFromChip(void);
extern void GyroSetFrequency(int Frequency);
#define GyroGetFrequency() GyroFrequency
extern int GyroGetFrequencyFromChip(void);
extern int32_t GyroDegreesToRaw(int Degrees);
extern int GyroDegreesPerSecToRaw(int Degrees);
extern int GyroRawToDegrees(int32_t Raw);
extern int GyroRawToDegreesPerSec(int Raw);
extern int GyroDegreesToStopFromRaw(int DegreesPerSecondRaw);
extern int GyroDegreesToStop(int DegreesPerSecond);
//
extern uint8_t GyroReadRegisters(uint8_t Reg, uint8_t *RxBuffer, uint8_t Length);
extern uint8_t GyroReadRegister(uint8_t Reg);
extern void GyroWriteRegisters(uint8_t Reg, uint8_t *TxBuffer, uint8_t Length);
extern void GyroWriteRegister(uint8_t Reg, uint8_t TxData);

// ***************************************************
// end Gyro
// ***************************************************




// ***************************************************
// Ringo I2C
// ***************************************************

//Note: Ringo uses a custom library called "RingoWire.h" for I2C communication with
//the Accelerometer and Gyroscope which is a chopped down version of the 
//official Arduino "Wire.h" library which eliminates portions of the 
//original Wire.h which are not necessary for Ringo and significantly reduces
//code space required.
extern uint8_t I2CReadRegs(uint8_t Device, uint8_t Reg, uint8_t *RxBuffer, uint8_t Length);
extern uint8_t I2CReadReg(uint8_t Device, uint8_t Reg);
extern void I2CWriteRegs(uint8_t Device, uint8_t Reg, uint8_t *TxBuffer, uint8_t Length);
extern void I2CWriteReg(uint8_t Device, uint8_t Reg, uint8_t TxData);
extern void I2CBegin(void);

// ***************************************************
// end Ringo I2C
// ***************************************************


// ***************************************************
// Gyroscope L3GD20 Register Defines
// ***************************************************

#define GYR_WHO_AM_I 0x0F
#define GYR_CTRL_REG1 0x20
#define GYR_CTRL_REG2 0x21
#define GYR_CTRL_REG3 0x22
#define GYR_CTRL_REG4 0x23
#define GYR_CTRL_REG5 0x24
#define GYR_REFERENCE 0x25
#define GYR_OUT_TEMP 0x26
#define GYR_STATUS_REG 0x27
#define GYR_OUT_X_L 0x28
#define GYR_OUT_X_H 0x29
#define GYR_OUT_Y_L 0x2A
#define GYR_OUT_Y_H 0x2B
#define GYR_OUT_Z_L 0x2C
#define GYR_OUT_Z_H 0x2D
#define GYR_FIFO_CTRL_REG 0x2E
#define GYR_FIFO_SRC_REG 0x2F
#define GYR_INT1_CFG 0x30
#define GYR_INT1_SRC 0x31
#define GYR_INT1_TSH_XH 0x32
#define GYR_INT1_TSH_XL 0x33
#define GYR_INT1_TSH_YH 0x34
#define GYR_INT1_TSH_YL 0x35
#define GYR_INT1_TSH_ZH 0x36
#define GYR_INT1_TSH_ZL 0x37
#define GYR_INT1_DURATION 0x38

// ***************************************************
// end Gyroscope L3GD20 Register Defines
// ***************************************************


// ***************************************************
// Accelerometer MMA8451Q Register Defines
// ***************************************************

#define ACC_F_STATUS 0x00
#define ACC_OUT_X_MSB 0x01
#define ACC_OUT_X_LSB 0x02
#define ACC_OUT_Y_MSB 0x03
#define ACC_OUT_Y_LSB 0x04
#define ACC_OUT_Z_MSB 0x05
#define ACC_OUT_Z_LSB 0x06
#define ACC_F_SETUP 0x09
#define ACC_TRIG_CFG 0x0A
#define ACC_SYSMOD 0x0B
#define ACC_INT_SOURCE 0x0C
#define ACC_WHO_AM_I 0x0D
#define ACC_XYZ_DATA_CFG 0x0E
#define ACC_HP_FILTER_CUTOFF 0x0F
#define ACC_PL_STATUS 0x10
#define ACC_PL_CFG 0x11
#define ACC_PL_COUNT 0x12
#define ACC_PL_BF_ZCOMP 0x13
#define ACC_P_L_THS_REG 0x14
#define ACC_FF_MT_CFG 0x15
#define ACC_FF_MT_SRC 0x16
#define ACC_FF_MT_THS 0x17
#define ACC_FF_MT_COUNT 0x18
#define ACC_TRANSIENT_CFG 0x1D
#define ACC_TRANSIENT_SRC 0x1E
#define ACC_TRANSIENT_THS 0x1F
#define ACC_TRANSIENT_COUNT 0x20
#define ACC_PULSE_CFG 0x21
#define ACC_PULSE_SRC 0x22
#define ACC_PULSE_THSX 0x23
#define ACC_PULSE_THSY 0x24
#define ACC_PULSE_THSZ 0x25
#define ACC_PULSE_TMLT 0x26
#define ACC_PULSE_LTCY 0x27
#define ACC_PULSE_WIND 0x28
#define ACC_ASLP_COUNT 0x29
#define ACC_CTRL_REG1 0x2A
#define ACC_CTRL_REG2 0x2B
#define ACC_CTRL_REG3 0x2C
#define ACC_CTRL_REG4 0x2D
#define ACC_CTRL_REG5 0x2E
#define ACC_OFF_X 0x2F
#define ACC_OFF_Y 0x30
#define ACC_OFF_Z 0x31


// ***************************************************
// end Accelerometer MMA8451Q Register Defines
// ***************************************************






#endif


