#ifndef WT901_I2C_H
#define WT901_I2C_H

#include "driver/i2c_master.h"


#ifdef __cplusplus
extern "C" {
#endif

#define SAVE 			0x00
#define CALSW 			0x01
#define RSW 			0x02
#define RRATE			0x03
#define BAUD 			0x04
#define AXOFFSET	0x05
#define AYOFFSET	0x06
#define AZOFFSET	0x07
#define GXOFFSET	0x08
#define GYOFFSET	0x09
#define GZOFFSET	0x0a
#define HXOFFSET	0x0b
#define HYOFFSET	0x0c
#define HZOFFSET	0x0d
#define D0MODE		0x0e
#define D1MODE		0x0f
#define D2MODE		0x10
#define D3MODE		0x11
#define D0PWMH		0x12
#define D1PWMH		0x13
#define D2PWMH		0x14
#define D3PWMH		0x15
#define D0PWMT		0x16
#define D1PWMT		0x17
#define D2PWMT		0x18
#define D3PWMT		0x19
#define IICADDR		0x1a
#define LEDOFF 		0x1b
#define GPSBAUD		0x1c
#define ORIENT		0x23
#define AXIS6 		0x24
#define LOCK 		  0x69
#define VERSION 	0x2E
#define BATVAL 	  0x5c

/************CALSW**************/
#define NORMAL          0x00
#define CALGYROACC      0x01
#define CALMAG          0x02
#define CALALTITUDE     0x03
#define CALANGLEZ       0x04
#define CALACCL         0x05
#define CALACCR         0x06
#define CALMAGMM        0x07
#define CALREFANGLE		0x08
#define CALMAG2STEP		0x09
#define CALHEXAHEDRON		0x12

/***********output head*********/
#define YYMM				0x30
#define DDHH				0x31
#define MMSS				0x32
#define MS					0x33
#define AX					0x34
#define AY					0x35
#define AZ					0x36
#define GX					0x37
#define GY					0x38
#define GZ					0x39
#define HX					0x3a
#define HY					0x3b
#define HZ					0x3c			
#define Roll				0x3d
#define Pitch				0x3e
#define Yaw					0x3f
#define TEMP				0x40
#define D0Status		0x41
#define D1Status		0x42
#define D2Status		0x43
#define D3Status		0x44
#define PressureL		0x45
#define PressureH		0x46
#define HeightL			0x47
#define HeightH			0x48
#define LonL				0x49
#define LonH				0x4a
#define LatL				0x4b
#define LatH				0x4c
#define GPSHeight   0x4d
#define GPSYAW      0x4e
#define GPSVL				0x4f
#define GPSVH				0x50
      
#define DIO_MODE_AIN 0
#define DIO_MODE_DIN 1
#define DIO_MODE_DOH 2
#define DIO_MODE_DOL 3
#define DIO_MODE_DOPWM 4
#define DIO_MODE_GPS 5		

struct STime
{
	unsigned char ucYear;
	unsigned char ucMonth;
	unsigned char ucDay;
	unsigned char ucHour;
	unsigned char ucMinute;
	unsigned char ucSecond;
	unsigned short usMiliSecond;
};

struct SAcc
{
    // Raw acceleration data from registers (short array)
    short a[3];        // a[0] = X axis, a[1] = Y axis, a[2] = Z axis (from register map)
    
    // Converted acceleration values (float array)
    float af[3];       // af[0] = X axis, af[1] = Y axis, af[2] = Z axis (converted to float)
};

struct STmp //temperature
{
	short tmp;
	float tmpf;
};

struct SGyr
{
    // Raw gyroscope data from registers (short array)
    short w[3];        // g[0] = X axis, g[1] = Y axis, g[2] = Z axis (from register map)
    
    // Converted gyroscope values (float array)
    float wf[3];       // gf[0] = X axis, gf[1] = Y axis, gf[2] = Z axis (converted to float)
};

struct SAtt
{
    // Raw angle data from registers (short array)
    short Att[3];    // Attitude[0] = Roll, Attitude[1] = Pitch, Attitude[2] = Yaw (from register map)
    
    // Converted angle values (float array)
    float Attf[3];   // Attitudef[0] = Roll, Attitudef[1] = Pitch, Attitudef[2] = Yaw (converted to float)
};

struct SMag
{
	short h[3];
};

struct SDStatus
{
	short sDStatus[4];
};

struct SBar
{
	long lPressure;
	long lHeight;
};

struct SLonLat
{
	long lLon;
	long lLat;
};

struct SGPSV
{
	short sGPSHeight;
	short sGPSYaw;
	long lGPSVelocity;
};

class CJY901 
{
  public: 
	struct STime		stcTime;
	struct STmp			stcTmp;
	struct SAcc 		stcAcc;
	struct SGyr 		stcGyr;
	struct SAtt 		stcAtt;
	struct SMag 		stcMag;
	struct SDStatus 	stcDStatus;
	struct SBar 		stcBar;
	struct SLonLat 		stcLonLat;
	struct SGPSV 		stcGPSV;
	
    CJY901(){}; 
	void StartIIC(i2c_master_dev_handle_t bus_handle);
    void CopeSerialData(unsigned char ucData);
	short ReadWord(unsigned char ucAddr);
	void WriteWord(unsigned char ucAddr,short sData);
	void ReadData(unsigned char ucAddr,unsigned char ucLength,char chrData[]);
	void updateTime();
	esp_err_t updateAcc();
	esp_err_t updateGyr();
	esp_err_t updateAtt();
	esp_err_t updateMag();
	esp_err_t updateBar();
	void updateDStatus();
	void updateLonLat();
	void updateGPSV();

  void Config();
  esp_err_t calibrateAcc();
  esp_err_t calibrateMag();
  esp_err_t stopCalibrating();
  // getOffsets();
  void checkOrientation();
  void checkAxis6Mode();
  void updateAll();
  void printAllData();
  esp_err_t unlock();
  esp_err_t lock();
  esp_err_t save();
  void reset();
	
  private: 
	//unsigned char ucDevAddr; 
	i2c_master_dev_handle_t bus_handle;
	esp_err_t readRegisters(uint8_t addressToRead, uint8_t bytesToRead, uint8_t *dest);
	esp_err_t writeRegister(uint8_t *dataToWrite, uint8_t bytesToWrite);
};
//extern CJY901 JY901;

#ifdef __cplusplus
}
#endif

#endif