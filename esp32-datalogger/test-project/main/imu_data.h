#ifndef IMU_DATA_H
#define IMU_DATA_H

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
    //short a[3];        // a[0] = X axis, a[1] = Y axis, a[2] = Z axis (from register map)
    
    // Converted acceleration values (float array)
    float af[3];       // af[0] = X axis, af[1] = Y axis, af[2] = Z axis (converted to float)
};

struct STmp //temperature
{
	//short tmp;
	float tmpf;
};

struct SGyr
{
    // Raw gyroscope data from registers (short array)
    //short w[3];        // g[0] = X axis, g[1] = Y axis, g[2] = Z axis (from register map)
    
    // Converted gyroscope values (float array)
    float wf[3];       // gf[0] = X axis, gf[1] = Y axis, gf[2] = Z axis (converted to float)
};

struct SAtt
{
    // Raw angle data from registers (short array)
    //short Att[3];    // Attitude[0] = Roll, Attitude[1] = Pitch, Attitude[2] = Yaw (from register map)
    
    // Converted angle values (float array)
    float Attf[3];   // Attitudef[0] = Roll, Attitudef[1] = Pitch, Attitudef[2] = Yaw (converted to float)
};

struct SMag
{
	short h[3];
	//float hf[3];
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

struct IMUData {
    SAcc acc;
    SGyr gyr;
    SAtt att;
    STmp tmp;
    SMag mag;
    SBar bar;
};

#endif