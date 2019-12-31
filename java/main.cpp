/* */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "SensorEvent.h"
#include "HeadTracker.h"
#include "HeadTransform.h"

// vector components
#define CHX 0
#define CHY 1
#define CHZ 2

// current hard iron offset x, y, z, (uT)
float fV[3] = {-0.631452, -10.438886, 64.399132};
// current inverse soft iron matrix
float finvW[3][3] =
{
	{0.977650, 0.025796, 0.003901,},
	{0.025796, 0.991753,-0.008408,},
	{0.003901,-0.008408, 1.032164,},
};
// http://www.unit-conversion.info/magnetic-field.html
// 1 Gauss   (G) = 100 Microtesla (µT)
// 1 mGauss (mG) = 0.1 Microtesla (µT)
// uT per count
float fuTPerCount = 0.1;
// counts per uT
float fCountsPeruT = 10;
// most recent unaveraged uncalibrated measurement (counts)
short iBs[3];
// averaged un-calibrated measurement (uT)
float fBsAvg[3];
// averaged calibrated measurement (uT)
float fBcAvg[3];
// averaged calibrated measurement (counts)
short iBcAvg[3];
// temporary array
float ftmp[3];

int main(int argc, char *argv[])
{
    const char *name = "sensors-data.txt";
    char title[128];
    FILE * fp;
    int count;
    char timestamp[32];
    float pressure;
    float temperature;
    float humidity;
    // original data
    int iax;
    int iay;
    int iaz;
    int igx;
    int igy;
    int igz;
    int imx;
    int imy;
    int imz;
    // bias
#if 1
    int ax_bias = -26;
    int ay_bias = -7;
    int az_bias = -26;
    int gx_bias = 1301; /* 1301 */
    int gy_bias = -721; /* -721 */
    int gz_bias = 6367; /* 6367 */
    int mx_bias = 0; /* -18 */
    int my_bias = 0; /* -97 */
    int mz_bias = 0; /* 652*/
#else
    int ax_bias = 0;
    int ay_bias = 0;
    int az_bias = 0;
    int gx_bias = 0;
    int gy_bias = 0;
    int gz_bias = 0;
    int mx_bias = 0;
    int my_bias = 0;
    int mz_bias = 0;
#endif
    // variables to hold latest sensor data values
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    float deltat = 0.005; // 5ms, 200HZ
    float yaw, pitch, roll;
    float eulerAngles[3];
    int i;
    HeadTracker tracker;
    HeadTransform head;
    long sensorTimeStamp = nanoTime();
    SensorEvent event;
    float headView[16] = {0};

    if (argc >= 2)
	name = argv[1];

    fp = fopen (name, "r");
    if (fp == NULL) {
	fprintf (stderr, "Can't open file: %s\n", name);
	exit (-1);
    }

    // ignore the title
    fgets(title, sizeof(title), fp);

    //
    // Initialize the HeadTracker system.
    //
    tracker.startTracking();


    count = 0;
    while (13 == fscanf(fp, "%s %f %f %f %d %d %d %d %d %d %d %d %d",
			    timestamp,
			    &pressure,
			    &temperature,
			    &humidity,
			    &iax,
			    &iay,
			    &iaz,
			    &igx,
			    &igy,
			    &igz,
			    &imx,
			    &imy,
			    &imz
			    ))
    {
        count++;

        ax = (iax - ax_bias) / 1000.0f * 9.81f;
        ay = (iay - ay_bias) / 1000.0f * 9.81f;
        az = (iaz - az_bias) / 1000.0f * 9.81f;
        gx = (igx - gx_bias) / 1000.0f * M_PI / 180.0f;
        gy = (igy - gy_bias) / 1000.0f * M_PI / 180.0f;
        gz = (igz - gz_bias) / 1000.0f * M_PI / 180.0f;

	// read and process the magnetometer sensor in every slot if magnetic algorithm is in use.
	iBs[CHX] = imx - mx_bias;
	iBs[CHY] = imy - my_bias;
	iBs[CHZ] = imz - mz_bias;

	fBsAvg[CHX] = iBs[CHX] * fuTPerCount;
	fBsAvg[CHY] = iBs[CHY] * fuTPerCount;
	fBsAvg[CHZ] = iBs[CHZ] * fuTPerCount;

	// remove the computed hard iron offsets (uT): ftmp[]=fBs[]-V[]
	for (i = CHX; i <= CHZ; i++)
	{
	    ftmp[i] = fBsAvg[i] - fV[i];
	}
	// remove the computed soft iron offsets (uT and counts): fBc=inv(W)*(fBs[]-V[])
	for (i = CHX; i <= CHZ; i++)
	{
	    fBcAvg[i] = finvW[i][CHX] * ftmp[CHX] + finvW[i][CHY] * ftmp[CHY] + finvW[i][CHZ] * ftmp[CHZ];
	    iBcAvg[i] = (short) (fBcAvg[i] * fCountsPeruT);
	}

	imx = iBcAvg[CHX];
	imy = iBcAvg[CHY];
	imz = iBcAvg[CHZ];
	//printf("imx = %5d, imy = %5d, imz = %5d, Total Mag = %7.2f,\n", imx, imy, imz, sqrt(imx * imx + imy * imy + imz * imz));
	//printf("iax = %5d, iay = %5d, iaz = %5d,\n", iax - ax_bias, iay - ay_bias, iaz - az_bias);

#if 0
	// uT -> T
	mx = fBcAvg[CHX] / 1000000.0f;
	my = fBcAvg[CHY] / 1000000.0f;
	mz = fBcAvg[CHZ] / 1000000.0f;
#else
	// uT -> Gauss
	mx = fBcAvg[CHX] / 1000.0f;
	my = fBcAvg[CHY] / 1000.0f;
	mz = fBcAvg[CHZ] / 1000.0f;
#endif

	/* ST X-NUCLEO-IKS01A1 扩展板
	 * 当Y轴指向北方，X轴指向东方，测试得知：
	 * 加速度数据
	 * +X: East
	 * +Y: North
	 * +Z: Up (指向天空为正)
	 *
	 * 磁力计校准后数据
	 * X: 273 (N)
	 * Y:  46 (W)(指向东方为正)
	 * Z:-371 (D)(指向天空为正)
	 * 
	 * 北京地区地磁数据:
	 * N: 27913.8 nT
	 * E: -3312.1 nT
	 * D: 46771.9 nT
	 *
	 * 如果选择NED坐标系，则：
	 * 加速度计: X <-> Y， Z符号相反。
	 * 磁力计: 数据校准后，Y符号相反。
	 */
#if 1
#if 0
	float tmp1;

	tmp1 = ax;
	ax = ay;
	ay = tmp1;
	az =-az;

	my = -my;
#else
	// 选择ENU坐标系
	// 加速度数值不变。
	// 磁力计数值：Y符号相反。X <-> Y。
	float tmp1;

	tmp1 = -my;
	my = mx;
	mx = tmp1;

#endif
#endif

	// yaw / pitch / roll 公式来自于：
	// ST集成传感器方案实现电子罗盘功能
	// http://ic.semi.org.cn/a/technology/mems/26112.html
        pitch = atan2f(ax, sqrtf(ay * ay + az * az));
        roll  = atan2f(ay, sqrtf(ax * ax + az * az));
        float Hy, Hx;
        Hy = my * cosf(roll)  + mx * sinf(roll) * sinf(pitch) - mz * cosf(pitch) * sinf(roll);
        Hx = mx * cosf(pitch) + mz * sinf(pitch);
        yaw = atan2f(Hy, Hx);

	//printf ("Tilted:   yaw = %7.2f, pitch = %7.2f, roll = %7.2f,\n", yaw * 180.0f / M_PI, pitch * 180.0f / M_PI, roll * 180.0f / M_PI);

	sensorTimeStamp += (long)(deltat * 1.0E9F); // S2NS

	event.type = SENSOR_TYPE_ACCELEROMETER;
	event.timestamp = sensorTimeStamp;
	event.values[0] = ax;
	event.values[1] = ay;
	event.values[2] = az;
	tracker.onSensorChanged(event);

#if 1
	event.type = SENSOR_TYPE_MAGNETIC_FIELD;
	event.timestamp = sensorTimeStamp;
	event.values[0] = mx;
	event.values[1] = my;
	event.values[2] = mz;
	tracker.onSensorChanged(event);
#endif

	event.type = SENSOR_TYPE_GYROSCOPE;
	event.timestamp = sensorTimeStamp;
	event.values[0] = gx;
	event.values[1] = gy;
	event.values[2] = gz;
	tracker.onSensorChanged(event);

	tracker.getLastHeadView(headView, 16, 0);

#if 0
	printf("float headView[16] = {\n");
	for (i = 0; i < 4; i++) {
	    printf("\t%9.6f, %9.6f, %9.6f, %9.6f,\n",
			headView[i * 4 + 0],
			headView[i * 4 + 1],
			headView[i * 4 + 2],
			headView[i * 4 + 3]
		  );
	}
	printf("};\n");
#endif
	tracker.getLastHeadView(head.getHeadView(), 16, 0);
	head.getEulerAngles(eulerAngles, 3, 0);
        pitch = eulerAngles[0];
        yaw   = eulerAngles[1];
        roll  = eulerAngles[2];
	printf ("Kalman:   yaw = %7.2f, pitch = %7.2f, roll = %7.2f,\n", yaw * 180.0f / M_PI, pitch * 180.0f / M_PI, roll * 180.0f / M_PI);
    }

    //
    // stop tracking.
    //
    tracker.stopTracking();

    fclose(fp);
   
    return(0);
}
