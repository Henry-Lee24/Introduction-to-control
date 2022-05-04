/*
	 WARNING !

	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include "stdio.h"
#include <ostream>
#include <fstream>

#define PI 3.141592653589793238462643383279

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void* pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo * modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_cruise";	// name of the module (short).
	modInfo[0].desc = "user module for CyberCruise";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void* pt)
{
	tUserItf* itf = (tUserItf*)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}


/*
	 WARNING!

	 DO NOT MODIFY CODES ABOVE!
*/

//**********Global variables for vehicle states*********//
static float _midline[200][2];							//
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;//
static int _gearbox;									//
//******************************************************//


bool parameterSet = false;								//
void PIDParamSetter();									//


//******************************************************//
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;												//
//******************************************************//

//********************PID parameters*************************//
double kp_s;	//kp for speed							     //
double ki_s;	//ki for speed							     //
double kd_s;	//kd for speed							     //
double kp_d;	//kp for direction						     //
double ki_d;	//ki for direction					    	 //
double kd_d;	//kd for direction
double k_s;//
// Direction Control Variables						         //
double D_err;//direction error					             //
double D_errDiff = 0;//direction difference(Differentiation) //
double D_errSum = 0;//sum of direction error(Integration)      //
// Speed Control Variables								     //
circle c, c_0, c_1, c_2;
double r;//
double expectedSpeed, expectedSteer;//      							     //
double curSpeedErr;//speed error   		                     //
double speedErrSum = 0;//sum of speed error(Integration)       //
int startPoint, startPoint_far;											     //
int delta;												 //
int mode = 0;
int count_time = 0;
double alpha, x, y;
double preaim;
int floor_preaim, ceil_preaim;
bool Rflag = false;
double len = 2.8;//车轴长
int tmp1;
int gear;
int emergency;
//***********************************************************//
//stanley variable
double distance;
double steer_angle;
double horizontal_angle;
double stanley_angle;
//*******************Other parameters*******************//
const int topGear = 6;									//
double tmp;												//
bool flag = true;											//
double offset = 0;										//
double Tmp = 0;
//******************************************************//

//******************************Helping Functions*******************************//
// Function updateGear:															//
//		Update Gear automatically according to the current speed.				//
//		Implemented as Schmitt trigger.											//
void updateGear(int* cmdGear);													//
// Function constrain:															//
//		Given input, outputs value with boundaries.								//
double constrain(double lowerBoundary, double upperBoundary, double input);		//
// Function getR:																//
//		Given three points ahead, outputs a struct circle.						//
//		{radius:[1,500], sign{-1:left,1:right}									//
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);		//
//******************************************************************************//
double get_closest_distance();

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	if (parameterSet == false)		// Initialization Part
	{
		PIDParamSetter();
	}

	else
	{
		// Speed Control
		/*
		You can modify the limited speed in this module
		Enjoy  -_-
		*/
		count_time++;
		if (count_time == 65 && _speed < 40.0) mode = 1;


		int delta0 = 5;
		int delta1 = 10;
		int delta2 = 15;
		startPoint = _speed * 0.2;
		if (mode) { startPoint_far = _speed * 0.5; }
		else { startPoint_far = _speed * 0.4; }
		c_0 = getR(_midline[0][0], _midline[0][1], _midline[0 + delta0][0], _midline[0 + delta0][1], _midline[0 + 2 * delta0][0], _midline[0 + 2 * delta0][1]);
		c_1 = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta1][0], _midline[startPoint + delta1][1], _midline[startPoint + 2 * delta1][0], _midline[startPoint + 2 * delta1][1]);
		c_2 = getR(_midline[startPoint_far][0], _midline[startPoint_far][1], _midline[startPoint_far + delta2][0], _midline[startPoint_far + delta2][1], _midline[startPoint_far + 2 * delta2][0], _midline[startPoint_far + 2 * delta2][1]);
		r = min(c_0.r, c_1.r, c_2.r);
		if (mode) {
			/*if (r <= 20) { expectedSpeed = constrain(30, 60, r * r * (-0.046) + r * 5.5 - 26.66); }//
			else if (r < 60) { expectedSpeed = constrain(60, 79, 0.375 * r + 57.5); }//
			else if (r < 100) { expectedSpeed = constrain(79, 85, r * 0.25 + 65); }//
			else if (r < 250) { expectedSpeed = constrain(85, 100, r * 2 / 15 + 230 / 3); }//
			else if (r < 450) { expectedSpeed = constrain(100, 120, r * 0.1 + 85); }
			else { expectedSpeed = 120; }*/
			if (r <= 20) { expectedSpeed = constrain(30, 60, r * r * (-0.046) + r * 5.5 - 31.66); }//
			else if (r < 60) { expectedSpeed = constrain(60, 79, 0.375 * r + 57.5); }//
			else if (r < 100) { expectedSpeed = constrain(79, 85, r * 0.25 + 65); }//
			else if (r < 250) { expectedSpeed = constrain(85, 100, r * 2 / 15 + 230 / 3); }//
			else if (r < 450) { expectedSpeed = constrain(100, 120, r * 0.1 + 85); }
			else { expectedSpeed = 120; }
		}
		else {
			if (r <= 20) { expectedSpeed = constrain(40, 65, r * r * (-0.046) + r * 5.5 - 22.66); }//
			else if (r < 60) { expectedSpeed = constrain(65, 80, 0.375 * r + 63); }//
			else if (r < 100) { expectedSpeed = constrain(80, 90, r * 0.25 + 67); }//
			else if (r < 250) { expectedSpeed = constrain(90, 110, r * 2 / 15 + 77); }//
			else if (r < 450) { expectedSpeed = constrain(110, 150, r * 0.2 + 59); }
			else { expectedSpeed = 160; }
		}


		kp_s = 0.03;
		ki_s = 0;
		kd_s = 0.35;//最好的速度参数

		curSpeedErr = expectedSpeed - _speed;
		speedErrSum = 0.1 * speedErrSum + curSpeedErr;
		if (mode)
		{
			if (curSpeedErr > 0)
			{

				if (count_time < 65)
				{
					*cmdAcc = 1;
					*cmdBrake = 0;
				}
				else if (abs(*cmdSteer) < 0.1)
				{
					*cmdAcc = constrain(0.0, 1.0, kp_s * curSpeedErr + ki_s * speedErrSum + offset);
					*cmdBrake = 0;
				}
				else if (abs(*cmdSteer) > 0.5 && gear != -1)
				{
					*cmdAcc = 0.005 + offset;
					*cmdBrake = 0;
				}
				else
				{
					*cmdAcc = 0.1 + offset;
					*cmdBrake = 0;
				}//调整的油门

			}
			else if (curSpeedErr < 0)
			{
				*cmdBrake = constrain(0.0, 1, -kp_s * curSpeedErr / 3 - offset / 3 + 0.2);
				*cmdAcc = 0;
			}
		}

		else
		{
			if (curSpeedErr > 0)
			{
				if (count_time < 100)
				{
					*cmdAcc = 1;
					*cmdBrake = 0;
				}
				else if (abs(*cmdSteer) < 0.06)
				{
					*cmdAcc = constrain(0.0, 1.0, kp_s * curSpeedErr + ki_s * speedErrSum + offset + 0.15);
					*cmdBrake = 0;
				}
				else if (abs(*cmdSteer) > 0.5 && gear != -1)
				{
					*cmdAcc = 0.01 + offset;
					*cmdBrake = 0;
				}
				else
				{
					*cmdAcc = 0.1 + offset;
					*cmdBrake = 0;
				}//调整的油门

			}
			else if (curSpeedErr < 0)
			{
				*cmdBrake = constrain(0.0, 1.0, -kp_s * curSpeedErr / 3.5 - offset / 3 + 0.1);
				*cmdAcc = 0;
			}
		}
		updateGear(cmdGear);

		/******************************************Modified by Yuan Wei********************************************/
		/*
		Please select a error model and coding for it here, you can modify the steps to get a new 'D_err',this is just a sample.
		Once you choose the error model, you can rectify the value of PID to improve your control performance.
		Enjoy  -_-
		*/
		// Direction Control
		//set the param of PID controller
		/*if (r > 120)
		{
			kp_d = 6;
			ki_d = 0;
			kd_d = 25;
			k_s = 1;
		}
		else if (r > 40)
		{
			kp_d = 7;
			ki_d = 0;
			kd_d = 25;
			k_s = 1;
		}
		else
		{
			kp_d = 8;
			ki_d = 0;
			kd_d = 25;
			k_s = 1;
		}
		if (mode)
		{
			kp_d -= 0;
			ki_d += 0;
			kd_d += 0;
		}*/

		if (r > 250)
		{
			kp_d = 12 * 0.7;
			ki_d = 0;
			kd_d = 20;
			k_s = 1;
		}
		if (r > 120)
		{
			kp_d = 13 * 0.7;
			ki_d = 0;
			kd_d = 20;
			k_s = 1;
		}
		else if (r > 40)
		{
			kp_d = 13 * 0.7;
			ki_d = 0;
			kd_d = 20;
			k_s = 1.45;
		}
		else
		{
			kp_d = 13 * 0.7;
			ki_d = 0;
			kd_d = 20;
			k_s = 1.45;
		}
		if (mode)
		{
			kp_d -= 0;
			ki_d += 0;
			kd_d += 0;
		}//midline[2]pid参数*/

		//std::fstream file;
		//FILE *fp;

		//get the error


		//纯追踪
		//preaim = constrain(0.5, 2.5, _speed / 60);
		//消除小数影响
		/*floor_preaim = floor(preaim);
		ceil_preaim = ceil(preaim);
		x = _midline[floor_preaim][0] + (_midline[ceil_preaim][0] - _midline[floor_preaim][0]) * (preaim - floor_preaim);
		y = _midline[floor_preaim][1] + (_midline[ceil_preaim][1] - _midline[floor_preaim][1]) * (preaim - floor_preaim);
		alpha = atan2(x, y) - _yaw;
		expectedSteer = atan2(2 * len * sin(alpha) / (0.1 * _speed + preaim), 1);
		D_err = -expectedSteer * 360 / (2 * PI);*/
		//double k1=constrain(2.5,10,r/30);
		double k1 = 4.53;
		if (mode)k1 = 0.7;
		if (count_time < 150) k1 = 1;
		//stanley
		distance = get_closest_distance();
		horizontal_angle = atan(k1 * distance / _speed);
		stanley_angle = horizontal_angle * 180 / PI;
		//the differential and integral operation
		D_err = -atan2(_midline[2][0], _midline[2][1]);
		D_errDiff = D_err - Tmp;
		D_errSum = D_errSum + D_err;
		Tmp = D_err;
		//if (count_time < 100) stanley_angle = 0;
		//set the error and get the cmdSteer
		*cmdSteer = constrain(-1, 1, kp_d * D_err - k_s * stanley_angle + ki_d * D_errSum + kd_d * D_errDiff);
		//if(count_time<150) *cmdSteer = constrain(-0.1, 0.1, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);

		//print some useful info on the terminal
		printf("D_err : %f \n", D_err);
		printf("D_errSum : %f \n", D_errSum);
		printf("D_errDiff : %f \n", D_errDiff);
		//printf("distance : %f \n", distance);
		//printf("horizontal_angle : %f \n", horizontal_angle);
		//printf("steer_angle : %f \n", steer_angle);
		//printf("stanley_angle : %f \n", stanley_angle);
		printf("cmdSteer %f \n", *cmdSteer);
		//printf("stanley_angle %f \n", stanley_angle);
		printf("err %f \n", _midline[0][0]);
		//printf("yaw:%f\n", _yaw);
		//printf("cmdAcc %f \n", *cmdAcc);
		//printf("c.r: %f \n", c.r);
		printf("r: %f \n", r);
		//printf("expectedspeed: %f \n", expectedSpeed);
		//printf("acc: %f \n", _acc);
		printf("mode: %d \n", mode);
		//printf("time: %d \n", count_time);
		//printf("speed: %f \n", _speed);
		//printf("cmdBrake %f \n", *cmdBrake);
		/******************************************End by Yuan Wei********************************************/
	}
}

void PIDParamSetter()
{

	kp_s = 0.02;
	ki_s = 0;
	kd_s = 0;
	kp_d = 1.35;
	ki_d = 0.151;
	kd_d = 0.10;
	parameterSet = true;

}

void updateGear(int* cmdGear)
{
	if (gear == -1)
	{
		*cmdGear = -1;
	}
	else if (_gearbox == 1)
	{
		if (_speed >= 60 && topGear > 1)
		{
			*cmdGear = 2;
		}
		else
		{
			*cmdGear = 1;
		}
	}
	else if (_gearbox == 2)
	{
		if (_speed <= 45)
		{
			*cmdGear = 1;
		}
		else if (_speed >= 105 && topGear > 2)
		{
			*cmdGear = 3;
		}
		else
		{
			*cmdGear = 2;
		}
	}
	else if (_gearbox == 3)
	{
		if (_speed <= 90)
		{
			*cmdGear = 2;
		}
		else if (_speed >= 145 && topGear > 3)
		{
			*cmdGear = 4;
		}
		else
		{
			*cmdGear = 3;
		}
	}
	else if (_gearbox == 4)
	{
		if (_speed <= 131)
		{
			*cmdGear = 3;
		}
		else if (_speed >= 187 && topGear > 4)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 4;
		}
	}
	else if (_gearbox == 5)
	{
		if (_speed <= 173)
		{
			*cmdGear = 4;
		}
		else if (_speed >= 234 && topGear > 5)
		{
			*cmdGear = 6;
		}
		else
		{
			*cmdGear = 5;
		}
	}
	else if (_gearbox == 6)
	{
		if (_speed <= 219)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 6;
		}
	}
	else
	{
		*cmdGear = 1;
	}
}
double get_closest_distance()
{
	double angle = atan(abs((_midline[1][0] - _midline[0][0]) / (_midline[1][1] - _midline[0][1])));
	double distance = abs(sqrt(_midline[1][0] * _midline[1][0] + _midline[1][1] * _midline[1][1]) * sin(angle));
	if (_midline[0][0] < 0) distance *= -1;
	return distance;
}

double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
	x = (b * f - e * c) / (b * d - e * a);
	y = (d * c - a * f) / (b * d - e * a);
	r = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 500.0, r);
	int sign = (x > 0) ? 1 : -1;
	if (isnan(r))
	{
		r = 100;
	}
	circle tmp = { r,sign };
	return tmp;
}

