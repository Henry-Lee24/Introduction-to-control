/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Description: User Module for CyberParking							█
█	Author: Chenxi Yang													█
█	Email: yangchenxi@sjtu.edu.cn										█
█	Create: 2022.04.03							    					█
█	Salute to Xuangui Huang												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/


#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"
#include <cmath>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*/
//Do NOT modify the code below!

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void* pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo * modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_parking";	// name of the module (short).
	modInfo[0].desc = "user module for CyberParking";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization
static int InitFuncPt(int, void* pt)
{
	tUserItf* itf = (tUserItf*)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	printf("OK!\n");
	return 0;
}

//Do Not modify the code above!
/*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;

static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;
float k1, k2;
float angleDiff = 0;
int mode = 0;
int reach = 0;
float distance;
float brake;
circle c;

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

	_lotX = lotX;
	_lotY = lotY;
	_lotAngle = lotAngle;
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;
	_caryaw = caryaw;
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	//printf("lotX %.6f  lotY %.6f", _lotX, _lotY);
}

double constrain(double lowerBoundary, double upperBoundary, double input) {
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
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

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█ Define your variables here											█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
static int nCrtStage = 0;	//Current stage

enum ListCrtStage {			//Stage list
	StageDrive,
	StageApproaching,
	StageDrift,
	StagePosition,
	StageEnterLot,
	StageBackToRoad,
	StageLeave
};

static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)
{
	/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
	█ Write your own code here												█
	\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	float fDis2Lot = (_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY);		//Distance to the parking lot
	float fVerticalDis2Lot;																		//Vertical distance to the parking lot
	float fHorizontalDis2Lot;
	c = getR(_midline[0][0], _midline[0][1], _midline[5][0], _midline[5][1], _midline[10][0], _midline[10][1]);

	if (abs(_lotAngle) > (PI / 2 - 0.05) && abs(_lotAngle) < (PI / 2 + 0.05)) {
		fVerticalDis2Lot = _carX - _lotX;
	}
	else {
		float k = tan(_lotAngle);
		float b = -k * _lotX + _lotY;
		fVerticalDis2Lot = (k * _carX - _carY + b) / sqrt(k * k + 1);
	}

	fHorizontalDis2Lot = sqrt(fDis2Lot - fVerticalDis2Lot * fVerticalDis2Lot);

	if (_caryaw > 0 && _lotAngle < 0)
		angleDiff = _caryaw - (_lotAngle + 2 * PI);
	else if (_caryaw < 0 && _lotAngle>0)
		angleDiff = _caryaw + 2 * PI - _lotAngle;
	else angleDiff = _caryaw - _lotAngle;

	printf("Stage: %d\n", nCrtStage);		//You can see the stage in the Console
	//printf("fDis2Lot:%f\n", fDis2Lot);
	printf("fVerticalDis2Lot:%f\n", fVerticalDis2Lot);
	printf("fHorizontalDis2Lot:%f\n", fHorizontalDis2Lot);
	//printf("lotX: %f\n", _lotX);
	//printf("lotY: %f\n", _lotY);
	//printf("_lotAngle: %f\n", _lotAngle);
	//printf("_caryaw: %f\n", _caryaw);
	printf("cmdSteer: %f\n", *cmdSteer);
	printf("angleDiff: %f\n", angleDiff);
	printf("speed: %f\n", _speed);
	//printf("cmdBrake: %f\n", *cmdBrake);
	printf("c.r: %f\n", c.r);
	printf("disatnce: %f\n", distance);
	switch (nCrtStage)
	{
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Approaching the parking lot when far from it							█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageDrive:
		*cmdSteer = constrain(-1.0, 1.0, (_yaw - 8 * atan2(_midline[10][0], _midline[10][1])) / 3.14);
		*cmdGear = 1;
		*cmdAcc = 1;
		*cmdBrake = 0;
		if (fDis2Lot < 5000)
			nCrtStage++;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Keep right to get more place to turn									█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageApproaching:
		*cmdSteer = constrain(-1.0, 1.0, (_yaw - 8 * atan2(_midline[10][0], _midline[10][1])) / 3.14);
		*cmdGear = 2;
		if (abs(_speed) < 80)
		{
			*cmdAcc = 0.5; *cmdBrake = 0.0;
		}
		else
		{
			*cmdAcc = 0; *cmdBrake = 0.2;
		}
		if (c.r > 490)
			distance = 15.2;
		else
			distance = 15.2 * sin(abs(angleDiff)) - 2.5 * cos(abs(angleDiff));
		if (abs(fVerticalDis2Lot) < distance)
			nCrtStage++;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Turn left into the parking lot										█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageDrift://飘移
		*cmdSteer = -1;
		*cmdGear = -1;
		*cmdAcc = 0.4;
		*cmdBrake = 0.354 + brake;
		if (c.r > 490)
			brake = 0;
		else
			brake = 0.055 * (PI / 2 - abs(angleDiff));
		if (_speed < 4)
			nCrtStage++;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Drive straight into the parking lot									█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StagePosition://调整位置
		static float X = fVerticalDis2Lot;
		printf("X: %f\n", X);
		*cmdSteer = constrain(-1, 1, 40 / PI * angleDiff - 3 * fVerticalDis2Lot);
		*cmdGear = -1;
		if (_speed < -30) {
			*cmdAcc = 0.0; *cmdBrake = 0.2;
		}
		else {
			*cmdAcc = 1; *cmdBrake = 0.0;
		}
		if (fDis2Lot < 30)
			nCrtStage++;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Drive straight out of the parking lot									█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageEnterLot:
		*cmdSteer = constrain(-1, 1, 40 / PI * angleDiff - 3 * fVerticalDis2Lot);
		*cmdGear = -1;
		if (fHorizontalDis2Lot > 2)
		{
			if (abs(_speed) > 20)
			{
				*cmdAcc = 0;
				*cmdBrake = 0.5;
			}
			else
			{
				*cmdAcc = 0.2;
				*cmdBrake = 0;
			}
		}
		else if (fHorizontalDis2Lot > 1.2)
		{
			if (abs(_speed) > 14)
			{
				*cmdAcc = 0;
				*cmdBrake = 0.6;
			}
			else
			{
				*cmdAcc = 0.2;
				*cmdBrake = 0;
			}
		}
		else if (fHorizontalDis2Lot > 0.5)
		{
			if (abs(_speed) > 7)
			{
				*cmdAcc = 0;
				*cmdBrake = 0.6;
			}
			else
			{
				*cmdAcc = 0.3;
				*cmdBrake = 0;
			}
		}
		else if (abs(fHorizontalDis2Lot) > 0.1 && reach == 0)
		{
			if (abs(_speed) > 5)
			{
				*cmdAcc = 0;
				*cmdBrake = 0.4;
			}
			else
			{
				*cmdAcc = 0.2;
				*cmdBrake = 0;
			}
		}
		else
		{
			if (abs(_speed) < 0.18)
			{
				*cmdAcc = 0.2;
				*cmdBrake = 0;
			}
			else
			{
				*cmdAcc = 0;
				*cmdBrake = 0.6;
			}
			if (abs(fHorizontalDis2Lot) < 0.005) reach = 1;//已到指定位置
			if (abs(_speed) < 0.2 && abs(fHorizontalDis2Lot) < 0.005)
			{
				*bFinished = true;
				nCrtStage++;
			}
			if (fHorizontalDis2Lot > 0.005 && reach == 1 && abs(_speed) < 0.2)//超过了指定位置
			{
				*bFinished = true;
				nCrtStage++;
			}
		}
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Turn left back to the main road										█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageBackToRoad:
		*cmdSteer = 1;
		*cmdGear = 1;
		if (_speed > 50) {
			*cmdAcc = 0; *cmdBrake = 0.1;
		}
		else {
			*cmdAcc = 0.55; *cmdBrake = 0;
		}
		if (abs(angleDiff) > 1.4)
			nCrtStage++;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Leave the parking lot													█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageLeave:
		*cmdSteer = constrain(-1.0, 1.0, (_yaw - 8 * atan2(_midline[25][0], _midline[25][1])) / 3.14);
		*cmdGear = 1;
		*cmdAcc = 0.9; *cmdBrake = 0;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Enjoy!																█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	default:
		break;
	}
}



