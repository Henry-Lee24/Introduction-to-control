/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*\
¨€	Description: User Module for CyberParking							¨€
¨€	Author: Chenxi Yang													¨€
¨€	Email: yangchenxi@sjtu.edu.cn										¨€
¨€	Create: 2022.04.03							    					¨€
¨€	Salute to Xuangui Huang												¨€
\*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/


#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"
#include <cmath>

/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*/
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
/*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/

static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;

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

/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*\
¨€ Define your variables here											¨€
\*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/
static int nCrtStage = 0;	//Current stage

enum ListCrtStage {			//Stage list
	StageApproaching,
	StageKeepRight,
	StageTurnLeft,
	StageEnterLot,
	StageLeaveLot,
	StageBackToRoad,
	StageLeave
};

static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)
{
	/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*\
	¨€ Write your own code here												¨€
	\*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/
	float fDis2Lot = (_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY);		//Distance to the parking lot
	float fVerticalDis2Lot;																		//Vertical distance to the parking lot

	if (abs(_lotAngle) > (PI / 2 - 0.05) && abs(_lotAngle) < (PI / 2 + 0.05)) {
		fVerticalDis2Lot = abs(_carX - _lotX);
	}
	else {
		float k = tan(_lotAngle);
		float b = -k * _lotX + _lotY;
		fVerticalDis2Lot = abs(k * _carX - _carY + b) / sqrt(k * k + 1);
	}

	printf("Stage: %d\n", nCrtStage);		//You can see the stage in the Console
	printf("lotX: %d\n", _lotX);
	printf("lotY: %d\n", _lotY);
	switch (nCrtStage)
	{
		/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*\
		¨€ Approaching the parking lot when far from it							¨€
		\*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/
	case StageApproaching:
		*cmdSteer = constrain(-1.0, 1.0, (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14);
		*cmdGear = 1;
		*cmdAcc = 1.0; *cmdBrake = 0.0;
		if (fDis2Lot < 4000)
			nCrtStage++;
		break;
		/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*\
		¨€ Keep right to get more place to turn									¨€
		\*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/
	case StageKeepRight:
		*cmdSteer = constrain(-1.0, 1.0, (_yaw - atan2(_midline[5][0] + _width / 3, _midline[5][1])) / 3.14);
		*cmdGear = 1;
		if (_speed > 15) {
			*cmdAcc = 0.0; *cmdBrake = 1;
		}
		else {
			*cmdAcc = 0.1; *cmdBrake = 0;
		}
		if (fVerticalDis2Lot < 8.2)
			nCrtStage++;
		break;
		/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*\
		¨€ Turn left into the parking lot										¨€
		\*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/
	case StageTurnLeft:
		*cmdSteer = constrain(-1.0, 1.0, 0.85+_speed*0.01);
		*cmdGear = 1;
		if (_speed > 15) {
			*cmdAcc = 0.0; *cmdBrake = 0.4;
		}
		else {
			*cmdAcc = 0.1; *cmdBrake = 0.0;
		}
		if (fDis2Lot < 35)
			nCrtStage++;
		break;
		/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*\
		¨€ Drive straight into the parking lot									¨€
		\*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/
	case StageEnterLot:
		*cmdSteer = constrain(-1.0, 1.0, -50 * (_lotAngle + _caryaw) / 3.14);
		*cmdGear = 1;
		

		if (_speed > 10) {
			*cmdAcc = 0.2; *cmdBrake = 0.1;
			if (fDis2Lot < 1.5)
			{
				*cmdBrake = 0.4;
			}
		}
		else if (_speed > 0.3) {
			*cmdAcc = 0.3; *cmdBrake = 0.1;
			if (fDis2Lot < 1.5)
			{
				*cmdBrake = 0.4;
			}
		}
		else if (_speed > 0.07)
		{
			for (int i = 0; i < 50; i++)
			{
				*cmdAcc = 0.1; *cmdBrake = 0.05;
			}
			*cmdAcc = 0.10; *cmdBrake = 0.10;
		}
		else {
			*bFinished = true;
			nCrtStage++;
		}
		break;
		/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*\
		¨€ Drive straight out of the parking lot									¨€
		\*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/
	case StageLeaveLot:
		*cmdSteer = constrain(-1.0, 1.0, -30 * (_lotAngle + _caryaw) / 3.14);
		*cmdGear = -1;
		if (_speed > 20) {
			*cmdAcc = 0.13; *cmdBrake = 0.05;
		}
		else {
			*cmdAcc = 0.25; *cmdBrake = 0;
		}
		if (_speed > 30)
		{
			*cmdBrake = 0.3;
		}
		if (fDis2Lot > 8)
			nCrtStage++;
		break;
		/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*\
		¨€ Turn left back to the main road										¨€
		\*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/
	case StageBackToRoad:
		*cmdSteer = 1;
		*cmdGear = -1;
		if (_speed > 20) {
			*cmdAcc = 0; *cmdBrake = 0.1;
		}
		else {
			*cmdAcc = 0.2; *cmdBrake = 0;
		}
		if (_speed > 30)
		{
			*cmdBrake = 0.3;
		}
		if (fVerticalDis2Lot > 6)
			nCrtStage++;
		break;
		/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*\
		¨€ Leave the parking lot													¨€
		\*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/
	case StageLeave:
		*cmdSteer = constrain(-1.0, 1.0, (_yaw - 8 * atan2(_midline[25][0], _midline[25][1])) / 3.14);
		*cmdGear = 1;
		*cmdAcc = 1; *cmdBrake = 0;
		break;
		/*¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹¨‹*\
		¨€ Enjoy!																¨€
		\*¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø¡ø*/
	default:
		break;
	}
}



