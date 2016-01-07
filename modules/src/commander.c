/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#include "commander.h"
#include "crtp.h"
#include "configblock.h"
#include "param.h"
#include "uart1.h"
#include "pid.h"
#include "controller.h"
#include "debug.h"

#include <math.h>

#define MIN_THRUST  1000
#define MAX_THRUST  60000

struct CommanderCrtpValues
{
  float roll;
  float pitch;
  float yaw;
  uint16_t thrust;
} __attribute__((packed));

struct MocapCrtp
{
	float x;
	float y;
	float z;
	float a;
} __attribute__((packed));

struct DestCrtp
{
	float x;
	float y;
	float z;
	float a;
} __attribute__((packed));

struct StateCrtp
{
	int curStat;
};

float mep(float x, float ma, float Ma, float mb, float Mb)
{
	return (x - ma) *  (Mb - mb) / (Ma - ma) + mb;
}

static struct CommanderCrtpValues targetVal[2];
static struct MocapCrtp mocapPose[2];
static struct DestCrtp destPose[2];
static struct StateCrtp Cstat[2];
static bool isInit;
static int side=0;
static int mc_side = 0;
static int dt_side = 0;
static int st_side = 0;
static int rf_update = 0;
static int state_curent = 0;
static uint32_t lastUpdate;
static bool isInactive;
static bool thrustLocked;
static bool altHoldMode = false;
static bool altHoldModeOld = false;
static uint16_t thrst;

static RPYType stabilizationModeRollPitch  = ANGLE;
static RPYType stabilizationModeRoll  = ANGLE; // Current stabilization type of roll (rate or angle)
static RPYType stabilizationModePitch = ANGLE; // Current stabilization type of pitch (rate or angle)
static RPYType stabilizationModeYaw   = RATE;  // Current stabilization type of yaw (rate or angle)

static YawModeType yawMode = DEFUALT_YAW_MODE; // Yaw mode configuration
static bool carefreeResetFront;             // Reset what is front in carefree mode

static cmdType cmdMode = ATTI;

static void commanderCrtpCB(CRTPPacket* pk);
static void mocapCrtpCB(CRTPPacket* pk);
static void destCrtpCB(CRTPPacket* pk);
static void stateCrtpCB(CRTPPacket* pk);
static void commanderWatchdogReset(void);

static uint8_t start;

void commanderInit(void)
{
  if(isInit)
    return;


  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_COMMANDER, commanderCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_MCPOSE, mocapCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_DTPOSE, destCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_STATE, stateCrtpCB);

  lastUpdate = xTaskGetTickCount();
  isInactive = true;
  thrustLocked = true;
  isInit = true;
}

bool commanderTest(void)
{
  crtpTest();
  return isInit;
}

static void mocapCrtpCB(CRTPPacket* pk)
{
	mocapPose[!mc_side] = *((struct MocapCrtp*)pk->data);
	mc_side = !mc_side;
	rf_update = 1;
	commanderWatchdogReset();
}

static void destCrtpCB(CRTPPacket* pk)
{
	destPose[!dt_side] = *((struct DestCrtp*)pk->data);
	dt_side = !dt_side;
	cmdMode = POSI;
	rf_update = 1;
	commanderWatchdogReset();
}

static void stateCrtpCB(CRTPPacket* pk)
{
	Cstat[!st_side] = *((struct StateCrtp*)pk->data);
	state_curent = Cstat[!st_side].curStat;
	st_side = !st_side;
}

static void commanderCrtpCB(CRTPPacket* pk)
{
  targetVal[!side] = *((struct CommanderCrtpValues*)pk->data);
  side = !side;
  cmdMode = ATTI;
  rf_update = 1;
  if (targetVal[side].thrust == 0) {
    thrustLocked = false;
  }

  commanderWatchdogReset();
}

void commanderWatchdog(void)
{
  int usedSide = side;
  uint32_t ticktimeSinceUpdate;

  ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;

  if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_STABALIZE)
  {
    targetVal[usedSide].roll = 0;
    targetVal[usedSide].pitch = 0;
    targetVal[usedSide].yaw = 0;
  }
  if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_SHUTDOWN)
  {
    targetVal[usedSide].thrust = 0;
    altHoldMode = false; // do we need this? It would reset the target altitude upon reconnect if still hovering
    isInactive = true;
    thrustLocked = true;
  }
  else
  {
    isInactive = false;
  }
}

static void commanderWatchdogReset(void)
{
  lastUpdate = xTaskGetTickCount();
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

void commanderGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired)
{
  int usedSide = side;
  uint16_t chval = getSBusChannel(5);
  if(chval<1020)
  {
	  *eulerRollDesired  = targetVal[usedSide].roll;
	  *eulerPitchDesired = targetVal[usedSide].pitch;
	  *eulerYawDesired   = targetVal[usedSide].yaw;
	  thrst              = targetVal[usedSide].thrust;
	  rf_update = 0;
  }
  else
  {
	  if(chval < 1030)
		stabilizationModeRollPitch = RATE;
	  else
	  	stabilizationModeRollPitch = ANGLE;
	  uint16_t pr = getSBusChannel(6);
	  uint16_t in = getSBusChannel(7);
	  uint16_t de = getSBusChannel(8);

	  float pro = mep((float)pr, 352.0, 1696.0, 0.0, 200.0);
	  float ine = mep((float)in, 352.0, 1696.0, 0.0, 20.0);
	  float der = mep((float)de, 352.0, 1696.0, 0.0, 20.0);
	  DEBUG_PRINT("P - %f", pro);
	  controllerSetPID(pro, ine, der);

	  float pi, ro, ya;
	  getSBusVals(&ro, &pi, &ya, &thrst);
	  *eulerPitchDesired = pi;
	  *eulerRollDesired = ro;
	  *eulerYawDesired = ya;
	  setSBusUpdate(0);
	  commanderWatchdogReset();
  }

}

void commanderGetXYZA(float* x, float* y, float* z, float* a)
{
	int usedside = mc_side;

	*x = mocapPose[usedside].x;
	*y = mocapPose[usedside].y;
	*z = mocapPose[usedside].z;
	*a = mocapPose[usedside].a;
}

void commanderGetDest(float* x, float* y, float* z, float* a)
{
	int usedside = dt_side;

	*x = destPose[usedside].x;
	*y = destPose[usedside].y;
	*z = destPose[usedside].z;
	*a = destPose[usedside].a;
}

int commanderGetState()
{
	int usedside = st_side;
	return Cstat[usedside].curStat;
}

void commanderSetState(int s)
{
	int usedside = st_side;
	Cstat[usedside].curStat = s;
}

void commanderGetAltHold(bool* altHold, bool* setAltHold, float* altHoldChange)
{
  *altHold = altHoldMode; // Still in altitude hold mode
  *setAltHold = !altHoldModeOld && altHoldMode; // Hover just activated
  *altHoldChange = altHoldMode ? ((float) targetVal[side].thrust - 32767.f) / 32767.f : 0.0; // Amount to change altitude hold target
  altHoldModeOld = altHoldMode;
}

bool commanderGetAltHoldMode(void)
{
	return (altHoldMode);
}

void commanderSetAltHoldMode(bool altHoldModeNew)
{
	altHoldMode = altHoldModeNew;

	/**
	 * Dirty trick to ensure the altHoldChange variable remains zero after next call to commanderGetAltHold().
	 *
	 * This is needed since the commanderGetAltHold calculates the altHoldChange to -1 if altHoldMode is enabled
	 * with a simultaneous thrust command of 0.
	 *
	 * When altHoldChange is calculated to -1 when enabling altHoldMode, the altTarget will steadily decrease
	 * until thrust is commanded to correct the altitude, which is what we want to avoid.
	 */
	if(altHoldModeNew) {
	  targetVal[side].thrust = 32767;
	}
}

void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType)
{
  *rollType  = stabilizationModeRollPitch;
  *pitchType = stabilizationModeRollPitch;
  *yawType   = stabilizationModeYaw;
}

int commanderGetCmdType()
{
	return cmdMode;
}

/*void commanderGetThrust(uint16_t* thrust)
{
  int usedSide = side;
  uint16_t rawThrust = targetVal[usedSide].thrust;

  if (thrustLocked)
  {
    *thrust = 0;
  }
  else
  {
    if (rawThrust > MIN_THRUST)
    {
      *thrust = rawThrust;
    }
    else
    {
      *thrust = 0;
    }

    if (rawThrust > MAX_THRUST)
    {
      *thrust = MAX_THRUST;
    }
  }

  commanderWatchdog();
}*/

void commanderGetThrust(uint16_t* thrust, float yawR)
{
  int usedSide = side;
  uint16_t rawThrust = thrst;

  float yR = fabs(yawR);

  if (yR > 1200.0)
    start = 1;

  if (start < 500*2 && start !=0 && rawThrust<11000)
  {
	if (yR < 20)
	{
		*thrust = 15000;
		start += 1;
	}
  }
  else
  {
    start = 0;
    *thrust = rawThrust;
  }
  commanderWatchdog();
}

YawModeType commanderGetYawMode(void)
{
  return DEFUALT_YAW_MODE;//yawMode;
}

bool commanderGetYawModeCarefreeResetFront(void)
{
  return carefreeResetFront;
}

// Params for flight modes
PARAM_GROUP_START(flightmode)
PARAM_ADD(PARAM_UINT8, althold, &altHoldMode)
PARAM_ADD(PARAM_UINT8, yawMode, &yawMode)
PARAM_ADD(PARAM_UINT8, yawRst, &carefreeResetFront)
PARAM_ADD(PARAM_UINT8, stabModeRoll, &stabilizationModeRoll)
PARAM_ADD(PARAM_UINT8, stabModePitch, &stabilizationModePitch)
PARAM_ADD(PARAM_UINT8, stabModeYaw, &stabilizationModeYaw)
PARAM_GROUP_STOP(flightmode)
