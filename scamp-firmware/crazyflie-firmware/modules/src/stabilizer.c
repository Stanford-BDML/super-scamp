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
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "ledseq.h"
#include "param.h"
//#include "ms5611.h"
#include "lps25h.h"
#include "debug.h"

#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

// Barometer/ Altitude hold stuff
#define ALTHOLD_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 500hz

// state machine stuff
#define PERCHING_PITCH_UP_TIME (300)
#define PERCHING_MOTOR_TIMEOUT (PERCHING_PITCH_UP_TIME + 300)
#define PERCHING_DONE (PERCHING_MOTOR_TIMEOUT + 400)
#define REATTACHING_FAILED (200)
#define RECOVERY_DONE (400)
#define RETURNING_TO_FLIGHT_DONE (2000)
#define RETURNING_TO_FLIGHT_MAX_DONE (800)
#define SERVOS_RESET_TIME (500)
#define START_DECIDING_TIME (3000)
#define DECISION_TIME (5000)
#define WIGGLE_TIME (3000)

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;
static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;
static float rollRateDesired;
static float pitchRateDesired;
static float yawRateDesired;

// stateMachine variables
static uint16_t state = DECISION_STATE; // FLYING; //
static uint16_t perching_timer = 0;
static uint16_t reattaching_timer = 0;
static uint16_t recovering_timer = 0;
static uint16_t returningtoflight_timer = 0;
static uint16_t reset_servos_timer = 0;
static uint16_t auto_fly_timer = 0;
static uint16_t auto_fly_time = 600;
static uint16_t cool_down_time = 2000;
static uint16_t launch_time = 100;
static uint16_t decision_timer = 0;
static uint16_t wiggle_timer = 0;
static float oldx = 0;
static float perch_threshold = .5;

// Baro variables
static float temperature; // temp from barometer
static float pressure;    // pressure from barometer
static float asl;     // smoothed asl
static float aslRaw;  // raw asl
static float aslLong; // long term asl

// Altitude hold variables
static PidObject altHoldPID; // Used for altitute hold mode. I gets reset when the bat status changes
bool altHold = false;          // Currently in altitude hold mode
bool setAltHold = false;      // Hover mode has just been activated
static float accWZ     = 0.0;
static float flatMAG   = 0.0;
static float accMAG    = 0.0;
static float vSpeedASL = 0.0;
static float vSpeedAcc = 0.0;
static float vSpeed    = 0.0; // Vertical speed (world frame) integrated from vertical acceleration
static float altHoldPIDVal;                    // Output of the PID controller
static float altHoldErr;                       // Different between target and current altitude

// Altitude hold & Baro Params
static float altHoldKp              = 0.5;  // PID gain constants, used everytime we reinitialise the PID controller
static float altHoldKi              = 0.18;
static float altHoldKd              = 0.0;
static float altHoldChange          = 0;     // Change in target altitude
static float altHoldTarget          = -1;    // Target altitude
static float altHoldErrMax          = 1.0;   // max cap on current estimated altitude vs target altitude in meters
static float altHoldChange_SENS     = 200;   // sensitivity of target altitude change (thrust input control) while hovering. Lower = more sensitive & faster changes
static float pidAslFac              = 13000; // relates meters asl to thrust
static float pidAlpha               = 0.8;   // PID Smoothing //TODO: shouldnt need to do this
static float vSpeedASLFac           = 0;    // multiplier
static float vSpeedAccFac           = -48;  // multiplier
static float vAccDeadband           = 0.05;  // Vertical acceleration deadband
static float vSpeedASLDeadband      = 0.005; // Vertical speed based on barometer readings deadband
static float vSpeedLimit            = 0.05;  // used to constrain vertical velocity
static float errDeadband            = 0.00;  // error (target - altitude) deadband
static float vBiasAlpha             = 0.98; // Blending factor we use to fuse vSpeedASL and vSpeedAcc
static float aslAlpha               = 0.92; // Short term smoothing
static float aslAlphaLong           = 0.93; // Long term smoothing
static uint16_t altHoldMinThrust    = 00000; // minimum hover thrust - not used yet
static uint16_t altHoldBaseThrust   = 43000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
static uint16_t altHoldMaxThrust    = 60000; // max altitude hold thrust


RPYType rollType;
RPYType pitchType;
RPYType yawType;

uint16_t actuatorThrust;
int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;

uint32_t motor_thrust = 0; // 10000;

static uint16_t launch_thrust = 60000;
static uint16_t auto_fly_thrust = 60000;
static uint16_t cool_down_thrust = 42000;
static uint16_t cool_down_thrust_dec = 42000;
int16_t launch_pitch = 0;
int16_t auto_fly_pitch = 30;
int16_t cool_down_pitch = 5;
static uint16_t cool_down_decrement = 30;

static bool isInit;

static void stabilizerAltHoldUpdate(void);
static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static float constrain(float value, const float minVal, const float maxVal);
static float deadband(float value, const float threshold);

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit();
  imu6Init();
  sensfusion6Init();
  controllerInit();

  rollRateDesired = 0;
  pitchRateDesired = 0;
  yawRateDesired = 0;

  xTaskCreate(stabilizerTask, (const signed char * const)STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= controllerTest();

  return pass;
}

static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  uint32_t altHoldCounter = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

    // Magnetometer not yet used more then for logging.
    imu9Read(&gyro, &acc, &mag);

    if (imu6IsCalibrated())
    {
    	// set RPY and T values for each state
    	switch(state)
    	{
//    	case FLYING:
//    		commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
//			commanderGetThrust(&actuatorThrust);
//			break;
    	case DECISION_STATE:
    		decision_timer++;
    		if(decision_timer > START_DECIDING_TIME)
    		{
				if(acc.x < -.5)
				{
					state = CLIMBING;
					decision_timer = 0;
				} else if (acc.z < -.5)
				{
					state = WIGGLE_SERVOS;
					decision_timer = 0;
				}
			}

    		if(decision_timer > DECISION_TIME)
    		{
    			state = FLYING;
    			decision_timer = 0;
    		}
    		commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
    		commanderGetThrust(&actuatorThrust);
    		break;
    	case WIGGLE_SERVOS:
    		wiggle_timer++;
    		if(wiggle_timer > WIGGLE_TIME)
    		{
    			state = AUTO_FLY;
    			wiggle_timer = 0;
    		}
    		commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
    		actuatorThrust = 10000;
    		break;
    	case TOWARDS_WALL:
    		flatMAG = sensfusion6GetAccXYMag2WRTGravity(acc.x, acc.y, acc.z);
    		if((acc.x < (-1.0*perch_threshold/2.0)) && (flatMAG > perch_threshold))
    		{
    			//	don't switch if the impact v is too high, just try and bump it again
    			if (acc.x > -2.5) // || pitch > 0
    			{
				state = PERCHING;
    			}
    		}
    		commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
    		commanderGetThrust(&actuatorThrust);
    		break;
    	case PERCHING:
    		perching_timer++;
    		uint16_t perchingThrust = 0;
    		if (perching_timer < PERCHING_MOTOR_TIMEOUT)
    		{
    			if ((perching_timer > PERCHING_PITCH_UP_TIME) && (acc.x < -1.0))
    			{
    				perching_timer = PERCHING_MOTOR_TIMEOUT;
    			}
    			perchingThrust = 60000;
    		} 
    		else if (perching_timer > PERCHING_DONE)
    		{
    			state = CLIMBING;
    			perching_timer = 0;
    		} else if (acc.x > -.5)
    		{
    			state = RECOVERING;
    			perching_timer = 0;
    		}
			eulerRollDesired = 0;
			eulerPitchDesired = -90;
			eulerYawDesired = 0;
			actuatorThrust = perchingThrust;
    		break;
    	case CLIMBING:
    		if(acc.x > -.5)
    		{
    			state = REATTACHING;
    		}
    		eulerRollDesired = 0;
    		eulerPitchDesired = 0;
    		eulerYawDesired = 0;
    		actuatorThrust = 0;
    		break;
    	case REATTACHING:
    		reattaching_timer++;
    		if(reattaching_timer > REATTACHING_FAILED)
    		{
    			state = RECOVERING;
    			reattaching_timer = 0;
    		}
    		if(acc.x <= -.5)
    		{
    			state = CLIMBING;
    			reattaching_timer = 0;
    		}
    		eulerRollDesired = 0;
    		eulerPitchDesired = 0;
    		eulerYawDesired = 0;
    		actuatorThrust = 0;
    		break;
    	case RECOVERING:
    		recovering_timer++;
    		uint16_t recoveringThrust = 0;
    		if(recovering_timer < RECOVERY_DONE)
    		{
    			recoveringThrust = 48000;
    		}
    		else
    		{
    			state = FLYING;
    			recovering_timer = 0;
    			recoveringThrust = 0;
    		}
    		eulerRollDesired = 0;
    		eulerPitchDesired = 0;
    		eulerYawDesired = 0;
    		actuatorThrust = recoveringThrust;
    		break;
    	case RETURNING_TO_FLIGHT:
    		returningtoflight_timer++;
    		uint16_t returningtoflightThrust = 0;
    		if(returningtoflight_timer < RETURNING_TO_FLIGHT_MAX_DONE)
    		{
    			returningtoflightThrust = 60000;
    		}
    		else if (returningtoflight_timer < RETURNING_TO_FLIGHT_DONE)
    		{
    			returningtoflightThrust = 42000;
    		}
    		else
    		{
    			state = FLYING;
    			returningtoflight_timer = 0;
    			returningtoflightThrust = 0;
    		}
    		eulerRollDesired = 0;
    		eulerPitchDesired = 0;
    		eulerYawDesired = 0;
    		actuatorThrust = returningtoflightThrust;
    		break;
    	case TAKEOFF:
    		if(acc.x > -.5)
    		{
    			state = RETURNING_TO_FLIGHT;
    		}
    		eulerRollDesired = 0;
    		eulerPitchDesired = 0;
    		eulerYawDesired = 0;
    		actuatorThrust = 10000;
    		break;
    	case RESET_SERVOS:
    		reset_servos_timer++;
    		if(reset_servos_timer > SERVOS_RESET_TIME)
    		{
    			state = FLYING;
    			reset_servos_timer = 0;
    		}
    		eulerRollDesired = 0;
    		eulerPitchDesired = 0;
    		eulerYawDesired = 0;
    		actuatorThrust = 0;
    		break;
    	case AUTO_FLY:
    		auto_fly_timer++;
    		flatMAG = sensfusion6GetAccXYMag2WRTGravity(acc.x, acc.y, acc.z);
    		if((auto_fly_timer<auto_fly_time) && (acc.x < (-1.0*perch_threshold/2.0)) && (flatMAG > perch_threshold))
    		{
    			//	don't switch if the impact v is too high, just try and bump it again
    			if (acc.x > -2.5) // || pitch > 0
    			{
				state = PERCHING;
				auto_fly_timer = 0;
    			}
    		}
    		if(auto_fly_timer < launch_time)
    		{
    		    cool_down_thrust_dec = cool_down_thrust;
    			eulerPitchDesired = -1*launch_pitch;
    			actuatorThrust = launch_thrust;
    		} else
    		{
    			eulerPitchDesired = -1*auto_fly_pitch;
    			actuatorThrust = auto_fly_thrust;
    		}
    		if(auto_fly_timer > auto_fly_time)
    		{
    			cool_down_thrust_dec-=cool_down_decrement;
    			actuatorThrust = cool_down_thrust_dec;
    			eulerPitchDesired = cool_down_pitch;
    		}
    		if(auto_fly_timer > cool_down_time)
    		{
    			state = FLYING;
    			reset_servos_timer = 0;
    			actuatorThrust = 0;
    			eulerPitchDesired = 0;
    			auto_fly_timer = 0;
    		}
    		eulerRollDesired = 0;
			eulerYawDesired = 0;
    		break;
    	default:
    		commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
    		commanderGetThrust(&actuatorThrust);
    	}
    	oldx = acc.x;
    	commanderGetRPYType(&rollType, &pitchType, &yawType);

      // 250HZ
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);

        accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);
        accMAG = (acc.x*acc.x) + (acc.y*acc.y) + (acc.z*acc.z);
        
        // Estimate speed from acc (drifts)
        vSpeed += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT;

        controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
                                     eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
                                     &rollRateDesired, &pitchRateDesired, &yawRateDesired);
        attitudeCounter = 0;
      }

      // 100HZ
      if (imuHasBarometer() && (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER))
      {
        stabilizerAltHoldUpdate();
        altHoldCounter = 0;
      }

      if (rollType == RATE)
      {
        rollRateDesired = eulerRollDesired;
      }
      if (pitchType == RATE)
      {
        pitchRateDesired = eulerPitchDesired;
      }
      if (yawType == RATE)
      {
        yawRateDesired = -eulerYawDesired;
      }

      // TODO: Investigate possibility to subtract gyro drift.
      controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
                               rollRateDesired, pitchRateDesired, yawRateDesired);

      controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

/*      if (!altHold || !imuHasBarometer())
      {
        // Use thrust from controller if not in altitude hold mode
		commanderGetThrust(&actuatorThrust);
      }
      else
      {
        // Added so thrust can be set to 0 while in altitude hold mode after disconnect
        commanderWatchdog();
      }*/

      if (actuatorThrust > 0)
      {
#if defined(TUNE_ROLL)
        distributePower(actuatorThrust, actuatorRoll, 0, 0);
#elif defined(TUNE_PITCH)
        distributePower(actuatorThrust, 0, actuatorPitch, 0);
#elif defined(TUNE_YAW)
        distributePower(actuatorThrust, 0, 0, -actuatorYaw);
#else
        distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);
#endif
      }
      else
      {
        distributePower(0, 0, 0, 0);
        controllerResetAllPID();
      }
    }
  }
}

static void stabilizerAltHoldUpdate(void)
{
  // Get altitude hold commands from pilot
  commanderGetAltHold(&altHold, &setAltHold, &altHoldChange);

  // Get barometer height estimates
  //TODO do the smoothing within getData
  lps25hGetData(&pressure, &temperature, &aslRaw);

  asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
  aslLong = aslLong * aslAlphaLong + aslRaw * (1 - aslAlphaLong);

  // Estimate vertical speed based on successive barometer readings. This is ugly :)
  vSpeedASL = deadband(asl - aslLong, vSpeedASLDeadband);

  // Estimate vertical speed based on Acc - fused with baro to reduce drift
  vSpeed = constrain(vSpeed, -vSpeedLimit, vSpeedLimit);
  vSpeed = vSpeed * vBiasAlpha + vSpeedASL * (1.f - vBiasAlpha);
  vSpeedAcc = vSpeed;

  // Reset Integral gain of PID controller if being charged
  if (!pmIsDischarging())
  {
    altHoldPID.integ = 0.0;
  }

  // Altitude hold mode just activated, set target altitude as current altitude. Reuse previous integral term as a starting point
  if (setAltHold)
  {
    // Set to current altitude
    altHoldTarget = asl;

    // Cache last integral term for reuse after pid init
    const float pre_integral = altHoldPID.integ;

    // Reset PID controller
    pidInit(&altHoldPID, asl, altHoldKp, altHoldKi, altHoldKd,
            ALTHOLD_UPDATE_DT);
    // TODO set low and high limits depending on voltage
    // TODO for now just use previous I value and manually set limits for whole voltage range
    //                    pidSetIntegralLimit(&altHoldPID, 12345);
    //                    pidSetIntegralLimitLow(&altHoldPID, 12345);              /

    altHoldPID.integ = pre_integral;

    // Reset altHoldPID
    altHoldPIDVal = pidUpdate(&altHoldPID, asl, false);
  }

  // In altitude hold mode
  if (altHold)
  {
    // Update target altitude from joy controller input
    altHoldTarget += altHoldChange / altHoldChange_SENS;
    pidSetDesired(&altHoldPID, altHoldTarget);

    // Compute error (current - target), limit the error
    altHoldErr = constrain(deadband(asl - altHoldTarget, errDeadband),
                           -altHoldErrMax, altHoldErrMax);
    pidSetError(&altHoldPID, -altHoldErr);

    // Get control from PID controller, dont update the error (done above)
    // Smooth it and include barometer vspeed
    // TODO same as smoothing the error??
    altHoldPIDVal = (pidAlpha) * altHoldPIDVal + (1.f - pidAlpha) * ((vSpeedAcc * vSpeedAccFac) +
                    (vSpeedASL * vSpeedASLFac) + pidUpdate(&altHoldPID, asl, false));

    // compute new thrust
    actuatorThrust =  max(altHoldMinThrust, min(altHoldMaxThrust,
                          limitThrust( altHoldBaseThrust + (int32_t)(altHoldPIDVal*pidAslFac))));

    // i part should compensate for voltage drop

  }
  else
  {
    altHoldTarget = 0.0;
    altHoldErr = 0.0;
    altHoldPIDVal = 0.0;
  }
}

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
#ifdef QUAD_FORMATION_X
	  int16_t r = roll >> 1;
	  int16_t p = pitch >> 1;
	  if (state == REATTACHING)
	  {
		  motorPowerM1 = 50000;
		  motorPowerM2 = 55000;
		  motorPowerM3 = 55000;
		  motorPowerM4 = 50000;
	  }
	  else if ((state == CLIMBING)||(state == TEST_MOTORS))
	  {
		  motorPowerM1 = motor_thrust;
		  motorPowerM2 = motor_thrust;
		  motorPowerM3 = motor_thrust;
		  motorPowerM4 = motor_thrust;

	  }
//	  else if (state == TAKEOFF)
//	  {
//		  motorPowerM1 = motor_thrust;
//		  motorPowerM2 = 0;
//		  motorPowerM3 = 0;
//		  motorPowerM4 = motor_thrust;
//	  }
	  else
	  {
		  motorPowerM1 = limitThrust(thrust - r + p + yaw);
		  motorPowerM2 = limitThrust(thrust - r - p - yaw);
		  motorPowerM3 = limitThrust(thrust + r - p + yaw);
		  motorPowerM4 = limitThrust(thrust + r + p - yaw);
	  }
#else // QUAD_FORMATION_NORMAL
  motorPowerM1 = limitThrust(thrust + pitch + yaw);
  motorPowerM2 = limitThrust(thrust - roll - yaw);
  motorPowerM3 =  limitThrust(thrust - pitch + yaw);
  motorPowerM4 =  limitThrust(thrust + roll - yaw);
#endif

  motorsSetRatio(MOTOR_M1, motorPowerM1);
  motorsSetRatio(MOTOR_M2, motorPowerM2);
  motorsSetRatio(MOTOR_M3, motorPowerM3);
  motorsSetRatio(MOTOR_M4, motorPowerM4);
}

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

// Constrain value between min and max
static float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

// Deadzone
static float deadband(float value, const float threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
//LOG_ADD(LOG_FLOAT, commanded_pitch, &eulerPitchDesired)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, zw, &accWZ)
LOG_ADD(LOG_FLOAT, flatmag, &flatMAG)
LOG_ADD(LOG_FLOAT, mag2, &accMAG)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)

// LOG altitude hold PID controller states
//LOG_GROUP_START(vpid)
//LOG_ADD(LOG_FLOAT, pid, &altHoldPID)
//LOG_ADD(LOG_FLOAT, p, &altHoldPID.outP)
//LOG_ADD(LOG_FLOAT, i, &altHoldPID.outI)
//LOG_ADD(LOG_FLOAT, d, &altHoldPID.outD)
//LOG_GROUP_STOP(vpid)

//LOG_GROUP_START(baro)
//LOG_ADD(LOG_FLOAT, asl, &asl)
//LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
//LOG_ADD(LOG_FLOAT, aslLong, &aslLong)
//LOG_ADD(LOG_FLOAT, temp, &temperature)
//LOG_ADD(LOG_FLOAT, pressure, &pressure)
//LOG_GROUP_STOP(baro)

//LOG_GROUP_START(altHold)
//LOG_ADD(LOG_FLOAT, err, &altHoldErr)
//LOG_ADD(LOG_FLOAT, target, &altHoldTarget)
//LOG_ADD(LOG_FLOAT, zSpeed, &vSpeed)
//LOG_ADD(LOG_FLOAT, vSpeed, &vSpeed)
//LOG_ADD(LOG_FLOAT, vSpeedASL, &vSpeedASL)
//LOG_ADD(LOG_FLOAT, vSpeedAcc, &vSpeedAcc)
//LOG_GROUP_STOP(altHold)

LOG_GROUP_START(stateMachine)
LOG_ADD(LOG_UINT16, state, &state)
LOG_GROUP_STOP(stateMachine)

// Params for altitude hold
//PARAM_GROUP_START(altHold)
//PARAM_ADD(PARAM_FLOAT, aslAlpha, &aslAlpha)
//PARAM_ADD(PARAM_FLOAT, aslAlphaLong, &aslAlphaLong)
//PARAM_ADD(PARAM_FLOAT, errDeadband, &errDeadband)
//PARAM_ADD(PARAM_FLOAT, altHoldChangeSens, &altHoldChange_SENS)
//PARAM_ADD(PARAM_FLOAT, altHoldErrMax, &altHoldErrMax)
//PARAM_ADD(PARAM_FLOAT, kd, &altHoldKd)
//PARAM_ADD(PARAM_FLOAT, ki, &altHoldKi)
//PARAM_ADD(PARAM_FLOAT, kp, &altHoldKp)
//PARAM_ADD(PARAM_FLOAT, pidAlpha, &pidAlpha)
//PARAM_ADD(PARAM_FLOAT, pidAslFac, &pidAslFac)
//PARAM_ADD(PARAM_FLOAT, vAccDeadband, &vAccDeadband)
//PARAM_ADD(PARAM_FLOAT, vBiasAlpha, &vBiasAlpha)
//PARAM_ADD(PARAM_FLOAT, vSpeedAccFac, &vSpeedAccFac)
//PARAM_ADD(PARAM_FLOAT, vSpeedASLDeadband, &vSpeedASLDeadband)
//PARAM_ADD(PARAM_FLOAT, vSpeedASLFac, &vSpeedASLFac)
//PARAM_ADD(PARAM_FLOAT, vSpeedLimit, &vSpeedLimit)
//PARAM_ADD(PARAM_UINT16, baseThrust, &altHoldBaseThrust)
//PARAM_ADD(PARAM_UINT16, maxThrust, &altHoldMaxThrust)
//PARAM_ADD(PARAM_UINT16, minThrust, &altHoldMinThrust)
//PARAM_GROUP_STOP(altHold)

PARAM_GROUP_START(stateMachine)
PARAM_ADD(PARAM_UINT16, state, &state)
PARAM_ADD(PARAM_FLOAT, perch_t, &perch_threshold)
PARAM_GROUP_STOP(stateMachine)

PARAM_GROUP_START(thrust)
PARAM_ADD(PARAM_INT32, thrust, &motor_thrust)
PARAM_ADD(PARAM_UINT16, lthrust, &launch_thrust)
PARAM_ADD(PARAM_UINT16, auto_thrust, &auto_fly_thrust)
PARAM_ADD(PARAM_UINT16, cool_thrust, &cool_down_thrust)
PARAM_ADD(PARAM_UINT16, cool_dec, &cool_down_decrement)
PARAM_GROUP_STOP(thrust)

PARAM_GROUP_START(auto_fly)
PARAM_ADD(PARAM_INT16, lpitch, &launch_pitch)
PARAM_ADD(PARAM_INT16, auto_pitch, &auto_fly_pitch)
PARAM_ADD(PARAM_UINT16, auto_fly_time, &auto_fly_time)
PARAM_ADD(PARAM_UINT16, launch_time, &launch_time)
PARAM_ADD(PARAM_INT16, cool_pitch, &cool_down_pitch)
PARAM_ADD(PARAM_UINT16, cool_time, &cool_down_time)
PARAM_GROUP_STOP(auto_fly)




