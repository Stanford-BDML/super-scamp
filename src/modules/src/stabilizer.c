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
#include "estimator_kalman.h"

#include "config.h"
#include "system.h"

#include "stabilizer.h"
#include "vl53l0x.h"
#include "commander.h"
//#include "controller.h"
#include "sensfusion6.h"
#include "sensors.h"
#include "crtp.h"
#include "position_estimator.h"
#include "buzzer.h"

#include "log.h"
//#include "pid.h"
#include "ledseq.h"
#include "param.h"
//#include "ms5611.h"
//#include "lps25h.h"
#include "debug.h"

#include "AttitudeController.h"
//#include "Z_estimator.

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  1
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

//static Axis3f gyro; // Gyro axis data in deg/s
//static Axis3f acc;  // Accelerometer axis data in mG
//static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;
static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;
static float rollRateDesired;
static float pitchRateDesired;
static float yawRateDesired;
static float AltitudeDesired;
static float ERD_app,EPD_app;

// stateMachine variables

static uint16_t STATE_MACHINE = LANDED; // FLYING; //
static uint16_t perching_timer = 0;
static uint16_t perching_attached_timer = 0;
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



static float accWZ  = 0.0;


static uint16_t actuatorThrust;

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

static void stabilizerTask(void* param);
//static float constrain(float value, const float minVal, const float maxVal);
//static float deadband(float value, const float threshold);

static struct AttitudeController AC;

static struct HeightController HC;
//static struct selfState_s EZ;



//GPIO_InitTypeDef DebugPin;
//bool debugFreq;



void stabilizerInit(void)
{
  if(isInit)
    return;


  motorsInit(motorMapDefaltConBrushless);
  sensorsInit();
  stateEstimatorInit();
  //imu6Init();
  sensfusion6Init();
  //controllerInit();
  //stateEstimatorInit();
  initUsecTimer();


  #if defined(SITAW_ENABLED)
	  sitAwInit();
  #endif


  set_AC(&AC);
  set_HC(&HC);
  
  rollRateDesired = 0;
  pitchRateDesired = 0;
  yawRateDesired = 0;


  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
                STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

zDistance_t ZRANGE_STAB;
bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  //pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= stateEstimatorTest();
  pass &= AC.isInit;
  pass &= HC.isInit;
  pass &= sensorsTest();

   return pass;

}

static state_t STATE;
static sensorData_t SENSORS;
static control_t CONTROL;

static uint8_t State_Joy;
static uint16_t M5power = 0; // maps to some PWM to power M5


// This function process the input provided by the joystick (State_Joy)
void processJoy()
{
	if (STATE_MACHINE==LANDED)
	{
		/*** TEMPORARY START ***/
		if (State_Joy & 1) // L1 button
		{
			M5power += 1000; // 32767 is probably 100% d.c.
			motor5SetRatio(M5power);
			move_takeoffarm(170); // switched for debugging. should be 150
		}
		if (State_Joy & 2) // L2 button
		{
			M5power = 32767;
			motor5SetRatio(M5power);
			move_takeoffarm(150); // switched for debugging. should be 170
		}
		if (State_Joy & 64) // Circle button
		{
			// emergency stop motor
			M5power = 0;
			motor5SetRatio(M5power);
		}
		/*** TEMP END ***/

		if (State_Joy & 16) // X button
		{
			STATE_MACHINE=FLYING;
		}
		if (State_Joy & 128) // Triangle button
		{
			STATE_MACHINE=DETACHING;
		}
		if (State_Joy & 32) // Square button
		{
			STATE_MACHINE=CLIMBING;
		}
	}
	
	if (STATE_MACHINE==FLYING)
	{
		if (State_Joy & 1) // L1 button
		{
			// increase altitude
			AltitudeDesired=AltitudeDesired+0.8f/RATE_MAIN_LOOP;
		}
		if (State_Joy & 2) // L2 button
		{
			// decrease altitude
			AltitudeDesired=AltitudeDesired-0.8f/RATE_MAIN_LOOP;
		}
		if (AltitudeDesired>5)
		{
			AltitudeDesired=5;
		}
		if (AltitudeDesired<-1)
		{
			AltitudeDesired=-1;
		}
		yawRateDesired=0;
		if (State_Joy & 4) // R1 button
		{
			// increase yaw
			yawRateDesired=25;
			eulerYawDesired=eulerYawActual;
		}
		if (State_Joy & 8) // R2 button
		{
			// decrease yaw
			yawRateDesired=-35;
			eulerYawDesired=eulerYawActual;
		}
		if (State_Joy & 32) // Square button
		{
			STATE_MACHINE=TOWARDS_WALL;
		}

		float Yawrad = eulerYawActual*(M_PI_F / 180.0f);
		ERD_app=cosf(Yawrad)*eulerRollDesired-sinf(Yawrad)*eulerPitchDesired;
		EPD_app=+sinf(Yawrad)*eulerRollDesired+cosf(Yawrad)*eulerPitchDesired;

	}
	if (STATE_MACHINE==TOWARDS_WALL)
	{

		if (State_Joy & 16)
		{
			STATE_MACHINE=FLYING;
		}

	}
	if (STATE_MACHINE==PERCHING_1)
	{

		if (State_Joy & 128)
		{
			STATE_MACHINE=PERCHING_2;
		}

	}
	if (STATE_MACHINE==CLIMBING)
	{

		if (State_Joy & 128)
		{
			// Deploy takeoff arm to ready to flip and takeoff
			STATE_MACHINE=DEPLOY_ARM;
		}

	}
	if (STATE_MACHINE==DEPLOY_ARM)
	{
		if (State_Joy & 16)
		{
			STATE_MACHINE=RETRACT_ARM;
		}
	}

	if (STATE_MACHINE==RETRACT_ARM)
	{
		if (State_Joy & 32)
		{
			STATE_MACHINE=DEPLOY_ARM;
		}
	}

	if (State_Joy & 64)
	{
		STATE_MACHINE=LANDED;
	}


}

static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  uint32_t altHoldCounter = 0;
  uint32_t lastWakeTime;
  float dt_loop=1/(float)RATE_500_HZ;
  float normG;
  bool Zsensor=false;

  eulerRollDesired = 0;
  eulerPitchDesired = 0;
  eulerYawDesired = 0;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

	// Wait for sensors to be calibrated
	lastWakeTime = xTaskGetTickCount ();
	while(!sensorsAreCalibrated()) {
	  vTaskDelayUntil(&lastWakeTime, F2T(RATE_500_HZ));
	}
// Initialize tick to something else then 0

	bool Flying = false;
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_500_HZ)); // 1000Hz

		// read sensors
		stateEstimatorUpdate(&STATE, &SENSORS, &CONTROL);
		// compute roll, pitch and yaw angles
		sensfusion6UpdateQ(SENSORS.gyro.x, SENSORS.gyro.y, SENSORS.gyro.z,SENSORS.acc.x,SENSORS.acc.y, SENSORS.acc.z, dt_loop);
		sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
		accWZ = sensfusion6GetAccZWithoutGravity(SENSORS.acc.x, SENSORS.acc.y, SENSORS.acc.z);
		positionUpdateVelocity(accWZ,dt_loop);

		//normG = sqrt(SENSORS.acc.x*SENSORS.acc.x + SENSORS.acc.y*SENSORS.acc.y +SENSORS.acc.z*SENSORS.acc.z);
		
		// process joystick input
		processJoy();

		switch(STATE_MACHINE)
		{
			
			case LANDED:
				Flying=false;
				AltitudeDesired=0;
				//
				M5power = 0;
				motor5SetRatio(M5power);
				//
				break;

			case FLYING:
				Flying=true;
				if (AltitudeDesired==-1)
					STATE_MACHINE=LANDED;
				break;
				
			case TOWARDS_WALL:

					if(SENSORS.acc.y>1.5f)
					{
						STATE_MACHINE = PERCHING_1;
						perching_timer = 0;
						perching_attached_timer=0;
					}
					 ERD_app=3;
					 EPD_app=0;
					 Flying=true;
			    	break;
					
			case PERCHING_1:
				Flying=true;
				ERD_app=8;
				EPD_app=0;
				
				break;
			case PERCHING_2:
				Flying=false;

				perching_vel(eulerRollActual,eulerPitchActual,eulerYawActual,eulerYawDesired, SENSORS.gyro.x, SENSORS.gyro.y, SENSORS.gyro.z, 500.0,90.0);
				
				if (!crtpIsConnected())
					motorSafe();
				break;
				
			// DETACHING is the FLIPPING: I will change the variable name	
			case DETACHING:
			
			    // To respect the convention
				if (eulerRollActual<0)
						eulerRollActual = eulerRollActual + 360;
					
				flipping(eulerRollActual,eulerPitchActual, SENSORS.gyro.x, SENSORS.gyro.y, SENSORS.gyro.z, 500.0,85.0);
				// When ROLL is smaller than 90°, turn off the motor 
				if (eulerRollActual<90.0f)
				{
					STATE_MACHINE = LANDED;
				}
				break;


			// these states are related to climbing
			case ATTACHING:
				Flying=false;
				setMotor(0.75);
				perching_attached_timer++;
				if (perching_attached_timer>250)
				{
					perching_attached_timer=0;
					
				}

				break;
			case CLIMBING:
				Flying=false;

				if(SENSORS.acc.y > 0.8f)
					turnOFFMotor();
				else
					setMotor(0.8);

				break;

			case DEPLOY_ARM:
				Flying=false;

				move_takeoffarm(170); // temp: should be 150

				// temp
				M5power += 1000; // 32767 is probably 100% d.c.
				motor5SetRatio(M5power);
				//
				break;
			case RETRACT_ARM:
				Flying=false;

				move_takeoffarm(150); // temp: should be 170
				// temp
				M5power = 32767;
				motor5SetRatio(M5power);
				//

			default:
				break;
		}
		
		// If Scamp fly
		if (Flying)
		{
			// test the z-ranger connection
			if(!vl53l0xTestConnection())
				turnOFFMotor();
			else
			{
				// read sensor and compute the Altitude controller
				Zsensor=vl53l0xReadRange2(&ZRANGE_STAB);
				compute_HC(&HC,ZRANGE_STAB.distance,AltitudeDesired,getVelocityPE(),Zsensor);

			}

			// Attitude controller
			compute_AC(&AC,eulerRollActual,eulerPitchActual,eulerYawActual,
    		  ERD_app-2,EPD_app-3,eulerYawDesired,yawRateDesired,
						SENSORS.gyro.x, SENSORS.gyro.y, SENSORS.gyro.z);
			
			// if the radio is connected, actuate the motor.
			if (crtpIsConnected())
				ActuateMotor(&AC,&HC,STATE.attitude.roll,STATE.attitude.pitch,&CONTROL);
			else
			{
				motorSafe();
				//ActuateMotor(&AC,&HC,STATE.attitude.roll,STATE.attitude.pitch,&CONTROL);
			}
		}



  }
}


// ############# LOG SECTION #############
/*
LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
//LOG_ADD(LOG_FLOAT, commanded_pitch, &eulerPitchDesired)
LOG_GROUP_STOP(stabilizer)
/*
/*
LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, zw, &accWZ)
LOG_ADD(LOG_FLOAT, flatmag, &flatMAG)
LOG_ADD(LOG_FLOAT, mag2, &accMAG)
LOG_GROUP_STOP(acc)*/

/*
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

*/

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
LOG_ADD(LOG_UINT16, state, &STATE_MACHINE)
LOG_ADD(LOG_UINT8, State_Joy, &State_Joy)
LOG_ADD(LOG_FLOAT, ERD, &eulerRollDesired)
LOG_ADD(LOG_FLOAT, EPD, &eulerPitchDesired)
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
PARAM_ADD(PARAM_UINT16, state, &STATE_MACHINE)
PARAM_ADD(PARAM_FLOAT, ERD, &eulerRollDesired)
PARAM_ADD(PARAM_FLOAT, EPD, &eulerPitchDesired)
PARAM_ADD(PARAM_UINT8, State_Joy, &State_Joy)
PARAM_GROUP_STOP(stateMachine)

/*

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


// Riattivare queste per fare andare tutto
LOG_GROUP_START(posEstimatorAlt)
LOG_ADD(LOG_FLOAT, estimatedZ, &(EZ.estimatedZ))
LOG_ADD(LOG_FLOAT, estVZ, &EZ.estimatedVZ)
LOG_ADD(LOG_FLOAT, velocityZ, &EZ.velocityZ)
LOG_GROUP_STOP(posEstimatorAlt)
*/

static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

/*
LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)
*/


LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &CONTROL.thrust)
LOG_ADD(LOG_FLOAT, AltitudeDesired, &AltitudeDesired)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &SENSORS.acc.x)
LOG_ADD(LOG_FLOAT, y, &SENSORS.acc.y)
LOG_ADD(LOG_FLOAT, z, &SENSORS.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &SENSORS.accSec.x)
LOG_ADD(LOG_FLOAT, y, &SENSORS.accSec.y)
LOG_ADD(LOG_FLOAT, z, &SENSORS.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &SENSORS.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &SENSORS.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &SENSORS.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &SENSORS.gyro.x)
LOG_ADD(LOG_FLOAT, y, &SENSORS.gyro.y)
LOG_ADD(LOG_FLOAT, z, &SENSORS.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif


LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &SENSORS.mag.x)
LOG_ADD(LOG_FLOAT, y, &SENSORS.mag.y)
LOG_ADD(LOG_FLOAT, z, &SENSORS.mag.z)
LOG_GROUP_STOP(mag)


LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &CONTROL.yaw)
LOG_ADD(LOG_FLOAT, z_fl,&(ZRANGE_STAB.distance))
LOG_GROUP_STOP(controller)

LOG_GROUP_START(stateEstimate)
LOG_ADD(LOG_FLOAT, x, &SENSORS.mag.x)
LOG_ADD(LOG_FLOAT, y,&SENSORS.mag.x)
LOG_ADD(LOG_FLOAT, z, &SENSORS.mag.x)
LOG_GROUP_STOP(stateEstimate)
