#include <math.h>
#include "motors.h"
#include "log.h"
#include <stdlib.h>
//#include "pm.h"

#define M_PI_F 3.14159265358979323846f

/*
  float appOC_Qpqr[SIZE_DF];
	float appIC_invGQpqr[SIZE_DF];
	float appOC_invGQpqr[SIZE_DF];

	// Tuning of the roll
	appIC_Qpqr[0]=0;
	appIC_Qpqr[1]= 0.012262903422;
	appIC_Qpqr[2]= 0.010547278666;
	appIC_Qpqr[3]= 0;

	appOC_Qpqr[0]=0;
	appOC_Qpqr[1]= 1.613296276384;
	appOC_Qpqr[2]= -0.636106458471;
	appOC_Qpqr[3]= 0;
  */

#define SIZE_DF 3


uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;


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

static uint16_t limitThrust_2(int32_t value)
{

  int32_t FSR=32767;
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < FSR)
  {
    value = FSR;
  }

  return (uint16_t)value;
}

static uint16_t limitThrust_3(int32_t value)
{

  int32_t FSR=32767;
  if(value > FSR)
  {
    value = FSR;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}


struct DigitalFilter
{
	float INdelays[SIZE_DF];
	float OUTdelays[SIZE_DF];
	float INcoeff[SIZE_DF];
	float OUTcoeff[SIZE_DF];
};

void reset_DF(struct DigitalFilter* DF)
{
	int i=0;
	for(i=0;i<SIZE_DF;i++)
	{
		DF->INdelays[i]=0;
		DF->OUTdelays[i]=0;
	}
}

void set_coeff_DF(struct DigitalFilter* DF,float* IC,float* OC)
{
	int i=0;
	for(i=0;i<SIZE_DF;i++)
	{
			DF->INcoeff[i]=IC[i];
			DF->OUTcoeff[i]=OC[i];
	}
	// This coefficient is always equal to 0
	DF->OUTcoeff[0]=0;
}

float compute_DF(struct DigitalFilter* DF,float IN)
{
	int i=0;

	// Update the inputs e outputs
	/*for(i=SIZE_DF-1;i>=1;i--)
	{
			DF->INdelays[i]=DF->INdelays[i-1];
			DF->OUTdelays[i]=DF->OUTdelays[i-1];
	}*/
	/*
	DF->INdelays[3]=DF->INdelays[2];
  */
	DF->INdelays[2]=DF->INdelays[1];
	DF->INdelays[1]=DF->INdelays[0];
	DF->INdelays[0]=IN;

	//DF->OUTdelays[3]=DF->OUTdelays[2];
	DF->OUTdelays[2]=DF->OUTdelays[1];
	DF->OUTdelays[1]=DF->OUTdelays[0];
	//DF->OUTdelays[0]=0;


	DF->OUTdelays[0] = (DF->INcoeff[0]*DF->INdelays[0]) + (DF->OUTdelays[1]*DF->OUTcoeff[1]) + (DF->INcoeff[1]*DF->INdelays[1]) + (DF->OUTdelays[2]*DF->OUTcoeff[2]) + (DF->INcoeff[2]*DF->INdelays[2]);
	//if ((DF->OUTdelays[0]>0.2)||(DF->OUTdelays[0]<-0.2))
	//	DF->OUTdelays[0]=0;


	// Compute the value of the output
	return DF->OUTdelays[0];

}


struct DistanceObserver
{
	float prev_act_T;
	float prevw;
	float SAT;
	float Inertia;
	struct DigitalFilter DFIL;
	float FREQ;

};


void set_DO(struct DistanceObserver* DO,float I,float S,float f,float* IC,float* OC)
{
		DO->SAT=S;
		DO->Inertia=I*1.0f;
		DO->prev_act_T=0;
		DO->prevw=0;
		DO->FREQ=f;
		reset_DF(&DO->DFIL);
		set_coeff_DF(&DO->DFIL,IC,OC);

}

float compute_DO(struct DistanceObserver* DO,float w,float tau_c)
{
		float newTau=0;
		float appT=0;

		appT=0.05f*DO->Inertia*w+(DO->Inertia*((w-DO->prevw)*DO->FREQ))-DO->prev_act_T;
		//appT=(DO->Inertia*((w-DO->prevw)*DO->FREQ))-DO->prev_act_T;
		appT=compute_DF(&(DO->DFIL),appT);

		DO->prevw=w;

		newTau=tau_c-appT;

		if (newTau>DO->SAT)
			newTau=DO->SAT;
		if (newTau<-DO->SAT)
			newTau=-DO->SAT;


		// Saturation of the actuated torque
		DO->prev_act_T=newTau;

		return newTau;
}



struct AttitudeController
{
	struct DistanceObserver DO[3];
	float KP[3];
	float KV[3];
	float Inertia[3];
	float PrevRPY[3];
	float tau_Motor[3];
	float Pole[3];
	float FREQ;
	float Length;
	float FMotorMax;

	bool isInit;
	float SAT;
	bool Landed;

};


void set_AC(struct AttitudeController* AC)
{

	AC->Length = 0.0605; // Kg
	AC->FMotorMax = (0.980665)*0.95; // N

	AC->SAT=(2*AC->Length*AC->FMotorMax)/(float)4.0; // Nm
	AC->SAT=AC->SAT*0.4f; // Factor

	AC->KP[0]=350;
	//AC->KP[1]=300;

	AC->KP[1]=350;


	AC->KP[2]=2400;

	AC->KV[0]=50;
	AC->KV[1]=50;

	AC->KV[2]=800;

	// Inertia crazyflie
	/*AC->Inertia[0]=0.00002;
	AC->Inertia[1]=0.00002;
	AC->Inertia[2]=0.0000323;*/

	AC->Inertia[0]=0.0002;
	AC->Inertia[1]=0.00025;
	AC->Inertia[2]=0.0002;


	AC->FREQ=RATE_500_HZ;

	AC->isInit=true;

	AC->Landed=true;

	AC->PrevRPY[0]=0;
	AC->PrevRPY[1]=0;
	AC->PrevRPY[2]=0;


	float appOC[SIZE_DF];
	float appIC[SIZE_DF];


	appIC[0]= 0;
	appIC[1]= 0*0.00114541764400166;
	appIC[2]= 0*0.00110767017305206;

	appOC[0]=0;
	appOC[1]= 0*1.90210402082127;
	appOC[2]= 0*-0.904357108638321;


	set_DO(&(AC->DO[0]),AC->Inertia[0],AC->SAT,AC->FREQ,appIC,appOC);


	appIC[0]= 0;
	appIC[1]= 0*0.00114541764400166;
	appIC[2]= 0*0.00110767017305206;

	appOC[0]=0;
	appOC[1]= 0*1.90210402082127;
	appOC[2]= 0*-0.904357108638321;

	set_DO(&(AC->DO[1]),AC->Inertia[1],AC->SAT,AC->FREQ,appIC,appOC);


}


void start_dist_obs(struct AttitudeController* AC)
{
	AC->Landed=true;
}

void reset_dist_obs(struct AttitudeController* AC)
{
	AC->Landed=true;

	// Valutare
	/*AC->DO[0].FIL.X=0;
	AC->DO[1].FIL.X=0;
	AC->DO[2].FIL.X=0;*/

}


void compute_AC(struct AttitudeController* AC,float eulerRollActual,float eulerPitchActual,float eulerYawActual,float eulerRollDesired,float eulerPitchDesired,float eulerYawDesired,float yawRateDesired, float wx,float wy,float wz)
{
	float val,dRoll,dPitch,dYaw;
	float tau_C;

	val = M_PI_F / 180.0f;

	eulerRollActual=eulerRollActual*val;
	eulerPitchActual=eulerPitchActual*val;
	eulerYawActual=eulerYawActual*val;

	eulerRollDesired=eulerRollDesired*val;
	eulerPitchDesired=eulerPitchDesired*val;
	eulerYawDesired=eulerYawDesired*val;
	yawRateDesired=yawRateDesired*val;

	wx=wx*val;
	// Coordinate changed
	wy=-wy*val;
	wz=wz*val;

	// Compute dt of RPY
	//dRoll=wx+sin(eulerRollActual)*tan(eulerPitchActual)*wy+cos(eulerRollActual)*tan(eulerPitchActual)*wz;
	//dPitch=cos(eulerRollActual)*wy-sin(eulerRollActual)*wz;
	//dYaw=(float)((sin(eulerRollActual)*wy+cos(eulerRollActual)*wz)/cos(eulerPitchActual));

	dRoll=(eulerRollActual-	AC->PrevRPY[0])*AC->FREQ;
	dPitch=(eulerPitchActual - AC->PrevRPY[1])*AC->FREQ;
	dYaw=(eulerYawActual - AC->PrevRPY[2])*AC->FREQ;

	// roll
	tau_C=(AC->KP[0]*(eulerRollDesired-eulerRollActual)-AC->KV[0]*dRoll)*AC->Inertia[0];
	if (AC->Landed)
	{
		if (tau_C>AC->SAT)
			tau_C=AC->SAT;
		else
			if (tau_C<-AC->SAT)
				tau_C=-AC->SAT;

		AC->tau_Motor[0]=tau_C;
	}
	else
	{
		AC->tau_Motor[0]=compute_DO(&(AC->DO[0]),wx,tau_C);
		//AC->tau_Motor[0]=0;
	}

	// pitch
	tau_C=(AC->KP[1]*(eulerPitchDesired-eulerPitchActual)-AC->KV[1]*dPitch)*AC->Inertia[1];

	if (AC->Landed)
	{
		if (tau_C>AC->SAT)
			tau_C=AC->SAT;
		else
			if (tau_C<-AC->SAT)
				tau_C=-AC->SAT;

		AC->tau_Motor[1]=tau_C;
	}
	else
	{
		AC->tau_Motor[1]=compute_DO(&(AC->DO[1]),wy,tau_C);
		//AC->tau_Motor[1]=AC->tau_Motor[1]=tau_C;
	}

	// yaw
	tau_C=(AC->KP[2]*(eulerYawDesired-eulerYawActual)+AC->KV[2]*(yawRateDesired-dYaw))*AC->Inertia[2];
	AC->tau_Motor[2]=tau_C;

	AC->PrevRPY[0]=eulerRollActual;
	AC->PrevRPY[1]=eulerPitchActual;
	AC->PrevRPY[2]=eulerYawActual;

}

/*
void get_ActuatorOutput(struct AttitudeController* AC,int32_t* roll, int32_t* pitch, int32_t* yaw)
{
	float app;

	float Volt;

	Volt=4.2;

	app=(AC->tau_Motor[0]/(AC->Length*AC->KM*Volt))*65535.0;
	*roll= (int32_t)(app);

	app=(AC->tau_Motor[1]/(AC->Length*AC->KM*Volt))*65535.0;
	*pitch= (int32_t)(app);

	app=(AC->tau_Motor[2]/(1))*65535.0;
	*yaw = (int32_t)(app);


}*/




struct HeightController
{

	float KP;
	float KV;
	float Mass;
	float ForceHeight;

	bool isInit;
	float SAT;
	bool Landed;

	float IntErr;

	float G;



};





void set_HC(struct HeightController* HC)
{

	HC->SAT=10000;

	HC->G=9.80665;
	HC->Mass=185*0.001;
	HC->ForceHeight=0.0;
	HC->isInit=true;
	HC->Landed=true;

	HC->IntErr=0;


}


void compute_HC(struct HeightController* HC,float HeigthActual,float HeigthDesired,float dHeigth,bool EnPos)
{
	float error,sigma,coeff;

	//sigma=0.05;
	HC->KP=10;
	HC->KV=6;
	uint64_t TimeZ;
	float appT;



	error=HeigthDesired-HeigthActual;
	if (!EnPos)
		error=0;


	// change function
	//coeff=fastPow(Nepero_inv,(dHeigth*dHeigth)/(2*sigma*sigma));//(dHeigth*dHeigth)/(2*sigma*sigma));

	//coeff=1;

	//HC->IntErr=HC->IntErr+(error*coeff);


	HC->ForceHeight=(HC->KP*error*HC->Mass)-(HC->KV*dHeigth*HC->Mass)+(HC->Mass*HC->G*1.0f);
	if (HC->ForceHeight<0)
		HC->ForceHeight=0;


}

void motorSafe(struct HeightController* HC)
{
	HC->ForceHeight=HC->Mass*HC->G*0.2f;
}

void ActuateMotor(struct AttitudeController* AC,struct HeightController* HC,float eulerRollActual,float eulerPitchActual,control_t* CONTROL)
{
	float app,val;
	int32_t app_roll,app_pitch,satRP,apptrust,app_yaw;
	int32_t M1,M2,M3,M4;
	bool sat;

	float FSR=32767.0;


	sat=false;
	val = M_PI_F / 180.0f;
	eulerRollActual=eulerRollActual*val;
	eulerPitchActual=eulerPitchActual*val;

	app=(AC->tau_Motor[0]/(2.0f*AC->Length*AC->FMotorMax))*FSR;
	app_roll=(int32_t)(app);
	app=(AC->tau_Motor[1]/(2.0f*AC->Length*AC->FMotorMax))*FSR;
	app_pitch=(int32_t)(app);

	app=HC->ForceHeight/(4.0f*cosf(eulerRollActual)*cosf(eulerPitchActual));
	app=(app/AC->FMotorMax)*FSR;
	apptrust=(int32_t)(app);


	satRP=abs(app_roll)+abs(app_pitch);
	if (apptrust<satRP)
	{
		//apptrust=satRP;
		sat=true;
	}

	if (apptrust+satRP>(FSR))
	{
		//apptrust=(FSR-satRP);
		sat=true;
	}

	//apptrust = 10000;

	 M1=(int32_t)(apptrust - app_roll + app_pitch);
	 M2=(int32_t)(apptrust - app_roll - app_pitch);
	 M3=(int32_t)(apptrust + app_roll - app_pitch);
	 M4=(int32_t)(apptrust + app_roll + app_pitch);


  if (!sat)
  {
	app=(AC->tau_Motor[2])*FSR;
	app_yaw = (int32_t)(app);

	/*if(M1-app_yaw>FSR)
	{
		app_yaw=M1-FSR;
	}
	if(M1-app_yaw<0)
	{
		app_yaw=M1;
	}

	if(M3-app_yaw>FSR)
	{
		app_yaw=M3-FSR;
	}
	if(M3-app_yaw<0)
	{
		app_yaw=M3;
	}

	if(M2+app_yaw>FSR)
	{
		app_yaw=FSR-M2;
	}
	if(M2+app_yaw<0)
	{
		app_yaw=-M2;
	}

	if(M4+app_yaw>FSR)
	{
		app_yaw=FSR-M4;
	}
	if(M4+app_yaw<0)
	{
		app_yaw=-M4;
	}*/

	if (app_yaw>18000)
			app_yaw=18000;
		if (app_yaw<-18000)
			app_yaw=-18000;

	M1=(int32_t)(M1 - app_yaw);
	M2=(int32_t)(M2 + app_yaw);
	M3=(int32_t)(M3 - app_yaw);
	M4=(int32_t)(M4 + app_yaw);
	}


  	motorPowerM1 = limitThrust_2(M1+FSR);
	motorPowerM2 = limitThrust_2(M2+FSR);
	motorPowerM3 = limitThrust_2(M3+FSR);
	motorPowerM4 = limitThrust_2(M4+FSR);

	motorsSetRatio(MOTOR_M1, motorPowerM1);
	motorsSetRatio(MOTOR_M2, motorPowerM2);
	motorsSetRatio(MOTOR_M3, motorPowerM3);
	motorsSetRatio(MOTOR_M4, motorPowerM4);

	/*motorsSetRatio(MOTOR_M1, 10000);
	motorsSetRatio(MOTOR_M2, 10000);
	motorsSetRatio(MOTOR_M3, 10000);
	motorsSetRatio(MOTOR_M4, 10000);*/


	// update
	CONTROL->thrust= apptrust;
	CONTROL->roll=(int16_t)app_roll;
	CONTROL->pitch=(int16_t)app_pitch;
	//CONTROL->yaw=(int16_t)app_yaw;


}

void turnOFFMotor()
{
	uint32_t FSR = 32767;
	motorsSetRatio(MOTOR_M1, FSR);
	motorsSetRatio(MOTOR_M2, FSR);
	motorsSetRatio(MOTOR_M3, FSR);
	motorsSetRatio(MOTOR_M4, FSR);
}

void setRatioMotor(float ratioM,float T)
{
	uint32_t Ratio;
	uint32_t Torque;
	Ratio=(int)ratioM*65535;
	Torque=(int)T*65535;

	Ratio=50000;
	Torque=0;

	motorsSetRatio(MOTOR_M1, (Ratio+Torque));
	motorsSetRatio(MOTOR_M2, (Ratio-Torque));
	motorsSetRatio(MOTOR_M3, (Ratio-Torque));
	motorsSetRatio(MOTOR_M4, (Ratio+Torque));
}

/*
static float prevRPYangle[3];
void perching(float eulerRollActual,float eulerPitchActual,float eulerYawActual, float wx,float wy,float wz,float FREQ)
{
	float val,dRoll,dPitch,dYaw;
	val = M_PI_F / 180.0f;

	eulerRollActual=eulerRollActual*val;
	eulerPitchActual=eulerPitchActual*val;
	eulerYawActual=eulerYawActual*val;

	wx=wx*val;
	wy=-wy*val;
	wz=wz*val;

	// Compute dt of RPY
	//dRoll=wx+sin(eulerRollActual)*tan(eulerPitchActual)*wy+cos(eulerRollActual)*tan(eulerPitchActual)*wz;
	//dPitch=cos(eulerRollActual)*wy-sin(eulerRollActual)*wz;
	//dYaw=(float)((sin(eulerRollActual)*wy+cos(eulerRollActual)*wz)/cos(eulerPitchActual));

	dRoll=(eulerRollActual-	prevRPYangle[0])*FREQ;
	dPitch=(eulerPitchActual - prevRPYangle[1])*FREQ;
	dYaw=(eulerYawActual - prevRPYangle[2])*FREQ;





	/*AC->KP[1]=300;
	AC->KP[1]=180;
	AC->KP[2]=60000;
	AC->KV[1]=50;
	AC->KV[2]=30000;
	AC->Inertia[1]=0.0002;
	AC->Inertia[2]=0.0001;*/

	/*
	// roll
	float tauRoll,tauPitch,tauYaw;
	float KP=250;
	float KV = 50;
	float I = 0.00008;
	// Roll
	tauRoll=(-KP*eulerRollActual-AC->KV[0]*dRoll)*AC->Inertia[0];
	// pitch
	tauPitch=(AC->KP[1]*(eulerPitchDesired-eulerPitchActual)-AC->KV[1]*dPitch)*AC->Inertia[1];
	tauYaw=(AC->KP[2]*(eulerYawDesired-eulerYawActual)+AC->KV[2]*(yawRateDesired-dYaw))*AC->Inertia[2];

	// Saturation




	prevRPYangle[0]=eulerRollActual;
	prevRPYangle[1]=eulerPitchActual;
	prevRPYangle[2]=eulerYawActual;

	float L,L0;
	uint32_t MPower[4];
	L = 10;
	L0=10;
	float FMotorMax1 = 0.980665;
	float FMotorMax2 = 0.980665;
	float rollForce,pitchForce,RollMotor,PitchMotor,RollMotorI,PitchMotorI;
	float app;

	int32_t app_roll,app_pitch,satRP,apptrust,app_yaw;
	int32_t M1,M2,M3,M4;
	bool sat;

	float FSR=32767.0;


	sat=false;



	// saturare con criterio
	rollForce=(tau_Roll/(2.0f*L);
	RollMotor = (int32_t) (rollForce * (FSR / FMotorMax1));
	RollMotorI = (int32_t)(rollForce * (FSR / FMotorMax2));

	pitchForce=(tau_Pitch/(2.0f*L);
	PitchMotor = (int32_t)(pitchForce * (FSR / FMotorMax1));
	PitchMotorI = (int32_t)(pitchForce * (FSR / FMotorMax2));

	dutyaw=(AC->tau_Motor[2])*FSR;
	app_yaw = (int32_t)(app);

   // New Configuration tutto importante
   MPower[0]=(int32_t)(- RollMotor + PitchMotor - app_yaw);
   MPower[1]=(int32_t)(+ RollMotorI - PitchMotorI - app_yaw);
   MPower[2]=(int32_t)(- RollMotorI - PitchMotorI + app_yaw);
   MPower[3]=(int32_t)(+ RollMotor + PitchMotor + app_yaw);

   float MinM1,MinM2;

   // find Minimum duty available
   if (MPower[0]<MPower[3])
	   MinM1 = MPower[0];
   else
	   MinM1 = MPower[3];

   if (MPower[1]<MPower[2])
   	   MinM2 = MPower[1];
   else
   	   MinM2 = MPower[2];

   // Verificare
   int32_t Fz;
   if (MinM2<MinM1*(L+L0))

   else


   for (int i=0;i<4:i++)
	   MPower[i] = MPower[0] + Fz;


	motorPowerM1 = limitThrust_2(M1+FSR);
	motorPowerM2 = limitThrust_3(M2);
	motorPowerM3 = limitThrust_3(M3);
	motorPowerM4 = limitThrust_2(M4+FSR);

	motorsSetRatio(MOTOR_M1, motorPowerM1);
	motorsSetRatio(MOTOR_M2, motorPowerM2);
	motorsSetRatio(MOTOR_M3, motorPowerM3);
	motorsSetRatio(MOTOR_M4, motorPowerM4);





}
*/



static float prevPvelocity;

void perch_takeoff_normal(float eulerRollActual,float eulerPitchActual,float eulerYawActual,float eulerYawDesired, float wx,float wy,float wz,float FREQ)
{
	float val,dRoll,dPitch,dYaw;
	val = M_PI_F / 180.0f;

	eulerRollActual=eulerRollActual*val;
	eulerPitchActual=eulerPitchActual*val;
	eulerYawActual=eulerYawActual*val;
	eulerYawDesired=eulerYawDesired*val;

	wx=wx*val;
	wy=-wy*val;
	wz=wz*val;

	// Compute dt of RPY
	dRoll=wx+sinf(eulerRollActual)*tanf(eulerPitchActual)*wy+cosf(eulerRollActual)*tanf(eulerPitchActual)*wz;
	dPitch=cosf(eulerRollActual)*wy-sinf(eulerRollActual)*wz;
	dYaw=(sinf(eulerRollActual)*wy+cosf(eulerRollActual)*wz)/cosf(eulerPitchActual);

	//dRoll=(eulerRollActual-	prevRPYangle[0])*FREQ;
	//dPitch=(eulerPitchActual - prevRPYangle[1])*FREQ;
	//dYaw=(eulerYawActual - prevRPYangle[2])*FREQ;

	/*AC->KP[1]=300;
	AC->KP[1]=180;
	AC->KP[2]=60000;
	AC->KV[1]=50;
	AC->KV[2]=30000;
	AC->Inertia[1]=0.0002;
	AC->Inertia[2]=0.0001;*/


	// roll
	float tauRoll,tauPitch,tauYaw;
	float KP=250;
	float KV = 50;
	float I = 0.00008;
	float Length = 0.0635; // Kg
	float mass = 0.145;
	float FMotorMax = 0.980665;
	float FSR=32767.0;

	tauRoll=(-KP*eulerRollActual-KV*dRoll)*I;

	KP=60000;
	KV = 30000;
	I = 0.0001;
	tauYaw=(KP*(eulerYawDesired-eulerYawActual)-KV*dYaw)*I;

	float pole = 30;
	KV=10;
	KP=KV*pole;
	I=0.0002;
	float refPvelocity;
	refPvelocity= (M_PI_F*0.5f-eulerPitchActual)*3;
	float accP = (dPitch - prevPvelocity)*FREQ;
	tauPitch=(KP*(refPvelocity-dPitch)-KV*dPitch)*I;
	prevPvelocity=dPitch;



	// Compute duty cycle
	int32_t RollMotor,PitchMotor,dutyYaw;

	RollMotor = (int32_t) ((tauRoll/(2.0f*Length)) * (FSR / FMotorMax));
	PitchMotor =(int32_t) ((tauPitch/(2.0f*Length)) * (FSR / FMotorMax));
	int32_t app_yaw;
	dutyYaw=(tauYaw)*FSR;
	app_yaw = (int32_t)(dutyYaw);
	int32_t FZ=(int32_t)(0.25f*mass*9.81f*cosf(eulerPitchActual)*(FSR / FMotorMax)); // Nm

	int32_t appSAT = (int32_t) 0.2f*FZ;
	if (RollMotor>appSAT)
		RollMotor=appSAT;
	else
		if (RollMotor<-appSAT)
			RollMotor=-appSAT;

	appSAT = (int32_t) 0.6f*FZ;
	if (PitchMotor>appSAT)
		PitchMotor=appSAT;
	else
		if (PitchMotor<-appSAT)
			PitchMotor=-appSAT;

	appSAT = (int32_t) 0.2f*FZ;
	if (app_yaw>appSAT)
		app_yaw=appSAT;
	else
		if (app_yaw<-appSAT)
			app_yaw=-appSAT;

   int32_t MPower[4];
   // New Configuration no priority
   MPower[0]=(int32_t)(- RollMotor + PitchMotor - app_yaw + FZ);
   MPower[1]=(int32_t)(- RollMotor - PitchMotor + app_yaw + FZ);
   MPower[2]=(int32_t)(+ RollMotor - PitchMotor - app_yaw + FZ);
   MPower[3]=(int32_t)(+ RollMotor + PitchMotor + app_yaw + FZ);


   motorPowerM1 = limitThrust_2(MPower[0]+FSR);
   motorPowerM2 = limitThrust_2(MPower[1]+FSR);
   motorPowerM3 = limitThrust_2(MPower[2]+FSR);
   motorPowerM4 = limitThrust_2(MPower[3]+FSR);

   motorsSetRatio(MOTOR_M1, motorPowerM1);
   motorsSetRatio(MOTOR_M2, motorPowerM2);
   motorsSetRatio(MOTOR_M3, motorPowerM3);
   motorsSetRatio(MOTOR_M4, motorPowerM4);

}



LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)
