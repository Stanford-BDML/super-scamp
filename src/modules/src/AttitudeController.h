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


uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;

const float Nepero_inv = 0.367879441171;

float fastPow(float a, float b) {
  union {
    float d;
    int x[2];
  } u = { a };
  u.x[1] = (int)(b * (u.x[1] - 1072632447) + 1072632447);
  u.x[0] = 0;
  return u.d;
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

struct Filter_new
{
	float X;
	float coeff_den;
	float coeff_num;
};

struct Filter
{
	float X;
	float coeff_den;
	float coeff_num;
};

void setFilter(struct Filter* F,float f,float p)
{
	F->X=0;
	// Cambio di function
	//F->coeff_den=exp(-p/f);
	F->coeff_den=fastPow(Nepero_inv,p/f);
	F->coeff_den=0.999;
	F->coeff_num=1-F->coeff_den;
}

float computeFilter(struct Filter* F,float Inp)
{
	F->X=F->coeff_den*F->X+F->coeff_num*Inp;
	return F->X;
}



struct DistanceObserver
{
	float prev_act_T;
	float prevw;
	float SAT;
	float Inertia;
	struct Filter FIL;
	float FREQ;

};


void set_DO(struct DistanceObserver* DO,float I,float S,float p,float f)
{
		DO->SAT=S;
		DO->Inertia=I*1.0f;
		DO->prev_act_T=0;
		DO->prevw=0;
		DO->FREQ=f;
		setFilter(&DO->FIL,p,DO->FREQ);

}

float compute_DO(struct DistanceObserver* DO,float w,float tau_c)
{
		float newTau=0;
		float appT=0;

		appT=0.05f*DO->Inertia*w+(DO->Inertia*((w-DO->prevw)*DO->FREQ))-DO->prev_act_T;
		//appT=(DO->Inertia*((w-DO->prevw)*DO->FREQ))-DO->prev_act_T;
		appT=computeFilter(&(DO->FIL),appT);

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

	AC->Length = 0.1; // Kg
	AC->FMotorMax = 0.980665; // N

	AC->SAT=(2*AC->Length*AC->FMotorMax)/(float)4.0; // Nm

	AC->KP[0]=300;
	AC->KP[1]=300;
	AC->KP[2]=2000;

	AC->KV[0]=50;
	AC->KV[1]=50;
	AC->KV[2]=30000;

	AC->Inertia[0]=0.00002;
	AC->Inertia[1]=0.00002;
	AC->Inertia[2]=0.0000323;

	AC->FREQ=RATE_500_HZ;
	AC->Pole[0]=3;
	AC->Pole[1]=3;
	AC->Pole[2]=30;

	AC->isInit=true;

	AC->Landed=true;

	AC->PrevRPY[0]=0;
	AC->PrevRPY[1]=0;
	AC->PrevRPY[2]=0;

	set_DO(&(AC->DO[0]),AC->Inertia[0],AC->SAT,AC->Pole[0],AC->FREQ);
	set_DO(&(AC->DO[1]),AC->Inertia[1],AC->SAT,AC->Pole[1],AC->FREQ);
	set_DO(&(AC->DO[2]),AC->Inertia[2],AC->SAT,AC->Pole[2],AC->FREQ);

}


void start_dist_obs(struct AttitudeController* AC)
{
	AC->Landed=false;
}

void reset_dist_obs(struct AttitudeController* AC)
{
	AC->Landed=true;
	AC->DO[0].FIL.X=0;
	AC->DO[1].FIL.X=0;
	AC->DO[2].FIL.X=0;

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
	HC->Mass=207*0.001;
	HC->ForceHeight=0.0;
	HC->isInit=true;
	HC->Landed=true;

	HC->IntErr=0;


}


void compute_HC(struct HeightController* HC,float HeigthActual,float HeigthDesired,float dHeigth,bool EnPos)
{
	float error,sigma,coeff;

	//sigma=0.05;
	HC->KP=8;
	HC->KV=4;
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
	HC->ForceHeight=HC->Mass*HC->G*0.8f;
}

void ActuateMotor(struct AttitudeController* AC,struct HeightController* HC,float eulerRollActual,float eulerPitchActual,control_t* CONTROL)
{
	float app,val;
	int32_t app_roll,app_pitch,satRP,apptrust,app_yaw;
	int32_t M1,M2,M3,M4;
	bool sat;



	sat=false;
	val = M_PI_F / 180.0f;
	eulerRollActual=eulerRollActual*val;
	eulerPitchActual=eulerPitchActual*val;

	app=(AC->tau_Motor[0]/(2.0f*AC->Length*AC->FMotorMax))*65535.0f;
	app_roll=(int32_t)(app);
	app=(AC->tau_Motor[1]/(2.0f*AC->Length*AC->FMotorMax))*65535.0f;
	app_pitch=(int32_t)(app);

	app=HC->ForceHeight/(4.0f*cosf(eulerRollActual)*cosf(eulerPitchActual));
	app=(app/AC->FMotorMax)*65535.0f;
	apptrust=(int32_t)(app);


	satRP=abs(app_roll)+abs(app_pitch);
	if (apptrust<satRP)
	{
		apptrust=satRP;
		sat=true;
	}

	if (apptrust+satRP>(65535))
	{
		apptrust=(65535-satRP);
		sat=true;
	}


  M1=(int32_t)(apptrust - app_roll + app_pitch);
  M2=(int32_t)(apptrust - app_roll - app_pitch);
  M3=(int32_t)(apptrust + app_roll - app_pitch);
  M4=(int32_t)(apptrust + app_roll + app_pitch);

  if (!sat)
  {
	app=(AC->tau_Motor[2])*65535.0f;
	app_yaw = (int32_t)(app);

	if(M1-app_yaw>65535)
	{
		app_yaw=M1-65535;
	}
	if(M1-app_yaw<0)
	{
		app_yaw=M1;
	}

	if(M3-app_yaw>65535)
	{
		app_yaw=M3-65535;
	}
	if(M3-app_yaw<0)
	{
		app_yaw=M3;
	}

	if(M2+app_yaw>65535)
	{
		app_yaw=65535-M2;
	}
	if(M2+app_yaw<0)
	{
		app_yaw=-M2;
	}

	if(M4+app_yaw>65535)
	{
		app_yaw=65535-M4;
	}
	if(M4+app_yaw<0)
	{
		app_yaw=-M4;
	}

		M1=(int32_t)(M1 - app_yaw);
		M2=(int32_t)(M2 + app_yaw);
		M3=(int32_t)(M3 - app_yaw);
		M4=(int32_t)(M4 + app_yaw);
	}

	motorPowerM1 = limitThrust(M1);
	motorPowerM2 = limitThrust(M2);
	motorPowerM3 = limitThrust(M3);
	motorPowerM4 = limitThrust(M4);

	motorsSetRatio(MOTOR_M1, motorPowerM1);
	motorsSetRatio(MOTOR_M2, motorPowerM2);
	motorsSetRatio(MOTOR_M3, motorPowerM3);
	motorsSetRatio(MOTOR_M4, motorPowerM4);

	// update
	CONTROL->thrust= apptrust;
	CONTROL->roll=(int16_t)app_roll;
	CONTROL->pitch=(int16_t)app_pitch;
	CONTROL->yaw=(int16_t)app_yaw;


}

void turnOFFMotor()
{
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);
}




LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)
