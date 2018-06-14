#include <math.h>
#include "motors.h"
#include "log.h"
#include <stdlib.h>
//#include "pm.h"

#define M_PI_F 3.14159265358979323846f


uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;


// Saturation for both direction of trust
static uint16_t limitThrust_1(int32_t value)
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


// Saturation for forward direction of trust
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

// Saturation for backward direction of trust
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





struct AttitudeController
{

	float KP[3];
	float KV[3];
	float Inertia[3];
	float PrevRPY[3];
	float tau_Motor[3];
	float Pole[3];
	float FREQ;
	float Length;
	float FMotorMax;

	float SAT;
	bool isInit;


};

// set the Attitude controller
void set_AC(struct AttitudeController* AC)
{

	AC->isInit =true;
	AC->Length = 0.05388; // Kg
	AC->FMotorMax = (0.980665)*0.95; // N

	AC->SAT=(2*AC->Length*AC->FMotorMax)/(float)4.0; // Nm
	AC->SAT=AC->SAT*0.4f; // Factor

	AC->KP[0]=300;
	AC->KP[1]=300;
	AC->KP[2]=6000;

	AC->KV[0]=45;
	AC->KV[1]=45;
	AC->KV[2]=800*2;

	AC->Inertia[0]=0.0002;
	AC->Inertia[1]=0.0002;
	AC->Inertia[2]=0.0002;


	AC->FREQ=RATE_500_HZ;



	AC->PrevRPY[0]=0;
	AC->PrevRPY[1]=0;
	AC->PrevRPY[2]=0;


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
	
	// Compute dt RPY
	dRoll=(eulerRollActual-	AC->PrevRPY[0])*AC->FREQ;
	dPitch=(eulerPitchActual - AC->PrevRPY[1])*AC->FREQ;
	dYaw=(eulerYawActual - AC->PrevRPY[2])*AC->FREQ;

	// roll
	tau_C=(AC->KP[0]*(eulerRollDesired-eulerRollActual)-AC->KV[0]*dRoll)*AC->Inertia[0];

	if (tau_C>AC->SAT)
		tau_C=AC->SAT;
	else
		if (tau_C<-AC->SAT)
			tau_C=-AC->SAT;

	AC->tau_Motor[0]=tau_C;


	// pitch
	tau_C=(AC->KP[1]*(eulerPitchDesired-eulerPitchActual)-AC->KV[1]*dPitch)*AC->Inertia[1];

	if (tau_C>AC->SAT)
		tau_C=AC->SAT;
	else
		if (tau_C<-AC->SAT)
			tau_C=-AC->SAT;

	AC->tau_Motor[1]=tau_C;

	// yaw
	tau_C=(AC->KP[2]*(eulerYawDesired-eulerYawActual)+AC->KV[2]*(yawRateDesired-dYaw))*AC->Inertia[2];
	AC->tau_Motor[2]=tau_C;

	AC->PrevRPY[0]=eulerRollActual;
	AC->PrevRPY[1]=eulerPitchActual;
	AC->PrevRPY[2]=eulerYawActual;

}


// Altitude controller
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


// set the Altitude controller
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
	float error;

	HC->KP=10;
	HC->KV=10;

	error=HeigthDesired-HeigthActual;
	if (!EnPos)
		error=0;

	HC->ForceHeight=(HC->KP*error*HC->Mass)-(HC->KV*dHeigth*HC->Mass)+(HC->Mass*HC->G*1.0f);
	if (HC->ForceHeight<0)
		HC->ForceHeight=0;

}

void mgcostheta(struct HeightController* HC,float eulerPitchActual)
{
	float val = M_PI_F / 180.0f;
	eulerPitchActual=eulerPitchActual*val;
	HC->ForceHeight = (HC->Mass*HC->G*1.0f)*cosf(eulerPitchActual);
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

	// motor saturation
  	motorPowerM1 = limitThrust_2(M1+FSR);
	motorPowerM2 = limitThrust_2(M2+FSR);
	motorPowerM3 = limitThrust_2(M3+FSR);
	motorPowerM4 = limitThrust_2(M4+FSR);

	// actuation
	motorsSetRatio(MOTOR_M1, motorPowerM3); // 1 connected with 3
	motorsSetRatio(MOTOR_M2, motorPowerM2); // Good
	motorsSetRatio(MOTOR_M3, motorPowerM1); //3 Connected with 1
	motorsSetRatio(MOTOR_M4, motorPowerM4); // Good*/
	// temp
	motor5SetRatio(motorPowerM4);
	//

	// probably can be deleted
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

// set constant motor speed
void setMotor(float ratioM)
{
	uint32_t Ratio;
	uint32_t FSR = 32767;
	Ratio = (1 + ratioM)*FSR;

	motorsSetRatio(MOTOR_M1, Ratio);
	motorsSetRatio(MOTOR_M2, Ratio);
	motorsSetRatio(MOTOR_M3, Ratio);
	motorsSetRatio(MOTOR_M4, Ratio);
}

void motorSafe()
{
	turnOFFMotor();
}





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



	// roll
	float tauRoll,tauPitch,tauYaw;
	float KP=350;
	float KV = 50;
	float I = 0.0002*0;
	float Length = 0.1077*0.5; // Kg
	float FMotorMax = (0.980665)*0.95;
	float FSR=32767.0;

	tauRoll=(-KP*eulerRollActual-KV*dRoll)*I;

	KP=2400*3;
	KV = 800;
	I = 0.0002*0;
	tauYaw=(KP*(eulerYawDesired-eulerYawActual)-KV*dYaw)*I;

	KV=0;
	KP=300;
	I=0.00025;
	float refPvelocity;
	// pay attention about the angle sign // check with the quad!!!!!!
	refPvelocity= (M_PI_F*0.5f-eulerPitchActual)*3.0f;

	tauPitch=(KP*(refPvelocity-dPitch)-KV)*I;



	// Compute duty cycle
	int32_t RollMotor,PitchMotor1,PitchMotor2,dutyYaw;

	RollMotor = (int32_t) ((tauRoll/(2.0f*Length)) * (FSR / FMotorMax));
	int32_t app_yaw;
	dutyYaw=(tauYaw)*FSR;
	app_yaw = (int32_t)(dutyYaw);


	//
	// Modificare
	float F1,F2;
	float L0 = 0.09;
	float l=Length;
	float Fref = 0.6;


	 F1 = ((L0)/(L0*L0+(2*l+L0)*(2*l+L0)))*tauPitch;
	 F2 = -((2*l+L0)/(L0*L0+(2*l+L0)*(2*l+L0)))*tauPitch;


	 Fref = Fref*(L0/(2*l));
	 F1 = F1 + ((2*l+L0)/L0)*Fref;
	 F2 = F2 + Fref;


	 F1 = F1*0.5f;
	 F2 = F2*0.5f;
	 if (eulerPitchActual>(0.9f*M_PI_F*0.5f))
		 F2=0;


	//Ftau = ((l+L0)/(L0*L0+2*l*l+2*l*L0)))*tauPitch;


	//F1 = ((2*l+L0)*Fref-(1)*tauPitch)/(2*l);
	//F2 =((L0)*Fref-(1)*tauPitch)/(2*l);


	//F1 = (F1+mass*cosf(0)*G*0.5f)*0.5f;
	//F2 = (F2+mass*cosf(0)*G*0.5f)*0.5f;


	// We need to transform the force in to motor
	PitchMotor1 =(int32_t) (F1 * (FSR / FMotorMax));
	PitchMotor2 =(int32_t) (F2 * (FSR / FMotorMax));


   // Nuova saturazione
	if (PitchMotor1>FSR)
		PitchMotor1=FSR;
	if (PitchMotor1<0)
		PitchMotor1=0;

	if (PitchMotor2>FSR)
		PitchMotor2=FSR;
	if (PitchMotor2<0)
		PitchMotor2=0;



   float appsat = abs(RollMotor+app_yaw);

   if ((PitchMotor2+appsat)>FSR)
   {
	   RollMotor = RollMotor * fabsf(FSR/(PitchMotor2+appsat));
	   app_yaw =   app_yaw * fabsf(FSR/(PitchMotor2+appsat));
   }

   if ((PitchMotor2-appsat)<0)
   {
	   RollMotor = RollMotor * fabsf(PitchMotor2/appsat);
	   app_yaw =   app_yaw * fabsf(PitchMotor2/appsat);
   }

   appsat = abs(RollMotor-app_yaw);

   if ((PitchMotor1+appsat)>FSR)
   {
	   RollMotor = RollMotor * fabsf(FSR/(PitchMotor1+appsat));
	   app_yaw =   app_yaw * fabsf(FSR/(PitchMotor1+appsat));
   }

   if ((PitchMotor1-appsat)<0)
   {
	   RollMotor = RollMotor * fabsf(PitchMotor1/appsat);
	   app_yaw =   app_yaw * fabsf(PitchMotor1/appsat);
   }

   RollMotor = 0;
   app_yaw = 0;
   int32_t MPower[4];
   // New Configuration no priority CHECK with the quad
   MPower[0]=(int32_t)(+ RollMotor - PitchMotor2 + app_yaw);
   MPower[1]=(int32_t)(- RollMotor + PitchMotor1 + app_yaw);
   MPower[2]=(int32_t)(+ RollMotor + PitchMotor1 - app_yaw);
   MPower[3]=(int32_t)(- RollMotor - PitchMotor2 - app_yaw);



   // // New Configuration tutto importante
   /*MPower[0]=(int32_t)(- RollMotor + PitchMotor - app_yaw);
   MPower[1]=(int32_t)(+ RollMotorI - PitchMotorI - app_yaw);
   MPower[2]=(int32_t)(- RollMotorI - PitchMotorI + app_yaw);
   MPower[3]=(int32_t)(+ RollMotor + PitchMotor + app_yaw);*/

   // cambiare le direzioni dei motori!
   motorPowerM1 = limitThrust_3(MPower[0]+FSR);
   motorPowerM2 = limitThrust_2(MPower[1]+FSR);
   motorPowerM3 = limitThrust_2(MPower[2]+FSR);
   motorPowerM4 = limitThrust_3(MPower[3]+FSR);




   motorsSetRatio(MOTOR_M1, motorPowerM3);
   motorsSetRatio(MOTOR_M2, motorPowerM2);
   motorsSetRatio(MOTOR_M3, motorPowerM1);
   motorsSetRatio(MOTOR_M4, motorPowerM4);

}



float prevRPY[3];

void perching_vel(float eulerRollActual,float eulerPitchActual,float eulerYawActual,float eulerYawDesired, float wx,float wy,float wz,float FREQ,float eulerRollDesired)
{
	float val,dRoll,dPitch,dYaw;
	val = M_PI_F / 180.0f;

	eulerRollActual=eulerRollActual*val;
	eulerPitchActual=eulerPitchActual*val;
	eulerYawActual=eulerYawActual*val;
	eulerYawDesired=eulerYawDesired*val;
	eulerRollDesired=eulerRollDesired*val;

	wx=wx*val;
	wy=-wy*val;
	wz=wz*val;

	// Compute dt of RPY
	dRoll=wx+sinf(eulerRollActual)*tanf(eulerPitchActual)*wy+cosf(eulerRollActual)*tanf(eulerPitchActual)*wz;
	dPitch=cosf(eulerRollActual)*wy-sinf(eulerRollActual)*wz;
	dYaw=(sinf(eulerRollActual)*wy+cosf(eulerRollActual)*wz)/cosf(eulerPitchActual);

	
	prevRPY[0]=eulerRollActual;
	prevRPY[1]=eulerPitchActual;
	prevRPY[2]=eulerYawActual;


	// roll
	float tauRoll,tauPitch,tauYaw;
	float KP=350;
	float KV = 50;
	float I = 0.0002;
	float Length = 0.1077*0.5; // Kg
	float FMotorMax = (0.980665)*0.95;
	float FSR=32767.0;

	tauPitch=(-KP*eulerPitchActual-KV*dPitch)*I;

	KP=1600;
	KV = 1600;
	I = 0.0002;
	tauYaw=(KP*(eulerYawDesired-eulerYawActual)-KV*dYaw)*I;

	KV=50*10*1;
	KP=350*10*2;
	I=0.00025;
	tauRoll=(KP*(eulerRollDesired-eulerRollActual)-KV*dRoll)*I;

	KP=300*4*2;
	I=0.00025;
	float refPvelocity;
	
	refPvelocity= 270.0f*(M_PI_F/180);
	tauRoll=(KP*(refPvelocity-dRoll))*I;

	// Duty cycle motor calculation
	int32_t PitchMotor,RollMotor1,RollMotor2,dutyYaw;
	// PITCH
	PitchMotor = (int32_t) ((tauPitch/(2*Length)) * (FSR / FMotorMax));
	// YAW
	int32_t app_yaw;
	dutyYaw=(tauYaw)*FSR;
	app_yaw = (int32_t)(dutyYaw);

	// Compensation of thrust forces (we are attached to the surface!)
	float F1,F2;
	float L0 = 0.045;
	float l=Length;
	float mass=220*0.001;
	float G = 9.80665;

	F1 = FMotorMax*1.5f;
	float Fadd = ((mass*G*cosf(eulerRollActual)*(l+L0))-(F1*L0))/(2*l+L0);
	F2 = tauRoll/(2*l+L0)+Fadd;

	// if Roll greater than 75°, SCAMP is pushed again the surface
	if (eulerRollActual>(75.0f*val))
	{
		F2 = FMotorMax*1.5f;
		app_yaw = 0;
		PitchMotor = 0;	
	}

	F1 = F1*0.5f;
	F2 = F2*0.5f;

	// ROLL duty cycle and saturation
	RollMotor1 =(int32_t) (F1 * (FSR / FMotorMax));
	RollMotor2 =(int32_t) (F2 * (FSR / FMotorMax));

	if (RollMotor2>FSR*0.95f)
		RollMotor2=FSR*0.95f;
	if (RollMotor2<0)
		RollMotor2=0;

	// PITCH and YAW saturation
   float appsat = abs(PitchMotor-app_yaw);

   if ((RollMotor1+appsat)>FSR)
   {
	   PitchMotor = PitchMotor * fabsf(FSR/(RollMotor1+appsat));
	   app_yaw =   app_yaw * fabsf(FSR/(RollMotor1+appsat));
   }

   if ((RollMotor1-appsat)<0)
   {
	   PitchMotor = PitchMotor * fabsf(RollMotor1/appsat);
	   app_yaw =   app_yaw * fabsf(RollMotor1/appsat);
   }

   appsat = abs(PitchMotor+app_yaw);

   if ((RollMotor2+appsat)>FSR)
   {
	   PitchMotor = PitchMotor * fabsf(FSR/(RollMotor2+appsat));
	   app_yaw =   app_yaw * fabsf(FSR/(RollMotor2+appsat));
   }

   if ((RollMotor2-appsat)<0)
   {
	   PitchMotor = PitchMotor * fabsf(RollMotor2/appsat);
	   app_yaw =   app_yaw * fabsf(RollMotor2/appsat);
   }


   int32_t MPower[4];
   // Motor configuration
   MPower[0]=(int32_t)(RollMotor1+PitchMotor-app_yaw);
   MPower[1]=(int32_t)(RollMotor1-PitchMotor+app_yaw);
   MPower[2]=(int32_t)(RollMotor2-PitchMotor -app_yaw);
   MPower[3]=(int32_t)(RollMotor2+ PitchMotor +app_yaw);


   // motor saturation
   motorPowerM1 = limitThrust_2(MPower[0]+FSR);
   motorPowerM2 = limitThrust_2(MPower[1]+FSR);

   motorPowerM3 = limitThrust_2(MPower[2]+FSR);
   motorPowerM4 = limitThrust_2(MPower[3]+FSR);
   
   // actuation
   motorsSetRatio(MOTOR_M1, motorPowerM3);
   motorsSetRatio(MOTOR_M2, motorPowerM2);
   motorsSetRatio(MOTOR_M3, motorPowerM1);
   motorsSetRatio(MOTOR_M4, motorPowerM4);
}


float prevRPY[3];

void flipping(float eulerRollActual,float eulerPitchActual, float wx,float wy,float wz,float FREQ,float eulerRollDesired)
{

	float val,dRoll;
	val = M_PI_F / 180.0f;

	eulerRollActual=eulerRollActual*val;
	eulerPitchActual=eulerPitchActual*val;
	eulerRollDesired = eulerRollDesired * val;

	wx=wx*val;
	wy=-wy*val;
	wz=wz*val;

	// Compute the derivative of roll
	dRoll=wx+sinf(eulerRollActual)*tanf(eulerPitchActual)*wy+cosf(eulerRollActual)*tanf(eulerPitchActual)*wz;

	float Length = 0.1077*0.5; // Kg
	float FMotorMax = (0.980665)*0.95;
	float FSR=32767.0;

	float KV,KP,I,tauRoll;
	KV=50*2;
	KP=350*2;
	I=0.00025;
	tauRoll=(KP*(-eulerRollActual)-KV*dRoll)*I;
	
	float F2;
	float L0 = 0.045;
	float l=Length;

	F2 = tauRoll/(2*l+L0);

	// Compute duty cycle and saturation
	int32_t RollMotor2;

	F2 = F2*0.5f;
	RollMotor2 =(int32_t) (F2 * (FSR / FMotorMax));
	if (RollMotor2>FSR)
			RollMotor2=FSR;
	if (RollMotor2<-FSR)
		RollMotor2=-FSR;


   int32_t MPower[4];
   // New Configuration no priority CHECK with the quad
   MPower[0]=(int32_t)(0);
   MPower[1]=(int32_t)(0);
   MPower[2]=(int32_t)(RollMotor2);
   MPower[3]=(int32_t)(RollMotor2);

   // saturation
   motorPowerM1 = limitThrust_1(MPower[0]+FSR);
   motorPowerM2 = limitThrust_1(MPower[1]+FSR);

   motorPowerM3 = limitThrust_1(MPower[2]+FSR);
   motorPowerM4 = limitThrust_1(MPower[3]+FSR);

   // actuation
   motorsSetRatio(MOTOR_M1, motorPowerM3);
   motorsSetRatio(MOTOR_M2, motorPowerM2);
   motorsSetRatio(MOTOR_M3, motorPowerM1);
   motorsSetRatio(MOTOR_M4, motorPowerM4);

}





LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)
