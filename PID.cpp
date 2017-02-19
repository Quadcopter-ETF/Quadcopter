#include "PID.h"

PID::PID(float Kp,float Ti, float Td, float Ts){
	this->Kp=Kp;
	this->Ti=Ti;
	this->Td=Td;
	this->Ts=Ts;
	this->Ui=0;
	this->stopInt=false;
	this->oldError=0;
}

void PID::setLimits(float max,float min){
	this->MaxOutput=max;
	this->MinOutput=min;
}

void PID::setRef(float Ref){
	this->Ref=Ref;
}

void PID::setOldError(float error){
	this->oldError=error;
}

float PID::calculate(float currentMeasurement){
	float Up,Ud,U;
	float error=this->Ref-currentMeasurement;
	float errorD=this->oldError-error;
	setOldError(error);
	
	Up=this->Kp*error;
	Ud=this->Kp*errorD*(this->Td/this->Ts);
	
	if(this->stopInt==false){
		this->Ui=this->Ui+this->Kp*error*(this->Ts/this->Ti);
	}
	
	U=Up+Ud+this->Ui;
	
	if(U>this->MaxOutput){
		this->stopInt=true;
		U=this->MaxOutput;
	}
	else if(U<this->MinOutput){
		this->stopInt=true;
		U=this->MinOutput;
	}
	else this->stopInt=false;
	
	return U;
}

void PID::reset(float manualU){
	this->Ui=manualU;
}
