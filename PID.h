#ifndef PID_H
#define PID_H

class PID{
	private:
		float Kp; // proportional gain
		float Ti; // integrator time constant
		float Td; // differential time constant
		float Ts; // sample time
		bool stopInt;
		float oldError;
		float Ui;
		float Ref;
		void setOldError(float);
		float MaxOutput;
		float MinOutput;
	public:
		PID(float,float,float,float);
		void setLimits(float,float);
		void setRef(float);
		float calculate(float);
		void reset(float);
};

#endif

