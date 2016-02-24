/**
 * @author Johnny Garcia
 * Guadalajara, Jalisco, Mexico.
 * Last revision 02/23/2016
 * 
 * @section LICENSE
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 01):
 * <jpgmar@outlook.com> wrote this file. As long as you 
 * retain this notice you can do whatever you want with this stuff. If we meet 
 * some day, and you think this stuff is worth it, you can buy me a beer in return.
 * ----------------------------------------------------------------------------
 *
 * @section DESCRIPTION
 * PID control library.
 * PID controllers are treated as objects from the PID class.
 */

#include "PIDcontrol.h"

PID::PID(double* _Input, double* _Output, double* _setPoint,
	       double _Kp, double _Ki, double _Kd, int controlDirect){

					 t.start();
					 
					 Output = _Output;
					 Input = _Input;
					 setPoint = _setPoint;
  
					 PID::setGains(_Kp, _Ki, _Kd);
					 
					 PID::setOutLimits(0, 255);
					 
					 
}

bool PID::run(){

		error = *setPoint - *Input;
		
		dTime = t.read_ms();
		t.reset();
		
	  P = Kp * error;
		I += error*dTime;
		D = (error - lastError)/dTime;
		
		//Now we add a filter to avoid the first big change when the controller starts to run. 
		D = lastD + (dTime/(dTime + TIME_FILTER)) * (D -lastD);
		
		double output = P + Ki*I + Kd*D;
		
		if(output > outMax) output = outMax;
		else if(output < outMin) output = outMin;
		
		*Output = output;
		
		lastError= error;
		lastD= D;
		return true;
}

void PID::setOutLimits(double max, double min){
	outMax = max;
	outMin = min;
}

void PID::setGains(double _kp, double _ki, double _kd){
	Kp = _kp;
	Ki = _ki;
	Kd = _kd;
}

double PID::getKp(){
	return Kp;
}
double PID::getKi(){
	return Ki;
}
double PID::getKd(){
	return Kd;
}

