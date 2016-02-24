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
 
 #ifndef PID_H
 #define PID_H
 
 #include "mbed.h"
 
 #define TIME_FILTER 	7.9577e-3
 
 class PID{
	 
	 public:
		 
		//Constructor.
		//Links the PID to the Input, Output and SetPoint,
		//Kp, Ki, Kd, Direction
		PID(double*,double*,double*,
			 double, double, double, int);
		
		// Performs the PID Calculation. Should be called in the loop cycles.
		bool run();
		
		//Sets Output limits (max, min)
		void setOutLimits(double, double);
		
		//Set the gains Kp, Ki, Kd
		void setGains(double, double, double);
		
		//Get values
		double getKp();
		double getKi();
		double getKd();
		
		
		
	 private:
		 
		double Kp,Ki,Kd;		//Gains
	  double P, I, D, lastD, lastI;
	  double error, lastError;
		unsigned long dTime;		//differential time
		double outMin, outMax;	//Output Limits
		bool mode;							//Mode of PID
	  Timer t;
	 
	  double *Input;
	  double *Output;
	  double *setPoint;
			
	 
 };
 #endif

 