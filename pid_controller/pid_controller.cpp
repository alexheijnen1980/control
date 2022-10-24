/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   // TODO: Initialize PID coefficients (and errors, if needed)
   Kp = Kpi;
   Ki = Kii;
   Kd = Kdi;
   output_lim_max = output_lim_maxi;
   output_lim_min = output_lim_mini;
   prev_cte = 0.0; // initialize as zero
   int_cte = 0.0; // initialize as zero
}

void PID::UpdateError(double cte) {
   // TODO: Update PID errors based on cte.
   
   // Calculate proportional part of control
   proportional_part = -Kp * cte;

   // Calculate integral part of control
   int_cte = int_cte + cte * delta_time;
   integral_part = -Ki * int_cte;

   // Calculate derivative part of control
   derivative_part = -Kd * (cte - prev_cte) / delta_time; 
   prev_cte = cte;
}

double PID::TotalError() {
   // TODO: Calculate and return the total error
   double control;
   control = proportional_part + integral_part + derivative_part;

   // Limit control value (if necessary)  
   if (control > output_lim_max) {
   	 control = output_lim_max;
   }
   else if (control < output_lim_min) {
     control = output_lim_min;
   }
   
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   // TODO: Update the delta time with new value
  	delta_time = new_delta_time;
}