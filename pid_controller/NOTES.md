# Step 1: build the PID controller object

```
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
```

**main.cpp**
```c++
////////////////////////////////////////
// Throttle control
////////////////////////////////////////

// Update the delta time with the previous command
pid_throttle.UpdateDeltaTime(new_delta_time);

// Compute error of speed
double error_throttle;
double target_velocity = v_points.back();
error_throttle = velocity - target_velocity;
cout << "target velocity: " << target_velocity;
cout << " actual velocity: " << velocity;
cout << " error_throttle: " << error_throttle << endl;

// Compute control to apply
double throttle_output;
double brake_output;
pid_throttle.UpdateError(error_throttle);
double throttle = pid_throttle.TotalError();

cout << "control command: " << throttle << endl;

// Adapt the negative throttle to break
if (throttle > 0.0) {
throttle_output = throttle;
brake_output = 0;
} else {
throttle_output = 0;
brake_output = -throttle;
}
```

# Step 2: PID controller for throttle

# Step 3: PID controller for throttle

# Step 4: Evaluate the PID efficiency




// Add the plots to your report and explain them (describe what you see)



// What is the effect of the PID according to the plots, how each part of the PID affects the control command?
*proportional - reacts to the absolute error - will continue to push even when the error starts to reduce derivative - reacts to the time derivative of the error - will start to push in the opposite direction when integral - reacts to the cumulative error - bias.*

// How would you design a way to automatically tune the PID parameters?
*Twiddle*

// PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?
*PID is such a controller that, when one look at its disadvantages……it looks dwarf in comparison to its tall stature in industrial applications. Let’s look at its advantages: It only acts on the error between the desired signal and the controlled signal. Hence, no extra measurements of the internal states are needed, which is one of the biggest advantage in applications because ….. more measurements mean more sensors, more signal conditioning, more maintenance, more cost.
Apart from certain structural properties of the plant/process, not much knowledge of the plant is required for tuning. The tuning can be done through trial and error or a look-up table. Not much expertise is needed for tuning, hence, few middle-skilled technicians may easily carry out the task….again cost cutting
It is efficient and robust against some common uncertainties if properly tuned (around an operating region).
Easy to implement in hardware (through filters) and also easy to implement though microcontrollers, PLC etc. No fancy codes to design… can be written by a middle-level programmer.
Disadvantages: The controller is not really suited for nonlinear plants in general or in layman language, the controller may not assure the desired performance for a changing environment/ operating points. In high-end applications like fighter aircraft, submarines, precision robotics, economic models (stock market predictions) etc, mere fulfilling of the stable performance around a fixed operating region. The controller should be able to track a reference signal under various conditions (even some of them may not be known).*

// (Optional) What would you do to improve the PID controller?
*link the two PID controllers - limit steer output as a function of velocity, limit throttle / brake as a function of steering angle*
