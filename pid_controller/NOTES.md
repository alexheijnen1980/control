# Step 1: build the PID controller object

Using the following parameters
Kp = 0.2
Ki = 0.004
Pd = 3.0
results in a car that is intructed to drive at high velocities, including those exceeding the max velocity of 3.0 m/s.

**pid_controller.cpp**
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
```
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