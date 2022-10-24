# Step 1: build the PID controller object

```
void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   proportional_error = -Kp * cte;
   derivative_error = 0.0; 
   integral_error = 0.0;  
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control;

   control = proportional_error + derivative_error + integral_error;
  
   if (control > output_lim_max) {
   	 control = output_lim_max;
   }
   else if (control < output_lim_min) {
     control = output_lim_min;
   }
   
   return control;
}
```

# Step 2: PID controller for throttle

# Step 3: PID controller for throttle

# Step 4: Evaluate the PID efficiency