import numpy as np

class PDController:
    """PD controller with configurable gains.

    compute_action(reference, observation) -> control action

    Includes:
    - Proportional and derivative terms
    """

    # Initialize controller with given gains
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.prev_error = 0.0
    
    def compute_action(self, reference, observation):     
        # Calculate error
        error = reference - observation
        
        # Calculate error derivative
        error_derivative = error - self.prev_error
        
        # Calculate control action using PD control law
        control_action = self.Kp * error + self.Kd * error_derivative
        
        # Update previous values for next iteration
        self.prev_error = error
        
        return control_action


class PIDController:
    """PID controller with configurable gains.

    compute_action(reference, observation) -> control action

    Features:
    - Proportional, integral and derivative terms
    - reset() method to clear internal state
    """

    # Initialize controller with given gains
    def __init__(self, Kp, Kd, Ki):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.prev_error = 0

        #Internal states
        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = 0.01 # Default time step

    def compute_action(self, reference, observation):

        # Compute error
        error = reference - observation

        # Proportional term
        P = self.Kp * error

        # Integral term (accumulate)
        self.integral += error * self.dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = error - self.prev_error
        D = self.Kd * derivative

        # Save error for next step
        self.prev_error = error

        # Return control action
        control_action = P + I + D
        return control_action

    def reset(self):
        """Reset internal integrator and derivative history."""
        self.integral = 0.0
        self.prev_error = 0.0
