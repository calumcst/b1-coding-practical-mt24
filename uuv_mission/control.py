import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

class PDController:
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.prev_error = 0
    
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
