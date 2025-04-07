import numpy as np

class PID:
    def __init__(self, kp, ki, kd):
        # 
        self.kp = np.array(kp)
        self.ki = np.array(ki)
        self.kd = np.array(kd)
       
        self.integral = np.zeros(3)  # Integral Part Vector 
        self.previous_error = np.zeros(3)  #Derivetive Part Vector  

    def compute(self, error, dt=0.1):
        """
        :param error: vector error
        :param dt: sample time
        :return: thrust
        """
        error = np.array(error)  
        self.integral += error * dt  
        derivative = (error - self.previous_error) / dt  
        self.previous_error = error   

        # calculate PID output (vector)
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output



