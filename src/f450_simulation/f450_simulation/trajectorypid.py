class TPID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
       
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self,error, dt=0.1):
        """
         PID 
        :param dt: sample time
        :return: force
        """
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        # calculate PID 
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output
