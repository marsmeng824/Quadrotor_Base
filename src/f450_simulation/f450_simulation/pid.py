class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, current_value, dt=0.1):
        """
         PID 
        :param current_value: current position
        :param dt: sample time
        :return: force
        """
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        # calculate PID 
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output
