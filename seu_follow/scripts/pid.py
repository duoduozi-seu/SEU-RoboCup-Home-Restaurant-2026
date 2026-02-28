#! /home/syw/anaconda3/envs/py3/bin/python
class PID:
    # 给pid的三个参数赋初值
    def __init__(self, kp, ki, kd, dz, th):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dz = dz  # 死区
        self.th = th  # 输出门限
        self.last_error = 0.0
        self.integral = 0.0

    def change_para(self, kp, ki, kd, dz, th):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dz = dz  # 死区
        self.th = th  # 输出门限

    def control_action(self, error, dt):
        """
        Args:
            error: 当前误差
            dt: 步长

        Returns: pid的输出
        """
        if abs(error) < self.dz:  # 误差值在死区内，控制量为0
            self.integral += error
            self.last_error = error
            return 0
        
        else:
            p = self.kp * error

            self.integral += error
            i = self.ki * self.integral

            derivative = (error - self.last_error) / dt
            d = self.kd * derivative
            self.last_error = error

            cmd = p + i + d
            
            # 未超出输出门限
            if abs(cmd) < self.th:  
                return p + i + d
            # 超出输出门限
            return self.th if cmd > 0 else -self.th

