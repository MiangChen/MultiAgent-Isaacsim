
class ControllerPID:
    """
    一个简单的 PID 控制器。
    """

    def __init__(self, Kp, Ki, Kd, target):
        """
        初始化 PID 控制器。

        Args:
            Kp: 比例增益 (Proportional gain).
            Ki: 积分增益 (Integral gain).
            Kd: 微分增益 (Derivative gain).
            target: 目标值 (Desired value).
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target

        self.previous_error = 0
        self.integral = 0
        self.output_min = None
        self.output_max = None

    def set_output_limits(self, min_val, max_val):
        """
        设置输出限制。

        Args:
            min_val: 最小输出值.
            max_val: 最大输出值.
        """
        self.output_min = min_val
        self.output_max = max_val

    def compute(self, current_value, dt):
        """
        计算控制信号。

        Args:
            current_value: 当前值 (Current value).
            dt: 时间间隔 (Time interval).

        Returns:
            控制信号 (Control signal).
        """
        error = self.target - current_value

        # 比例项
        proportional = self.Kp * error

        # 积分项
        self.integral += error * dt
        integral = self.Ki * self.integral

        # 微分项
        derivative = self.Kd * (error - self.previous_error) / dt

        # 总控制信号
        output = proportional + integral + derivative

        # 限制输出
        if self.output_min is not None and self.output_max is not None:
            output = max(self.output_min, min(output, self.output_max))

        # 更新状态
        self.previous_error = error

        return output

# 示例用法
if __name__ == '__main__':
    # 创建 PID 控制器
    pid = ControllerPID(Kp=1.0, Ki=0.1, Kd=0.01, target=10.0)

    # 设置输出限制 (可选)
    pid.set_output_limits(min_val=-10.0, max_val=10.0)

    # 模拟控制循环
    current_value = 0.0
    dt = 0.1  # 时间间隔

    for i in range(100):
        # 计算控制信号
        control_signal = pid.compute(current_value, dt)

        # 模拟系统响应 (这里只是一个简单的示例)
        current_value += control_signal * dt

        # 打印信息
        print(f"Iteration {i+1}: Current Value = {current_value:.2f}, Control Signal = {control_signal:.2f}")
