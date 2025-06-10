import matplotlib.pyplot as plt

# 设置中文字体
plt.rcParams['font.family'] = ['SimHei']  
plt.rcParams['axes.unicode_minus'] = False  

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, T):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.T = T
        self.prev_error = 0
        self.prev_error2 = 0
        self.prev_output = 0
        self.I = 0

    def compute(self, current_temp):
        # 计算当前误差
        error = self.setpoint - current_temp

        # 比例项：当前误差
        P = self.Kp * error

        # 积分项：累计误差
        self.I += self.Ki * error if error <= 10 else 0 # 每次增量乘以采样周期

        # 微分项：误差变化率
        delta_error = error - self.prev_error  # 当前误差与前一个误差之差
        D = self.Kd * delta_error  # 微分项（差分）

        # 总输出（加权）
        output =  P + self.I + D

        # 更新历史值
        self.prev_error2 = self.prev_error

        return output, P, self.I, D


def simulate_boiler_temperature(pid, initial_temp, ambient_temp, heat_loss_coeff,
                                 steps=200, cooling_n=2, include_heat_loss=True):
    """
    模拟锅炉温度变化，加入加热、散热和加水降温功能：
    - steps: 模拟总步数，默认 200
    - cooling_n: 每 100 ms 加水降温 n 摄氏度，默认 2℃
      在第 101 到 150 步期间生效
    - include_heat_loss: 是否考虑散热，默认为True（有散热）
    """
    temperature = initial_temp
    time_steps = [0]
    temperatures = [temperature]
    real_times = [0.0]
    control_times = []

    current_time = 0.0

    for step in range(steps):
        output, P, I, D = pid.compute(temperature)

        # 根据控制信号计算每100ms升温
        heating_rate = output / 10  # 控制信号决定的升温量
        temperature += heating_rate  # 温度增加

        # 散热，速率与温差和时间成正比，只有当include_heat_loss为True时才考虑散热
        dt = 0.1  # 每步 100ms
        if include_heat_loss:
            heat_loss = heat_loss_coeff * (temperature - ambient_temp) * dt
            temperature -= heat_loss

        # 第 101 到 150 步期间，每 100 ms 降温 cooling_n 摄氏度
        if 100 < step + 1 <= 150:
            intervals = 1  # 每次步长即100ms
            temperature -= cooling_n * intervals

        # 记录数据
        time_steps.append(step + 1)
        temperatures.append(temperature)
        control_times.append(dt)
        current_time += dt
        real_times.append(current_time)

    return real_times, temperatures


# 参数设置
Kp = 2.0
Ki = 0.75
Kd = 2.5
setpoint = 70
T = 0.5

# 环境温度和散热系数
ambient_temp = 25    # 环境温度 (℃)
heat_loss_coeff = 0.06  # 散热系数
# 每 100 ms 加水降温 n 摄氏度
cooling_n = 0.5
steps = 300

pid = PIDController(Kp, Ki, Kd, setpoint, T)

# 模拟有散热和无散热的情况
real_times_with_heat_loss, temperatures_with_heat_loss = simulate_boiler_temperature(
    pid, initial_temp=30, ambient_temp=ambient_temp, heat_loss_coeff=heat_loss_coeff,
    steps=steps, cooling_n=cooling_n, include_heat_loss=True
)

real_times_without_heat_loss, temperatures_without_heat_loss = simulate_boiler_temperature(
    pid, initial_temp=30, ambient_temp=ambient_temp, heat_loss_coeff=heat_loss_coeff,
    steps=steps, cooling_n=cooling_n, include_heat_loss=False
)

# 绘制有散热和无散热的温度对比图
plt.figure(figsize=(10, 6))
plt.plot(real_times_with_heat_loss, temperatures_with_heat_loss, label='有散热 (℃)', color='b')
plt.plot(real_times_without_heat_loss, temperatures_without_heat_loss, label='无散热 (℃)', color='r')
plt.axhline(y=pid.setpoint, color='g', linestyle='--', label='目标温度 (℃)')
plt.xlabel('累积时间 (秒)')
plt.ylabel('温度 (℃)')
plt.title('有散热与无散热的锅炉温度对比')
plt.legend()
plt.grid(True)
plt.show()
