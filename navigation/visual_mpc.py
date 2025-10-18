# visualize_mpc.py

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# 从您的文件中导入类
from traj_manager import TrajectoryManager
from casadi_mpc import CasadiMpcController

# --- 1. 设置仿真参数 ---
robot_dynamics = {
    "max_velocity": 2.0, "max_acceleration": 1.0,
    "max_omega": 1.5, "max_alpha": 2.0
}
MPC_HORIZON = 20  # N: 预测时域
MPC_DT = 0.1  # dt: 控制步长 (10 Hz)
SIM_DURATION = 15.0  # 仿真总时长 (秒)


# --- 2. 创建一个更有趣的3D参考轨迹 (螺旋线) ---
def create_helix_trajectory(duration, dt):
    t_points = np.arange(0, duration, dt)
    num_points = len(t_points)

    radius = 5.0
    pitch = 2.0

    # 位置 (x, y, z, yaw)
    x = radius * np.cos(2 * np.pi * t_points / duration)
    y = radius * np.sin(2 * np.pi * t_points / duration)
    z = pitch * t_points
    # Yaw 始终朝向切线方向
    yaw = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))

    positions = np.column_stack((x, y, z, yaw))

    # 速度 (vx, vy, vz, omega)
    velocities = np.gradient(positions, dt, axis=0)

    return {
        'success': True,
        'timestamps': t_points.tolist(),
        'positions': positions.tolist(),
        'velocities': velocities.tolist()
    }


# --- 3. 仿真和可视化主类 ---
class MpcVisualSimulator:
    def __init__(self):
        # 初始化轨迹和控制器
        mock_traj_data = create_helix_trajectory(SIM_DURATION, MPC_DT)
        self.traj_manager = TrajectoryManager(mock_traj_data)
        self.mpc_controller = CasadiMpcController(robot_dynamics, N=MPC_HORIZON, dt=MPC_DT)

        # 初始化仿真状态
        self.current_time = 0.0
        # 让机器人从一个有初始误差的位置开始
        self.current_state = np.array(mock_traj_data['positions'][0]) + np.array([0.5, -0.5, 0.2, 0.1])
        self.actual_path_history = [self.current_state.copy()]

        # 设置Matplotlib图形
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self._init_plot()

    def _init_plot(self):
        # 绘制静态的全局参考轨迹
        full_ref_pos = np.array(self.traj_manager.positions)
        self.ax.plot(full_ref_pos[:, 0], full_ref_pos[:, 1], full_ref_pos[:, 2], ':', color='gray',
                     label='Global Reference Trajectory')

        # 创建动态更新的绘图对象
        self.line_actual, = self.ax.plot([], [], [], '-', color='b', lw=2, label='Actual Path')
        self.line_ref_slice, = self.ax.plot([], [], [], '--', color='g', lw=2, label='Short-term Reference')
        self.line_predicted, = self.ax.plot([], [], [], '-o', color='r', markersize=3, alpha=0.7,
                                            label='MPC Prediction')
        self.marker_robot, = self.ax.plot([], [], [], 'o', color='b', markersize=8, label='Robot Position')

        # 设置图形属性
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.legend()
        self.ax.grid(True)
        self.time_text = self.ax.text2D(0.05, 0.95, '', transform=self.ax.transAxes)
        # 设置固定的坐标轴范围
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.ax.set_zlim(0, SIM_DURATION * 1.5)

    def _update_plot(self, frame):
        # --- 仿真核心步骤 ---
        if self.current_time >= self.traj_manager.duration:
            self.anim.event_source.stop()  # 仿真结束时停止动画
            return

        # 1. 获取参考切片
        ref_states, ref_controls = self.traj_manager.get_reference_slice(
            t_start=self.current_time,
            horizon_N=MPC_HORIZON,
            dt=MPC_DT
        )

        # 2. 求解MPC
        optimal_command, predicted_states = self.mpc_controller.solve_and_get_prediction(
            self.current_state, ref_states, ref_controls
        )

        # 3. 更新机器人状态 (使用简单的欧拉积分模拟)
        self.current_state = self.mpc_controller.f(self.current_state, optimal_command).full().flatten()
        self.actual_path_history.append(self.current_state.copy())
        self.current_time += MPC_DT

        # --- 更新绘图数据 ---
        path_hist = np.array(self.actual_path_history)
        self.line_actual.set_data(path_hist[:, 0], path_hist[:, 1])
        self.line_actual.set_3d_properties(path_hist[:, 2])

        self.line_ref_slice.set_data(ref_states[:, 0], ref_states[:, 1])
        self.line_ref_slice.set_3d_properties(ref_states[:, 2])

        self.line_predicted.set_data(predicted_states[:, 0], predicted_states[:, 1])
        self.line_predicted.set_3d_properties(predicted_states[:, 2])

        self.marker_robot.set_data([self.current_state[0]], [self.current_state[1]])
        self.marker_robot.set_3d_properties([self.current_state[2]])

        self.time_text.set_text(f'Time: {self.current_time:.2f}s')

        # 返回需要重绘的艺术家对象
        return self.line_actual, self.line_ref_slice, self.line_predicted, self.marker_robot, self.time_text

    def run(self):
        # 创建并启动动画
        num_frames = int(SIM_DURATION / MPC_DT)
        self.anim = FuncAnimation(
            self.fig, self._update_plot, frames=num_frames,
            init_func=lambda: None,  # init_plot 已经完成初始化
            interval=MPC_DT * 1000,  # interval 单位是毫秒
            blit=False  # 3D动画中 blit=False 更稳定
        )
        plt.show()


if __name__ == '__main__':
    simulator = MpcVisualSimulator()
    simulator.run()