import numpy as np
from scipy.interpolate import interp1d
from typing import Dict, Any, Tuple


class TrajectoryManager:
    """
    管理和插值由 toppra 生成的离散轨迹数据。
    为MPC控制器提供一个连续的、可查询的参考信号。
    """

    def __init__(self, trajectory_data: Dict[str, Any]):
        """
        Args:
            trajectory_data: toppra 函数返回的字典。
        """
        if not trajectory_data['success']:
            raise ValueError("Cannot initialize TrajectoryManager with a failed trajectory.")

        self.timestamps = np.array(trajectory_data['timestamps'])
        self.positions = np.array(trajectory_data['positions'])  # Shape: (N, 4) -> x,y,z,yaw
        self.velocities = np.array(trajectory_data['velocities'])  # Shape: (N, 4) -> vx,vy,vz,omega

        self.duration = self.timestamps[-1]
        self.dof = self.positions.shape[1]  # 自由度应为 4

        # --- [核心] 创建插值器 ---
        # 我们为每个自由度的位置和速度都创建一个独立的线性插值函数。
        # 'linear' 速度快且稳定。'cubic' 更平滑但可能在端点产生过冲。
        # bounds_error=False, fill_value=...: 当查询时间超出范围时，
        # 不会报错，而是使用最后一个点的值。这对于控制器更鲁棒。
        self.pos_interpolators = [
            interp1d(self.timestamps, self.positions[:, i], kind='linear',
                     bounds_error=False, fill_value=self.positions[-1, i])
            for i in range(self.dof)
        ]
        self.vel_interpolators = [
            interp1d(self.timestamps, self.velocities[:, i], kind='linear',
                     bounds_error=False, fill_value=self.velocities[-1, i])
            for i in range(self.dof)
        ]

    def get_reference_at_time(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取单个时间点的参考状态和参考控制量。

        Args:
            t: 查询的时间点 (秒)。

        Returns:
            一个元组 (reference_state, reference_control):
            - reference_state: [px, py, pz, yaw]
            - reference_control: [vx, vy, vz, omega]
        """
        ref_pos = np.array([interp(t) for interp in self.pos_interpolators])
        ref_vel = np.array([interp(t) for interp in self.vel_interpolators])
        return ref_pos, ref_vel

    def get_reference_slice(self, t_start: float, horizon_N: int, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取MPC预测时域内的参考轨迹切片。(这是MPC主循环会调用的函数)

        Args:
            t_start: 切片的起始时间 (通常是当前时间)。
            horizon_N: MPC的预测步数。
            dt: MPC的控制时间步长。

        Returns:
            一个元组 (reference_states, reference_controls):
            - reference_states: 形状为 (N+1, dof) 的参考状态矩阵。
            - reference_controls: 形状为 (N, dof) 的参考控制矩阵。
        """
        # 生成未来 N+1 个时间点 (包括当前点)
        future_times = np.linspace(t_start, t_start + horizon_N * dt, horizon_N + 1)

        # 矢量化调用插值器，一次性计算所有未来点，性能极高
        ref_states = np.array([interp(future_times) for interp in self.pos_interpolators]).T

        # 控制量对应的是每个时间段，所以是 N 个点
        ref_controls = np.array([interp(future_times[:-1]) for interp in self.vel_interpolators]).T

        return ref_states, ref_controls


# --- 使用示例 ---
if __name__ == '__main__':
    # 1. 模拟一个成功的 toppra 输出
    sim_timestamps = np.linspace(0, 5.21, 260)
    # 模拟一个 (260, 4) 的位置和速度数据
    sim_positions = np.random.rand(260, 4)
    sim_velocities = np.random.rand(260, 4)

    mock_trajectory_data = {
        'success': True,
        'timestamps': sim_timestamps.tolist(),
        'positions': sim_positions.tolist(),
        'velocities': sim_velocities.tolist(),
        'accelerations': []  # 这里用不到
    }

    # 2. 初始化 TrajectoryManager
    trajectory_manager = TrajectoryManager(mock_trajectory_data)
    print("TrajectoryManager 初始化成功！")
    print(f"  轨迹总时长: {trajectory_manager.duration:.2f} s")
    print(f"  自由度 (dof): {trajectory_manager.dof}")

    # 3. 模拟MPC调用
    current_time = 1.5  # 假设机器人已经运行了1.5秒
    mpc_horizon = 20  # MPC向前看20步
    mpc_dt = 0.05  # MPC频率为 1/0.05 = 20Hz

    print(f"\n在 {current_time}s 时刻，为MPC生成参考切片...")
    ref_states, ref_controls = trajectory_manager.get_reference_slice(
        t_start=current_time,
        horizon_N=mpc_horizon,
        dt=mpc_dt
    )

    print(f"  生成的参考状态矩阵形状: {ref_states.shape}")  # 应为 (21, 4)
    print(f"  生成的参考控制矩阵形状: {ref_controls.shape}")  # 应为 (20, 4)
    print("\n参考切片的第一个状态 (t=1.5s):")
    print(ref_states[0])
    print("\n参考切片的最后一个状态 (t=1.5 + 20*0.05 = 2.5s):")
    print(ref_states[-1])
