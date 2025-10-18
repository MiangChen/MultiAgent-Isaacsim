# mpc_controller.py

import casadi as ca
import numpy as np
from typing import Dict


class CasadiMpcController:
    def __init__(self, robot_dynamics: Dict, N: int = 20, dt: float = 0.05):
        """
        初始化CasADi MPC控制器。

        Args:
            robot_dynamics: 包含机器人动力学约束的字典。
            N: MPC的预测时域步数 (Horizon)。
            dt: 控制时间步长 (s)。
        """
        self.N = N
        self.dt = dt
        self.robot_dynamics = robot_dynamics

        # 提取动力学参数
        self.v_max = robot_dynamics['max_velocity']
        self.a_max = robot_dynamics['max_acceleration']
        self.omega_max = robot_dynamics['max_omega']
        self.alpha_max = robot_dynamics['max_alpha']

        self.dof_x = 4  # 状态自由度: [px, py, pz, yaw]
        self.dof_u = 4  # 控制自由度: [vx, vy, vz, omega]

        self._build_mpc_problem()

    def _build_mpc_problem(self):
        """
        构建 CasADi 优化问题。
        """
        # 1. 定义符号变量
        # 状态 x = [px, py, pz, yaw]
        x = ca.SX.sym('x', self.dof_x)
        # 控制 u = [vx, vy, vz, omega]
        u = ca.SX.sym('u', self.dof_u)

        # 2. 定义系统模型 (x_dot = f(x, u))
        #    对于全向模型，状态变化直接等于控制输入
        x_dot = ca.vertcat(
            u[0],  # d(px)/dt = vx
            u[1],  # d(py)/dt = vy
            u[2],  # d(pz)/dt = vz
            u[3]  # d(yaw)/dt = omega
        )

        # 将连续模型转换为离散函数 (Forward Euler)
        # x_next = x_k + dt * f(x_k, u_k)
        self.f = ca.Function('f', [x, u], [x + self.dt * x_dot])

        # 3. 定义优化问题变量和参数
        #    - U: 未来 N 步的所有控制指令 (决策变量)
        #    - X: 未来 N+1 步的所有预测状态 (依赖于U)
        #    - P: 参数，包含当前状态 x0 和未来 N+1 步的参考轨迹
        U = ca.SX.sym('U', self.dof_u, self.N)
        X = ca.SX.sym('X', self.dof_x, self.N + 1)
        # 参数 P 是一个长向量，包含了 x_current 和 x_ref_slice, u_ref_slice
        num_params = self.dof_x + (self.N + 1) * self.dof_x + self.N * self.dof_u
        P = ca.SX.sym('P', num_params)

        # 4. 定义代价函数 (Cost Function)
        obj = 0  # 初始化目标函数
        # 权重矩阵
        # Q: 状态跟踪误差权重
        Q = ca.diag([10.0, 10.0, 10.0, 1.0])  # 位置权重 > 姿态权重
        # R: 控制量大小权重
        R = ca.diag([0.5, 0.5, 0.5, 0.1])  # 线速度权重 > 角速度权重

        # 提取参数中的参考轨迹
        x_ref_slice = ca.reshape(P[self.dof_x: self.dof_x + (self.N + 1) * self.dof_x], self.dof_x, self.N + 1)
        u_ref_slice = ca.reshape(P[self.dof_x + (self.N + 1) * self.dof_x:], self.dof_u, self.N)

        # 循环构建代价
        for k in range(self.N):
            state_error = X[:, k] - x_ref_slice[:, k]
            control_error = U[:, k] - u_ref_slice[:, k]
            obj = obj + ca.mtimes([state_error.T, Q, state_error]) \
                  + ca.mtimes([control_error.T, R, control_error])

        # 加上终点状态的误差
        final_state_error = X[:, self.N] - x_ref_slice[:, self.N]
        obj = obj + ca.mtimes([final_state_error.T, Q, final_state_error])

        # 5. 定义约束 (Constraints)
        g = []  # 约束向量

        # 初始状态约束
        x0 = P[:self.dof_x]  # P的前4个元素是当前状态
        g.append(X[:, 0] - x0)

        # 动力学约束 (x_{k+1} == f(x_k, u_k))
        for k in range(self.N):
            g.append(X[:, k + 1] - self.f(X[:, k], U[:, k]))

        # 6. 创建 NLP 求解器
        #    将所有决策变量整合成一个长向量
        OPT_variables = ca.vertcat(
            ca.reshape(X, self.dof_x * (self.N + 1), 1),
            ca.reshape(U, self.dof_u * self.N, 1)
        )

        nlp_prob = {
            'f': obj,
            'x': OPT_variables,
            'g': ca.vertcat(*g),
            'p': P
        }

        opts = {
            'ipopt': {'max_iter': 100, 'print_level': 0, 'acceptable_tol': 1e-8, 'acceptable_obj_change_tol': 1e-6},
            'print_time': 0
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        # 7. 定义约束的上下界
        self.lbg = [0.0] * (self.dof_x * (self.N + 1))
        self.ubg = [0.0] * (self.dof_x * (self.N + 1))

        self.lbx = []
        self.ubx = []
        # 状态X的边界 (这里假设无边界)
        self.lbx.extend([-ca.inf] * self.dof_x * (self.N + 1))
        self.ubx.extend([ca.inf] * self.dof_x * (self.N + 1))

        # 控制U的边界
        for _ in range(self.N):
            self.lbx.extend([-self.v_max, -self.v_max, -self.v_max, -self.omega_max])
            self.ubx.extend([self.v_max, self.v_max, self.v_max, self.omega_max])

    def solve_and_step(self, current_state: np.ndarray, ref_states: np.ndarray, ref_controls: np.ndarray):
        """
        求解MPC问题并返回第一个最优控制指令。

        Args:
            current_state: 机器人当前状态 [px,py,pz,yaw]。
            ref_states: 参考状态切片, shape (N+1, 4)。
            ref_controls: 参考控制切片, shape (N, 4)。

        Returns:
            第一个最优控制指令 [vx,vy,vz,omega]。
        """
        # 构建参数向量 P
        p_vec = np.concatenate([
            current_state.flatten(),
            ref_states.flatten(),
            ref_controls.flatten()
        ])

        # 设置初始猜测 (可选，但有助于热启动)
        # initial_guess = ...

        # 求解
        sol = self.solver(
            x0=0,  # 使用默认初始猜测
            lbx=self.lbx,
            ubx=self.ubx,
            lbg=self.lbg,
            ubg=self.ubg,
            p=p_vec
        )

        # 提取最优控制序列
        optimal_U_flat = sol['x'][self.dof_x * (self.N + 1):].full().flatten()

        # 只返回第一个控制指令
        return optimal_U_flat[:self.dof_u]

    def solve_and_get_prediction(self, current_state: np.ndarray, ref_states: np.ndarray, ref_controls: np.ndarray):
        """
        求解MPC并返回第一个控制指令以及完整的预测状态轨迹。
        (专为可视化和调试设计)
        """
        # 构建参数向量 P
        p_vec = np.concatenate([
            current_state.flatten(),
            ref_states.flatten(),
            ref_controls.flatten()
        ])

        # 求解
        sol = self.solver(
            x0=0, lbx=self.lbx, ubx=self.ubx,
            lbg=self.lbg, ubg=self.ubg, p=p_vec
        )

        # 提取最优控制序列
        solution_vars = sol['x'].full().flatten()
        optimal_U = solution_vars[self.dof_x * (self.N + 1):].reshape(self.N, self.dof_u)

        # 提取最优状态序列
        optimal_X = solution_vars[:self.dof_x * (self.N + 1)].reshape(self.N + 1, self.dof_x)

        # 返回第一个控制指令和完整的状态预测
        return optimal_U[0, :], optimal_X

# --- 独立测试 ---
if __name__ == '__main__':
    from traj_manager import TrajectoryManager  # 假设您的 TrajectoryManager 在这里

    robot_dynamics = {
        "max_velocity": 1.5, "max_acceleration": 0.8,
        "max_omega": 1.0, "max_alpha": 1.5
    }

    mpc_controller = CasadiMpcController(robot_dynamics, N=20, dt=0.05)
    print("MPC控制器初始化成功！")

    # 模拟一个 TrajectoryManager
    mock_timestamps = np.linspace(0, 5, 100)
    mock_positions = np.zeros((100, 4))
    mock_velocities = np.zeros((100, 4))
    mock_traj_data = {'success': True, 'timestamps': mock_timestamps, 'positions': mock_positions,
                      'velocities': mock_velocities}
    traj_manager = TrajectoryManager(mock_traj_data)

    # 模拟当前状态和参考切片
    current_state = np.array([0.1, -0.1, 0.05, 0.02])
    ref_states, ref_controls = traj_manager.get_reference_slice(t_start=0, horizon_N=20, dt=0.05)

    # 求解
    optimal_command = mpc_controller.solve_and_step(current_state, ref_states, ref_controls)

    print("\n求解完成！")
    print(f"当前状态: {current_state}")
    print(f"第一个参考控制: {ref_controls[0]}")
    print(f"计算出的最优指令: {optimal_command}")

