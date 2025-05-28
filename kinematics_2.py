from minimal_rotation_z_axis_aligner import align_z_axis_to_global_z, rotation_matrix_intrinsic_ZYZ, plot_coordinate_system
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
import copy

class RobotKinematics:
    """机器人运动学库，包含正运动学和逆运动学计算"""
    
    def __init__(self, L1, L2, L3, L4):
        """
        初始化机器人参数
        
        参数:
            L1-L4: 机器人各连杆长度
        """
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L4 = L4
        self.PI = np.pi
        
        # 存储各关节变换矩阵
        self.__T0 = np.eye(4)
        self.__T1 = np.eye(4)
        self.__T2 = np.eye(4)
        self.__T3 = np.eye(4)
        self.__T4 = np.eye(4)
        
        # 存储关节角度
        self.__theta0 = 0.0
        self.__theta1 = 0.0
        self.__theta2 = 0.0
        self.__theta3 = 0.0
        self.__theta4 = 0.0
    
    def dh_transform(self, theta, d, a, alpha):
        """
        计算标准DH参数变换矩阵
        
        参数:
            theta: 绕z轴的旋转角度
            d: 沿z轴的平移
            a: 沿x轴的平移
            alpha: 绕x轴的旋转角度
            
        返回:
            4x4齐次变换矩阵
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        
        return T
    
    
    def forward_kinematics(self, theta0, theta1, theta2, theta3, theta4):
        """
        正运动学计算
        
        参数:
            theta0-theta4: 各关节角度
            
        返回:
            末端执行器的4x4变换矩阵
        """
        # 存储关节角度
        self.__theta0 = theta0
        self.__theta1 = theta1
        self.__theta2 = theta2
        self.__theta3 = theta3
        self.__theta4 = theta4
        
        # 计算各关节变换矩阵
        self.__T0 = np.eye(4)
        self.__T1 = self.__T0 @ self.dh_transform(theta0, 0, 0, self.PI/2)
        self.__T2 = self.__T1 @ self.dh_transform(theta1, 0, self.L1, 0)
        self.__T3 = self.__T2 @ self.dh_transform(theta2, 0, self.L2, 0)
        
        # 特殊处理关节4（包含固定变换和旋转）
        T14 = self.dh_transform(self.PI/2, 0, 0, self.PI/2)
        T4 = T14 @ self.dh_transform(theta4, 0, self.L4, 0)
        self.__T4 = self.__T3 @ T4
        
        return self.__T4
        
    def inverse_kinematics(self, T_target):
        """
        逆运动学计算
        
        参数:
            T_target: 目标位姿的4x4变换矩阵
            
        返回:
            各关节角度 [theta0, theta1, theta2, theta3, theta4]
        """
        # 提取末端执行器的位置和姿态
        px, py, pz = T_target[0, 3], T_target[1, 3], T_target[2, 3]
        R_target = T_target[:3, :3]
        
        # 计算手腕中心位置
        # 从末端位置减去手腕长度向量（沿z轴方向）
        nx, ny, nz = R_target[0, 2], R_target[1, 2], R_target[2, 2]
        wx = px - self.L4 * nx
        wy = py - self.L4 * ny
        wz = pz - self.L4 * nz
        
        # 关节0：绕基坐标系z轴旋转
        theta0 = np.arctan2(wy, wx)
        
        # 简化计算，考虑关节0旋转后的坐标系
        r = np.sqrt(wx**2 + wy**2)
        z = wz - self.L1
        
        # 关节1和关节2：解两连杆平面机构
        D = (r**2 + z**2 - self.L2**2) / (2 * self.L2)
        if abs(D) > 1:
            raise ValueError("目标位置超出工作空间")
        
        # 计算theta3（肘关节角度）
        theta3 = np.arccos(D)
        
        # 计算theta2（肩关节角度）
        k1 = self.L2 * np.sin(theta3)
        k2 = self.L2 * np.cos(theta3) + self.L2
        
        theta2 = np.arctan2(z, r) - np.arctan2(k1, k2)
        
        # 关节3：通常设置为0或其他固定值，这里假设为0
        theta3 = 0.0
        
        # 计算关节4（手腕旋转）
        # 构造当前姿态矩阵
        self.forward_kinematics(theta0, theta1, theta2, theta3, 0)
        R_current = self.__T3[:3, :3]
        
        # 计算所需的手腕旋转
        R_wrist = R_current.T @ R_target
        # 提取绕z轴的旋转角度
        theta4 = np.arctan2(R_wrist[1, 0], R_wrist[0, 0])
        
        return [theta0, theta1, theta2, theta3, theta4]

    def _wrap_angle(self, angle):
        """将角度规范化到 [-π, π] 范围内"""
        while angle > self.PI:
            angle -= 2 * self.PI
        while angle < -self.PI:
            angle += 2 * self.PI
        return angle    
    def plot_robot(self):
        """Visualize robot configuration"""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot joint positions
        p0 = self.__T0[:3, 3]
        p1 = self.__T1[:3, 3]
        p2 = self.__T2[:3, 3]
        p3 = self.__T3[:3, 3]
        p4 = self.__T4[:3, 3]
        
        # Plot links
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], 'b-', linewidth=3)
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'g-', linewidth=3)
        ax.plot([p2[0], p3[0]], [p2[1], p3[1]], [p2[2], p3[2]], 'r-', linewidth=3)
        ax.plot([p3[0], p4[0]], [p3[1], p4[1]], [p3[2], p4[2]], 'm-', linewidth=3)
        
        # Plot joints
        ax.scatter(p0[0], p0[1], p0[2], c='k', s=50, marker='o')
        ax.scatter(p1[0], p1[1], p1[2], c='k', s=50, marker='o')
        ax.scatter(p2[0], p2[1], p2[2], c='k', s=50, marker='o')
        ax.scatter(p3[0], p3[1], p3[2], c='k', s=50, marker='o')
        ax.scatter(p4[0], p4[1], p4[2], c='k', s=50, marker='o')
        
        # Plot coordinate systems
        self._plot_coordinate_system(ax, self.__T0, scale=0.2, label='Base')
        self._plot_coordinate_system(ax, self.__T1, scale=0.2, label='Joint1')
        self._plot_coordinate_system(ax, self.__T2, scale=0.2, label='Joint2')
        self._plot_coordinate_system(ax, self.__T3, scale=0.2, label='Joint3')
        self._plot_coordinate_system(ax, self.__T4, scale=0.3, label='End Effector')
        
        # Set plot properties
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])
        ax.set_zlim([0, 2.0])
        ax.set_title('Robot Kinematics Visualization')
        
        # Display joint angles
        angles_deg = [np.rad2deg(angle) for angle in 
                    [self.__theta0, self.__theta1, self.__theta2, self.__theta3, self.__theta4]]
        ax.text2D(0.05, 0.95, 
                f'theta0: {angles_deg[0]:.1f}, theta1: {angles_deg[1]:.1f}, theta2: {angles_deg[2]:.1f}\n'
                f'theta3: {angles_deg[3]:.1f}, theta4: {angles_deg[4]:.1f}', 
                transform=ax.transAxes)
        
        plt.tight_layout()
        plt.show()
    
    def _plot_coordinate_system(self, ax, T, scale=0.2, label=None):
        """Helper function: Plot coordinate system"""
        origin = T[:3, 3]
        R = T[:3, :3]
        
        # X-axis (red)
        end_x = origin + R[:, 0] * scale
        ax.plot([origin[0], end_x[0]], [origin[1], end_x[1]], [origin[2], end_x[2]], 'r-')
        
        # Y-axis (green)
        end_y = origin + R[:, 1] * scale
        ax.plot([origin[0], end_y[0]], [origin[1], end_y[1]], [origin[2], end_y[2]], 'g-')
        
        # Z-axis (blue)
        end_z = origin + R[:, 2] * scale
        ax.plot([origin[0], end_z[0]], [origin[1], end_z[1]], [origin[2], end_z[2]], 'b-')
        
        # Add label
        if label:
            ax.text(origin[0], origin[1], origin[2], label)
    
    def interactive_visualization(self):
        """Create interactive visualization interface"""
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        plt.subplots_adjust(bottom=0.35)
        
        # Initial joint angles (radians)
        theta0_init = 0.0
        theta1_init = self.PI/4
        theta2_init = -self.PI/4
        theta3_init = self.PI/6
        theta4_init = 0.0
        
        # Calculate and plot initial configuration
        self.forward_kinematics(theta0_init, theta1_init, theta2_init, theta3_init, theta4_init)
        self._plot_robot_interactive(ax)
        
        # Create sliders
        ax_theta0 = plt.axes([0.25, 0.25, 0.65, 0.03])
        ax_theta1 = plt.axes([0.25, 0.20, 0.65, 0.03])
        ax_theta2 = plt.axes([0.25, 0.15, 0.65, 0.03])
        ax_theta3 = plt.axes([0.25, 0.10, 0.65, 0.03])
        ax_theta4 = plt.axes([0.25, 0.05, 0.65, 0.03])
        
        slider_theta0 = Slider(ax_theta0, 'theta0 (deg)', -180, 180, valinit=np.rad2deg(theta0_init))
        slider_theta1 = Slider(ax_theta1, 'theta1 (deg)', -90, 90, valinit=np.rad2deg(theta1_init))
        slider_theta2 = Slider(ax_theta2, 'theta2 (deg)', -135, 45, valinit=np.rad2deg(theta2_init))
        slider_theta3 = Slider(ax_theta3, 'theta3 (deg)', -90, 90, valinit=np.rad2deg(theta3_init))
        slider_theta4 = Slider(ax_theta4, 'theta4 (deg)', -180, 180, valinit=np.rad2deg(theta4_init))
        
        def update(val):
            # Get slider values and convert to radians
            theta0 = np.deg2rad(slider_theta0.val)
            theta1 = np.deg2rad(slider_theta1.val)
            theta2 = np.deg2rad(slider_theta2.val)
            theta3 = np.deg2rad(slider_theta3.val)
            theta4 = np.deg2rad(slider_theta4.val)
            
            # Calculate forward kinematics
            self.forward_kinematics(theta0, theta1, theta2, theta3, theta4)
            
            # Clear and redraw
            ax.clear()
            self._plot_robot_interactive(ax)
            fig.canvas.draw_idle()
        
        # Register update function
        slider_theta0.on_changed(update)
        slider_theta1.on_changed(update)
        slider_theta2.on_changed(update)
        slider_theta3.on_changed(update)
        slider_theta4.on_changed(update)
        
        plt.show()
    
    def _plot_robot_interactive(self, ax):
        """Helper function: Plot robot for interactive visualization"""
        # Plot joint positions
        p0 = self.__T0[:3, 3]
        p1 = self.__T1[:3, 3]
        p2 = self.__T2[:3, 3]
        p3 = self.__T3[:3, 3]
        p4 = self.__T4[:3, 3]
        
        # Plot links
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], 'b-', linewidth=3)
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'g-', linewidth=3)
        ax.plot([p2[0], p3[0]], [p2[1], p3[1]], [p2[2], p3[2]], 'r-', linewidth=3)
        ax.plot([p3[0], p4[0]], [p3[1], p4[1]], [p3[2], p4[2]], 'm-', linewidth=3)
        
        # Plot joints
        ax.scatter(p0[0], p0[1], p0[2], c='k', s=50, marker='o')
        ax.scatter(p1[0], p1[1], p1[2], c='k', s=50, marker='o')
        ax.scatter(p2[0], p2[1], p2[2], c='k', s=50, marker='o')
        ax.scatter(p3[0], p3[1], p3[2], c='k', s=50, marker='o')
        ax.scatter(p4[0], p4[1], p4[2], c='k', s=50, marker='o')
        
        # Plot coordinate systems
        self._plot_coordinate_system(ax, self.__T0, scale=0.2, label='Base')
        self._plot_coordinate_system(ax, self.__T1, scale=0.2, label='J1')
        self._plot_coordinate_system(ax, self.__T2, scale=0.2, label='J2')
        self._plot_coordinate_system(ax, self.__T3, scale=0.2, label='J3')
        self._plot_coordinate_system(ax, self.__T4, scale=0.3, label='EE')
        
        # Set plot properties
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])
        ax.set_zlim([0, 2.0])
        ax.set_title('Interactive Robot Kinematics')
        
        # Display joint angles and end effector position
        angles_deg = [np.rad2deg(angle) for angle in 
                     [self.__theta0, self.__theta1, self.__theta2, self.__theta3, self.__theta4]]
        end_pos = self.__T4[:3, 3]
        
        ax.text2D(0.05, 0.95, 
                 f'theta0: {angles_deg[0]:.1f}, theta1: {angles_deg[1]:.1f}, theta2: {angles_deg[2]:.1f}\n'
                 f'theta3: {angles_deg[3]:.1f}, theta4: {angles_deg[4]:.1f}\n'
                 f'End Effector: [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]', 
                 transform=ax.transAxes)


# Example usage
if __name__ == "__main__":
    # Create robot instance (units: meters)
    robot = RobotKinematics(L1=0.5, L2=0.4, L3=0.3, L4=0.2)
    
    # ===== Forward Kinematics Example =====
    print("\n===== Forward Kinematics Example =====")
    # Set joint angles (radians)
    theta0 = np.deg2rad(30)
    theta1 = np.deg2rad(45)
    theta2 = np.deg2rad(-30)
    theta3 = np.deg2rad(60)
    theta4 = np.deg2rad(15)
    
    # Calculate forward kinematics
    T_end = robot.forward_kinematics(theta0, theta1, theta2, theta3, theta4)
    
    # Display end effector position and orientation
    end_position = T_end[:3, 3]
    end_orientation = T_end[:3, :3]
    
    print(f"End Effector Position: [{end_position[0]:.4f}, {end_position[1]:.4f}, {end_position[2]:.4f}]")
    print("End Effector Rotation Matrix:")
    print(end_orientation)
    
    # Visualize
    robot.plot_robot()
    
    # ===== Inverse Kinematics Example =====
    print("\n===== Inverse Kinematics Example =====")
    # Use the result from forward kinematics as target pose
    T_target = T_end.copy()
    
    # Calculate inverse kinematics
    theta0_ik, theta1_ik, theta2_ik, theta3_ik, theta4_ik = robot.inverse_kinematics(T_target)
    
    # Display calculated joint angles
    print("Calculated Joint Angles:")
    print(f"theta0: {np.rad2deg(theta0_ik):.2f}, theta1: {np.rad2deg(theta1_ik):.2f}, theta2: {np.rad2deg(theta2_ik):.2f}")
    print(f"theta3: {np.rad2deg(theta3_ik):.2f}, theta4: {np.rad2deg(theta4_ik):.2f}")
    
    # Verify: Run forward kinematics with calculated joint angles
    T_verify = robot.forward_kinematics(theta0_ik, theta1_ik, theta2_ik, theta3_ik, theta4_ik)
    
    # Calculate errors
    position_error = np.linalg.norm(T_verify[:3, 3] - T_target[:3, 3])
    rotation_error = np.arccos((np.trace(T_verify[:3, :3] @ T_target[:3, :3].T) - 1) / 2)
    
    print(f"Position Error: {position_error:.6f} m")
    print(f"Orientation Error: {np.rad2deg(rotation_error):.6f}")
    
    # Visualize results
    robot.plot_robot()
    
    # ===== Interactive Visualization =====
    print("\n===== Interactive Visualization =====")
    print("Close current figure window to start interactive visualization...")
    # robot.interactive_visualization()  # Uncomment to enable interactive visualization