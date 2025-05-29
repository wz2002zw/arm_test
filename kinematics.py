import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import math
import copy
from minimal_rotation_z_axis_aligner  import align_z_axis_to_global_z
PI=np.pi

def T_rotate_z(theta):
    """
    生成绕Z轴旋转的4x4齐次变换矩阵
    
    参数:
    theta (float): 旋转角度（弧度），正值表示逆时针旋转
    
    返回:
    np.ndarray: 4x4齐次变换矩阵
    """
    # 计算旋转矩阵的元素
    c = np.cos(theta)
    s = np.sin(theta)
    
    # 构建4x4齐次变换矩阵
    T = np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])
    
    return T
    
def T_rotate_x(theta):
    """
    生成绕X轴旋转的4x4齐次变换矩阵
    
    参数:
    theta (float): 旋转角度（弧度），正值表示逆时针旋转
    
    返回:
    np.ndarray: 4x4齐次变换矩阵
    """
    c= np.cos(theta)
    s= np.sin(theta)
    T=np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]
    ])
    return T
def T_rotate_y(theta):
    """
    生成绕Y轴旋转的4x4齐次变换矩阵

    参数:
    theta (float): 旋转角度（弧度），正值表示逆时针旋转
    
    返回:
    np.ndarray: 4x4齐次变换矩阵
    """
    c = np.cos(theta)
    s = np.sin(theta)
    
    T = np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1]
    ])
    
    return T
def normalize_angle(theta, start=0, is_deg=False):
    """
    将角度归一化到以`start`为起点、长度为360°的区间内。
    
    参数：
    theta (float)：输入角度值。
    start (float)：目标区间起点（如-180、0等），区间为[start, start+360)。
    is_deg (bool)：是否为角度值（True）或弧度值（False），默认True。
    
    返回：
    float：归一化后的角度值，位于[start, start+360)区间内。
    """
    if is_deg:
        # 角度值处理（单位：度）
        interval_length = 360.0
        # 先将角度归一化到[0, 360)
        while theta< 0:
            theta += 360
        normalized_0_360 = theta % interval_length
        # 映射到以start为起点的区间
        normalized = normalized_0_360 + start
        # 确保结果在[start, start+360)内
        return normalized 
    else:
        # 弧度值处理（单位：弧度，区间长度为2π）
        interval_length = 2 * math.pi
        while theta< 0:
            theta += 2 * math.pi
        normalized_0_2pi = theta % interval_length
        normalized = normalized_0_2pi + start
        return normalized


def rotx(theta):
    """绕x轴旋转的旋转矩阵"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])
def roty(theta):
    """绕y轴旋转的旋转矩阵"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])
def rotz(theta):
    """绕z轴旋转的旋转矩阵"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1
                      ]])
    
    
    
class RobotKinematics:
    """机器人运动学模型类"""
    def __init__(self, link_lengths=[0.5, 0.5, 0.5, 0]):
        self.__type1="back" #theta1的类型
        self.__type2="lower" #theta2的类型
        self.L1, self.L2, self.L3, self.L4 = link_lengths
        self.__x_adjust=0 # 初x始位置偏移量
        self.__y_adjust=0 # 初y始位置偏移量
        self.__z_adjust=0 # 初z始位置偏移量
        self.__translation_vector=np.array([0, 0, 0])
        self.__alpha_adjust_z=0
        self.__beta_adjust_y=0
        self.__gamma_adjust_z=0 
        self.__angles_adjust_zyz=np.zeros(3) # 关于基座的偏移角度
        self.__rotation_matrix=np.eye(3)
        self.__transformation_matrix=np.eye(4)
        
        self.__theta0_adjust = 0  # 初始关节0角度调整
        self.__theta1_adjust = 0  # 初始关节1角度调整
        self.__theta2_adjust = 0  # 初始关节2角度调整
        self.__theta3_adjust = 0  # 初始关节3角度调整
        self.__theta4_adjust = 0  # 初始关节4角度调整
        self.__theta0_math=0
        self.__theta1_math=0
        self.__theta2_math=0
        self.__theta3_math=0
        self.__theta4_math=0
        
        self.__p0_math = np.zeros(3)  # 基原点
        self.__p1_math = np.zeros(3)
        self.__p2_math = np.zeros(3)
        self.__p3_math = np.zeros(3)
        self.__p4_math = np.zeros(3)
        
        self.__T0_math = np.zeros(4)
        self.__T1_math = np.zeros(4)
        self.__T2_math = np.zeros(4)
        self.__T3_math = np.zeros(4)
        self.__T4_math = np.zeros(4)
        self.__T4_math = np.zeros(4)
        
        self.__theta0=0
        self.__theta1=0
        self.__theta2=0
        self.__theta3=0
        self.__theta4=0
        
        self.__p0 = np.zeros(3)  # 基原点
        self.__p1 = np.zeros(3)
        self.__p2 = np.zeros(3)
        self.__p3 = np.zeros(3)
        self.__p4 = np.zeros(3)
        
        
        self.__T0 = np.eye(4)
        self.__T1 = np.eye(4)
        self.__T2 = np.eye(4)
        self.__T3 = np.eye(4)
        self.__T4 = np.eye(4)
        
        

        self.forward_kinematics(0,0,0,0,0)
    

    
    @property
    def theta0(self)->float:
        return self.__theta0
    @property
    def theta1(self)->float:
        return self.__theta1
    @property
    def theta2(self)->float:
        return self.__theta2
    @property
    def theta3(self)->float:
        return self.__theta3
    @property
    def theta4(self)->float:
        return self.__theta4
    
    @property
    def p0(self)->float:
        return self.__p0
    @property
    def p1(self)->float:
        return self.__p1
    @property
    def p2(self)->float:
        return self.__p2
    @property
    def p3(self)->float:
        return self.__p3
    
    @property
    def p4(self)->float:
        return self.__p4
    @property
    def T4(self)->np.ndarray:
        return self.__T4
    @property
    def theta0_adjust(self)->float:
        return self.__theta0_adjust
    @property
    def theta1_adjust(self)->float:
        return self.__theta1_adjust
    @property
    def theta2_adjust(self)->float:
        return self.__theta2_adjust
    @property
    def theta3_adjust(self)->float:
        return self.__theta3_adjust
    @property
    def theta4_adjust(self)->float:
        return self.__theta4_adjust
    
    @property
    def adjust_x(self)->float:
        return self.__x_adjust
    @property
    def adjust_y(self)->float:
        return self.__y_adjust
    @property
    def adjust_z(self)->float:
        return self.__z_adjust
    
    @property
    def adjust_angles(self)->np.ndarray:
        return self.__angles_adjust_zyz
    
    @property
    def adjust_translation_vector(self)->np.ndarray:
        return self.__translation_vector
    
    @property
    def adjust_rotation_matrix(self)->np.ndarray:
        return self.__rotation_matrix
    
    @property
    def type_1(self)->str:
        return self.__type1
    @property
    def type_2(self)->str:
        return self.__type2
    @staticmethod
    def __zyz_rotation_matrix(alpha, beta, gamma):
        """
        计算 ZYZ 欧拉角对应的旋转矩阵（绕固定坐标系）
        
        参数：
        alpha (float)：绕 Z 轴的第一个旋转角（弧度）
        beta (float)：绕 Y 轴的中间旋转角（弧度）
        gamma (float)：绕 Z 轴的第二个旋转角（弧度）
        
        返回：
        np.ndarray：3x3 旋转矩阵
        """
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        cos_beta = np.cos(beta)
        sin_beta = np.sin(beta)
        cos_gamma = np.cos(gamma)
        sin_gamma = np.sin(gamma)
        
        # 计算矩阵元素
        r11 = cos_alpha * cos_beta * cos_gamma - sin_alpha * sin_gamma
        r12 = -cos_alpha * cos_beta * sin_gamma - sin_alpha * cos_gamma
        r13 = cos_alpha * sin_beta
        
        r21 = sin_alpha * cos_beta * cos_gamma + cos_alpha * sin_gamma
        r22 = -sin_alpha * cos_beta * sin_gamma + cos_alpha * cos_gamma
        r23 = sin_alpha * sin_beta
        
        r31 = -sin_beta * cos_gamma
        r32 = sin_beta * sin_gamma
        r33 = cos_beta
        
        return np.array([
            [r11, r12, r13],
            [r21, r22, r23],
            [r31, r32, r33]
        ])
    
    def __transformation_matrix_update_rotation_part(self, rotation_matrix):
        """
        更新变换矩阵中的旋转部分
        
        参数:
            rotation_matrix (np.ndarray): 3x3旋转矩阵，将替换当前变换矩阵的旋转部分
            
        功能:
        - 保持变换矩阵的平移部分不变
        - 只替换左上角3x3的旋转矩阵部分
        """
        self.__transformation_matrix[0:3, 0:3] = rotation_matrix
    
    def __transformation_matrix_update_translation_part(self, translation_vector):
        """
        更新变换矩阵中的平移部分

        参数:
            translation_vector (np.ndarray): 3x1平移向量，将替换当前变换矩阵的平移部分

        功能:
        - 保持变换矩阵的旋转部分不变
        - 只替换右下角3x1的平移向量部分
        """
        self.__transformation_matrix[:3, 3] = translation_vector
        
    def __set_adjust_angles_zyz(self,alpha,beta,gamma):
        self.__alpha_adjust_z=alpha
        self.__beta_adjust_y=beta
        self.__gamma_adjust_z=gamma
        self.__rotation_matrix=self.__zyz_rotation_matrix(alpha,beta,gamma)
        self.__transformation_matrix_update_rotation_part(self.__rotation_matrix)
      
    def set_adjust_rotation_vector(self, vector:np.ndarray):
        alpha=vector[0]
        beta=vector[1]
        gamma=vector[2]
        self.__set_adjust_angles_zyz(alpha,beta,gamma)
        
    def set_adjust_alpha_z(self,alpha):
        self.__alpha_adjust_z=alpha
        beta=self.__beta_adjust_y
        gamma=self.__gamma_adjust_z
        self.__rotation_matrix=self.__set_adjust_angles_zyz(alpha,beta,gamma)
        
        
    def set_adjust_beta_y(self,beta):
        self.__beta_adjust_y=beta
        alpha=self.__alpha_adjust_z
        gamma=self.__gamma_adjust_z
        self.__rotation_matrix=self.__set_adjust_angles_zyz(alpha,beta,gamma)
        
        
    def set_adjust_gamma_z(self,gamma):
        self.__gamma_adjust_z=gamma
        alpha=self.__alpha_adjust_z
        beta=self.__beta_adjust_y
        self.__rotation_matrix=self.__set_adjust_angles_zyz(alpha,beta,gamma)
        
    
    def set_adjust_translation_vector(self, vector:np.ndarray):
        self.__x_adjust = vector[0]
        self.__y_adjust = vector[1]
        self.__z_adjust = vector[2]
        self.__translation_vector = vector
        self.__transformation_matrix_update_translation_part(self.__translation_vector)    
        
    def set_adjust_x(self, value)->float:
        x=value
        y=self.__y_adjust
        z=self.__z_adjust
        self.set_adjust_translation_vector(np.array([x,y,z]))
        
    def set_adjust_y(self, value)->float:
        x=self.__x_adjust
        y=value
        z=self.__z_adjust
        self.set_adjust_translation_vector(np.array([x,y,z]))
        
    def set_adjust_z(self, value)->float:
        x=self.__x_adjust
        y=self.__y_adjust
        z=value
        self.set_adjust_translation_vector(np.array([x,y,z]))
    
    
        
    def __set_theta0_math(self, value)->float:
        self.__theta0_math=normalize_angle(self.__theta0_adjust+value, start=0, is_deg=False)
    def __set_theta1_math(self, value)->float:
        self.__theta1_math=normalize_angle(self.__theta1_adjust+value, start=0, is_deg=False)
    def __set_theta2_math(self, value)->float:
        self.__theta2_math=normalize_angle(self.__theta2_adjust+value, start=0, is_deg=False)
    def __set_theta3_math(self, value)->float:
        self.__theta3_math=normalize_angle(self.__theta3_adjust+value, start=0, is_deg=False)
    def __set_theta4_math(self, value)->float:
        self.__theta4_math=normalize_angle(self.__theta4_adjust+value, start=0, is_deg=False)
        
    
        
    def dh_transform(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def turn_mathmatical_physical(self):
        """
        将数学计算坐标系下的结果转换为物理坐标系下的表示
        
        功能说明:
        1. 转换数学坐标系下的关节角度到物理坐标系
        2. 转换数学坐标系下的位置坐标到物理坐标系
        3. 转换数学坐标系下的变换矩阵到物理坐标系
        
        注意:
        - 该方法通常在正向运动学计算后被调用
        - 物理坐标系考虑了初始位置偏移和角度调整
        """
        # 转换角度（数学坐标系→物理坐标系）
        self.__theta0 = self.__theta0_math - self.__theta0_adjust
        self.__theta1 = self.__theta1_math - self.__theta1_adjust
        self.__theta2 = self.__theta2_math - self.__theta2_adjust
        self.__theta3 = self.__theta3_math - self.__theta3_adjust
        self.__theta4 = self.__theta4_math - self.__theta4_adjust
        

        # 转换变换矩阵（应用初始旋转）
        
        self.__T0 = np.dot(self.__transformation_matrix, self.__T0_math)
        self.__T1 = np.dot(self.__transformation_matrix, self.__T1_math)
        self.__T2 = np.dot(self.__transformation_matrix, self.__T2_math)
        self.__T3 = np.dot(self.__transformation_matrix, self.__T3_math)
        self.__T4 = np.dot(self.__transformation_matrix, self.__T4_math)

        # 转换位置坐标（应用初始偏移）
        self.__p0 = self.__T0[:3, 3]
        self.__p1 = self.__T1[:3, 3]
        self.__p2 = self.__T2[:3, 3]
        self.__p3 = self.__T3[:3, 3]
        self.__p4 = self.__T4[:3, 3]
        
            
    def forward_kinematics(self, theta0, theta1, theta2, theta3, theta4):
        """
        计算机械臂的正向运动学（Forward Kinematics），根据给定的关节角度计算末端执行器的位姿。

        参数:
            theta0 (float): 第0个关节的角度（单位：弧度）
            theta1 (float): 第1个关节的角度（单位：弧度）
            theta2 (float): 第2个关节的角度（单位：弧度）
            theta3 (float): 第3个关节的角度（单位：弧度）
            theta4 (float): 第4个关节的角度（单位：弧度）

        返回值:
            None: 该函数通过内部方法更新机械臂的状态，不直接返回计算结果。
        """
        
        theta0=normalize_angle(theta0, start=0, is_deg=False)
        theta1=normalize_angle(theta1, start=0, is_deg=False)
        theta2=normalize_angle(theta2, start=0, is_deg=False)
        theta3=normalize_angle(theta3, start=0, is_deg=False)
        theta4=normalize_angle(theta4, start=0, is_deg=False)
        
        self.__theta0=theta0
        self.__theta1=theta1
        self.__theta2=theta2
        self.__theta3=theta3
        self.__theta4=theta4
        
        self.__set_theta0_math(theta0) 
        self.__set_theta1_math(theta1)
        self.__set_theta2_math(theta2)
        self.__set_theta3_math(theta3)
        self.__set_theta4_math(theta4)
        target=PI/2
        if(normalize_angle(self.__theta1_math+PI-target))>PI:
            self.__type1="back"
        else:
            self.__type1="front"
        target=0
        if(normalize_angle(self.__theta3_math+PI-target))>PI:
            self.__type2="lower"
        else:
            self.__type2="upper"

        
        self.__forward_kinematics_math()
        self.turn_mathmatical_physical()
        return self.__T0, self.__T1, self.__T2, self.__T3, self.__T4, self.__p0, self.__p1, self.__p2, self.__p3, self.__p4
        
    def inverse_kinematics(self, T_target):
        """
        逆运动学计算
        
        参数:
            T_target: 目标位姿的4x4变换矩阵
            
        返回:
            各关节角度 [theta0, theta1, theta2, theta3, theta4]
        """
        
        transformation = np.eye(4)
        translation_vector = np.array([x, y, z])
        rotation_matrix=self.__zyz_rotation_matrix(alpha,beta,gamma)
        transformation[:3, :3] = rotation_matrix
        transformation[:3, 3] = translation_vector
        self.inverse_kinematics_core(x,y,z,transformation)
        
    def normalize_angle(self,angle):
        while angle<0:
            angle += 2*PI
        if angle>2*PI:
            angle=angle%(2*PI)
        return angle
    def inverse_kinematics_core(self,x,y,z,trans):
        # 填充旋转部分
        transformation=np.eye(4)
        transformation[:3, :3]=trans
        transformation[:3,3]=[x,y,z]
        
        rotation_matrix= trans
        # 填充平移部分
        self.__T4=copy.deepcopy(transformation)
        # self.__thet4=self.__get_theta4(self.__T4)
        rotation_matrix_dot=align_z_axis_to_global_z(rotation_matrix)
        cos_tmp=rotation_matrix_dot[0,0]
        sin_tmp=rotation_matrix_dot[1,0]
        self.__theta4=copy.deepcopy(np.arctan2(sin_tmp,cos_tmp))
        self.__theta4=self.__theta4-PI
        self.__theta4=self.normalize_angle(self.__theta4)
        
        self.normalize_angle(self.__theta4)
        T14=self.dh_transform(PI/2, 0, 0, PI/2)  # 关节4相对于关节3的变换
        T04 = np.dot(T14,self.dh_transform(self.__theta4, 0, self.L4, 0)) 
        transformation=self.__T4 @ np.linalg.inv(T04)
        self.__T3=copy.deepcopy(transformation)
        
        z = self.__T4[2, :3]  # 变换矩阵T4的第三行（Z轴方向）
        x = self.__T3[0, :3]  # 变换矩阵T3的第一行（X轴方向）

        # 归一化向量
        z_normalized = z / np.linalg.norm(z)
        x_normalized = x / np.linalg.norm(x)

        # 计算点积（注意要限制在[-1, 1]范围内，避免数值误差）
        dot_product = np.clip(np.dot(z_normalized, x_normalized), -1.0, 1.0)

        # 计算夹角（弧度）
        angle = np.arccos(dot_product)

        # 用π减去这个角度
        result_angle = np.pi - angle
        self.__theta3=copy.deepcopy(result_angle)
        self.__T2=copy.deepcopy(self.__T3 @ np.linalg.inv(self.dh_transform(-angle, 0, -self.L3, 0)))
        
        
        x_T3 = self.__T3[0, :3]  # 变换矩阵T3的第一行（X轴方向）
        x_T2 = self.__T2[0, :3]  # 变换矩阵T2的第一行（X轴方向）
        x_T3_normalized = x_T3 / np.linalg.norm(x_T3)
        x_T2_normalized = x_T2 / np.linalg.norm(x_T2)
        
        
        dot_product = np.clip(np.dot(x_T3_normalized, x_T2_normalized), -1.0, 1.0)
        angle = np.arccos(dot_product)
        self.__theta2=copy.deepcopy(angle)
        self.__T1=copy.deepcopy(self.__T2 @ np.linalg.inv(self.dh_transform(-angle, 0, -self.L2, 0)))
        
        x_T1 = self.__T1[0, :3]
        x_T1_normalized = x_T1 / np.linalg.norm(x_T1)
        dot_product = np.clip(np.dot(x_T1_normalized, x_T2_normalized), -1.0, 1.0)
        angle = np.arccos(dot_product)
        self.__theta1=copy.deepcopy(angle)
        T0_tmp=self.__T1@self.dh_transform(self.__theta1, 0, self.L1, 0)
        T0_tmp=T0_tmp@self.dh_transform(0, 0, 0,-PI/2) 
        cos_tmp=T0_tmp[0,0]
        sin_tmp=T0_tmp[1,0]
        self.__theta0=copy.deepcopy(np.arctan2(sin_tmp,cos_tmp))
        return self.__theta0, self.__theta1, self.__theta2, self.__theta3, self.__theta4
        
        
    def __forward_kinematics_math(self):
        """
        计算五关节机器人的正运动学
        :param theta0: 关节0角度（弧度）
        : 关节1角度（弧度）
        :param theta2: 关节2角度（弧度）
        :param theta3: 关节3角度（弧度）
        :param theta4: 关节4角度（弧度）
        :return: 各个关节的变换矩阵和坐标
        """
        
        T00 = self.dh_transform(self.__theta0_math, 0, 0,PI/2)  
        T01 = self.dh_transform(self.__theta1_math, 0, self.L1, 0)  
        T02 = self.dh_transform(self.__theta2_math, 0, self.L2, 0)  
        T03 = self.dh_transform(self.__theta3_math, 0, self.L3, 0)      
        T14=self.dh_transform(PI/2, 0, 0, PI/2)  # 关节4相对于关节3的变换
        T04 = np.dot(T14,self.dh_transform(self.__theta4_math, 0, self.L4, 0))  
        T01_total = np.dot(T00, T01)
        T02_total = np.dot(T01_total, T02)
        T03_total = np.dot(T02_total, T03)
        T04_total = np.dot(T03_total, T04)
        p0 = np.zeros(3)
        p1 = T01_total[:3, 3]
        p2 = T02_total[:3, 3]
        p3 = T03_total[:3, 3]
        p4 = T04_total[:3, 3]
        self.__p0_math = p0  # 基原点
        self.__p1_math = p1
        self.__p2_math = p2
        self.__p3_math = p3
        self.__p4_math = p4
        self.__T0_math=_math=T00
        self.__T1_math=T01_total
        self.__T2_math = T02_total
        self.__T3_math = T03_total
        self.__T4_math = T04_total
        return T00, T01_total, T02_total, T03_total, T04_total, p0, p1, p2, p3, p4
    

    def __forward_kinematics_math_for_training_core(self, theta0, theta1, theta2, theta3, theta4):
        """
        计算机械臂的正向运动学（用于训练的核心数学计算部分）
        
        使用DH参数法计算从基座到末端执行器的变换矩阵，并提取位置和旋转矩阵元素。
        
        参数:
            theta0 (float): 关节0的角度（弧度）
            theta1 (float): 关节1的角度（弧度）
            theta2 (float): 关节2的角度（弧度）
            theta3 (float): 关节3的角度（弧度）
            theta4 (float): 关节4的角度（弧度）
            
        返回:
            tuple: 包含以下元素的元组
                x, y, z (float): 末端执行器的3D坐标
                r00-r22 (float): 旋转矩阵的9个元素（3x3矩阵按行展开）
                self.type_1: 类型参数1（具体含义取决于类实现）
                self.type_2: 类型参数2（具体含义取决于类实现）
        """
        
        # 使用DH参数法计算各关节的变换矩阵
        T00 = self.dh_transform(theta0, 0, 0, PI/2)  # 基座到关节0的变换
        T01 = self.dh_transform(theta1, 0, self.L1, 0)  # 关节0到关节1的变换
        T02 = self.dh_transform(theta2, 0, self.L2, 0)  # 关节1到关节2的变换
        T03 = self.dh_transform(theta3, 0, self.L3, 0)  # 关节2到关节3的变换
        
        # 特殊处理关节4的变换（包含固定偏移）
        T14 = self.dh_transform(PI/2, 0, 0, PI/2)  # 关节3到关节4的固定变换
        T04 = np.dot(T14, self.dh_transform(theta4, 0, self.L4, 0))  # 完整的关节4变换
        
        # 累积计算从基座到末端执行器的总变换矩阵
        T01_total = np.dot(T00, T01)
        T02_total = np.dot(T01_total, T02)
        T03_total = np.dot(T02_total, T03)
        T04_total = np.dot(T03_total, T04)
        
        # 从总变换矩阵中提取旋转矩阵元素
        r00, r01, r02 = T04_total[0, :3]
        r10, r11, r12 = T04_total[1, :3]
        r20, r21, r22 = T04_total[2, :3]
        
        # 提取末端执行器的位置坐标
        x, y, z = T04_total[:3, 3]
        
        return x, y, z, r00, r01, r02, r10, r11, r12, r20, r21, r22, self.type_1, self.type_2
    
    def forward_kinematics_math_for_training(self, s0, c0, s1, c1, s2, c2, s3, c3, s4, c4)->tuple:
        """
        计算五关节机器人的正运动学（用于训练场景）。
        
        通过给定的关节角度的正弦和余弦值，计算各关节的实际角度，并调用核心正运动学计算函数。
        
        参数:
            s0 (float): 第0关节角度的正弦值（sin(theta0)）
            c0 (float): 第0关节角度的余弦值（cos(theta0)）
            s1 (float): 第1关节角度的正弦值（sin(theta1)）
            c1 (float): 第1关节角度的余弦值（cos(theta1)）
            s2 (float): 第2关节角度的正弦值（sin(theta2)）
            c2 (float): 第2关节角度的余弦值（cos(theta2)）
            s3 (float): 第3关节角度的正弦值（sin(theta3)）
            c3 (float): 第3关节角度的余弦值（cos(theta3)）
            s4 (float): 第4关节角度的正弦值（sin(theta4)）
            c4 (float): 第4关节角度的余弦值（cos(theta4)）
            
        返回:
            tuple: 包含以下元素的元组
                x, y, z (float): 末端执行器的3D坐标
                r00-r22 (float): 旋转矩阵的9个元素（3x3矩阵按行展开）
                type_1: 类型参数1（1为front,0为back）
                type_2: 类型参数2（1为lower,0为upper） 
        """
        # 通过arctan2计算各关节的实际角度（避免象限问题）
        theta0 = np.arctan2(s0, c0)
        theta1 = np.arctan2(s1, c1)
        theta2 = np.arctan2(s2, c2)
        theta3 = np.arctan2(s3, c3)
        theta4 = np.arctan2(s4, c4)
            # 调用核心计算方法
        x, y, z, r00, r01, r02, r10, r11, r12, r20, r21, r22, type1, type2 = \
            self._RobotKinematics__forward_kinematics_math_for_training_core(theta0, theta1, theta2, theta3, theta4)
        self._RobotKinematics__type1 = type1
        self._RobotKinematics__type2 = type2
        # 调用核心正运动学计算函数并返回结果
        # x, y, z, r00, r01, r02, r10, r11, r12, r20, r21, r22, self.type_1, self.type_2=self.__forward_kinematics_math_for_training_core(theta0, theta1, theta2, theta3, theta4)
        if self.type_1=="front":
            type_1=0
        else:
            type_1=1
        if self.type_2=="lower":
            type_2=0
        else:
            type_2=1
        return x, y, z, r00, r01, r02, r10, r11, r12, r20, r21, r22, type_1,type_2
    
    def inverse_kinematics(self,T_target,type="lower"):
        """
        逆运动学计算
        
        参数:
            T_target: 目标位姿的4x4变换矩阵
            
        返回:
            各关节角度 [theta0, theta1, theta2, theta3, theta4]
        """
        """
        逆运动学计算
        
        参数:
            T_target: 目标位姿的4x4变换矩阵
            
        返回:
            各关节角度 [theta0, theta1, theta2, theta3, theta4]
        """
        cosine_law_angle = lambda a, b, r: np.arccos((a**2 + b**2 - r**2) / (2 * a * b))
        x,y,z=T_target[:3,3]
        theta0=np.arctan2(y,x)+np.pi
        T_target_mod=T_rotate_z(-theta0)@T_target
        R_part=T_target_mod[:3,:3]
        R_tmp=align_z_axis_to_global_z(R_part)
        theta4=np.arctan2(R_tmp[1,0],R_tmp[0,0])+np.pi
        
        
        # 提取末端执行器的位置和姿态
        T_target_mod_xz=T_target_mod@T_rotate_z(-theta4)
        px, py, pz = T_target_mod_xz[:3, 3]
        R_target = T_target_mod_xz[:3, :3]
        
        # 计算手腕中心位置
        # 从末端位置减去手腕长度向量（沿z轴方向）
        nx, ny, nz = R_target[:3, 2]
        wx = px - self.L3 * nx
        wy = py - self.L3 * ny
        wz = pz - self.L3 * nz
        alpha = np.arctan2(wz, wx)
        r = np.sqrt(wx**2 + wz**2)
        theta1_tmp=cosine_law_angle(self.L1,r,self.L2)
        if type=="lower":
            theta1=alpha-theta1_tmp
        else:
            theta1=alpha+theta1_tmp
            
        theta2_tmp=cosine_law_angle(self.L1,self.L2,r)
        theta2=np.pi-theta2_tmp
        T00=np.eye(4)@self.dh_transform(0,0,0,PI/2)
        T01 = self.dh_transform(theta1, 0, self.L1, 0)
        T02 = self.dh_transform(theta2, 0, self.L2, 0) 
        T01_total = np.dot(T00, T01)
        T02_total = np.dot(T01_total, T02)
        a=np.array([T02_total[0,0],T02_total[2,0]])
        b=np.array([T_target_mod_xz[0,0],T_target_mod_xz[2,0]])
        cross = a[0] * b[1] - a[1] * b[0]  # 二维叉积（行列式）
        dot = np.dot(a, b)
        theta3 = np.arctan2(cross, dot)  # 直接使用ar
        theta3 = np.pi-theta3
        return [theta0, theta1, theta2, theta3, theta4]


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
  
    def plot_robot(self,ax=None):
        """Visualize robot configuration"""
        if ax is None:
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
        
        
class RobotVisualizer:
    """Robot Visualization Class"""
    def __init__(self, kinematics_model):
        self.kinematics = kinematics_model
        self.theta0 = 0
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.theta4 = 0
        self.init_plot()
        self.init_widgets()
        
    def init_plot(self):
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_axes([0.1, 0.3, 0.6, 0.6], projection='3d')
        
        # Set fixed axis limits and equal aspect ratio
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.5, 1.5)
        self.ax.set_zlim(0, 1.5)
        self.ax.set_box_aspect([1, 1, 1])  # Key: Set equal axis ratio
        
        self.ax.set_xlabel('X Axis')
        self.ax.set_ylabel('Y Axis')
        self.ax.set_zlabel('Z Axis')
        self.ax.set_title('5-DOF Robot Kinematics Visualization (Equal Aspect Ratio)')
        
    def init_widgets(self):
        slider_color = 'lightgoldenrodyellow'
        self.sliders = [
            Slider(self.fig.add_axes([0.1, 0.22, 0.6, 0.03], facecolor=slider_color), 
                   'Base Rotation θ0', -PI/2, PI/2, valinit=self.theta0),
            Slider(self.fig.add_axes([0.1, 0.19, 0.6, 0.03], facecolor=slider_color), 
                   'Joint 1 θ1', -PI, PI, valinit=self.theta1),
            Slider(self.fig.add_axes([0.1, 0.16, 0.6, 0.03], facecolor=slider_color), 
                   'Joint 2 θ2', -PI, PI, valinit=self.theta2),
            Slider(self.fig.add_axes([0.1, 0.13, 0.6, 0.03], facecolor=slider_color), 
                   'Joint 3 θ3', -PI, PI, valinit=self.theta3),
            Slider(self.fig.add_axes([0.1, 0.10, 0.6, 0.03], facecolor=slider_color), 
                   'Joint 4 θ4', -PI, PI, valinit=self.theta4)
        ]
        self.button_reset = Button(self.fig.add_axes([0.8, 0.15, 0.1, 0.04], facecolor=slider_color), 
                                   'Reset', hovercolor='0.975')
        self.check = CheckButtons(self.fig.add_axes([0.8, 0.25, 0.15, 0.20]), 
                                  ['Base Frame', 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4'], 
                                  [True, True, True, True, True])
        self.text_display = self.fig.add_axes([0.1, 0.02, 0.8, 0.05]).text(
            0.5, 0.5, "", ha='center', va='center', transform=self.fig.transFigure
        )
        for slider in self.sliders:
            slider.on_changed(self.update)
        self.button_reset.on_clicked(self.reset)
        self.check.on_clicked(self.toggle_frame)
        
    def update(self, val):
        self.theta0, self.theta1, self.theta2, self.theta3, self.theta4 = [s.val for s in self.sliders]
        T00, T01, T02, T03, T04, p0, p1, p2, p3, p4 = self.kinematics.forward_kinematics(
            self.theta0, self.theta1, self.theta2, self.theta3, self.theta4
        )
        
        self.ax.clear()
        
        # Fixed axis limits (prevent auto-scaling during updates)
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.5, 1.5)
        self.ax.set_zlim(0, 1.5)
        self.ax.set_box_aspect([1, 1, 1])  # Reset aspect ratio during update
        
        self.plot_links([p0, p1, p2, p3, p4])
        self.plot_joints([p0, p1, p2, p3, p4])
        self.plot_frames([T00, T01, T02, T03, T04])
        
        self.text_display.set_text(
            f"End Effector Position: ({p4[0]:.3f}, {p4[1]:.3f}, {p4[2]:.3f})\n"
            f"θ0={self.theta0:.1f}° | θ1={self.theta1:.1f}° | θ2={self.theta2:.1f}° | θ3={self.theta3:.1f}° | θ4={self.theta4:.1f}°"
        )
        self.ax.legend(loc='upper left')
        self.fig.canvas.draw_idle()
        print("type_1:"+self.kinematics.type_1)
        print("type_2:"+self.kinematics.type_2)
        
    def plot_links(self, points):
        links = [('b-', 'Link 1'), ('g-', 'Link 2'), ('r-', 'Link 3')]
        for i in range(len(links)):
            self.ax.plot(
                [points[i][0], points[i+1][0]],
                [points[i][1], points[i+1][1]],
                [points[i][2], points[i+1][2]],
                links[i][0], linewidth=5, label=links[i][1]
            )
            
    def plot_joints(self, points):
        colors = ['black', 'black', 'black', 'black', 'purple']
        labels = ['Base Origin', 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4 (End Effector)']
        for i, (p, c, lbl) in enumerate(zip(points, colors, labels)):
            self.ax.scatter(p[0], p[1], p[2], color=c, s=70 if i else 100, label=lbl)
            
    def plot_frames(self, transforms):
        labels = ['Base Frame', 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4']
        status = self.check.get_status()
        for i in range(5):
            if status[i]:
                self.plot_coordinate_system(transforms[i], labels[i], scale=0.15 if i else 0.2)
                
    def plot_coordinate_system(self, transform, label, scale=0.1):
        origin = transform[:3, 3]
        x = transform[:3, 0] * scale
        y = transform[:3, 1] * scale
        z = transform[:3, 2] * scale
        self.ax.quiver(*origin, *x, color='r', linewidth=2, arrow_length_ratio=0.2)
        self.ax.quiver(*origin, *y, color='g', linewidth=2, arrow_length_ratio=0.2)
        self.ax.quiver(*origin, *z, color='b', linewidth=2, arrow_length_ratio=0.2)
        self.ax.text(*(origin + x), "X", color='r', fontweight='bold')
        self.ax.text(*(origin + y), "Y", color='g', fontweight='bold')
        self.ax.text(*(origin + z), "Z", color='b', fontweight='bold')
        self.ax.text(*origin, label, bbox=dict(facecolor='white', alpha=0.7))
        
    def reset(self, event):
        for slider in self.sliders:
            slider.reset()
        self.check.set_status([True]*5)
        self.update(None)
        
    def toggle_frame(self, label):
        idx = ['Base Frame', 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4'].index(label)
        status = self.check.get_status()
        status[idx] = not status[idx]
        self.check.set_status(status)
        self.update(None)    
        
        
if __name__ == "__main__":
    kinematics = RobotKinematics(link_lengths=[0.5, 0.5, 0.5, 0])
    kinematics.forward_kinematics(np.pi/4, np.pi/3, np.pi/4, np.pi/4, np.pi/3)
    # visualizer = RobotVisualizer(kinematics)
    kinematics.inverse_kinematics(kinematics.T4)

    # plt.tight_layout()
    # plt.show()