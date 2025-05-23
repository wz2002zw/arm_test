#!/usr/bin/python3
from pca9685_servo_controller import *
import json
import time
import math
import smbus
from typing import Dict, List, Tuple, Optional
import threading

class Joint:
    """机器人关节基类"""
    
    def __init__(self, controller: ServoController, channel: int, name: str, 
                 min_pulse: int, max_pulse: int, max_angle: int, 
                 home_angle: float = 0, inverse: bool = False):
        """
        初始化关节
        
        :param controller: 舵机控制器实例
        :param channel: PCA9685通道号
        :param name: 关节名称
        :param min_pulse: 最小脉冲宽度
        :param max_pulse: 最大脉冲宽度
        :param max_angle: 最大旋转角度
        :param home_angle: 初始位置角度
        :param inverse: 是否反转控制逻辑
        """
        self.controller = controller
        self.channel = channel
        self.name = name
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.max_angle = max_angle
        self.home_angle = home_angle
        self.inverse = inverse
        
        # 注册到控制器
        self.controller.add_servo(
            channel, 
            min_pulse=min_pulse, 
            max_pulse=max_pulse, 
            max_angle=max_angle
        )
        
        # 移动到初始位置
        self.move_to(home_angle, speed=30)
    
    def move_to(self, angle: float, speed: float = 30, fixed_delay: float = 0.01):
        """移动关节到指定角度"""
        actual_angle = angle
        if self.inverse:
            actual_angle = self.max_angle - angle
            
        self.controller.set_angle_with_speed(
            self.channel, 
            actual_angle, 
            angular_speed=speed,  # 这里设置舵机转动的速度
            fixed_delay=fixed_delay
        )
        print(f"关节 {self.name}({self.channel}) 移动到 {angle}°")
        
    def move_to_home(self, speed: float = 30):
        """移动关节到初始位置"""
        self.move_to(self.home_angle, speed)
        
    def get_current_angle(self) -> float:
        """获取当前角度"""
        angle = self.controller.get_angle(self.channel)
        return angle if not self.inverse else self.max_angle - angle

class Robot:
    """机器人基类"""
    
    def __init__(self, config_file: str = None):
        """
        初始化机器人
        
        :param config_file: 配置文件路径
        """
        self.controller = ServoController()
        self.joints = {}
        
        if config_file:
            self.load_config(config_file)
            
    def load_config(self, config_file: str):
        """从配置文件加载机器人参数"""
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
                
            # 加载关节配置
            if 'joints' in config:
                for joint_config in config['joints']:
                    self.add_joint(**joint_config)
                    
            # 加载控制器配置
            if 'controller' in config:
                controller_config = config['controller']
                self.controller.pwm.setPWMFreq(controller_config.get('pwm_freq', 50))
                
            print(f"已从 {config_file} 加载机器人配置")
                
        except Exception as e:
            print(f"加载配置文件失败: {e}")
            
    def add_joint(self, channel: int, name: str, min_pulse: int, max_pulse: int, 
                  max_angle: int = 180, home_angle: float = 0, inverse: bool = False):
        """添加关节"""
        joint = Joint(
            controller=self.controller,
            channel=channel,
            name=name,
            min_pulse=min_pulse,
            max_pulse=max_pulse,
            max_angle=max_angle,
            home_angle=home_angle,
            inverse=inverse
        )
        self.joints[name] = joint
        return joint
        
    def move_to_home_position(self, speed: float = 30):
        """移动所有关节到初始位置"""
        print("移动到初始位置...")
        for name, joint in self.joints.items():
            joint.move_to_home(speed)
        print("已到达初始位置")
    
    def move(self, angles: Dict[str, float], speeds: Dict[str, float]):
        """同时移动所有关节"""
        if len(angles) != len(speeds):
            raise ValueError("angles 和 speeds 字典的长度必须一致")

        threads = []
        for name, joint in self.joints.items():
            if name in angles and name in speeds:
                angle = angles[name]
                speed = speeds[name]
                # 创建线程来控制每个关节的移动
                thread = threading.Thread(target=joint.move_to, args=(angle, speed))
                threads.append(thread)
                thread.start()

        # 等待所有线程完成
        for thread in threads:
            thread.join()
        
    def get_joint_positions(self) -> Dict[str, float]:
        """获取所有关节的当前位置"""
        return {name: joint.get_current_angle() for name, joint in self.joints.items()}
        
    def save_config(self, config_file: str):
        """保存当前配置到文件"""
        config = {
            'controller': {
                'pwm_freq': 50  # 可以扩展保存更多控制器参数
            },
            'joints': []
        }
        
        for name, joint in self.joints.items():
            joint_config = {
                'channel': joint.channel,
                'name': joint.name,
                'min_pulse': joint.min_pulse,
                'max_pulse': joint.max_pulse,
                'max_angle': joint.max_angle,
                'home_angle': joint.home_angle,
                'inverse': joint.inverse
            }
            config['joints'].append(joint_config)
            
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=4)
        print(f"已保存配置到 {config_file}")

class ArmRobot(Robot):
    """机械臂机器人"""
    
    def __init__(self, config_file: str = None):
        super().__init__(config_file)
        
def main():
    # 从配置文件加载机器人
    robot = ArmRobot("robot_config.json")

    # 保存配置以便将来使用
    robot.save_config("robot_config.json")

    try:
        # 移动到初始位置
        robot.move_to_home_position()

        # 定义要验证的关节角度和速度，包含 joint_0 到 joint_4
        angles = {
            "joint_0": 30,  # 关节 joint_0 目标角度 30 度
            "joint_1": 45,  # 关节 joint_1 目标角度 45 度
            "joint_2": 90,  # 关节 joint_2 目标角度 90 度
            "joint_3": 120, # 关节 joint_3 目标角度 120 度
            "joint_4": 150  # 关节 joint_4 目标角度 150 度
        }
        speeds = {
            "joint_0": 20,  # 关节 joint_0 移动速度 20 度/秒
            "joint_1": 30,  # 关节 joint_1 移动速度 30 度/秒
            "joint_2": 40,  # 关节 joint_2 移动速度 40 度/秒
            "joint_3": 50,  # 关节 joint_3 移动速度 50 度/秒
            "joint_4": 60   # 关节 joint_4 移动速度 60 度/秒
        }

        # 调用 move 函数
        print("开始移动关节...")
        robot.move(angles, speeds)
        print("关节移动完成")

        # 获取当前关节位置
        current_positions = robot.get_joint_positions()

        # 验证关节是否移动到预期位置
        tolerance = 2  # 允许的误差范围，单位：度
        success = True
        for joint_name, target_angle in angles.items():
            current_angle = current_positions.get(joint_name, None)
            if current_angle is None:
                print(f"未找到关节 {joint_name} 的位置信息")
                success = False
            elif abs(current_angle - target_angle) > tolerance:
                print(f"关节 {joint_name} 未移动到预期位置。预期: {target_angle}°, 实际: {current_angle}°")
                success = False

        if success:
            print("move 函数验证成功")
        else:
            print("move 函数验证失败")

    except KeyboardInterrupt:
        print("程序已停止")
    finally:
        # 确保机器人回到安全位置
        robot.move_to_home_position(speed=100)

if __name__ == "__main__":
    main()