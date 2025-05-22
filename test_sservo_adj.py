#!/usr/bin/python3

import time
import math
import smbus

# PCA9685寄存器地址
_SUBADR1 = 0x02
_SUBADR2 = 0x03
_SUBADR3 = 0x04
_MODE1 = 0x00
_PRESCALE = 0xFE
_LED0_ON_L = 0x06
_LED0_ON_H = 0x07
_LED0_OFF_L = 0x08
_LED0_OFF_H = 0x09
_ALLLED_ON_L = 0xFA
_ALLLED_ON_H = 0xFB
_ALLLED_OFF_L = 0xFC
_ALLLED_OFF_H = 0xFD

class PCA9685:
    """控制PCA9685 16通道PWM驱动模块的类"""
    
    def __init__(self, address=0x40, debug=False):
        """初始化PCA9685驱动"""
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        if self.debug:
            print("Reseting PCA9685")
        self.write(_MODE1, 0x00)

    def write(self, reg, value):
        """向指定寄存器写入一个8位值"""
        self.bus.write_byte_data(self.address, reg, value)
        if self.debug:
            print(f"I2C: Write 0x{value:02X} to register 0x{reg:02X}")

    def read(self, reg):
        """从I2C设备读取一个无符号字节"""
        result = self.bus.read_byte_data(self.address, reg)
        if self.debug:
            print(f"I2C: Device 0x{self.address:02X} returned 0x{result & 0xFF:02X} from reg 0x{reg:02X}")
        return result

    def setPWMFreq(self, freq):
        """设置PWM输出频率"""
        prescaleval = 50000000.0    # 晶振频率为50MHz
        prescaleval /= 4096.0       # 分辨率为12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        if self.debug:
            print(f"Setting PWM frequency to {freq} Hz")
            print(f"Estimated pre-scale: {prescaleval}")
        prescale = math.floor(prescaleval + 0.5)
        if self.debug:
            print(f"Final pre-scale: {prescale}")

        oldmode = self.read(_MODE1)
        newmode = (oldmode & 0x7F) | 0x10  # 进入睡眠模式以修改预分频器
        self.write(_MODE1, newmode)  # 写入新配置
        self.write(_PRESCALE, int(math.floor(prescale)))
        self.write(_MODE1, oldmode)
        time.sleep(0.005)
        self.write(_MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        """设置单个PWM通道的ON和OFF值"""
        self.write(_LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(_LED0_ON_H + 4 * channel, on >> 8)
        self.write(_LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(_LED0_OFF_H + 4 * channel, off >> 8)
        if self.debug:
            print(f"channel: {channel}  LED_ON: {on} LED_OFF: {off}")

    def setServoPulse(self, channel, pulse):
        """设置伺服电机脉冲宽度"""
        pulse = pulse * 4096 / 20000  # 对应50Hz的周期（20000微秒）
        self.setPWM(channel, 0, int(pulse))

class ServoController:
    """舵机控制器，封装不同类型舵机的控制逻辑"""
    
    def __init__(self, pwm_freq=50):
        """初始化舵机控制器"""
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(pwm_freq)
        self.servo_config = {}  # 存储各通道舵机配置
        
    def add_servo(self, channel, min_pulse=500, max_pulse=2500, max_angle=180):
        """
        添加舵机配置
        
        :param channel: 舵机通道
        :param min_pulse: 最小脉冲宽度(us)
        :param max_pulse: 最大脉冲宽度(us)
        :param max_angle: 最大角度范围
        """
        self.servo_config[channel] = {
            'min_pulse': min_pulse,
            'max_pulse': max_pulse,
            'max_angle': max_angle,
            'current_angle': 0  # 新增：记录当前角度，初始为0度
        }
        
    def set_angle(self, channel, angle):
        """
        设置指定通道舵机角度
        
        :param channel: 舵机通道
        :param angle: 目标角度
        """
        if channel not in self.servo_config:
            raise ValueError(f"通道 {channel} 未配置舵机")
            
        config = self.servo_config[channel]
        # 限制角度范围
        angle = max(0, min(config['max_angle'], angle))
        
        # 线性映射角度到脉冲宽度
        pulse_range = config['max_pulse'] - config['min_pulse']
        pulse = config['min_pulse'] + (angle / config['max_angle']) * pulse_range
        
        # 设置PWM脉冲
        self.pwm.setServoPulse(channel, pulse)
        config['current_angle'] = angle
        return angle, pulse
    
    def get_angle(self, channel):
        """
        获取指定通道舵机的当前角度
        
        :param channel: 舵机通道
        :return: 当前角度（浮点数），若通道未配置则返回 None
        """
        if channel not in self.servo_config:
            print(f"警告：通道 {channel} 未配置舵机，无法获取角度")
            return None
        return self.servo_config[channel]['current_angle']
    
    def set_angle_with_speed(self, channel, target_angle, angular_speed=30, fixed_delay=0.005, min_step=1):
        """
        设置指定通道舵机角度，基于固定延时的速度控制
        
        :param channel: 舵机通道
        :param target_angle: 目标角度
        :param angular_speed: 角速度 (度/秒)
        :param fixed_delay: 固定延时时间(秒)，默认0.01秒
        :param min_step: 最小步长，确保步长不小于此值
        """
        if channel not in self.servo_config:
            raise ValueError(f"通道 {channel} 未配置舵机")
            
        config = self.servo_config[channel]
        
        # 限制角度范围
        target_angle = max(0, min(config['max_angle'], target_angle))
        
        # 当前角度
        current_angle = config['current_angle']
        
        # 如果目标角度与当前角度相同，直接返回
        if abs(target_angle - current_angle) < 0.1:
            return
            
        # 计算所需的步长
        direction = 1 if target_angle > current_angle else -1
        step_size = max(min_step, angular_speed * fixed_delay)
        
        # 计算步数
        steps = int(abs(target_angle - current_angle) / step_size) + 1
        
        # 逐步移动到目标角度
        for _ in range(steps):
            current_angle += direction * step_size
            
            # 确保不超过目标角度
            if (direction > 0 and current_angle > target_angle) or (direction < 0 and current_angle < target_angle):
                current_angle = target_angle
                
            self.set_angle(channel, current_angle)
            time.sleep(fixed_delay)
def demo_180_degree_servo():
    """180度舵机控制示例"""
    controller = ServoController(50)
    
    # 添加舵机配置 (通道0, 180度舵机)
    controller.add_servo(0, min_pulse=250, max_pulse=1250, max_angle=270)
    
    try:
        print("180度舵机控制示例")
        while True:
            # 从0度到180度循环 - 慢速
            print("慢速转动: 0° -> 180°")
            controller.set_angle_with_speed(0, 180, angular_speed=1000)
            
            # 从180度到0度循环 - 快速
            print("快速转动: 180° -> 0°")
            # controller.set_angle_with_speed(0, 90, angular_speed=10000)
            controller.set_angle_with_speed(0, 0, angular_speed=10000)
            print("-"*30)
    except KeyboardInterrupt:
        print("程序已停止")

if __name__ == "__main__":
    demo_180_degree_servo()    