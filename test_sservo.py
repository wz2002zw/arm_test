#!/usr/bin/python

import time
import math
import smbus

# PCA9685寄存器地址（改为单下划线前缀）
_SUBADR1            = 0x02
_SUBADR2            = 0x03
_SUBADR3            = 0x04
_MODE1              = 0x00
_PRESCALE           = 0xFE
_LED0_ON_L          = 0x06
_LED0_ON_H          = 0x07
_LED0_OFF_L         = 0x08
_LED0_OFF_H         = 0x09
_ALLLED_ON_L        = 0xFA
_ALLLED_ON_H        = 0xFB
_ALLLED_OFF_L       = 0xFC
_ALLLED_OFF_H       = 0xFD

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
            print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))

    def read(self, reg):
        """从I2C设备读取一个无符号字节"""
        result = self.bus.read_byte_data(self.address, reg)
        if self.debug:
            print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
        return result

    def setPWMFreq(self, freq):
        """设置PWM输出频率"""
        prescaleval = 50000000.0    # 晶振频率为50MHz
        prescaleval /= 4096.0       # 分辨率为12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        if self.debug:
            print("Setting PWM frequency to %d Hz" % freq)
            print("Estimated pre-scale: %d" % prescaleval)
        prescale = math.floor(prescaleval + 0.5)
        if self.debug:
            print("Final pre-scale: %d" % prescale)

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
            print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel, on, off))

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
        print(f"通道 {channel} 设置角度: {angle}°, 脉冲宽度: {pulse:.2f}us")
        # 关键修复：更新当前角度
        config['current_angle'] = angle
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
    
    def set_angle_with_speed(self, channel, target_angle, speed=100):   
        """
        设置指定通道舵机角度，带速度控制
        
        :param channel: 舵机通道
        :param target_angle: 目标角度
        :param speed: 速度百分比 (0-100)，值越大速度越快
        """
        min_delay=0.0001
        max_delay=0.01
        step_size = 2  # 每步角度变化量
        if channel not in self.servo_config:
            raise ValueError(f"通道 {channel} 未配置舵机")
            
        config = self.servo_config[channel]
        
        # 限制角度范围
        target_angle = max(0, min(config['max_angle'], target_angle))
        
        # 当前角度
        current_angle = config['current_angle']
        
        # 如果目标角度与当前角度相同，直接返回
        if abs(target_angle - current_angle) < 1:
            return
            
        # 计算步数和延时
        
        direction = 1 if target_angle > current_angle else -1
        
        # 根据速度计算延时（速度越大，延时越小）
        # 基础延时为0.02秒，速度100%时延时为0.005秒
        delay_time = max_delay - (0.015 * (speed / 100.0))
        delay_time = max(min_delay, delay_time)  # 确保最小延时
        print(delay_time)
        # 逐步移动到目标角度
        for angle in range(int(current_angle), int(target_angle) + direction, direction * step_size):
            self.set_angle(channel, angle)
            time.sleep(delay_time)
            
        # 确保最终到达目标角度
        self.set_angle(channel, target_angle)

def demo_180_degree_servo():
    """180度舵机控制示例"""
    controller = ServoController(50)
    
    # 添加舵机配置 (通道0, 180度舵机)
    controller.add_servo(0, min_pulse=250, max_pulse=1250, max_angle=270)
    
    try:
        print("180度舵机控制示例")
        while True:
            # 从0度到180度循环
            for angle in range(0, 180, 5):
                print(f"forward:{angle}")
                speed=angle
                controller.set_angle_with_speed(0, angle,1)
                # time.sleep(1)
            print("="*10)
            # 从180度到0度循环
            for angle in range(180-5, -1, -5):
                print("inverse")
                controller.set_angle_with_speed(0, angle,1)
                # time.sleep(1)
            print("-"*10)
    except KeyboardInterrupt:
        print("程序已停止")

# def demo_270_degree_servo():
#     """270度舵机控制示例"""
#     controller = ServoController()
    
#     # 添加舵机配置 (通道1, 270度舵机)
#     controller.add_servo(1, min_pulse=500, max_pulse=2500, max_angle=270)
    
#     try:
#         print("270度舵机控制示例")
#         while True:
#             # 从0度到270度循环
#             for angle in range(0, 271, 15):
#                 controller.set_angle(1, angle)
#                 time.sleep(0.1)
                
#             # 从270度到0度循环
#             for angle in range(270, -1, -15):
#                 controller.set_angle(1, angle)
#                 time.sleep(0.1)
#     except KeyboardInterrupt:
#         print("程序已停止")

# def demo_360_degree_servo():
#     """360度连续旋转舵机控制示例"""
#     controller = ServoController()
    
#     # 添加舵机配置 (通道2, 360度连续旋转舵机)
#     controller.add_servo(2, min_pulse=500, max_pulse=2500, max_angle=100)  # 使用百分比控制速度
    
#     try:
#         print("360度连续旋转舵机控制示例")
#         while True:
#             print("正向最大速度")
#             controller.set_angle(2, 0)  # 0% 对应最大正向速度
#             time.sleep(2)
            
#             print("慢速正向旋转")
#             controller.set_angle(2, 40)  # 40% 对应慢速正向旋转
#             time.sleep(2)
            
#             print("停止")
#             controller.set_angle(2, 50)  # 50% 对应停止
#             time.sleep(2)
            
#             print("慢速反向旋转")
#             controller.set_angle(2, 60)  # 60% 对应慢速反向旋转
#             time.sleep(2)
            
#             print("反向最大速度")
#             controller.set_angle(2, 100)  # 100% 对应最大反向速度
#             time.sleep(2)
            
#             print("停止")
#             controller.set_angle(2, 50)  # 50% 对应停止
#             time.sleep(2)
#     except KeyboardInterrupt:
#         print("程序已停止")
def demo_servo_pulse_control():
    """脉冲宽度控制舵机测试 - 允许输入脉冲值(us)直接控制舵机"""
    controller = ServoController(50)
    
    # 添加舵机配置（通道0，设置较大的脉冲范围以支持270度舵机）
    controller.add_servo(0, min_pulse=250, max_pulse=2500, max_angle=270)
    print("脉冲宽度控制舵机测试已启动")
    print("支持的脉冲范围: 500-2500us (建议从1000开始测试)")
    print("输入 'q' 或 'quit' 退出测试")
    
    try:
        while True:
            # 获取用户输入的脉冲值
            pulse_input = input("\n请输入脉冲宽度(us)或输入'q'退出: ")
            
            # 退出条件
            if pulse_input.lower() in ['q', 'quit', 'exit']:
                break
                
            # 验证输入是否为数字
            if not pulse_input.isdigit():
                print("错误: 请输入有效的数字")
                continue
                
            pulse = int(pulse_input)
            
            # 限制脉冲范围
            if pulse < 500 or pulse > 2500:
                print(f"警告: 脉冲值应在500-2500us之间，当前输入{pulse}us已被限制")
                pulse = max(500, min(2500, pulse))
            
            # 发送脉冲控制舵机
            controller.pwm.setServoPulse(0, pulse)
            current_angle = controller.get_angle(0)
            print(f"已发送脉冲: {pulse}us, 当前角度: {current_angle:.1f}°")
            
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 测试完成后将舵机归位到中间位置
        print("测试结束，将舵机归位...")
        controller.pwm.setServoPulse(0, 1500)  # 中间位置脉冲
        time.sleep(1)
        print("舵机已归位")
if __name__ == "__main__":
    # 选择要运行的示例
    # demo_180_degree_servo()
    # demo_270_degree_servo()
    # demo_360_degree_servo() 
    demo_servo_pulse_control()   