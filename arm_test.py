#!/usr/bin/python3 sudo reboot Q

import time
import math
import smbus

class PCA9685:
    __MODE1              = 0x00
    __PRESCALE           = 0xFE
    __LED0_ON_L          = 0x06
    __LED0_ON_H          = 0x07
    __LED0_OFF_L         = 0x08
    __LED0_OFF_H         = 0x09

    def __init__(self, address=0x40, debug=False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def setPWMFreq(self, freq):
        prescaleval = 25000000.0
        prescaleval /= 4096.0
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = math.floor(prescaleval + 0.5)

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, int(prescale))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    def setServoPulse(self, channel, pulse):
        pulse = pulse * 4096 / 20000  # 20ms -> 50Hz
        self.setPWM(channel, 0, int(pulse))

    def angle_to_pulse(self, angle):
        angle = max(0, min(270, angle))  # 限制在 0~270 度
        return 500 + (angle / 270.0) * 2000

    def set_servo_angle(self, channel, angle):
        pulse = self.angle_to_pulse(angle)
        self.setServoPulse(channel, pulse)
def close_gripper(pwm):
    pwm.set_servo_angle(5, 135)  # 通道5 = 舵机6，设置为125°
    print("夹爪已闭合")
def open_gripper(pwm):
    pwm.set_servo_angle(5, 90)  # 通道5 = 舵机6，设置为125°
    print("夹爪已open")
# 慢速移动单个舵机
def slow_move_servo(pwm, channel, start_angle, end_angle, step=1, delay=0.1):
    if start_angle < end_angle:
        angle_range = range(start_angle, end_angle + 1, step)
    else:
        angle_range = range(start_angle, end_angle - 1, -step)

    for angle in angle_range:
        pwm.set_servo_angle(channel, angle)
        time.sleep(delay)    
# 慢速移动多个舵机
def slow_move_to_angles(pwm, current_angles, target_angles, step=1, delay=0.1):
    for i in range(len(target_angles)):
        slow_move_servo(pwm, i, current_angles[i], target_angles[i], step, delay)
    return target_angles  
def come_home_position(pwm, current_angles):
    target_angles = [135, 135, 150, 153, 135, 90]
    slow_move_to_angles(pwm, current_angles, target_angles)
    print("机械臂已移动到起始位置")
    return target_angles   
def move_aim_position_1(pwm, current_angles):
    target_angles = [135, 135, 80, 90, 135, 90]
    slow_move_to_angles(pwm, current_angles, target_angles)
    print("机械臂已移动到 aim 1位置")
    return target_angles
def move_aim_position_2(pwm, current_angles):
    target_angles = [200, 135, 80, 90, 135, 125]
    slow_move_to_angles(pwm, current_angles, target_angles)
    print("机械臂已移动到 aim 2位置")
    return target_angles
if __name__ == '__main__':
    pwm = PCA9685()
    pwm.setPWMFreq(50)
    current_angles = [135, 135, 150, 153, 135, 90]
    print("初始化完成，等待 2 秒")
    time.sleep(2)  # 延时2秒
    current_angles = move_aim_position_1(pwm, current_angles)
    print("识别到物块，开始夹取")
    # current_angles = come_home_position(pwm, current_angles)
    time.sleep(1)
    close_gripper(pwm)
    current_angles = move_aim_position_2(pwm, current_angles)
    print("到达目标位置，开始释放物块")
    open_gripper(pwm)
    print("任务完成开始返回")
    time.sleep(1)
    current_angles = come_home_position(pwm, current_angles)

