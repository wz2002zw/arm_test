#!/usr/bin/python3 sudo reboot Q

from PCA9685 import PCA9685
import time
import math
import smbus

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

