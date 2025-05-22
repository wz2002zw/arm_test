#!/usr/bin/python

import time
import math
import smbus

# ============================================================================
# Raspi PCA9685 16-Channel PWM Servo Driver
# ============================================================================

class PCA9685:
    """
    控制 PCA9685 16通道 PWM 驱动模块的类。
    
    支持设置 PWM 频率、占空比以及控制舵机角度等功能。
    """

    # 寄存器地址定义
    __SUBADR1            = 0x02
    __SUBADR2            = 0x03
    __SUBADR3            = 0x04
    __MODE1              = 0x00
    __PRESCALE           = 0xFE
    __LED0_ON_L          = 0x06
    __LED0_ON_H          = 0x07
    __LED0_OFF_L         = 0x08
    __LED0_OFF_H         = 0x09
    __ALLLED_ON_L        = 0xFA
    __ALLLED_ON_H        = 0xFB
    __ALLLED_OFF_L       = 0xFC
    __ALLLED_OFF_H       = 0xFD

    def __init__(self, address=0x40, debug=False):
        """
        初始化 PCA9685 驱动。

        :param address: I2C 设备地址，默认为 0x40
        :param debug: 是否启用调试输出
        """
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        if self.debug:
            print("Reseting PCA9685")
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        """
        向指定寄存器写入一个 8 位值。

        :param reg: 寄存器地址
        :param value: 要写入的值
        """
        self.bus.write_byte_data(self.address, reg, value)
        if self.debug:
            print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))

    def read(self, reg):
        """
        从 I2C 设备读取一个无符号字节。

        :param reg: 寄存器地址
        :return: 读取到的值
        """
        result = self.bus.read_byte_data(self.address, reg)
        if self.debug:
            print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
        return result

    def setPWMFreq(self, freq):
        """
        设置 PWM 输出频率。

        :param freq: 目标频率（Hz）
        """
        prescaleval = 50000000.0    # 晶振频率为 50MHz
        prescaleval /= 4096.0       # 分辨率为 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        if self.debug:
            print("Setting PWM frequency to %d Hz" % freq)
            print("Estimated pre-scale: %d" % prescaleval)
        prescale = math.floor(prescaleval + 0.5)
        if self.debug:
            print("Final pre-scale: %d" % prescale)

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10  # 进入睡眠模式以修改预分频器
        self.write(self.__MODE1, newmode)  # 写入新配置
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        """
        设置单个 PWM 通道的 ON 和 OFF 值。

        :param channel: 通道号（0~15）
        :param on: 开始时间点
        :param off: 结束时间点
        """
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)
        if self.debug:
            print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel, on, off))

    def setServoPulse(self, channel, pulse):
        """
        设置伺服电机脉冲宽度。

        :param channel: 通道号（0~15）
        :param pulse: 脉冲宽度（单位：微秒）
        """
        pulse = pulse * 4096 / 20000  # 对应 50Hz 的周期（20000 微秒）
        self.setPWM(channel, 0, int(pulse))

    def set_servo_angle(self, channel, angle):
        """
        设置伺服电机的角度。

        :param channel: 通道号（0~15）
        :param angle: 角度（0~270 度）
        """
        pulse = self.angle_to_pulse(angle)
        self.setServoPulse(channel, pulse)

    def angle_to_pulse(self, angle):
        """
        将角度转换为对应的脉冲宽度。

        :param angle: 角度（0~270 度）
        :return: 脉冲宽度（单位：微秒）
        """
        angle = max(0, min(270, angle))  # 限制在 0~270 度
        return 500 + (angle / 270.0) * 2000  # 映射到 500~2500 微秒


if __name__ == '__main__':
    pwm = PCA9685(0x40, debug=False)
    pwm.setPWMFreq(50)  # 设置 PWM 频率为 50Hz
    while True:
        # 从 500 到 2500 微秒逐步增加脉冲宽度
        for i in range(500, 2500, 10):
            pwm.setServoPulse(0, i)
            time.sleep(0.02)

        # 从 2500 到 500 微秒逐步减少脉冲宽度
        for i in range(2500, 500, -10):
            pwm.setServoPulse(0, i)
            time.sleep(0.02)