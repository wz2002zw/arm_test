#!/usr/bin/python3

import time
import math
import smbus
import os

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
    MAX_ANGULAR_SPEED = 280  # 添加最大速度限制常量
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
        
    def set_angle(self, channel, angle, angular_speed=None, fixed_delay=0.01):
        """
        设置指定通道舵机角度，支持速度限制
        
        :param channel: 舵机通道
        :param angle: 目标角度
        :param angular_speed: 角速度 (度/秒)，如果为None则立即设置角度
        :param fixed_delay: 固定延时时间(秒)，默认0.01秒
        """
        if angular_speed is not None:  # ✅ 应添加限制
         angular_speed = min(angular_speed, self.MAX_ANGULAR_SPEED)
        if channel not in self.servo_config:
            raise ValueError(f"通道 {channel} 未配置舵机")
            
        config = self.servo_config[channel]
        # 限制角度范围
        angle = max(0, min(config['max_angle'], angle))
        
        # 如果未指定角速度，则立即设置角度
        if angular_speed is None:
            # 线性映射角度到脉冲宽度
            pulse_range = config['max_pulse'] - config['min_pulse']
            pulse = config['min_pulse'] + (angle / config['max_angle']) * pulse_range
            
            # 设置PWM脉冲
            self.pwm.setServoPulse(channel, pulse)
            config['current_angle'] = angle
            return angle, pulse
        
        # 如果指定了角速度，则逐步移动到目标角度
        current_angle = config['current_angle']
        if abs(angle - current_angle) < 0.1:
            return
            
        # 计算步长
        direction = 1 if angle > current_angle else -1
        step_size = angular_speed * fixed_delay
        
        # 计算步数
        steps = int(abs(angle - current_angle) / step_size) + 1
        
        # 逐步移动到目标角度
        for _ in range(steps):
            current_angle += direction * step_size
            
            # 确保不超过目标角度
            if (direction > 0 and current_angle > angle) or (direction < 0 and current_angle < angle):
                current_angle = angle
                
            # 线性映射角度到脉冲宽度
            pulse_range = config['max_pulse'] - config['min_pulse']
            pulse = config['min_pulse'] + (current_angle / config['max_angle']) * pulse_range
            
            # 设置PWM脉冲
            self.pwm.setServoPulse(channel, pulse)
            config['current_angle'] = current_angle
            time.sleep(fixed_delay)
        
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
    def reset_all_servos_to_zero(self):
        """
        将所有已配置的舵机重置到零位（初始位置）
        """
        for channel in self.servo_config.keys():
            self.set_angle(channel, 0)
            time.sleep(1)
    def set_angle_with_speed(self, channel, target_angle, angular_speed=30, fixed_delay=0.005, min_step=1):
        """

        设置指定通道舵机角度，基于固定延时的速度控制
        
        :param channel: 舵机通道
        :param target_angle: 目标角度
        :param angular_speed: 角速度 (度/秒)
        :param fixed_delay: 固定延时时间(秒)，默认0.01秒
        :param min_step: 最小步长，确保步长不小于此值
        """
        angular_speed = min(angular_speed, self.MAX_ANGULAR_SPEED)  # 限制最大速度
        if angular_speed > self.MAX_ANGULAR_SPEED:
            print(f"警告: 速度 {angular_speed}°/秒 超过限制，已设置为 {self.MAX_ANGULAR_SPEED}°/秒")
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
    def synchronized_servo_control(self, servo_commands, fixed_delay=0.01, accel_time=0.5):
        """
        同步控制多个舵机，添加加速度控制功能
        :param servo_commands: 字典列表，每个字典包含:
            - channel: 舵机通道
            - target_angle: 目标角度
            - angular_speed: 角速度
        :param fixed_delay: 固定延时时间(秒)
        :param accel_time: 加速到最大速度的时间(秒)
        """
        
        processed_commands = []
        for cmd in servo_commands:
            # 复制命令并限制速度
            processed_cmd = cmd.copy()
            if 'angular_speed' in processed_cmd:
                processed_cmd['angular_speed'] = min(processed_cmd['angular_speed'], self.MAX_ANGULAR_SPEED)
                if cmd['angular_speed'] > self.MAX_ANGULAR_SPEED:
                    print(f"警告: 通道 {cmd['channel']} 速度 {cmd['angular_speed']}°/秒 超过限制，已设置为 {self.MAX_ANGULAR_SPEED}°/秒")
            processed_commands.append(processed_cmd)
        # 初始化所有舵机的状态
        servo_states = []
        for cmd in servo_commands:
            channel = cmd['channel']
            current_angle = self.get_angle(channel)
            target_angle = cmd['target_angle']
            max_speed = cmd.get('angular_speed', 30)
            
            servo_states.append({
                'channel': channel,
                'current_angle': current_angle,
                'target_angle': target_angle,
                'max_speed': max_speed,
                'current_speed': 0,  # 初始速度为0
                'direction': 1 if target_angle > current_angle else -1,
                'completed': False
            })
        
        # 计算加速度相关参数
        accel_steps = int(accel_time / fixed_delay)  # 加速步数
        speed_increment = max_speed / accel_steps if accel_steps > 0 else max_speed
        
        # 主循环，直到所有舵机到达目标位置
        step_count = 0
        while not all(state['completed'] for state in servo_states):
            start_time = time.time()
            
            # 更新当前速度（加速度控制）
            if step_count < accel_steps:
                for state in servo_states:
                    state['current_speed'] = min(state['max_speed'], state['current_speed'] + speed_increment)
            
            # 遍历所有舵机，执行一个时间片
            for state in servo_states:
                if state['completed']:
                    continue
                    
                # 计算步长
                # step_size = max(1, state['current_speed'] * fixed_delay)
                step_size = state['current_speed'] * fixed_delay
                
                # 计算下一个角度
                next_angle = state['current_angle'] + state['direction'] * step_size
                
                # 检查是否到达目标
                if (state['direction'] > 0 and next_angle >= state['target_angle']) or \
                (state['direction'] < 0 and next_angle <= state['target_angle']):
                    next_angle = state['target_angle']
                    state['completed'] = True
                    
                # 更新角度
                state['current_angle'] = next_angle
                self.set_angle(state['channel'], next_angle)
            
            step_count += 1
            
            # 计算本轮执行时间，调整延时
            elapsed = time.time() - start_time
            if elapsed < fixed_delay:
                time.sleep(fixed_delay - elapsed)
    def time_based_servo_control(self, servo_commands, update_freq=100):
        """
        基于时间点的多舵机同步控制
        
        :param servo_commands: 字典列表，每个字典包含:
            - channel: 舵机通道
            - target_angle: 目标角度
            - angular_speed: 角速度
        :param update_freq: 更新频率(Hz)
        """
        for cmd in servo_commands:
            if 'angular_speed' in cmd:
                cmd['angular_speed'] = min(cmd['angular_speed'], self.MAX_ANGULAR_SPEED)
        update_interval = 1.0 / update_freq  # 计算更新间隔
        
        # 初始化舵机状态
        servo_states = []
        for cmd in servo_commands:
            channel = cmd['channel']
            current_angle = self.get_angle(channel)
            target_angle = cmd['target_angle']
            angular_speed = cmd.get('angular_speed', 30)
            
            # 计算到达目标所需时间和总步数
            angle_diff = abs(target_angle - current_angle)
            total_time = angle_diff / angular_speed
            total_steps = max(1, int(total_time * update_freq))
            
            # 计算每步角度变化
            angle_step = (target_angle - current_angle) / total_steps if total_steps > 0 else 0
            
            servo_states.append({
                'channel': channel,
                'current_angle': current_angle,
                'target_angle': target_angle,
                'angle_step': angle_step,
                'steps_remaining': total_steps,
                'completed': total_steps == 0
            })
        
        # 主循环
        while not all(state['completed'] for state in servo_states):
            start_time = time.time()
            
            # 更新所有舵机
            for state in servo_states:
                if state['completed']:
                    continue
                    
                # 计算下一个角度
                next_angle = state['current_angle'] + state['angle_step']
                
                # 检查是否到达目标
                if state['steps_remaining'] <= 1:
                    next_angle = state['target_angle']
                    state['completed'] = True
                else:
                    state['steps_remaining'] -= 1
                    
                # 更新角度
                state['current_angle'] = next_angle
                self.set_angle(state['channel'], next_angle)
            
            # 等待下一个更新周期
            elapsed = time.time() - start_time
            if elapsed < update_interval:
                time.sleep(update_interval - elapsed)
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
            controller.set_angle_with_speed(0, 180, angular_speed=10)
            # 从180度到0度循环 - 快速
            print("快速转动: 180° -> 0°")
            # controller.set_angle_with_speed(0, 90, angular_speed=10000)
            controller.set_angle_with_speed(0, 0, angular_speed=10)
            print("-"*30)
    except KeyboardInterrupt:
        print("程序已停止")
def demo_demo_180_degree_servo_test():
    """测试基于当前位置偏移的舵机控制，带速度限制和电流保护"""
    controller = ServoController(50)
    
    # 添加多个舵机配置（根据实际舵机型号调整参数）
    controller.add_servo(0, min_pulse=250, max_pulse=1282, max_angle=270)
    controller.add_servo(1, min_pulse=250, max_pulse=1383, max_angle=270)
    controller.add_servo(2, min_pulse=250, max_pulse=1383, max_angle=270)
    controller.add_servo(3, min_pulse=250, max_pulse=1383, max_angle=270)
    controller.reset_all_servos_to_zero()
    
    try:
        # 全局速度限制（度/秒）- 降低此值以减小电流
        max_speed = 5  # 建议初始值为5-10，根据实际情况调整
        angle_offset = 90 # 偏移角度
        
        print(f"基于当前位置偏移{angle_offset}度的舵机控制测试（速度限制: {max_speed}度/秒）")
        
        # 初始化: 将所有舵机移动到初始位置
        print("\n初始化: 将所有舵机移动到初始位置")
        controller.synchronized_servo_control([
            {'channel': 0, 'target_angle': 30, 'angular_speed': max_speed},
            {'channel': 1, 'target_angle': 60, 'angular_speed': max_speed},
            {'channel': 2, 'target_angle': 120, 'angular_speed': max_speed},
            {'channel': 3, 'target_angle': 150, 'angular_speed': max_speed}
        ])
        time.sleep(1)
        
        # 示例1: 所有舵机在当前位置基础上顺时针偏移
        print(f"\n示例1: 所有舵机顺时针偏移{angle_offset}度")
        commands = []
        for channel in range(4):
            current_angle =controller.get_angle(channel)
            target_angle = min(current_angle + angle_offset, 180)  # 限制最大角度
            commands.append({
                'channel': channel,
                'target_angle': target_angle,
                'angular_speed': max_speed  # 使用限速值
            })
            print(f"通道 {channel}: 当前位置 {current_angle:.1f}°, 目标位置 {target_angle:.1f}°")
        
        controller.synchronized_servo_control(commands)
        time.sleep(1)
        
        # 示例2: 所有舵机在当前位置基础上逆时针偏移
        print(f"\n示例2: 所有舵机逆时针偏移{angle_offset}度")
        commands = []
        for channel in range(4):
            current_angle = controller.get_angle(channel)
            target_angle = max(current_angle - angle_offset, 0)  # 限制最小角度
            commands.append({
                'channel': channel,
                'target_angle': target_angle,
                'angular_speed': max_speed  # 使用限速值
            })
            print(f"通道 {channel}: 当前位置 {current_angle:.1f}°, 目标位置 {target_angle:.1f}°")
        
        controller.synchronized_servo_control(commands)
        time.sleep(1)
        
        # 示例3: 智能偏移 - 根据当前位置选择最短路径偏移
        print(f"\n示例3: 智能偏移 - 选择最短路径偏移{angle_offset}度")
        commands = []
        for channel in range(4):
            current_angle = controller.get_angle(channel)
            
            # 计算两个可能的目标角度: +angle_offset度和-angle_offset度
            option1 = current_angle + angle_offset
            option2 = current_angle - angle_offset
            
            # 检查两个选项是否在有效范围内
            valid_options = []
            if 0 <= option1 <= 180:
                valid_options.append(option1)
            if 0 <= option2 <= 180:
                valid_options.append(option2)
            
            # 选择与当前位置距离最近的有效目标角度
            if valid_options:
                target_angle = min(valid_options, key=lambda x: abs(x - current_angle))
                commands.append({
                    'channel': channel,
                    'target_angle': target_angle,
                    'angular_speed': max_speed  # 使用限速值
                })
                direction = "顺时针" if target_angle > current_angle else "逆时针"
                print(f"通道 {channel}: 当前位置 {current_angle:.1f}°, 选择{direction}偏移到 {target_angle:.1f}°")
            else:
                print(f"通道 {channel}: 当前位置 {current_angle:.1f}°, 无法安全偏移{angle_offset}度")
        
        if commands:
            controller.synchronized_servo_control(commands)
        
        print("\n测试完成!")
        
    except KeyboardInterrupt:
        print("程序已停止")
    except Exception as e:
        print(f"发生错误: {e}")
        # 错误处理：紧急停止所有舵机
        try:
            for channel in range(4):
                controller.set_angle(channel, controller.get_angle(channel))  # 保持当前位置
        except:
            pass
        print("程序已停止")
def test_servo_reset_and_sync():
    """测试舵机归零功能及同步到达性能，包含测试后缓慢归零"""
    controller = ServoController(50)
    
    # 添加六个舵机配置
    controller.add_servo(0, min_pulse=250, max_pulse=1306, max_angle=270)
    controller.add_servo(1, min_pulse=250, max_pulse=1283, max_angle=270)
    controller.add_servo(2, min_pulse=250, max_pulse=1283, max_angle=270)
    controller.add_servo(3, min_pulse=250, max_pulse=1283, max_angle=270)
    controller.add_servo(4, min_pulse=250, max_pulse=1283, max_angle=270)
    controller.add_servo(5, min_pulse=250, max_pulse=1306, max_angle=270)
    controller.reset_all_servos_to_zero()
    
    try:
        max_speed = 5  # 建议初始值为5-10，根据实际情况调整
        angle_offset = 90 # 偏移角度
        
        print(f"基于当前位置偏移{angle_offset}度的舵机控制测试（速度限制: {max_speed}度/秒）")
        
        # 初始化: 将所有舵机移动到初始位置
        print("\n初始化: 将所有舵机移动到初始位置")
        test_targets = [
            {'channel': 0, 'angle': 30, 'speed': 2},   # 30°@2°/s (15秒)
            {'channel': 1, 'angle': 30, 'speed': 2},  
            {'channel': 2, 'angle': 30, 'speed': 2},   
            {'channel': 3, 'angle': 30, 'speed': 2},
            {'channel': 4, 'angle': 30, 'speed': 2},
            {'channel': 5, 'angle': 30, 'speed': 2}   
        ]
        time.sleep(1)
        print("==== 舵机归零与同步到达测试 ====")
        
        # 1. 执行舵机归零（测试前归零）
        print("[归零验证] 检查各舵机角度:")
        for target in test_targets:
            angle = controller.get_angle(target['channel'])
            print(f"舵机 {target['channel']}: 当前角度 {angle:.1f}° {'✅' if angle < 5 else '❌'}")
        time.sleep(1)
        
        # 2. 同步控制测试（理论上15秒同时到达）
        print("\n[测试步骤2/3] 同步控制测试...")
        print("目标参数:")
        for target in test_targets:
            print(f"舵机 {target['channel']}: {target['angle']}° @ {target['speed']}°/s")
        
        # 发送同步控制命令
        test_commands = [
            {'channel': t['channel'], 'target_angle': t['angle'], 'angular_speed': t['speed']}
            for t in test_targets
        ]
        
        start_time = time.time()
        controller.synchronized_servo_control(test_commands)
        end_time = time.time()
        total_time = end_time - start_time
        
        # 3. 验证同步到达结果
        print(f"\n[测试步骤3/3] 同步性验证（总耗时: {total_time:.2f}秒）")
        is_sync = all(abs(total_time - (t['angle']/t['speed'])) < 1 for t in test_targets)
        
        for target in test_targets:
            actual_angle = controller.get_angle(target['channel'])
            target_time = target['angle'] / target['speed']
            print(f"舵机 {target['channel']}: "
                  f"目标角度 {target['angle']}° → 实际角度 {actual_angle:.1f}° "
                  f"(理论时间 {target_time:.2f}秒 → 实际耗时 {total_time:.2f}秒)")
        
        print(f"\n同步性结果: {'✅ 成功' if is_sync else '❌ 失败'}")
        
    except KeyboardInterrupt:
        print("程序已停止")
    except Exception as e:
        print(f"测试错误: {e}")
    finally:
        # 测试结束后执行缓慢归零（核心修改点）
        print("\n[等待2秒后执行舵机缓慢归零...]")
        time.sleep(2)  # 添加2秒延时
        print("\n[测试后处理] 执行舵机缓慢归零...")
        for channel in sorted(controller.servo_config.keys()):
            print(f"舵机 {channel} 开始缓慢归零...")
            controller.set_angle_with_speed(
                channel, 
                target_angle=0, 
                angular_speed=3,  # 3°/秒更低速度，确保安全
                fixed_delay=0.02   # 增加延时，进一步降低电流
            )
            time.sleep(1)  # 每个舵机归零后等待1秒，避免同时运动
        
        print("所有舵机已安全归零，测试完成")
def demo_servo_pulse_control():
    """脉冲宽度控制舵机测试 - 允许输入脉冲值(us)直接控制舵机"""
    controller = ServoController(50)
    
    # 添加舵机配置（通道0，设置较大的脉冲范围以支持270度舵机）
    controller.add_servo(5, min_pulse=250, max_pulse=2500, max_angle=270)
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
def test_servo_speed_incremental_sync():
    """测试舵机在不同速度下的同步性能，支持速度递增测试并记录数据"""
    controller = ServoController(50)

    # 添加六个舵机配置
    controller.add_servo(0, min_pulse=250, max_pulse=1306, max_angle=270)
    controller.add_servo(1, min_pulse=250, max_pulse=1283, max_angle=270)
    controller.add_servo(2, min_pulse=250, max_pulse=1283, max_angle=270)
    controller.add_servo(3, min_pulse=250, max_pulse=1283, max_angle=270)
    controller.add_servo(4, min_pulse=250, max_pulse=1283, max_angle=270)
    controller.add_servo(5, min_pulse=250, max_pulse=1306, max_angle=270)
    controller.reset_all_servos_to_zero()

    # 创建data文件夹（关键修改点）
    data_dir = "data"
    os.makedirs(data_dir, exist_ok=True)
    
    # 创建汇总数据记录文件
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    summary_file = os.path.join(data_dir, f"servo_test_summary_{timestamp}.txt")
    
    try:
        # 写入汇总文件头部
        with open(summary_file, "w") as f:
            f.write("速度(°/秒),角度(°),是否达标,总耗时(秒)\n")
        
        # 定义测试角度
        test_angles = [45, 90, 135, 180]
        print(f"测试角度: {test_angles}°")
        print(f"汇总数据将记录到: {summary_file}")
        
        current_speed = 280  # 初始速度为1°/秒
        
        while True:
            print(f"\n==== 开始速度 {current_speed}°/秒 的同步测试 ====")
            
            # 1. 测试前归零
            print("[测试前处理] 执行舵机归零...")
            max_speed = 5  # 建议初始值为5-10，根据实际情况调整
            angle_offset = 90 # 偏移角度
            
            print(f"基于当前位置偏移{angle_offset}度的舵机控制测试（速度限制: {max_speed}度/秒）")
            
            # 初始化: 将所有舵机移动到初始位置
            print("\n初始化: 将所有舵机移动到初始位置")
            # for channel in controller.servo_config.keys():
            #     controller.set_angle_with_speed(channel, 0, angular_speed=3, fixed_delay=0.02)
            time.sleep(1)
            print("[归零验证] 检查各舵机角度:")
            for channel in controller.servo_config.keys():
                angle = controller.get_angle(channel)
                print(f"舵机 {channel}: 当前角度 {angle:.1f}° {'✅' if angle < 5 else '❌'}")
            
            # 2. 执行角度测试（每个角度测试后都归零）
            test_results = []
            for angle in test_angles:
                print(f"\n--- 测试角度: {angle}° ---")
                
                # 创建当前测试的独立数据文件（关键修改点）
                test_timestamp = time.strftime("%Y%m%d_%H%M%S")
                test_file = os.path.join(data_dir, f"{angle}°_{current_speed}°ps_{test_timestamp}.txt")
                
                # 写入独立文件头部
                with open(test_file, "w") as f:
                    f.write("舵机通道,理论时间(秒),实际耗时(秒),时间差(秒),目标角度(°),实际角度(°),角度差(°),是否达标\n")
                
                # 配置测试命令
                test_commands = [
                    {'channel': channel, 'target_angle': angle, 'angular_speed': current_speed}
                    for channel in controller.servo_config.keys()
                ]
                
                # 记录开始时间
                start_time = time.time()
                
                # 获取初始角度
                initial_angles = {channel: controller.get_angle(channel) for channel in controller.servo_config.keys()}
                
                # 执行同步控制
                controller.synchronized_servo_control(test_commands)
                
                # 等待所有舵机到达目标位置（添加超时机制）
                timeout = angle / current_speed + 5  # 额外增加5秒超时
                servo_arrival_times = {channel: None for channel in controller.servo_config.keys()}
                
                while time.time() - start_time < timeout:
                    all_arrived = True
                    
                    for channel in controller.servo_config.keys():
                        # 跳过已记录到达时间的舵机
                        if servo_arrival_times[channel] is not None:
                            continue
                            
                        current_angle = controller.get_angle(channel)
                        
                        # 改进到达检测逻辑
                        if (abs(current_angle - angle) <= 1.0 and 
                            abs(current_angle - initial_angles[channel]) > 1.0):
                            servo_arrival_times[channel] = time.time() - start_time
                        else:
                            all_arrived = False
                    
                    if all_arrived:
                        break
                    
                    time.sleep(0.1)
                
                # 计算总耗时（取最后一个到达的舵机时间）
                total_time = max(t for t in servo_arrival_times.values() if t is not None)
                
                # 验证每个舵机的到达时间
                is_valid = True
                for channel in controller.servo_config.keys():
                    target_time = angle / current_speed
                    actual_time = servo_arrival_times[channel]
                    
                    # 处理未到达的情况
                    if actual_time is None:
                        actual_time = timeout
                        time_diff = abs(actual_time - target_time)
                        print(f"警告: 舵机 {channel} 在超时时间内未到达目标位置")
                    else:
                        time_diff = abs(actual_time - target_time)
                    
                    # 使用动态阈值（理论时间的2%或1秒，取较大值）
                    threshold = max(1.0, target_time * 0.02)
                    
                    actual_angle = controller.get_angle(channel)
                    angle_diff = abs(actual_angle - angle)
                    passed = time_diff <= threshold
                    
                    print(f"舵机 {channel}: "
                          f"理论时间 {target_time:.2f}秒 → 实际耗时 {actual_time:.2f}秒 "
                          f"(时间差: {time_diff:.2f}秒, 阈值: {threshold:.2f}秒), "
                          f"目标角度 {angle}° → 实际角度 {actual_angle:.1f}° "
                          f"(角度差: {angle_diff:.1f}°) {'✅' if passed else '❌'}")
                    
                    # 写入独立测试文件
                    with open(test_file, "a") as f:
                        f.write(f"{channel},{target_time:.4f},{actual_time:.4f},"
                                f"{time_diff:.4f},{angle},{actual_angle:.1f},{angle_diff:.1f},{1 if passed else 0}\n")
                    
                    if time_diff > threshold:
                        is_valid = False
                
                test_results.append((angle, current_speed, is_valid, total_time))
                
                # 写入汇总文件
                with open(summary_file, "a") as f:
                    f.write(f"{current_speed},{angle},{1 if is_valid else 0},{total_time:.4f}\n")
                
                print(f"测试数据已保存至: {test_file}")
                
                # 单个角度测试完成后立即归零
                print(f"\n[角度{angle}测试完成] 执行舵机归零...")
                zero_commands = [
                    {'channel': channel, 'target_angle': 0, 'angular_speed': 5}
                    for channel in controller.servo_config.keys()
                ]
                controller.synchronized_servo_control(zero_commands)
                time.sleep(1)  # 等待归零完成
                
                # 验证归零结果
                print("[归零验证] 检查各舵机角度:")
                for channel in controller.servo_config.keys():
                    angle = controller.get_angle(channel)
                    print(f"舵机 {channel}: 当前角度 {angle:.1f}° {'✅' if angle < 5 else '❌'}")
                time.sleep(0.5)  # 短暂停顿，便于观察
            
            # 4. 检查测试结果
            failed_test = None
            for angle, speed, is_valid, total_time in test_results:
                if not is_valid:
                    failed_test = (angle, speed)
                    break
            
            if failed_test:
                angle, speed = failed_test
                print(f"\n==== 测试失败 ====")
                print(f"速度 {speed}°/秒 时，角度 {angle}° 的测试时间差超过阈值")
                print("测试终止")
                break
            
            # 5. 速度递增，继续下一轮测试
            current_speed += 2
            print(f"\n速度递增至 {current_speed}°/秒，准备下一轮测试...")
            time.sleep(2)

    except KeyboardInterrupt:
        print("\n程序已中断")
    except Exception as e:
        print(f"测试错误: {e}")
    finally:
        # 最终归零
        print("\n[最终处理] 执行舵机归零...")
        for channel in controller.servo_config.keys():
            controller.set_angle_with_speed(channel, 0, angular_speed=3, fixed_delay=0.02)
        print(f"所有舵机已安全归零，汇总数据已保存至 {summary_file}")
if __name__ == "__main__":
    # demo_180_degree_servo()
    # demo_demo_180_degree_servo_test()
    test_servo_speed_incremental_sync()
    # demo_servo_pulse_control()