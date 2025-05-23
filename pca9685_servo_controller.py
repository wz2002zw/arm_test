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
        
    def set_angle(self, channel, angle, angular_speed=None, fixed_delay=0.01):
        """
        设置指定通道舵机角度，支持速度限制
        
        :param channel: 舵机通道
        :param angle: 目标角度
        :param angular_speed: 角速度 (度/秒)，如果为None则立即设置角度
        :param fixed_delay: 固定延时时间(秒)，默认0.01秒
        """
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
    """测试舵机归零功能及同步到达性能"""
    controller = ServoController(50)
    
    # 添加四个舵机配置
    controller.add_servo(0, min_pulse=250, max_pulse=1282, max_angle=270)
    controller.add_servo(1, min_pulse=250, max_pulse=1383, max_angle=270)
    controller.add_servo(2, min_pulse=250, max_pulse=1383, max_angle=270)
    controller.add_servo(3, min_pulse=250, max_pulse=1383, max_angle=270)
    controller.reset_all_servos_to_zero()
    # 等待舵机归零完成
    time.sleep(2)
    try:
        # 定义测试目标（角度与速度比例为1:1）
        test_targets = [
            {'channel': 0, 'angle': 120, 'speed': 8},   # 30°@1°/s (30秒)
            {'channel': 1, 'angle': 120, 'speed': 8},   # 60°@2°/s (30秒)
            {'channel': 2, 'angle': 120, 'speed': 8},   # 90°@3°/s (30秒)
            {'channel': 3, 'angle': 120, 'speed': 8}   # 120°@4°/s (30秒)
        ]
        
        print("==== 舵机归零与同步到达测试 ====")
        
        # 1. 执行舵机归零（逐个设置到0°，每个间隔1秒）
        print("\n[测试步骤1/3] 执行舵机归零...")
        controller.reset_all_servos_to_zero()
        
        # 验证归零结果
        print("[归零验证] 检查各舵机角度:")
        for target in test_targets:
            angle = controller.get_angle(target['channel'])
            print(f"舵机 {target['channel']}: 当前角度 {angle:.1f}° {'✅' if angle < 5 else '❌'}")
        time.sleep(1)
        
        # 2. 同步控制测试（理论上30秒同时到达）
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
        print("==== 测试完成 ====")
        
    except KeyboardInterrupt:
        print("程序已停止")
    except Exception as e:
        print(f"测试错误: {e}")
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
if __name__ == "__main__":
    # demo_180_degree_servo()
    # demo_demo_180_degree_servo_test()
    # test_servo_reset_and_sync()
    demo_servo_pulse_control()