
import RPi.GPIO as GPIO
import time

# 设置GPIO模式
GPIO.setmode(GPIO.BCM)

# 设置GPIO引脚
servo_pin = 17
GPIO.setup(servo_pin, GPIO.OUT)

# 创建PWM实例
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz

# 设置初始角度
angle = 0

# 开始PWM
pwm.start(0)

# 循环控制舵机
for i in range(180):  # 旋转180度
    angle = i
    dutyCycle = angle / 18.3  # 计算占空比
    pwm.ChangeDutyCycle(dutyCycle)  # 更改占空比
    time.sleep(0.02)  # 延时

# 停止PWM
pwm.stop()

# 清理GPIO资源
GPIO.cleanup()