from test_sservo import *
controller = ServoController()
# 添加舵机配置 (通道0, 180度舵机)
controller.add_servo(0, min_pulse=250, max_pulse=1250, max_angle=270)
angle=controller.get_angle(0)
print(angle)
