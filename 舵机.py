import RPi.GPIO as gpio # type: ignore
import time

# 定义电机引脚
spin1 = 17
spin2 = 27

# 初始化GPIO模式
gpio.setmode(gpio.BCM)
# 设置引脚为输出模式
gpio.setup(spin1, gpio.OUT)
gpio.setup(spin2, gpio.OUT)

# 控制电机正转
gpio.output(spin1, gpio.HIGH)
gpio.output(spin2, gpio.LOW)
time.sleep(2)

# 控制电机反转
gpio.output(spin1, gpio.LOW)
gpio.output(spin2, gpio.HIGH)
time.sleep(2)

# 清理GPIO资源
gpio.cleanup()