import RPi.GPIO as GPIO # type: ignore
import time

# 定义电机引脚
spin1 = 17
spin2 = 27

# 初始化GPIO模式
GPIO.setmode(GPIO.BCM)
# 设置引脚为输出模式
GPIO.setup(spin1, GPIO.OUT)
GPIO.setup(spin2, GPIO.OUT)

# 控制电机正转
GPIO.output(spin1, GPIO.HIGH)
GPIO.output(spin2, GPIO.LOW)
time.sleep(2)

# 控制电机反转
GPIO.output(spin1, GPIO.LOW)
GPIO.output(spin2, GPIO.HIGH)
time.sleep(2)

# 清理GPIO资源
GPIO.cleanup()