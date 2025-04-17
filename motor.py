import RPi.GPIO as GPIO #type: ignore
import time

# 5v引脚为2号物理引脚
IN1=17, IN2=18, speed=50
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
in1=GPIO.PWM(IN1, 50)
in2=GPIO.PWM(IN2, 50)
in1.start(0)
in2.start(0)

# IN1高电平，IN2低电平，电机正转
# IN1低电平，IN2高电平，电机反转
# 定义向左、向右、停止函数
def left(ts):
    in1.start(speed)
    in2.start(0)
    time.sleep(ts)
    GPIO.cleanup()

def right(ts):
    in1.start(speed)
    in2.start(0)
    time.sleep(ts)
    GPIO.cleanup()

def stop():
    in1.start(0)
    in2.start(0)
    GPIO.cleanup()