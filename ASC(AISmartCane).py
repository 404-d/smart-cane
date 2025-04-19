from picamera2 import Picamera2 #type: ignore
from ultralytics import YOLO #type: ignore
import RPi.GPIO as GPIO #type: ignore
import VL53L0X #type: ignore
import cv2 #type: ignore
import sys
import pyttsx3 
import time

# # 映射函数
# def map():
#     return 0

# 绝对值函数
def abs(x):
    if x > 0:
        return x
    else:
        return -x


# 电机初始化、定义函数
# 5v引脚为2号物理引脚
IN1=17, IN2=18
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
in1=GPIO.PWM(IN1, 50)
in2=GPIO.PWM(IN2, 50)
in1.start(0)
in2.start(0)

def left(ts, speed):
    in1.start(speed)
    in2.start(0)
    time.sleep(ts)
    GPIO.cleanup()

def right(ts, speed):
    in1.start(speed)
    in2.start(0)
    time.sleep(ts)
    GPIO.cleanup()

def stop():
    in1.start(0)
    in2.start(0)
    GPIO.cleanup()


# 声音播放初始化
engine = pyttsx3.init()
engine.setProperty('rate', 200)


# YOLO模型
model = YOLO("yolo-v5.pt")

picam2 = Picamera2()
picam2.preview_configuration.main.size = (3280, 2464)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()


# 运行目标检测
image = picam2.capture_array()
results = model(image)

# 提取检测结果并绘制矩形框
for result in results:
    boxes = result.boxes # 获取边界框信息
    for box in boxes:
        # 坐标
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        # 标签和准确度
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        cls_name = model.names[cls_id]

# 矩形框
cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
# 添加标签和准确度
label = f"{cls_name} {conf:.2f}"
cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# 计算中心点坐标
center_x = int((x1 + x2) / 2 / 3280)
# center_y = int((y1 + y2) / 2)


# 避障部分 ,此部分暂未完成
# 由坐标位置判断障碍物位置并决定转动时间
if center_x>0 :
    X=3.2
    deltaX = center_x-x1 + X/2
    n=5 # 转速 
    if center_x > 0.5 : 
        engine.say("前方右侧有障碍物")
        engine.runAndWait()
        left(0.5, 50)
        t=3.2(abs(1845-x1)+256.25)/(3280*2*3.14*0.035*n)
        time.sleep(t)
        stop()

    if center_x < 0.5:
        engine.say("前方左侧有障碍物")
        engine.runAndWait()
        right(0.5, 50)
        t=3.2(abs(1332.5-x2)+256.25)/(3280*2*3.14*0.035*n)
        time.sleep(t)
        
    if label == "stairs":
        engine.say("前方有楼梯，请注意安全")
        engine.runAndWait()
        stop()


# 盲道


# 加速度传感器


# VL53L0X


# 映射函数
def map (vin, iMin, iMax, oMin, oMax):
    return (vin-iMin)*(oMax-oMin)/(iMax-iMin)+oMin

# 传感器1的关闭引脚
tof_shut = 20

GPIO.setwarnings(False)
 
# 初始化传感器1的关闭引脚
GPIO.setmode(GPIO.BCM)
GPIO.setup(tof_shut, GPIO.OUT) 
 
# 设置关闭引脚为低电平,使传感器1关闭
GPIO.output(tof_shut, GPIO.LOW)

# 中断3s,等待传感器完成初始化
time.sleep(3) 

# 创建地址为0x29的传感器对象
tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
 
tof.open()

# 开始测量
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.HIGH_SPEED)
tof.start_ranging(VL53L0X.Vl53l0xDeviceMode.CONTINUOUS_RANGING)

timing = tof.get_timing()
if timing < 20000:
    timing = 20000


# 语音