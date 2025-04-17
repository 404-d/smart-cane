from ultralytics import YOLO #type: ignore
from picamera2 import Picamera2 #type: ignore
import RPi.GPIO as GPIO #type: ignore
import cv2 #type: ignore
import sys #type: ignore
import pyttsx3 
import time

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
center_x = int((x1 + x2) / 2)
center_y = int((y1 + y2) / 2)


# 避障部分 ,此部分暂未完成
# 由坐标位置判断障碍物位置并决定转动时间
if center_x>0 and center_y>0:
    if center_x < 1640: 
        engine.say("前方左侧有障碍物")
        engine.runAndWait()
        right(0.5, 50)
# -->
        time.sleep(2)
    if center_x > 1640:
        engine.say("前方右侧有障碍物")
        engine.runAndWait()
        left(0.5, 50)
# -->
