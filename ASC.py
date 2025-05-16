from picamera2 import Picamera2 #type: ignore
from ultralytics import YOLO #type: ignore
from collections import deque
# import adafruit_vl53l0x #type: ignore
import RPi.GPIO as GPIO #type: ignore
import cv2 #type: ignore
import board #type: ignore
import sys
import pyttsx3 
import time
import smbus #type: ignore
import threading
from collections import deque
import Adafruit_VL53L0X as VL53L0X #type: ignore


#====================VL53L0X传感器========================

# 初始化VL53L0X传感器
vl53 = VL53L0X.VL53L0X()
vl53.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)  # 启动高精度测距模式

D = 0  # 全局距离变量
distance_queue = deque(maxlen=5)  # 创建固定长度的滤波队列（移动平均滤波）

def read_sensor():
    # 持续读取传感器数据并进行滤波的线程函数
    global D
    while True:
        # 获取原始距离数据
        raw_distance = vl53.get_distance()
        
        # 数据有效性校验（丢弃0和负值）
        if raw_distance > 0:
            distance_queue.append(raw_distance)
            
            # 计算移动平均值（队列满时才开始更新）
            if len(distance_queue) == distance_queue.maxlen:
                D = sum(distance_queue) / len(distance_queue)
                
        # 控制采样频率 20Hz
        time.sleep(0.05)

# 创建并启动传感器读取线程
sensor_thread = threading.Thread(target=read_sensor, daemon=True)
sensor_thread.start()

try:
    while True:
        # 主线程保持运行
        time.sleep(1)
except KeyboardInterrupt:
    vl53.stop_ranging()  # 停止传感器
    print("VL53L0X quipped")


#====================MPU-6050传感器=======================

# 初始化
MPU6050_ADDR = 0x68  # I2C地址
ACCEL_SCALE = 16384.0  # ±2g量程时的灵敏度(LSB/g)
AXIS = 1              # 0-X轴, 1-Y轴, 2-Z轴
THRESHOLD = 0.75      # 触发阈值（g）
BUS = smbus.SMBus(1)  # 使用I2C通道1

def setup_mpu():
    # 初始化MPU6050设置
    BUS.write_byte_data(MPU6050_ADDR, 0x6B, 0)   # 解除休眠状态
    BUS.write_byte_data(MPU6050_ADDR, 0x1C, 0x00) # 设置加速度计±2g量程

def read_accel():
    # 读取三轴加速度数据并转换为g值
    accel_data = BUS.read_i2c_block_data(MPU6050_ADDR, 0x3B, 6)
    
    # 处理16位有符号数转换
    accel = []
    for i in range(3):
        raw = (accel_data[i*2] << 8) | accel_data[i*2+1]
        if raw > 0x7FFF:  # 处理负数
            raw -= 0x10000
        accel.append(raw / ACCEL_SCALE)
    return accel

def interrupt_handler():
    # 中断触发后的处理函数
    # print(f"[警报] 检测到异常加速度！时间：{time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(1)
    # 此处可扩展其他操作：触发GPIO、保存数据、发送通知等

if __name__ == "__main__":
    setup_mpu()
    print("MPU-6050 started")
    
    try:
        while True:
            x, y, z = read_accel()
            vertical = [x, y, z][AXIS]  # 选择监测的轴
            
            # 判断是否超过阈值 (绝对值)
            if abs(vertical) > THRESHOLD:
                interrupt_handler()
                time.sleep(0.5)  # 防抖延迟
            
            time.sleep(0.01)  # 循环间隔
            
    except KeyboardInterrupt:
        print("MPU-6050 quipped")




#====================GPIO电机控制=========================

# 电机初始化、定义函数
# 5v引脚为2号物理引脚
IN1=17
IN2=18
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
in1=GPIO.PWM(IN1, 50)
in2=GPIO.PWM(IN2, 50)
in1.start(0)
in2.start(0)
speed=0
# t为转动时间，speed为占空比
def left(t, speed):
    in1.start(speed)
    in2.start(0)
    time.sleep(t)
    GPIO.cleanup()

def right(t, speed):
    in1.start(0)
    in2.start(speed)
    time.sleep(t)
    GPIO.cleanup()

def stop():
    in1.start(0)
    in2.start(0)
    GPIO.cleanup()


#====================杂项函数=============================
# 映射函数
def map(input, imin, imax, omin, omax):
    return (input - imin) * (omax - omin) / (imax - imin) + omin

# 绝对值函数
def abs(x):
    if x > 0:
        return x
    else:
        return -x

# 声音播放初始化
engine = pyttsx3.init()
engine.setProperty('rate', 200)

#====================YOLO模型============================

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

#=========================避障主代码============================

pi=3.14
# omega=map(speed,0,100,0,10)*pi
# 由坐标位置判断障碍物位置并决定转动时间
# 由于电机反装,函数中旋转方向与实际方向相反(详见139-146等)
if abs(1640-x1)/abs(1640-x2)>1: #障碍物在左
    if x2<820:  #向右
        delta_X=1-(1.6*D-(x2/3280)*3.2*D)
        speed=100
        omega=map(speed,0,100,0,10)*pi
        t=delta_X/omega
        engine.say("前方有障碍物,请右转")
        engine.runAndWait()
        left(t, speed)

    else:  #向左
        delta_X=(abs(1640-x1)*3.2*D)/3280
        speed=100
        omega=map(speed,0,100,0,10)*pi
        t=delta_X/omega
        engine.say("前方有障碍物,请左转")
        engine.runAndWait()
        right(t, speed)

if abs(1640-x1)/abs(1640-x2)<1: #障碍物在右
    # if x1>2460:  #向左
    #     可以不做避让
    #     delta_X=(((3280-x1)/3280)*3.2*D)/2
    #     time.sleep(0)

    # else:  #向左
    if x1<2460 & x2<2460:  #向左
        delta_X=abs(1640-x2)/3280*3.2*D
        speed=100
        omega=map(speed,0,100,0,10)*pi
        t=delta_X/omega
        right(t, speed)




    
    
    
    
    
    
    #专项避障
    
    
    
    
    
    
    
    
    
    if cls_name == "stairs":
        engine.say("前方有楼梯，请注意安全")
        engine.runAndWait()
        stop()


# 盲道
# 盲道的tag 
# 如果center_x偏离中心区域，则发出提示向左或向右



# 语音







