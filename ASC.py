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


#====================杂项函数=============================
# 映射函数
def map(input, imin, imax, omin, omax):
    return (input-imin)*(omax-omin)/(imax-imin)+omin

# 绝对值函数
def abs(x):
    if x>0:
        return x
    else:
        return -x

# 声音播放初始化
engine=pyttsx3.init()
engine.setProperty('rate', 200)


#====================GPIO电机控制=========================

# 电机初始化、定义函数
# 5v引脚为2号物理引脚
IN1=17 #GPIO0
IN2=18 #GPIO1
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


#====================VL53L0X传感器========================


# VCC -> 3.3V, GND -> GND, SDA -> SDA.1, SCL -> SCL.1

# 初始化VL53L0X传感器
vl53=VL53L0X.VL53L0X()
vl53.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)  # 启动高精度测距模式

D=0 # 全局距离变量
distance_queue=deque(maxlen=5)  # 创建固定长度的滤波队列（移动平均滤波）

def read_sensor():
    # 持续读取传感器数据并进行滤波的线程函数
    global D
    while True:
        raw=vl53.get_distance() # 获取原始距离数据
        # 数据有效性校验（丢弃0和负值）
        if raw>0:
            distance_queue.append(raw)
            # 计算移动平均值（队列满时才开始更新）
            if len(distance_queue)==distance_queue.maxlen:
                D=sum(distance_queue)/len(distance_queue)
                
        # 控制采样频率 20Hz
        time.sleep(0.05)

# 创建并启动传感器读取线程
sensor_thread=threading.Thread(target=read_sensor, daemon=True)
sensor_thread.start()

try:
    while True:
        # 主线程保持运行
        time.sleep(1)
except KeyboardInterrupt:
    vl53.stop_ranging()  # 停止传感器
    print("VL53L0X quipped")


#====================MPU-6050传感器=======================

# VCC -> 5V, GND -> GND, SDA -> GPIO2, SCL -> GPIO3

MPU6050_ADDR=0x68 # I2C地址
ACCEL_SCALE=16384.0 # ±2g量程时的灵敏度(LSB/g)
THRESHOLD=0.75 # 触发阈值（单位：g）
BUTTON_PIN = 23 # GPIO.4
BUS=smbus.SMBus(1) # 使用I2C通道1

# 初始化GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 启用内部上拉电阻

# 优化方案:在5分钟时提醒用户按下按钮以确认安全,若超时则播放警报
class AlertSystem:
    def __init__(self):
        self.alert_active=False  # 报警状态标志
        self.lock=threading.Lock()

    def wait_for_response(self):
        # 等待按钮响应的线程函数
        start_time = time.time()
        print("waiting for button...")
        
        while time.time()-start_time<60:  # 1分钟倒计时
            # 检测按钮是否按下(低)
            if GPIO.input(BUTTON_PIN)==GPIO.LOW:
                with self.lock:
                    self.alert_active=False
                print("has been answered")
                return
            time.sleep(0.1)  # 减少CPU占用
        
        # 超时未响应
        with self.lock:
            if self.alert_active:
                print("no answered")
                engine.say("紧急! 有人跌倒, 请帮忙拨打120")
                engine.runAndWait()
                # [TODO] 发送警报到云端
                self.alert_active=False

    def trigger_alert(self):
        # 触发警报主函数
        with self.lock:
            if not self.alert_active:
                self.alert_active=True
                # 启动监测线程
                t = threading.Thread(target=self.wait_for_response)
                t.daemon=True
                t.start()

alert_system=AlertSystem()

def setup_mpu():
    # 初始化MPU6050设置
    BUS.write_byte_data(MPU6050_ADDR, 0x6B, 0) # 解除休眠状态
    BUS.write_byte_data(MPU6050_ADDR, 0x1C, 0x00) # 设置加速度计±2g量程

def read_accel():
    # 读取三轴加速度数据并转换为g值
    accel_data=BUS.read_i2c_block_data(MPU6050_ADDR, 0x3B, 6)
    
    # 处理16位有符号数转换
    accel=[]
    for i in range(3):
        raw=(accel_data[i*2]<<8) | accel_data[i*2+1]
        if raw>0x7FFF:  # 处理负数
            raw-=0x10000
        accel.append(raw/ACCEL_SCALE)
    return accel

def interrupt_handler(triggered_axis, accel_value):
    # 中断触发后的处理函数
    axis_names=['X', 'Y', 'Z']
    print(f"[ERROR] {axis_names[triggered_axis]}轴加速度超标！当前值: {accel_value:.2f}g")
    alert_system.trigger_alert()

if __name__=="__main__":
    setup_mpu()
    print("MPU-6050 started")
    print(f"阈值设定：{THRESHOLD}g")
    print(f"MPU6050 address: 0x{MPU6050_ADDR:02X}")
    print(f"button pin: GPIO{BUTTON_PIN}")

    try:
        while True:
            x, y, z=read_accel()
            # 检查任意轴加速度是否超标
            if any(abs(a)>THRESHOLD for a in [x, y, z]):
                interrupt_handler()
                time.sleep(0.5)  # 防抖延迟
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("MPU-6050 quipped")
    

#====================YOLO模型============================

model=YOLO("yolo-v5.pt")

picam2=Picamera2()
picam2.preview_configuration.main.size=(3280, 2464)
picam2.preview_configuration.main.format="RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# 运行目标检测
image=picam2.capture_array()
results=model(image)

# 提取检测结果并绘制矩形框
for result in results:
    boxes=result.boxes # 获取边界框信息
    for box in boxes:
        # 坐标
        x1, y1, x2, y2=map(int, box.xyxy[0])
        # 标签和准确度
        cls_id=int(box.cls[0])
        conf=float(box.conf[0])
        cls_name=model.names[cls_id]

# 矩形框
cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
# 添加标签和准确度
label=f"{cls_name} {conf:.2f}"
cv2.putText(image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
# 'stop_sign','person','bicycle','bus','truck','car','motorbike','reflective_cone','ashcan','warning_column','spherical_roadblock','pole','dog','tricycle','fire_hydrant','stairs'

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
        match cls_name:
            case "stairs":
                engine.say("前方有楼梯，请注意安全")
                engine.runAndWait()
            case "person":
                engine.say("前方有行人，请小心避让")
                engine.runAndWait()
            case "reflective_cone" | "warning_column" | "spherical_roadblock":
                engine.say("前方有路障，请小心避让")
                engine.runAndWait()
            case "bicycle" | "bus" | "truck" | "car" | "motorbike" | "tricycle":
                engine.say("前方有车辆，请注意安全")
                engine.runAndWait()
            case "ashcan":
                engine.say("前方有垃圾桶，请小心避让")
                engine.runAndWait()
            case "stop_sign":
                engine.say("前方有告示牌，请注意安全")
                engine.runAndWait()
            case "pole":
                engine.say("前方有电线杆，请小心避让")
                engine.runAndWait()
            case "dog":
                engine.say("前方有动物，请注意安全")
                engine.runAndWait()
            case _:
                engine.say("前方有障碍物,请小心避让")
                engine.runAndWait()

        left(t, speed)

    else:  #向左
        delta_X=(abs(1640-x1)*3.2*D)/3280
        speed=100
        omega=map(speed,0,100,0,10)*pi
        t=delta_X/omega
        match cls_name:
            case "stairs":
                engine.say("前方有楼梯，请注意安全")
                engine.runAndWait()
            case "person":
                engine.say("前方有行人，请小心避让")
                engine.runAndWait()
            case "reflective_cone" | "warning_column" | "spherical_roadblock":
                engine.say("前方有路障，请小心避让")
                engine.runAndWait()
            case "bicycle" | "bus" | "truck" | "car" | "motorbike" | "tricycle":
                engine.say("前方有车辆，请注意安全")
                engine.runAndWait()
            case "ashcan":
                engine.say("前方有垃圾桶，请小心避让")
                engine.runAndWait()
            case "stop_sign":
                engine.say("前方有告示牌，请注意安全")
                engine.runAndWait()
            case "pole":
                engine.say("前方有电线杆，请小心避让")
                engine.runAndWait()
            case "dog":
                engine.say("前方有动物，请注意安全")
                engine.runAndWait()
            case _:
                engine.say("前方有障碍物,请小心避让")
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
        match cls_name:
            case "stairs":
                engine.say("前方有楼梯，请注意安全")
                engine.runAndWait()
            case "person":
                engine.say("前方有行人，请小心避让")
                engine.runAndWait()
            case "reflective_cone" | "warning_column" | "spherical_roadblock":
                engine.say("前方有路障，请小心避让")
                engine.runAndWait()
            case "bicycle" | "bus" | "truck" | "car" | "motorbike" | "tricycle":
                engine.say("前方有车辆，请注意安全")
                engine.runAndWait()
            case "ashcan":
                engine.say("前方有垃圾桶，请小心避让")
                engine.runAndWait()
            case "stop_sign":
                engine.say("前方有告示牌，请注意安全")
                engine.runAndWait()
            case "pole":
                engine.say("前方有电线杆，请小心避让")
                engine.runAndWait()
            case "dog":
                engine.say("前方有动物，请注意安全")
                engine.runAndWait()
            case _:
                engine.say("前方有障碍物,请小心避让")
                engine.runAndWait()
        right(t, speed)
