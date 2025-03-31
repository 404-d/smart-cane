import time
import VL53L0X #type: ignore
import RPi.GPIO as GPIO #type: ignore

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

#当
i=0
while i<=1:
    distance = tof.get_distance()
    if distance > 0:
        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(12, GPIO.HIGH)
    time.sleep(timing/1000000.00)
    i+=1
# IMX219分辨率 3280*2464
# 设定障碍物的坐标为 (x, y)
# 坐标映射: 
x=1, y=1
map(x, 1, 1, 0, 3280)
