import time
import VL53L0X
import RPi.GPIO as GPIO #type: ignore
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
 
# Create one object per VL53L0X passing the address to give to
# each.
tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
 
tof.open()

# 开始测量
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

timing = tof.get_timing()
if timing < 20000:
    timing = 20000
print("Timing %d ms" % (timing/1000))

for count in range(1, 101):
    distance = tof.get_distance()
    if distance > 0:
        print("%d mm, %d cm, %d" % (distance, (distance/10), count))

    time.sleep(timing/1000000.00)

tof.stop_ranging()
tof.close()