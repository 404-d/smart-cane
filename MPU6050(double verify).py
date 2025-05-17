import smbus #type: ignore
import time #type: ignore
import RPi.GPIO as GPIO #type: ignore
import threading #type: ignore
import os #type: ignore

# 硬件配置
MPU6050_ADDR = 0x68     # MPU605C I2C地址
ACCEL_SCALE = 16384.0    # ±2g量程灵敏度
THRESHOLD = 0.75         # 加速度阈值（单位：g）
BUTTON_PIN = 17          # 按钮GPIO引脚
BUS = smbus.SMBus(1)     # I2C通道1

# 初始化GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

class AlertSystem:
    def __init__(self):
        self.alert_lock = threading.Lock()
        self.current_stage = 0  # 0:空闲 1:第一等待期 2:第二等待期
        self.alert_timer = None

    def _alert_sequence(self, stage):
        """报警处理核心逻辑"""
        with self.alert_lock:
            if stage != self.current_stage:  # 状态校验
                return
            
            # 各阶段参数设置
            if stage == 1:
                timeout = 60
                alert_msg = "第一次警报！请按下按钮确认安全"
            elif stage == 2:
                timeout = 300  # 5分钟=300秒
                alert_msg = "第二次警报！请再次按下按钮确认"
            else:
                return

        # 播放语音提示
        os.system(f'espeak -v zh+f5 "{alert_msg}"')
        print(f"\n[阶段 {stage}] {alert_msg}")

        # 倒计时监测
        start_time = time.time()
        while time.time() - start_time < timeout:
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                with self.alert_lock:
                    if self.current_stage == stage:
                        print(f"阶段 {stage} 已确认")
                        self.current_stage = 0
                        return
                time.sleep(0.3)  # 按钮防抖
            time.sleep(0.1)

        # 超时处理
        with self.alert_lock:
            if self.current_stage == stage:
                if stage == 1:
                    print("第一阶段未响应，进入二次确认")
                    self.current_stage = 2
                    threading.Thread(target=self._alert_sequence, args=(2,)).start()
                else:
                    print("最终警报：未收到任何响应！")
                    os.system('espeak -v zh+f5 "需要紧急救援！"')
                    self.current_stage = 0

    def trigger_alert(self):
        """触发警报入口"""
        with self.alert_lock:
            if self.current_stage == 0:
                self.current_stage = 1
                threading.Thread(target=self._alert_sequence, args=(1,)).start()

alert_system = AlertSystem()

def setup_mpu():
    """初始化MPU6050"""
    BUS.write_byte_data(MPU6050_ADDR, 0x6B, 0)
    BUS.write_byte_data(MPU6050_ADDR, 0x1C, 0x00)

def read_accel():
    """读取加速度数据"""
    accel_data = BUS.read_i2c_block_data(MPU6050_ADDR, 0x3B, 6)
    return [(raw if (raw := (accel_data[i*2]<<8)|accel_data[i*2+1]) < 0x7FFF 
            else raw-0x10000)/ACCEL_SCALE for i in range(3)]

if __name__ == "__main__":
    setup_mpu()
    print("""双阶段跌倒监测系统已启动
    工作流程：
    1. 首次触发 → 1分钟确认期
    2. 未响应 → 播放语音并进入5分钟二次确认
    3. 最终未响应 → 播放紧急警报""")

    try:
        while True:
            x, y, z = read_accel()
            if any(abs(a) > THRESHOLD for a in [x, y, z]):
                alert_system.trigger_alert()
                time.sleep(0.5)  # 硬件防抖
            time.sleep(0.05)

    except KeyboardInterrupt:
        GPIO.cleanup()
        print("系统已安全关闭")