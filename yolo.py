from ultralytics import YOLO #type: ignore
from picamera2 import Picamera2 #type: ignore
import cv2 #type: ignore
import sys #type: ignore

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



# # 矩形框
# cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
# # 添加标签和准确度
# label = f"{cls_name} {conf:.2f}"
# cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
