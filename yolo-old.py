import cv2 #type: ignore
from picamera2 import Picamera2 #type: ignore
from ultralytics import YOLO #type: ignore

# 初始化树莓派摄像头
picam2 = Picamera2()
picam2.preview_configuration.main.size = (3280, 2464)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# 加载YOLOv5模型
model = YOLO("yolov5-16.pt")

while True:
    # 截取一帧图像
    frame = picam2.capture_array()
    
    # 识别并处存数据
    results = model(frame)
    
    # Output the visual detection data, we will draw this on our camera preview window
    annotated_frame = results[0].plot()
    
    # Get inference time
    inference_time = results[0].speed['inference']
    fps = 1000 / inference_time  # Convert to milliseconds
    text = f'FPS: {fps:.1f}'

    # 定义标签、位置
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size = cv2.getTextSize(text, font, 1, 2)[0]
    text_x = annotated_frame.shape[1] - text_size[0] - 10  # 10 pixels from the right
    text_y = text_size[1] + 10  # 10 pixels from the top

    # Draw the text on the annotated frame
    cv2.putText(annotated_frame, text, (text_x, text_y), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

    # Display the resulting frame
    cv2.imshow("Camera", annotated_frame)

    # Exit the program if q is pressed
    # if cv2.waitKey(1) == ord("q"):
    #     break
    
# Close all windows
# cv2.destroyAllWindows()