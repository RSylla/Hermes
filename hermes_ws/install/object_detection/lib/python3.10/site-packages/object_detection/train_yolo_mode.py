from ultralytics import YOLO

# Load a pre-trained YOLO model for transfer learning (e.g., yolov8n)
model = YOLO('src/object_detection/object_detection/yolov11n.pt') 

# Train the model on your custom dataset
model.train(data='src/object_detection/object_detection/yolov11n_with_cones/data.yaml', epochs=100, imgsz=640)
