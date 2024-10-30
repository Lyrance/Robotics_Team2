from ultralytics import YOLO
import torch.multiprocessing as mp
mp.set_start_method('spawn', force=True)
model = YOLO('D:/Personal documents/Study/University/Msc/Robotic_System_Design/Detection/yolov8/ultralytics/models/v8/yolov8-seg.yaml')
model = YOLO('D:/Personal documents/Study/University/Msc/Robotic_System_Design/Detection/yolov8/yolov8n-seg.pt')

if __name__ == '__main__':
    model.train(
        data='D:/Personal documents/Study/University/Msc/Robotic_System_Design/Detection/yolov8/data.yaml',
        epochs=50,
        imgsz=640,
        device='cuda:0'
    )
