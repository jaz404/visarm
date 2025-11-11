from ultralytics import YOLO
import cv2
import numpy as np


class Camera:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise ValueError("Could not read frame from camera")
        return frame

    def release(self):
        self.cap.release()


class ObjectDetector:
    def __init__(self, model_path='yolov8n.pt', classes_path=None, conf_threshold=0.5):
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.classes = []
        if classes_path:
            with open(classes_path, 'r') as f:
                self.classes = [line.strip() for line in f.readlines()]

    def detect_objects(self, frame):
        results = self.model(frame)[0]
        detections = []
        for result in results.boxes:
            if result.conf[0] >= self.conf_threshold:
                x1, y1, x2, y2 = map(int, result.xyxy[0])
                conf = float(result.conf[0])
                cls_id = int(result.cls[0])
                class_name = None
                if self.classes and 0 <= cls_id < len(self.classes):
                    class_name = self.classes[cls_id]
                detections.append({
                    'bbox': (x1, y1, x2, y2),
                    'confidence': conf,
                    'class_id': cls_id,
                    'class_name': class_name
                })
        return detections


def main():
    camera = Camera()
    detector = ObjectDetector(
        model_path='../models/MSD_Final.pt', classes_path='classes.txt')

    try:
        while True:
            frame = camera.get_frame()
            detections = detector.detect_objects(frame)

            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                conf = det['confidence']
                cls_id = det['class_id']
                cls_name = det['class_name'] if det['class_name'] else 'Unknown'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'ID: {cls_id} Name: {cls_name} Conf: {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow('Object Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
