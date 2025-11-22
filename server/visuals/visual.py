import argparse
import sys
from typing import Tuple, Optional

from ultralytics import YOLO
import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except Exception:
    rs = None


class Camera:
    """Unified camera wrapper supporting OpenCV webcams and Intel RealSense depth cameras.

    get_frame() returns (color_frame, depth_frame) where depth_frame may be None if not available.
    """

    def __init__(self, camera_type: str = "opencv", camera_index: int = 0, align_depth: bool = True):
        self.camera_type = camera_type.lower()
        self.camera_index = camera_index
        self.align_depth = align_depth
        self.pipeline = None
        self.align = None
        self.depth_scale = 1.0

        if self.camera_type == "opencv":
            self.cap = cv2.VideoCapture(camera_index)
            if not self.cap.isOpened():
                raise RuntimeError(
                    f"Failed to open OpenCV camera index {camera_index}")
        elif self.camera_type == "realsense":
            if rs is None:
                raise RuntimeError(
                    "pyrealsense2 not installed. Install with: pip install pyrealsense2")
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            profile = self.pipeline.start(config)
            # depth scale
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()  # meters per unit
            if self.align_depth:
                self.align = rs.align(rs.stream.color)
        else:
            raise ValueError("camera_type must be 'opencv' or 'realsense'")

    def get_frame(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        if self.camera_type == "opencv":
            ret, frame = self.cap.read()
            if not ret:
                raise ValueError("Could not read frame from OpenCV camera")
            return frame, None
        else:  # realsense
            frames = self.pipeline.wait_for_frames()
            if self.align:
                frames = self.align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                raise ValueError("Incomplete RealSense frames")
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            return color_image, depth_image

    def release(self):
        if self.camera_type == "opencv":
            self.cap.release()
        elif self.camera_type == "realsense" and self.pipeline is not None:
            self.pipeline.stop()


class ObjectDetector:
    def __init__(self, model_path='yolov8n.pt', classes_path=None, conf_threshold=0.5):
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.classes = []
        if classes_path:
            with open(classes_path, 'r') as f:
                self.classes = [line.strip() for line in f.readlines()]

    def detect_objects(self, frame_or_tuple, depth_image: Optional[np.ndarray] = None, depth_scale: float = 1.0):
        # Support (color, depth) tuple or just color frame
        if isinstance(frame_or_tuple, tuple):
            frame = frame_or_tuple[0]
            if depth_image is None and frame_or_tuple[1] is not None:
                depth_image = frame_or_tuple[1]
        else:
            frame = frame_or_tuple

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
                depth_m = None
                if depth_image is not None:
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    if 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]:
                        raw_depth = depth_image[cy, cx]
                        depth_m = raw_depth * depth_scale  # meters
                detections.append({
                    'bbox': (x1, y1, x2, y2),
                    'confidence': conf,
                    'class_id': cls_id,
                    'class_name': class_name,
                    'distance_m': depth_m
                })
        return detections


def parse_args():
    p = argparse.ArgumentParser(
        description="Unified vision inference with OpenCV or Intel RealSense")
    p.add_argument('--camera', choices=['opencv', 'realsense'],
                   default='opencv', help='Camera backend to use')
    p.add_argument('--index', type=int, default=0,
                   help='OpenCV camera index (ignored for realsense)')
    p.add_argument('--model', default='../models/MSD_Final.pt',
                   help='YOLO model path')
    p.add_argument('--classes', default='classes.txt',
                   help='Path to classes file')
    p.add_argument('--conf', type=float, default=0.5,
                   help='Confidence threshold')
    p.add_argument('--no-align', action='store_true',
                   help='Disable RealSense depth/color alignment')
    return p.parse_args()

"""
Example usage:
python3 visual.py --camera opencv --index 0 --model ../models/MSD_Final.pt --classes classes.txt
python3 visual.py --camera realsense --model ../models/MSD_Final.pt --classes classes.txt
"""
def main():
    args = parse_args()
    try:
        camera = Camera(camera_type=args.camera,
                        camera_index=args.index, align_depth=not args.no_align)
    except Exception as e:
        print(f"Camera initialization failed: {e}", file=sys.stderr)
        sys.exit(1)

    # detector = ObjectDetector(
    #     model_path=args.model, classes_path=args.classes, conf_threshold=args.conf)

    print(f"Using camera: {args.camera}")
    if args.camera == 'realsense':
        print("Depth scale (m per unit):", camera.depth_scale)

    try:
        while True:
            color_frame, depth_frame = camera.get_frame()
            # detections = detector.detect_objects(
                # (color_frame, depth_frame), depth_image=depth_frame, depth_scale=camera.depth_scale)

            # Draw detections on color frame
            # for det in detections:
            #     x1, y1, x2, y2 = det['bbox']
            #     conf = det['confidence']
            #     cls_id = det['class_id']
            #     cls_name = det['class_name'] if det['class_name'] else 'Unknown'
            #     distance_m = det['distance_m']
            #     cv2.rectangle(color_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            #     label = f'ID:{cls_id} {cls_name} {conf:.2f}'
            #     if distance_m is not None:
            #         label += f' {distance_m:.2f}m'
            #     cv2.putText(color_frame, label, (x1, max(0, y1 - 10)),
            #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow('Object Detection', color_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
