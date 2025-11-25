import pyrealsense2 as rs
import numpy as np
import json
import cv2

from camera_model import compute_camera_pose, pixel_to_robot


class Camera:
    def __init__(self, param_file="camera_params.json",
                 resolution=(640, 480), fps=30):
        
        self.param_file = param_file 

        with open(param_file) as f:
            params = json.load(f)

        self.params = params 
        self.K = np.array(params["intrinsics"]["K"], dtype=np.float32)

        radial = params["intrinsics"]["radial"]
        tangential = params["intrinsics"]["tangential"]
        self.dist = np.array(
            [radial[0], radial[1], tangential[0], tangential[1], 0.0],
            dtype=np.float32
        )

        self.pattern_size = (9, 5)
        self.square = 23.0

        self.world_points = np.array(
            [
                [col * self.square, row * self.square]
                for row in range(self.pattern_size[1])
                for col in range(self.pattern_size[0])
            ],
            dtype=np.float32
        )

        self.T_O_from_checker = np.array(
            params["checker_to_robot"], dtype=np.float32
        )

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(
            rs.stream.color,
            resolution[0], resolution[1],
            rs.format.bgr8, fps
        )
        self.pipeline.start(config)

        def get_frame_for_pose():
            return self.get_frame_raw()
        

        R, t, pose, corners, undist = compute_camera_pose(
            get_raw_image_fn=get_frame_for_pose,
            K=self.K,
            dist_coeffs=self.dist,
            world_points=self.world_points,
            pattern_size=self.pattern_size
        )

        self.R = R
        self.t = t
        self.pose = pose
        self.corners = corners
        self.outline = self._compute_full_board_outline(self.corners)
        # self._save_board_outline(self.outline)

    def get_frame_raw(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

    def get_undistorted_frame(self):
        raw = self.get_frame_raw()
        if raw is None:
            return None
        return cv2.undistort(raw, self.K, self.dist)
    
    def get_frame(self):
        return self.get_frame_raw()

    def pixel_to_robot(self, u, v):
        return pixel_to_robot(u, v, self.K, self.R, self.t, self.T_O_from_checker)
    

    def _compute_full_board_outline(self, corners):
        w, h = self.pattern_size
        tl = corners[0]
        tr = corners[w - 1]
        bl = corners[(h - 1) * w]
        br = corners[(h * w) - 1]

        dx = (tr - tl) / (w - 1)
        dy = (bl - tl) / (h - 1)

        tl_o = tl - dx - dy
        tr_o = tr + dx - dy
        br_o = br + dx + dy
        bl_o = bl - dx + dy

        return np.array([tl_o, tr_o, br_o, bl_o], dtype=float).tolist()

    def save_board_outline_to_json(self):
        self.params["board_outline_px"] = self.outline
        with open(self.param_file, "w") as f:
            json.dump(self.params, f, indent=2)

    def shutdown(self):
        self.pipeline.stop()

def click_handler(event, x, y, flags, param):
    cam, frame_ref = param
    if event == cv2.EVENT_LBUTTONDOWN:
        P = cam.pixel_to_robot(x, y)
        print("Pixel:", (x, y), "→ Robot:", P)
        cv2.circle(frame_ref[0], (x, y), 6, (0, 0, 255), -1)

def main():
    cam = Camera("camera_params.json")
    frame_ref = [None]

    cv2.namedWindow("RGB")
    cv2.setMouseCallback("RGB", click_handler, (cam, frame_ref))

    while True:
        frame = cam.get_undistorted_frame()
        frame_ref[0] = frame.copy()

        cv2.imshow("RGB", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cam.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
