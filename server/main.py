import cv2
import os
import numpy as np
import time
import sys
import select

from visarm import VisArm
import variables as v
from camera.camera import Camera
from camera.image_processing import ImageProcessing


def user_input_available():
    return select.select([sys.stdin], [], [], 0)[0] != []


def main():

    visarm = VisArm()

    if not visarm.connect():
        return

    # SET TO SURVEY!
    visarm.set_survey()
    time.sleep(2)

    print("[SYSTEM] Initializing Camera…")
    # Use absolute path for camera_params.json
    script_dir = os.path.dirname(os.path.abspath(__file__))
    params_path = os.path.join(script_dir, "camera", "camera_params.json")
    cam = Camera(params_path)
    if cam:
        cam.save_board_outline_to_json()

    print("\n[SYSTEM] Camera calibration complete.")
    print("[SYSTEM] You may now place objects inside the board region.\n")

    # Load saved outline from camera_params.json (computed already by Camera)
    outline_px = cam.outline

    # Create image processing instance
    ip = ImageProcessing(cam.K, cam.dist, outline_px)

    # Wait for user to place objects and press Enter
    print("[SYSTEM] Press Enter to start sorting...")
    while not user_input_available():
        time.sleep(0.1)
    sys.stdin.readline()  # Clear the input buffer

    while True:
        print("\n[SORTING] Moving to Survey Position...")
        visarm.set_survey()
        time.sleep(2)

        # Raw frame
        raw = cam.get_frame_raw()
        if raw is None:
            continue

        # Ensure we always have a processed image to show (avoid NameError
        # if no color branches produce a `processed` variable)
        processed = raw.copy()

        detected_objects = []

        # Iterate over all colors in BINS
        for color_name, bin_pos in v.BINS.items():
            # Get the HSV range from variables
            if not hasattr(v, color_name):
                continue
            hsv_range = getattr(v, color_name)

            # Process image
            processed, centers, mask = ip.process(raw, hsv_range)

            for center in centers:
                detected_objects.append({
                    "color": color_name,
                    "center": center,
                    "bin_pos": bin_pos
                })
                # Draw on processed image for visualization
                cx, cy = center
                cv2.circle(processed, (cx, cy), 8, (0, 0, 255), -1)
                cv2.putText(processed, color_name, (cx + 10, cy - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Ensure windows are created (helps some backends)
        try:
            cv2.namedWindow("Raw feed", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Processed View", cv2.WINDOW_NORMAL)
            cv2.imshow("Raw feed", raw)
            cv2.imshow("Processed View", processed)
            cv2.waitKey(1)
        except cv2.error:
            pass

        print(f"[SORTING] Detected {len(detected_objects)} objects.")

        if not detected_objects:
            print("[SORTING] No objects found. Waiting...")
            time.sleep(1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        # Sort Objects
        for obj in detected_objects:
            cx, cy = obj['center']
            color = obj['color']
            # Refresh GUI so frames remain visible during long operations
            raw = cam.get_frame_raw()
            processed, centers, _ = ip.process(raw, getattr(v, color))
            for center in centers:
                detected_objects.append({
                    "color": color_name,
                    "center": center,
                    "bin_pos": bin_pos
                })
                # Draw on processed image for visualization
                cx, cy = center
                cv2.circle(processed, (cx, cy), 8, (0, 0, 255), -1)
                cv2.putText(processed, color_name, (cx + 10, cy - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Ensure windows are created (helps some backends)
            try:
                cv2.namedWindow("Raw feed", cv2.WINDOW_NORMAL)
                cv2.namedWindow("Processed View", cv2.WINDOW_NORMAL)
                cv2.imshow("Raw feed", raw)
                cv2.imshow("Processed View", processed)
                cv2.waitKey(1)
            except cv2.error:
                pass

            print(f"[SORTING] Processing {color} object at {cx}, {cy}")

            # Calculate Robot Coordinates
            P_robot_mm = cam.pixel_to_robot(cx, cy)
            P_robot_cm = P_robot_mm / 10.0
            P_robot_cm[2] = 5.0  # Pickup height

            target_pos = np.array(
                [P_robot_cm[0], P_robot_cm[1], P_robot_cm[2]])

            if target_pos[1] < 0:
                offset = [-6, 0, 0, 0, 0]
            else:
                offset = [0, 0, 0, 0, 0]
            print(f"[SORTING] Moving to object {color}...")
            # Move to object (Open)
            visarm.move_to_position(target_pos, steps=2, add_offset=offset)
            time.sleep(1)

            # Close Gripper
            print("[SORTING] Grabbing object...")
            visarm.close_gripper()
            time.sleep(0.5)

            # Move to POS_2 (Closed)
            print("[SORTING] Moving to Bin Position)...")
            bin_pos = v.BINS[color]
            visarm.move_to_position(bin_pos, steps=2)
            time.sleep(1)

            # Open Gripper to release above bin
            print("[SORTING] Releasing object into bin...")
            visarm.open_gripper()
            time.sleep(0.5)

            # Return to Survey loop (will happen at start of loop or next iteration)
            print("[SORTING] Returning to Survey...")
            visarm.set_survey()
            time.sleep(1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exiting...")
            break

    cam.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
