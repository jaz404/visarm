import cv2
import os
import numpy as np
import time
import threading
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

    visarm.set_survey()
    time.sleep(2)

    print("[SYSTEM] Initializing Camera…")
    # Use absolute path for camera_params.json
    script_dir = os.path.dirname(os.path.abspath(__file__))
    params_path = os.path.join(script_dir, "camera", "camera_params.json")
    cam = Camera(params_path)
    # Calibration / outline saving is intentionally disabled for normal runs.
    # If you need to recalibrate, uncomment the next lines.
    # if cam:
    #     cam.save_board_outline_to_json()
    #
    # print("\n[SYSTEM] Camera calibration complete.")
    # print("[SYSTEM] You may now place objects inside the board region.\n")

    # Load saved outline from camera_params.json (computed already by Camera)
    outline_px = cam.outline

    # Create image processing instance
    ip = ImageProcessing(cam.K, cam.dist, outline_px)

    # Shared frame store for communication between video thread and main loop
    frame_store = {
        'raw': None,
        'processed': None,
        'lock': threading.Lock(),
        'stop': False
    }

    def video_thread(cam, store):
        """Continuously grab frames from camera and show them in a separate
        window so the GUI stays responsive while main loop runs robot logic.
        The main thread can write `store['processed']` to show overlays.
        """
        try:
            cv2.namedWindow("Raw feed", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Processed View", cv2.WINDOW_NORMAL)
        except cv2.error:
            pass

        last_frame = None
        last_proc = None

        while not store['stop']:
            # Always attempt to grab a frame from the camera first so capture
            # happens regardless of any locking or display work.
            try:
                frame = cam.get_frame_raw()
            except Exception:
                frame = None

            if frame is None:
                # small sleep to avoid busy-looping when camera not ready
                time.sleep(0.005)
                continue

            # keep a local copy so display can work even if lock is busy
            last_frame = frame.copy()

            # Try to update shared raw frame without blocking for long
            acquired = store['lock'].acquire(blocking=False)
            if acquired:
                try:
                    store['raw'] = last_frame.copy()
                    # fetch processed overlay if available
                    last_proc = store.get('processed')
                finally:
                    store['lock'].release()
            else:
                # If lock busy, avoid blocking; we'll still display latest_local
                try:
                    # attempt a quick read of processed overlay
                    if store['lock'].acquire(blocking=False):
                        try:
                            last_proc = store.get('processed')
                        finally:
                            store['lock'].release()
                except Exception:
                    last_proc = None

            # Display windows using local copies
            try:
                if last_proc is not None:
                    cv2.imshow("Processed View", last_proc)
                else:
                    cv2.imshow("Processed View", last_frame)
                cv2.imshow("Raw feed", last_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    store['stop'] = True
                    break
            except cv2.error:
                # ignore display errors but keep capturing frames
                pass

        # Cleanup windows when exiting thread
        try:
            cv2.destroyWindow("Raw feed")
            cv2.destroyWindow("Processed View")
        except cv2.error:
            pass

    # Start video/display thread (non-daemon so we can join it on shutdown)
    vt = threading.Thread(target=video_thread, args=(cam, frame_store))
    vt.start()

    # Wait for user to place objects and press Enter
    print("[SYSTEM] Press Enter to start sorting...")
    while not user_input_available():
        time.sleep(0.1)
    sys.stdin.readline()  # Clear the input buffer

    try:
        while True:
            print("\n[SORTING] Moving to Survey Position...")
            visarm.set_survey()
            time.sleep(2)

            # Raw frame (captured while in survey pose) - read from video thread
            with frame_store['lock']:
                raw = frame_store['raw'].copy(
                ) if frame_store['raw'] is not None else None

            if raw is None:
                # wait shortly for the video thread to populate frames
                time.sleep(0.05)
                if frame_store['stop']:
                    break
                continue

            # Ensure we always have a processed image to show
            processed = raw.copy()

            detected_objects = []

            # Contour area thresholds to filter out large objects like a hand
            # and very small noise. Tune these if needed for your camera.
            MIN_CONTOUR_AREA = 5000
            MAX_CONTOUR_AREA = 7000

            # Iterate over all colors in BINS and detect contours in the mask.
            for color_name, bin_pos in v.BINS.items():
                # Get the HSV range from variables
                if not hasattr(v, color_name):
                    continue
                hsv_range = getattr(v, color_name)

                # Process image -> returns processed view, centers (not used), mask
                processed, _, mask = ip.process(raw, hsv_range)

                # Find contours on the mask and inspect/draw all of them so the
                # user can see what's being filtered. Accepted contours are
                # added to detected_objects; rejected contours are still drawn
                # (in yellow) so they are visible.
                contours, _ = cv2.findContours(
                    mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    M = cv2.moments(cnt)
                    cx = cy = None
                    if M.get('m00', 0) != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])

                    # Draw the contour itself (thin line)
                    cv2.drawContours(processed, [cnt], -1, (200, 200, 0), 2)

                    # If contour is within accepted area range, mark as accepted
                    print(area, cx, cy)
                    if area >= MIN_CONTOUR_AREA and area <= MAX_CONTOUR_AREA and cx is not None:
                        detected_objects.append({
                            "color": color_name,
                            "center": (cx, cy),
                            "bin_pos": bin_pos,
                            "area": area
                        })
                        # Draw accepted center (red) and label with color
                        cv2.circle(processed, (cx, cy), 8, (0, 0, 255), -1)
                        cv2.putText(processed, f"{color_name} A={int(area)}", (cx + 10, cy - 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    else:
                        # Draw rejected/ignored centroid (yellow) if available
                        if cx is not None:
                            cv2.circle(processed, (cx, cy),
                                       6, (0, 200, 200), -1)
                            cv2.putText(processed, f"rej A={int(area)}", (cx + 10, cy - 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 200), 1)

                # Defer updating the shared processed view until after we've
                # finished checking all colors so the display always shows the
                # latest annotated frame even when no objects are accepted.
            # Update processed view in the shared store for the video thread
            with frame_store['lock']:
                frame_store['processed'] = processed.copy()

            print(f"[SORTING] Detected {len(detected_objects)} objects.")

            if not detected_objects:
                print("[SORTING] No objects found. Waiting...")
                time.sleep(1)
                if frame_store['stop']:
                    break
                continue

            for obj in list(detected_objects):
                cx, cy = obj['center']
                color = obj['color']

                # Refresh frame from the video thread so we act on a fresh view
                with frame_store['lock']:
                    latest_raw = frame_store['raw'].copy(
                    ) if frame_store['raw'] is not None else None
                if latest_raw is not None:
                    processed, _, _ = ip.process(latest_raw, getattr(v, color))
                    # Update processed overlay for visualization
                    with frame_store['lock']:
                        frame_store['processed'] = processed.copy()
                else:
                    # if no fresh frame, skip this object iteration
                    continue

                print(f"[SORTING] Processing {color} object at {cx}, {cy}")

                # Calculate Robot Coordinates
                P_robot_mm = cam.pixel_to_robot(cx, cy)
                P_robot_cm = P_robot_mm / 10.0
                P_robot_cm[2] = 5.0  # Pickup height

                target_pos = np.array(
                    [P_robot_cm[0], P_robot_cm[1], P_robot_cm[2]])

                if target_pos[1] < 6.7:
                    offset = np.deg2rad([-6, 0, 0, 0, 0])
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

            detected_objects.clear()

            if frame_store['stop']:
                print("Exiting...")
                break
    except KeyboardInterrupt:
        print("Received Ctrl+C — shutting down gracefully...")
        frame_store['stop'] = True
    finally:
        # Ensure video thread is stopped and joined, shutdown camera and robot
        frame_store['stop'] = True
        try:
            vt.join(timeout=2.0)
        except Exception:
            pass
        try:
            if hasattr(visarm, 'disconnect'):
                visarm.disconnect()
        except Exception:
            pass
        cam.shutdown()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


if __name__ == "__main__":
    main()
