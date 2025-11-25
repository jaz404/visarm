import pyrealsense2 as rs
import numpy as np
import cv2

def nothing(x):
    pass

# Automatically adjust sliders around clicked pixel's HSV
def set_hsv_window(hsv_val, window=20):
    h, s, v = hsv_val

    # Compute ranges
    hL = max(h - window, 0)
    hH = min(h + window, 179)
    sL = max(s - window, 0)
    sH = min(s + window, 255)
    vL = max(v - window, 0)
    vH = min(v + window, 255)

    # Update sliders
    cv2.setTrackbarPos("H_low",  "HSV Tuner", hL)
    cv2.setTrackbarPos("H_high", "HSV Tuner", hH)
    cv2.setTrackbarPos("S_low",  "HSV Tuner", sL)
    cv2.setTrackbarPos("S_high", "HSV Tuner", sH)
    cv2.setTrackbarPos("V_low",  "HSV Tuner", vL)
    cv2.setTrackbarPos("V_high", "HSV Tuner", vH)

    print("\n[COLOR PICKED]")
    print("HSV =", hsv_val)
    print(f"New window: H[{hL}, {hH}]  S[{sL}, {sH}]  V[{vL}, {vH}]")

def main():

    # ------------------------------
    # RealSense pipeline
    # ------------------------------
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # ------------------------------
    # Trackbars
    # ------------------------------
    cv2.namedWindow("HSV Tuner")

    cv2.createTrackbar("H_low",  "HSV Tuner", 0,   179, nothing)
    cv2.createTrackbar("H_high", "HSV Tuner", 179, 179, nothing)
    cv2.createTrackbar("S_low",  "HSV Tuner", 0,   255, nothing)
    cv2.createTrackbar("S_high", "HSV Tuner", 255, 255, nothing)
    cv2.createTrackbar("V_low",  "HSV Tuner", 0,   255, nothing)
    cv2.createTrackbar("V_high", "HSV Tuner", 255, 255, nothing)

    print("[INFO] RealSense HSV + Color Picker active.")
    print("[INFO] Click on the RGB image to auto-adjust HSV threshold.")
    print("[INFO] Press q to quit.\n")

    clicked_hsv = None
    frame_ref = [None]  # shared ref for click handler

    # ------------------------------
    # Click handler
    # ------------------------------
    def click_event(event, x, y, flags, param):
        nonlocal clicked_hsv

        if event == cv2.EVENT_LBUTTONDOWN and frame_ref[0] is not None:
            bgr = frame_ref[0][y, x]
            hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
            clicked_hsv = hsv
            set_hsv_window(hsv)

    cv2.namedWindow("RealSense RGB")
    cv2.setMouseCallback("RealSense RGB", click_event)

    # ------------------------------
    # Main loop
    # ------------------------------
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        frame_ref[0] = frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get slider values
        hL = cv2.getTrackbarPos("H_low",  "HSV Tuner")
        hH = cv2.getTrackbarPos("H_high", "HSV Tuner")
        sL = cv2.getTrackbarPos("S_low",  "HSV Tuner")
        sH = cv2.getTrackbarPos("S_high", "HSV Tuner")
        vL = cv2.getTrackbarPos("V_low",  "HSV Tuner")
        vH = cv2.getTrackbarPos("V_high", "HSV Tuner")

        lower = np.array([hL, sL, vL])
        upper = np.array([hH, sH, vH])

        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Show windows
        cv2.imshow("RealSense RGB", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Result", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
