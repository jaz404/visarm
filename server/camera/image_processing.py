import cv2
import numpy as np
from camera import Camera


class ImageProcessing:
    def __init__(self, K, dist, outline_px):
        """
        outline_px: list or np.array of shape (4,2) in pixel coords
        """
        self.K = K
        self.dist = dist

        # Ensure numpy int32 array
        self.outline = np.array(outline_px, dtype=np.int32)

    def undistort(self, frame):
        return cv2.undistort(frame, self.K, self.dist)

    def equalize(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        hsv[:, :, 2] = clahe.apply(hsv[:, :, 2])
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    def draw_outline(self, frame):
        out = frame.copy()
        poly = self.outline.reshape((-1, 1, 2))
        cv2.polylines(out, [poly], True, (0, 255, 0), 3)
        return out

    def process(self, frame, hsv_range):
        f = self.undistort(frame)
        f = self.equalize(f)

        # apply board mask
        mask = np.zeros(f.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [self.outline.reshape((-1, 1, 2))], 255)
        f_masked = cv2.bitwise_and(f, f, mask=mask)

        # extract green centers
        centers, color_mask = self.extract_color_centers(f_masked, hsv_range)

        # visualize
        vis = f_masked.copy()
        for (cx, cy) in centers:
            cv2.circle(vis, (cx, cy), 8, (0, 200, 0), -1)
            cv2.putText(vis, f"{cx},{cy}", (cx+5, cy-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)

        # draw outline on final result
        vis = self.draw_outline(vis)

        return vis, centers, color_mask

    def extract_color_centers(self, frame, hsv_range):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, hsv_range["lower"], hsv_range["upper"])

        # Clean mask
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))

        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return [], mask

        # Choose the STRONGEST detection = largest area contour
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        cnt = contours[0]          # strongest one
        area = cv2.contourArea(cnt)

        # Reject micro-noise
        if area < 400:   # tune this
            return [], mask

        # Compute centroid
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            return [], mask

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        return [(cx, cy)], mask

    def is_object_picked(self, frame):
        # Ensure survey position before checking!
        h, w = frame.shape[:2]
        roi = frame[int(h*0.65):h, int(w*0.30):int(w*0.70)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # Strong saturation mask
        mask = cv2.inRange(s, 70, 255)

        # Slight blur
        mask = cv2.medianBlur(mask, 7)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return False

        # Keep ONLY the largest contour
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)

        # Must be large enough (>1500 px)
        if area < 1500:
            return False

        # Bounding box
        x, y, w, h = cv2.boundingRect(cnt)

        # Must be near the bottom of the ROI (cube is close to camera)
        roi_h = roi.shape[0]
        if y + h < roi_h * 0.55:   # not low enough
            return False

        # Must be roughly square-ish or tall-ish (cube face)
        ratio = w / float(h)
        if ratio < 0.4 or ratio > 2.5:
            return False

        # Must be centered horizontally (within 30-40% from center)
        roi_w = roi.shape[1]
        center_x = x + w/2
        if center_x < roi_w * 0.20 or center_x > roi_w * 0.80:
            return False

        return True


def main():
    cam = Camera("camera_params.json")
    ip = ImageProcessing(cam.K, cam.dist, cam.outline)

    HSV_DARK_GREEN = {
        "lower": np.array([30, 71, 0]),
        "upper": np.array([54, 255, 110])
    }

    while True:
        raw = cam.get_frame_raw()

        # processed, centers, color_mask = ip.process(raw, HSV_DARK_GREEN)

        # Show visual result
        # cv2.imshow("Processed", processed)
        # cv2.imshow("Mask", color_mask)

        cv2.imshow("raw", raw)


        # Print centers live
        # if len(centers) > 0:
        #     print("Detected centers:", centers)

        if ip.is_object_picked(raw):
            print("[SORTING] Cube detected in gripper.")
        else:
            print("[SORTING] No cube detected. Will retry offsets.")
            continue   # retry next offset or next loop

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cam.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
