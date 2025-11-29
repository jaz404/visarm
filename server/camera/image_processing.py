import cv2
import numpy as np
from .camera import Camera


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


def main():
    cam = Camera("camera_params.json")
    ip = ImageProcessing(cam.K, cam.dist, cam.outline)

    HSV_DARK_GREEN = {
        "lower": np.array([30, 71, 0]),
        "upper": np.array([54, 255, 110])
    }

    while True:
        raw = cam.get_frame_raw()

        processed, centers, color_mask = ip.process(raw, HSV_DARK_GREEN)

        # Show visual result
        cv2.imshow("Processed", processed)
        cv2.imshow("Mask", color_mask)

        # Print centers live
        if len(centers) > 0:
            print("Detected centers:", centers)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cam.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
