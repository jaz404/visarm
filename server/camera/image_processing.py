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

        # extract centers
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

        if contours:
            cnt = max(contours, key=cv2.contourArea)
            (cx, cy), r = cv2.minEnclosingCircle(cnt)
            cx, cy = int(cx), int(cy)

        centers = []
        # Minimum area to consider a valid detection (in px)
        MIN_AREA_PX = 150

        # for cnt in contours:
        #     area = cv2.contourArea(cnt)
        #     if area < MIN_AREA_PX:
        #         continue
        #     M = cv2.moments(cnt)
        #     if M.get('m00', 0) == 0:
        #         continue
        #     cx = int(M['m10'] / M['m00'])
        #     cy = int(M['m01'] / M['m00'])
        #     centers.append((cx, cy))

        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_AREA_PX:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            cx, cy = int(cx), int(cy)
            centers.append((cx, cy))

        # Apply linear y-direction correction relative to checkerboard outline
        # Empirical error (in cm) increases linearly from top to bottom:
        # Top -> 0cm error, Bottom -> 1.8cm error.
        # Convert cm to pixels using the checkerboard's known physical width (138mm).
        if len(centers) > 0 and hasattr(self, "outline") and self.outline is not None and self.outline.size >= 2:
            # Normalize outline shape and order: BL, BR, TR, TL
            outline = np.array(self.outline, dtype=np.int32)
            # Accept shapes (4,2) or (4,1,2); reshape to (4,2)
            if outline.ndim == 3:
                outline = outline.reshape(4, 2)
            # Indices: 0=BL, 1=BR, 2=TR, 3=TL
            BL = outline[0]
            BR = outline[1]
            TR = outline[2]
            TL = outline[3]

            # Vertical bounds from defined corners
            y_top = int(min(TR[1], TL[1]))
            y_bottom = int(max(BL[1], BR[1]))
            H_px = max(1, abs(y_bottom - y_top))  # avoid div by zero

            # Physical scaling from bottom edge width: BR.x - BL.x corresponds to 138mm => 13.8cm
            W_px = max(1, int(BR[0]) - int(BL[0]))
            board_width_cm = 13.8  # 138mm
            cm_per_px = board_width_cm / float(W_px)
            px_per_cm = 1.0 / cm_per_px
            max_err_cm = 1.0  # maximum empirical error at the bottom
            corrected_centers = []
            for (cx, cy) in centers:
                y_norm = abs(cy - y_top) / float(H_px)
                y_norm = float(np.clip(y_norm, 0.0, 1.0))
                err_cm = max_err_cm * y_norm
                err_px = err_cm * px_per_cm
                corr_px_int = int(round(err_px))
                if y_norm > 0.0 and err_px > 0.0 and corr_px_int == 0:
                    corr_px_int = 1
                cy_corr = int(cy - corr_px_int)
                corrected_centers.append((cx, cy_corr))
            centers = corrected_centers
        return centers, mask

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

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

    HSV_BLUE = {
        "lower": np.array([83, 184, 0]),
        "upper": np.array([179, 255, 108])
    }
    print(cam.outline)
    while True:
        raw = cam.get_frame_raw()

        processed, centers, color_mask = ip.process(raw, HSV_BLUE)

        # Show visual result
        cv2.imshow("Processed", processed)
        cv2.imshow("Mask", color_mask)

        cv2.imshow("raw", raw)

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
