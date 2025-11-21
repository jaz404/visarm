import pyrealsense2 as rs
import numpy as np
import cv2
import math

# RealSense setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

print("[INFO] RGB Cube Detector Active — Press ESC to quit")

def to_homog(p):
    return np.array([float(p[0]), float(p[1]), 1.0])


def normalize_line(L):
    n = np.linalg.norm(L[:2])
    return L / n if n > 1e-9 else L

# aligning with the verical lines should be easier
def pick_opposite_vertical_edges(box):
    p0, p1, p2, p3 = box

    # Opposite edge pairs
    E1A, E1B = p0, p1
    E1C, E1D = p2, p3

    E2A, E2B = p1, p2
    E2C, E2D = p3, p0

    def edge_verticality(a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        angle = abs(math.degrees(math.atan2(dy, dx)))
        return abs(angle - 90)   # closer to 90 = more vertical

    # compute score for both opposite pairs
    v1 = edge_verticality(E1A, E1B) + edge_verticality(E1C, E1D)
    v2 = edge_verticality(E2A, E2B) + edge_verticality(E2C, E2D)

    # lower score = more vertical
    if v1 < v2:
        return (E1A, E1B), (E1C, E1D)
    else:
        return (E2A, E2B), (E2C, E2D)


while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue

    color = np.asanyarray(color_frame.get_data())
    vis = color.copy()

    # Edge image 
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 1.2)
    edges = cv2.Canny(blur, 40, 130)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 200 < area < 250000:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            if 4 <= len(approx) <= 8:
                candidates.append(cnt)

    if len(candidates) == 0:
        cv2.putText(vis, "Cube not detected", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

    else:

        # process ALL detected candidates
        for cube_id, cnt in enumerate(candidates, start=1):

            # ---- Rectangle fit ----
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect).astype(np.int32)

            cv2.polylines(vis, [box], True, (0, 255, 0), 2)

            for (x, y) in box:
                cv2.circle(vis, (x, y), 5, (0, 0, 255), -1)

            (pA1, pB1), (pA2, pB2) = pick_opposite_vertical_edges(box)

            # Draw finite segments
            cv2.line(vis, tuple(pA1), tuple(pB1), (0, 0, 255), 3) 
            cv2.line(vis, tuple(pA2), tuple(pB2), (255, 0, 0), 3)  

            L1 = normalize_line(np.cross(to_homog(pA1), to_homog(pB1)))
            L2 = normalize_line(np.cross(to_homog(pA2), to_homog(pB2)))

            # Display line parameters 
            cx = int((box[0][0] + box[2][0]) / 2)
            cy = int((box[0][1] + box[2][1]) / 2)

            cv2.putText(vis,
            f"C{cube_id} L1[{L1[0]:.2f},{L1[1]:.2f}] L2[{L2[0]:.2f},{L2[1]:.2f}]",
            (cx, cy),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.3, (255,255,255), 1)


    cv2.imshow("Edges", edges)
    cv2.imshow("Cube Detection (RGB)", vis)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

pipeline.stop()
cv2.destroyAllWindows()
