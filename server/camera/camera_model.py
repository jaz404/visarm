import json
import numpy as np
import cv2

def load_camera(filename="camera_params.json"):
    with open(filename, "r") as f:
        params = json.load(f)

    # Intrinsics
    K = np.array(params["intrinsics"]["K"])

    # Extrinsics
    R = np.array(params["extrinsics"]["R"])
    t = np.array(params["extrinsics"]["t"]).reshape(3, 1)

    # Checkerboard -> Robot
    T_O_from_checker = np.array(params["checker_to_robot"])

    return K, R, t, T_O_from_checker


def pixel_to_robot(u, v, K, R, t, T_O_from_checker):
    """Convert pixel -> robot coordinates using Z=0 intersection."""

    Kinv = np.linalg.inv(K)

    # pixel → normalized ray
    p = np.array([u, v, 1])
    ray_cam = Kinv @ p
    ray_cam /= np.linalg.norm(ray_cam)

    # Convert into checkerboard frame
    R_inv = R.T
    t_inv = -R_inv @ t

    cam_origin_chk = t_inv.flatten()
    ray_chk = (R_inv @ ray_cam).flatten()

    # Ray-plane intersection (checkerboard Z=0)
    s = -cam_origin_chk[2] / ray_chk[2]
    P_chk = cam_origin_chk + s * ray_chk

    # Homogeneous
    P_chk_h = np.append(P_chk, 1)

    # Checkerboard → Robot
    P_robot = T_O_from_checker @ P_chk_h

    return P_robot[:3]

def compute_camera_pose(get_raw_image_fn, K, dist_coeffs, world_points, pattern_size):
    """
    Replicates MATLAB calculateCameraPos().
    Detects checkerboard → solves PnP → returns (R, t, pose4x4).
    """

    raw = get_raw_image_fn()

    # Undistort
    img = cv2.undistort(raw, K, dist_coeffs)

    # Detect checkerboard
    ret, corners = cv2.findChessboardCorners(img, pattern_size)
    if not ret:
        raise RuntimeError("Checkerboard NOT detected")

    # Sub-pixel refinement 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        ),
    )

    # repare 3D world points (z = 0)
    world_points = np.array(world_points, dtype=np.float32)
    objp = np.hstack([world_points, np.zeros((len(world_points), 1), dtype=np.float32)])

    # Solve extrinsics (PnP)
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist_coeffs)
    if not ok:
        raise RuntimeError("solvePnP failed")

    # Convert rvec → R
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3, 1)

    # 6. Build pose matrix
    pose = np.eye(4)
    pose[:3, :3] = R
    pose[:3, 3:4] = t

    return R, t, pose, corners, img


def save_camera_extrinsics(R, t, pose, filename="camera_params.json"):
    """
    Updates the extrinsic values inside camera_params.json.
    """

    with open(filename, "r") as f:
        params = json.load(f)

    params["extrinsics"]["R"] = R.tolist()
    params["extrinsics"]["t"] = t.flatten().tolist()

    with open(filename, "w") as f:
        json.dump(params, f, indent=2)

    print("[INFO] Updated extrinsics saved to", filename)
