import numpy as np

def compute_checkerboard_errors(corners, world_points, K, R, t, T_O_from_checker, pixel_to_robot_fn):
    """
    Compute positional errors (in robot coordinates) for all checkerboard inner corners.

    Parameters
    ----------
    corners : (N,1,2) list of detected pixel points from cv2.findChessboardCorners
    world_points : (N,2) checkerboard ground truth points in mm
    K, R, t : camera intrinsics and extrinsics
    T_O_from_checker : 4x4 transform: checkerboard -> robot
    pixel_to_robot_fn : function(u, v, K, R, t, T_O_from_checker) -> (x,y,z)

    Returns
    -------
    dict with:
        theory_robot_pts   : (N,3)
        detected_robot_pts : (N,3)
        error_vectors      : (N,3)
        error_magnitudes   : (N,)
        mean_error         : float
        max_error          : float
        rms_error          : float
    """

    N = len(world_points)

    theory_robot_pts = []
    detected_robot_pts = []

    # Step 1: For each checkerboard point
    for i in range(N):
        # --------------------------
        # 1. Theoretical robot point
        # --------------------------
        wx, wy = world_points[i]
        checker_h = np.array([wx, wy, 0, 1], dtype=np.float32)
        robot_th = T_O_from_checker @ checker_h
        robot_th = robot_th[:3]
        theory_robot_pts.append(robot_th)

        # --------------------------
        # 2. Detected robot point (pixel -> robot)
        # --------------------------
        u, v = corners[i,0]  # cv2 gives corners as (N,1,2)
        robot_det = pixel_to_robot_fn(u, v, K, R, t, T_O_from_checker)
        detected_robot_pts.append(robot_det)

    theory_robot_pts   = np.vstack(theory_robot_pts)
    detected_robot_pts = np.vstack(detected_robot_pts)

    # --------------------------
    # 3. Error calculation
    # --------------------------
    error_vectors = detected_robot_pts - theory_robot_pts
    error_magnitudes = np.linalg.norm(error_vectors, axis=1)

    mean_error = float(np.mean(error_magnitudes))
    max_error  = float(np.max(error_magnitudes))
    rms_error  = float(np.sqrt(np.mean(error_magnitudes**2)))

    return {
        "theory_robot_pts": theory_robot_pts,
        "detected_robot_pts": detected_robot_pts,
        "error_vectors": error_vectors,
        "error_magnitudes": error_magnitudes,
        "mean_error": mean_error,
        "max_error": max_error,
        "rms_error": rms_error,
    }

errors = compute_checkerboard_errors(
    corners=corners,
    world_points=world_points,           # 45 theoretical checkerboard points
    K=K,
    R=R_new,
    t=t_new,
    T_O_from_checker=T_O_from_checker,
    pixel_to_robot_fn=pixel_to_robot
)

print("\n================ ERROR ANALYSIS ================")
print("Mean error   :", errors["mean_error"], "mm")
print("Max error    :", errors["max_error"],  "mm")
print("RMS error    :", errors["rms_error"],  "mm")
print("===============================================")

import matplotlib.pyplot as plt
import numpy as np

def visualize_checkerboard_errors(errors, title="Checkerboard Robot-Frame Error"):
    theory = errors["theory_robot_pts"]
    detected = errors["detected_robot_pts"]
    err_vec = errors["error_vectors"]
    err_mag = errors["error_magnitudes"]

    plt.figure(figsize=(10, 8))
    
    # ---------------------------------------------------------
    # 1. Plot theoretical grid
    # ---------------------------------------------------------
    plt.scatter(theory[:,0], theory[:,1], 
                c="green", s=60, label="Theoretical", marker="o")

    # ---------------------------------------------------------
    # 2. Plot detected positions with error magnitude color
    # ---------------------------------------------------------
    sc = plt.scatter(detected[:,0], detected[:,1],
                     c=err_mag, cmap="jet", s=60,
                     label="Detected")

    plt.colorbar(sc, label="Error magnitude (mm)")

    # ---------------------------------------------------------
    # 3. Draw lines (error vectors)
    # ---------------------------------------------------------
    for i in range(len(theory)):
        x_vals = [theory[i,0], detected[i,0]]
        y_vals = [theory[i,1], detected[i,1]]
        plt.plot(x_vals, y_vals, "r-", linewidth=0.8)

    # ---------------------------------------------------------
    # 4. Annotate largest error
    # ---------------------------------------------------------
    idx_max = np.argmax(err_mag)
    plt.text(detected[idx_max,0], detected[idx_max,1], 
             f"{err_mag[idx_max]:.1f} mm",
             color="red", fontsize=12)

    # ---------------------------------------------------------
    # 5. Title and labels
    # ---------------------------------------------------------
    plt.title(
        f"{title}\n"
        f"Mean = {errors['mean_error']:.2f} mm | "
        f"Max = {errors['max_error']:.2f} mm | "
        f"RMS = {errors['rms_error']:.2f} mm"
    )

    plt.xlabel("Robot X (mm)")
    plt.ylabel("Robot Y (mm)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")

    plt.show()
