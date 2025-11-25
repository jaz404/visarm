import cv2
import numpy as np
import time
from camera import Camera
from image_processing import ImageProcessing

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from visarm import VisArm

# Dark Green HSV
HSV_DARK_GREEN = {
    "lower": np.array([30, 71, 0]),
    "upper": np.array([54, 255, 110])
}
import select

def user_input_available():
    """Returns True if the user typed something in the terminal (non-blocking)."""
    return select.select([sys.stdin], [], [], 0)[0] != []

def main():

    visarm = VisArm()

    if not visarm.connect():
        return
    
    # SET TO SURVEY!
    visarm.set_survey()
    time.sleep(2)

    print("[SYSTEM] Initializing Camera…")  
    cam = Camera("camera_params.json")
    if cam:
        cam.save_board_outline_to_json()

    
    print("\n[SYSTEM] Camera calibration complete.")
    print("[SYSTEM] You may now place objects inside the board region.\n")

    # Load saved outline from camera_params.json (computed already by Camera)
    outline_px = cam.outline

    # Create image processing instance
    ip = ImageProcessing(cam.K, cam.dist, outline_px)


    last_detected = None  # store latest good detection

    while True:

        # Raw frame
        raw = cam.get_frame_raw()
        if raw is None:
            continue

        # Process
        processed, centers, mask = ip.process(raw, HSV_DARK_GREEN)

        # Track latest detected center
        if len(centers) > 0:
            last_detected = centers[0]

            cx, cy = last_detected
            P_robot_mm = cam.pixel_to_robot(cx, cy)

            # draw live overlay
            cv2.circle(processed, (cx, cy), 8, (0, 0, 255), -1)
            cv2.putText(
                processed,
                f"{P_robot_mm[0]:.1f},{P_robot_mm[1]:.1f},{P_robot_mm[2]:.1f}",
                (cx + 10, cy - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
            )

        # Show windows
        cv2.imshow("Raw feed", raw)
        cv2.imshow("Processed View (Outline + Detection)", processed)
        cv2.imshow("Mask (color only)", mask)

        # 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exiting...")
            break

        # --- Non-blocking terminal input check ---
        if user_input_available():
            user_text = sys.stdin.readline().strip()
            if user_text != "":
                print("\n[INPUT RECEIVED] ->", user_text)

                if last_detected is None:
                    print("[ERROR] No object detected yet.")
                else:
                    cx, cy = last_detected
                    P_robot_mm = cam.pixel_to_robot(cx, cy)

                    # Convert mm → cm
                    P_robot_cm = P_robot_mm / 10.0

                    # Set Z = 6 cm
                    P_robot_cm[2] = 4.0

                    print(f"[FINAL] Pixel center = ({cx}, {cy})")
                    print(f"[FINAL] Robot coordinates (cm) = {P_robot_cm}")

                    # finding ik solution
                    ik_solution = visarm.inv_kinematics(np.array([P_robot_cm[0], P_robot_cm[1], P_robot_cm[2]]))
                    if ik_solution:
                        best_angles, best_err = min(ik_solution, key=lambda x: x[1])
                        print(np.round(np.degrees(best_angles), 3),
                              f"with pos error: {best_err:.4f} cm")
                        print("[VisArm] Moving to IK solution...")
                        print("[VisArm] Solution found")
                        print("[VisArm] moving to origin")
                        visarm.set_home()
                        time.sleep(2)
                        if visarm.set_joint_angles(np.degrees(best_angles)):
                            print("[VisArm] Move successful.")
                        
                        # reached now close gripper
                        best_angles_EE = np.insert(best_angles, 5, 30.0)
                        print("[VisArm] initiating object pickup!")
                        visarm.set_joint_angles(np.degrees(best_angles_EE))


                        visarm.set_survey()
                        visarm.set_home()


                    else:
                        print("[VisArm] No IK solution found.")


                    # now set it to the actual position



                break  # exit loop after handling input

    cam.shutdown()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()
