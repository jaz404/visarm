function validate_click_points()

    % === Load MATLAB camera calibration (RGB intrinsics + extrinsics) ===
    S = load("calibrationSession.mat");
    session = S.calibrationSession;
    cameraParams = session.CameraParameters;
    % === Your Checkerboard → Robot transform ===
    T_O_from_checker = [
        0  -1  0  305;
       -1   0  0  117;
        0   0 -1    0;
        0   0  0    1 ];

    % === CAPTURE FROM REALSENSE RGB ===
    % You must already have RealSense pipeline running.
    % Here, you load a saved frame OR replace with your RealSense code.
    rgb = imread("test_img_Color.png");  

    % === Undistort using MATLAB calibration of the RGB camera ===
    intrinsics_sdk = [];
    [img, ~] = undistortImage(rgb, cameraParams.Intrinsics);

    figure; imshow(img);
    title("Click on ANY checkerboard point. Press ENTER to stop.");

    all_errors = [];

    while true
        [u,v] = ginput(1);
        if isempty(u)
            break;
        end

        plot(u,v,'ro','MarkerSize',10,'LineWidth',2);

        fprintf("\n--- POINT CLICKED ---\n");
        fprintf("Pixel = (%.2f, %.2f)\n", u, v);

        Xc = input("Enter checkerboard X (mm): ");
        Yc = input("Enter checkerboard Y (mm): ");

        p_checker_true = [Xc; Yc; 0; 1];
        p_robot_true = T_O_from_checker * p_checker_true;

        p_robot_est = pixelToRobot(u, v, cameraParams, T_O_from_checker);

        err = abs(p_robot_est - p_robot_true(1:3));

        fprintf("Estimate (robot): [%.3f %.3f %.3f]\n", p_robot_est);
        fprintf("Ground truth     : [%.3f %.3f %.3f]\n", p_robot_true(1:3));
        fprintf("Error (mm)       : [%.3f %.3f %.3f]\n", err);

        all_errors = [all_errors; err'];

    end

    fprintf("\n=== FINAL SUMMARY ===\n");
    fprintf("Mean error (mm): [%.3f %.3f %.3f]\n", mean(all_errors));
    fprintf("Max  error (mm): [%.3f %.3f %.3f]\n", max(all_errors));

end
