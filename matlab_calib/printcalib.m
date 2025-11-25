% script to print all the intrinsics/extrinsics as a result from the
% caliberation performed from matlab's caliberation module

load("calibrationSession.mat");

disp("====== INTRINSICS ======");
K = cameraParams.Intrinsics.IntrinsicMatrix';
disp(K);

disp("Radial Distortion:");
disp(cameraParams.Intrinsics.RadialDistortion);

disp("Tangential Distortion:");
disp(cameraParams.Intrinsics.TangentialDistortion);

disp("====== WORLD POINTS ======");
disp(cameraParams.WorldPoints);

disp("====== EXTRINSICS (Pattern 1) ======");
tform = cameraParams.PatternExtrinsics(1);
R = tform.R;
t = tform.Translation;

disp("R:");
disp(R);

disp("t:");
disp(t);

T_checker_to_cam = [R, t'; 0 0 0 1];
disp("T_checker_to_camera:");
disp(T_checker_to_cam);

R_inv = R';
t_inv = -R_inv * t';

T_cam_to_checker = [R_inv, t_inv; 0 0 0 1];
disp("T_camera_to_checker:");
disp(T_cam_to_checker);

validate_click_points();