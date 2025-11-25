
% === Load calibration parameters ===
load("calibrationSession.mat");   % loads the variable cameraParams

% === Confirm the object really has WorldPoints ===
disp("Fields inside cameraParams:");
disp(properties(cameraParams));

if ~isprop(cameraParams, "WorldPoints")
    error("cameraParams does NOT contain WorldPoints. You loaded the wrong file.");
end

% === Your checkerboard → robot transform ===
T_O_from_checker = [
    0  -1  0  305;
   -1   0  0  117;
    0   0 -1    0;
    0   0  0    1 ];

% === Pick an easily identifiable checkerboard corner ===
corner_index = 1;

world_xy = cameraParams.WorldPoints(corner_index, :); % [X Y]
p_checker_true = [world_xy 0];                         % [X Y 0]

% === Get corresponding pixel for pattern #1 ===
img_pt = cameraParams.ReprojectedPoints(corner_index, :, 1);
u = img_pt(1);
v = img_pt(2);

% === Convert pixel → robot ===
p_robot = pixelToRobot(u, v, cameraParams, T_O_from_checker);

fprintf("\n===== VALIDATION =====\n");
fprintf("Pixel (u,v) = (%.2f, %.2f)\n", u, v);
fprintf("Checkerboard corner world XY = [%.2f, %.2f]\n", p_checker_true(1), p_checker_true(2));
fprintf("Robot-frame computed XYZ     = [%.2f, %.2f, %.2f]\n", ...
    p_robot(1), p_robot(2), p_robot(3));
