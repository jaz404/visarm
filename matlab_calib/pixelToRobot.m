function p_robot = pixelToRobot(u, v, cameraParams, T_O_from_checker)
% Corrected version - produces proper 4×1 homogeneous points

    % === 1. Intrinsics ===
    K = cameraParams.Intrinsics.IntrinsicMatrix';
    Kinv = inv(K);

    % === 2. Choose extrinsics (pattern 1) ===
    tform = cameraParams.PatternExtrinsics(1);
    R = tform.R;
    t = tform.Translation';

    % === 3. Build camera→checkerboard transform ===
    R_inv = R';
    t_inv = -R_inv * t;

    T_cam_to_checker = [R_inv, t_inv;
                        0 0 0 1];

    % === 4. Pixel to ray (camera frame) ===
    p_img = [u; v; 1];
    ray_cam = Kinv * p_img;
    ray_cam = ray_cam / norm(ray_cam);

    % === 5. Transform ray into checkerboard frame ===
    ray_chk = R_inv * ray_cam;
    cam_origin_chk = t_inv;

    % === 6. Intersect ray with Z=0 plane in checkerboard frame ===
    s = -cam_origin_chk(3) / ray_chk(3);
    p_checker = cam_origin_chk + s * ray_chk;  % 3×1

    % === 7. Convert to homogeneous (correct size 4×1) ===
    p_checker_h = [p_checker; 1];   % 4×1

    % === 8. Checkerboard → Robot frame ===
    p_robot_h = T_O_from_checker * p_checker_h;

    % === 9. Return XYZ ===
    p_robot = p_robot_h(1:3);

end
