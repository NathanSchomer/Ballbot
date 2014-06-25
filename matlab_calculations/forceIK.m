function motor_tau = forceIK (ball_force)

    % Inputs:       vel_x - rolling velocity in robot's forward direction, inches/sec
    %               vel_y - rolling velocity in robot's left direction, inches/sec
    %               omega_z - angular velocity about robot's Z axis, deg/sec
    %
    % Outputs:      motor_cps[1] - speed of forward motor, counts per second
    %               motor_cps[2] - speed of right motor, counts per second
    %               motor_cps[3] - speed of left motor, counts per second

    % Coordinates:  x - forward, y - left, z - up
    % Motors:       a - forward, b - right, c - left

    % Ballbot constants
    d_wheel = 0.070;        % diameter of omniwheel, m
    d_ball = 0.254;         % diameter of basketball, m
    theta = 45;             % included angle between omniwheel's axis and vertical
    
    pinion = 16;            % motor pinion teeth
    cluster_gear = 50;      % cluster gear teeth
    cluster_pulley = 18;    % cluster pully teeth
    output_pulley = 32;     % output pulley teeth
    gear_ratio = (cluster_gear/pinion) * (output_pulley/cluster_pulley) * (d_ball/d_wheel);
%     gear_ratio = 1;
    
    ball_tau(1) = ball_force(2) * d_ball/2;
    ball_tau(2) = ball_force(1) * d_ball/2;
    ball_tau(3) = ball_force(3);
    
    % transformation from cartesian coordinates to motor axis coordinates
    T = [sind(theta)        -cosd(60)*sind(theta)   -cosd(60)*sind(theta);
        0                	-sind(60)*sind(theta)    sind(60)*sind(theta);
        -cosd(theta)        -cosd(theta)            -cosd(theta)]^-1;
    
    assignin('base', 'T', T);
    
    motor_tau = T * ball_tau';
    
    % Scale torque based on gear ratio of train
    motor_tau = motor_tau / gear_ratio;

end