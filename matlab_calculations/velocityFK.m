function omega = velocityFK (robot_velocity)
    
    % omega - angular velocity of motor in revs/sec
    % ball_velocity - the linear and angular components of the 
    % Coordinates:  x - forward, y - left, z - up
    % Motors:       a - forward, b - right, c - left

    % Ballbot constants
    d_wheel = 2.5;          % diameter of omniwheel
    d_ball = 10;            % diameter of basketball
    theta = 45;             % included angle between omniwheel's axis and vertical
    
    pinion = 16;            % motor pinion teeth
    cluster_gear = 50;      % cluster gear teeth
    cluster_pulley = 18;    % cluster pully teeth
    output_pulley = 32;     % output pulley teeth
    gear_ratio = (cluster_gear/pinion) * (output_pulley/cluster_pulley) * (d_ball/d_wheel);
    
    % Transform tangent velocities X and Y into angular velociites, Z is
    % already angualr. Scale to rads/second
    ball_angular_velocity(1) = robot_velocity(2) / (d_ball/2);
    ball_angular_velocity(2) = robot_velocity(1) / (d_ball/2);
    ball_angular_velocity(3) = robot_velocity(3) * (2*pi);
    
    
    T = [sind(theta)                0                       -cosd(theta);
        -cosd(60)*sind(theta)       -sind(60)*sind(theta)   -cosd(theta);
        -cosd(60)*sind(theta)       sind(60)*sind(theta)    -cosd(theta)];
    
    assignin('base', 'T', T);
    
    % Convert from ball (Cartesian) coordinates to ball axis coordiantes
    omega = T * ball_angular_velocity';
    
    % Scale by gear ratio. Convert to revs/sec.
    omega = omega * gear_ratio / (2*pi);

end