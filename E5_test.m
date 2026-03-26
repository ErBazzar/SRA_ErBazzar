% clear memory 
clearvars -except tbot

% start TurtleBot connection (tbot object), only if required
if ( ~exist("tbot", "var") )
    IP_TURTLEBOT = "192.168.21.136";         
    IP_HOST_COMPUTER = "192.168.21.1";       
    tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER);   
    if( tbot.getVersion() < 0.9 ) error ('TurtleBot v09 required'); end 
end 

% Normalize angle in range [-pi, pi]
normalizeAngle = @(angle) atan2(sin(angle), cos(angle));

% # of max iterations 
maxIterations = 5000;

% Buffer for the path of the bot
Trajectory = zeros(maxIterations, 3); 

% Load the generated A* path
load('astar_path.mat'); 


% Convert the A* pixel indices back to real-world coordinates [m]

offset_m = 10; % In the gazebo map of the house the origin is in the middle of a 20x20 grid
path_m = zeros(size(path));
path_m(:, 1) = (path(:, 1) / mscale) - offset_m; 
path_m(:, 2) = (path(:, 2) / mscale) - offset_m; 

% Pure Pursuit & Path Setup 
num_waypoints = size(path_m, 1); % Checks how many points is the path made up of
target_idx = 1;                  % Initializes the index of the points in the path
lookahead_dist = 0.4;            % Lookahead radius for path pursuit

% Set TurtleBot's 2D start pose (x, y, theta)
start_x = path_m(1, 1);
start_y = path_m(1, 2);

% Directs the orientation of the bot towards the second point of the tracking
if num_waypoints > 1
    start_theta = atan2(path_m(2, 2) - start_y, path_m(2, 1) - start_x);
else
    start_theta = 0;
end
tbot.setPose(start_x, start_y, start_theta);

% Tolerances and Control Variables
Eps = 0.35;         
d_star = 0.005;     
IntegralError = 0;  
previousError = 0;            

% Proportional gains for velocity control
kp = 1;
ks = 1;
ki = 0.011;

% Init ratecontrol obj 
r = rateControl(5);    % run at 5Hz
dt = r.DesiredPeriod;  % Loop time period: dt = 1/5 Hz = 0.2s 

% Tracking loop

figure(1); clf; % Prepares the figure for visualization

for it = 1:1:maxIterations  
    
    % Read TurtleBot pose
    [x, y, theta, timestamp] = tbot.readPose();
    Trajectory(it, 1) = x;
    Trajectory(it, 2) = y;
    Trajectory(it, 3) = theta;
    
    % Check if we reached the final goal
    dist_to_final = sqrt((x - path_m(end, 1))^2 + (y - path_m(end, 2))^2);
    if dist_to_final <= Eps
        disp('Goal Reached! Stopping the robot.');
        v = 0; w = 0; 
        tbot.setVelocity(v, w); 
        break; 
    end
    
    % Update the target waypoint
    dist_to_target = sqrt((x - path_m(target_idx, 1))^2 + (y - path_m(target_idx, 2))^2);
    
    % Checks waypoints until it finds one outside the lookahead radius
    while dist_to_target < lookahead_dist && target_idx < num_waypoints
         target_idx = target_idx + 1;
         dist_to_target = sqrt((x - path_m(target_idx, 1))^2 + (y - path_m(target_idx, 2))^2);
    end
    
    % Updates the reference with the waypoint found
    ref_x = path_m(target_idx, 1);
    ref_y = path_m(target_idx, 2);
    
    % PI Control Law (Linear Velocity)
    Error = sqrt((x-ref_x)^2 + (y-ref_y)^2) - d_star; 
    IntegralError = IntegralError + (Error + previousError) * dt / 2;
    IntegralError = min(max(IntegralError, -0.1), 0.1); % Anti-windup cap
    
    v = kp * Error + ki * IntegralError;
    
    % Cap the linear velocity to match the physical limitations of the bot
    v = min(max(v, -0.22), 0.22); 
    
    % P Control Law (Angular Velocity)
    theta_ref = atan2(ref_y-y, ref_x-x);
    w = ks * normalizeAngle(theta_ref - theta);
    
    % Cap the angular velocity to match the physical limitations of the bot
    w = min(max(w, -2.84), 2.84);
    
    % Velocity commands to the motors
    tbot.setVelocity(v, w);
    
    % Display 
    cla; hold on; 
    
    plot(path_m(:,1), path_m(:,2), 'k--', 'LineWidth', 1.5, 'DisplayName', 'A* Path');
    plot(path_m(end,1), path_m(end,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal');
    
    drawTurtleBot(x, y, theta);         
    plot(Trajectory(1:it,1), Trajectory(1:it,2), 'b', 'LineWidth', 1.5, 'DisplayName', 'Robot Trajectory');
    
    axis([-10, 10, -10, 10]);            
    grid on;                            
    xlabel('x [m]'); ylabel('y [m]');
    title('A* Path Tracking');                         

    drawnow limitrate; 
    
    waitfor(r);
end

tbot.stop();