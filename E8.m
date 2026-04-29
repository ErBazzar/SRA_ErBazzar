clearvars -except tbot

% Start TurtleBot connection (tbot object)
if ( ~exist("tbot", "var") )
    IP_TURTLEBOT = "192.168.21.136";         
    IP_HOST_COMPUTER = "192.168.21.1";       
    tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER);   
    if( tbot.getVersion() < 0.9 ) error ('TurtleBot v09 required'); end 
    tbot.gazeboDeleteAllModels();
    tbot.gazeboPlace3DCylinder(-2, 0, 1, 0.5, 'r');        
    %tbot.gazeboPlace3DCardboardBox(2, 3, 0);
end 

% Normalize angle in range [-pi, pi]
normalizeAngle = @(angle) atan2(sin(angle), cos(angle));

% # of max iterations 
maxIterations = 5000;

% Set the starting pose of the robot in the world frame
xri = -6.5; yri = -3; theta = pi/2;   
tbot.setPose(xri, yri, theta);
d = 0.0305;

% Map Setup 
mscale    = 20;              
worldSize = 20;             
hm = worldSize*mscale; wm = worldSize*mscale;
map = 0.5 * ones(hm, wm);

% Use an offset to shift negative coordinates into positive matrix indices
offset_m = 10; 

% A*-generated waypoints 
% Goal pose in world coordinates [m]
xgoal = 3; ygoal = 4;

% Number of waypoints to extract from the A* path 
N_waypoints = 5;

% Load the prior map used for planning 
plannerData = load('turtlebot_map.mat');   % requires a previous mapping run
plannerMap  = plannerData.occVisFinal;     % inflated binary map

% Convert start / goal from world meters to grid indices 
start_idx = [max(1, round((xri   + offset_m) * mscale)), ...
             max(1, round((yri   + offset_m) * mscale))];
goal_idx  = [max(1, round((xgoal + offset_m) * mscale)), ...
             max(1, round((ygoal + offset_m) * mscale))];

% Plan with A*
path_px = A_star(plannerMap, start_idx, goal_idx);
if isempty(path_px)
    error('A* failed: no feasible path between start and goal.');
end

% Convert pixel path back to world meters
path_m = (path_px / mscale) - offset_m;

% Drop consecutive duplicates so cumulative arc length is strictly monotonic
keep   = [true; sqrt(sum(diff(path_m).^2, 2)) > 0];
path_m = path_m(keep, :);

% Segment by equal arc length: N_waypoints + 1 samples, drop the start
cumDist  = [0; cumsum(sqrt(sum(diff(path_m).^2, 2)))];
totalLen = cumDist(end);
sampleD  = linspace(0, totalLen, N_waypoints + 1);
sampleD  = sampleD(2:end);                          % Skip the robot's start
t        = interp1(cumDist, path_m, sampleD.');     % [N_waypoints x 2] in [m]
n        = size(t, 1);
allTrajectory = []; 

% Setup the robot safety radius and inflation disk
r_robot_m = 0.105; % Safety radius
se = strel('disk', ceil(r_robot_m * mscale));

% Initialize log odds values and control gains
l0 = 0; l_occ = 0.65; l_free = -0.65;
ks = 0.7; Eps = 0.1;
v=0; w=0;

% VFH parameters
searchWindow = 1.0;     % VFH active window radius [m]
v_max = 0.22;           % TurtleBot3 max linear velocity [m/s]
w_max = 2.84;           % TurtleBot3 max angular velocity [rad/s]

% PI linear-velocity control (pure-pursuit along the VFH steering direction)
cStar = 1.0;            % Lookahead distance [m]
dStar = 0.05;           % Standoff to lookahead point [m]
kv    = 0.055;          % Linear velocity proportional gain
ki    = 0.011;          % Linear velocity integral gain

% Graphics Setup 
fig = figure(1); clf(fig);
mainAx  = axes('Parent', fig, 'Position', [0.1, 0.1, 0.8, 0.8]); 
insetAx = axes('Parent', fig, 'Position', [0.7, 0.7, 0.2, 0.2]);

% Init ratecontrol obj 
r  = rateControl(5);
dt = r.DesiredPeriod;   % Loop period for trapezoidal integration

% Main Control Loop
for i=1:n
    % Reset local trajectory buffer for each waypoint
    Trajectory = zeros(maxIterations, 3); 

    % Reset PI integrator state at the start of every new waypoint
    IntegralError = 0;
    previousError = 0;
    
    for it = 1:maxIterations  
        % Read the current pose and lidar data from the robot
        [x, y, theta, timestamp] = tbot.readPose();
        Trajectory(it, :) = [x, y, theta]; 
        [scanMsg, lddata, ldtimestamp] = tbot.getLidarData();
    
        % Compensate for the time delay between pose and lidar data
        dt_lag = ldtimestamp - timestamp;
        theta_corr = normalizeAngle(theta + (w * dt_lag));
        x_corr = x + (v * dt_lag) * cos(theta);
        y_corr = y + (v * dt_lag) * sin(theta);

        % Transform coordinates to positive indices for mapping
        map_x = x_corr + offset_m;
        map_y = y_corr + offset_m;
         
        % Mapping
        map = MapUpdate(map, [map_x, map_y, theta_corr], lddata, mscale, l_occ, l_free, l0);
        
        % Check distance to current waypoint 
        dist = sqrt((x-t(i, 1))^2 + (y-t(i, 2))^2); 
        if dist <= Eps, break; end
        
        % VFH: build the inflated occupancy map used by the planner
        % VFH treats cells with value > 0.5 as obstacles. We build a binary
        % obstacle map from the probability grid (threshold = 0.65), then
        % inflate it by the robot safety radius so the histogram already
        % accounts for the robot footprint.
        occVisRaw = map > 0.65;
        map_infl  = imdilate(occVisRaw, se);
        map_infl  = double(map_infl);    % VFH expects numeric input

        % Robot/target positions in the shifted frame used by the house map
        robotPose_shift  = [x + offset_m, y + offset_m];
        targetPose_shift = [t(i,1) + offset_m, t(i,2) + offset_m];

        % Compute VFH steering direction + smoothed polar histogram
        [steerDir, h_smooth, bin_edges] = VFH(robotPose_shift, ...
                                              targetPose_shift, ...
                                              map_infl, mscale, searchWindow);

        % Completely blocked: stop and abandon this waypoint
        if isnan(steerDir)
            v = 0; w = 0;
            tbot.setVelocity(v, w);
            fprintf('Waypoint %d/%d: blocked by obstacles, stopping.\n', i, n);
            break;
        end

        % Control Law: PI on linear velocity, P on angular
        % Pure-pursuit lookahead point along the VFH steering direction
        L     = min(cStar, dist);
        xStar = x + L * cos(steerDir);
        yStar = y + L * sin(steerDir);

        % Distance error to the lookahead
        % Near the goal, switch to using dist directly so the controller
        % does not overshoot when the lookahead exceeds the target
        if dist > dStar
            Error = sqrt((xStar - x)^2 + (yStar - y)^2) - dStar;
        else
            Error = dist;
        end
        Error = max(0, Error);    % Keep Error non-negative

        % Trapezoidal integration 
        if Error > 0
            IntegralError = IntegralError + (Error + previousError) * dt / 2;
        end
        previousError = Error;

        % PI control on linear velocity, then saturate to TurtleBot3 limits
        v = kv * Error + ki * IntegralError;
        v = max(-v_max, min(v, v_max));

        % Angular velocity: P on the VFH steering direction
        w = ks * normalizeAngle(steerDir - theta);
        w = max(-w_max, min(w, w_max));
        
        % Extract occupied cells for the main plot display
        occVis = map > 0.65;
        [rowm, colm] = find(occVis);
        xObs = (rowm ./ mscale) - offset_m;
        yObs = (colm ./ mscale) - offset_m;
        
       
        figure(1); clf; hold on;
        
        % Main plot
        plot(xObs, yObs, 'ks', 'MarkerSize', 4, 'MarkerFaceColor', 'k');

        % Planned A* path
        plot(path_m(:,1), path_m(:,2), '--', 'Color', [0.6 0.6 0.6], ...
             'LineWidth', 1);
        % All segmented waypoints (small dots) + current target (blue x)
        plot(t(:,1), t(:,2), '.', 'Color', [0.3 0.3 0.8], 'MarkerSize', 10);

        plot(Trajectory(1:it, 1), Trajectory(1:it, 2), 'b');
        plot(t(i, 1), t(i, 2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);     
        drawTurtleBot(x, y, theta);

        % VFH search window
        th_circ = linspace(0, 2*pi, 200);
        plot(x + searchWindow*cos(th_circ), y + searchWindow*sin(th_circ), ...
             'b--', 'LineWidth', 1);

        % VFH polar histogram
        drawPolarHistogram(x, y, 0, h_smooth, bin_edges, ...
                           searchWindow*0.6, 'r');

        % Steering direction arrow
        quiver(x, y, 0.5*cos(steerDir), 0.5*sin(steerDir), 0, ...
               'r', 'LineWidth', 2, 'MaxHeadSize', 0.8);

        % Pure-pursuit lookahead point
        plot(xStar, yStar, 'r*', 'MarkerSize', 8);

        % Draw the coordinate axes and set the display limits
        quiver(0,0,1,0,0.5,'r'); 
        quiver(0,0,0,1,0.5,'g');
        axis([-10, 10, -10, 10]); 
        grid on; 
        xlabel('x'); ylabel('y'); title('Map Building + VFH Navigation');
        
        % Update the small inset map in the top right corner
        ax = axes('Position', [0.655 0.655 0.275 0.275]);
        imagesc(rot90(map));        
        colormap(gray);               
        axis image;                   
        axis off;                     
        title('map');
        
        pause(0.02); 
        
        % Velocity commands
        tbot.setVelocity(v, w);
        waitfor(r);
    end
    
    % Store the results of this segment
    allTrajectory = [allTrajectory; Trajectory(1:it, :); [NaN NaN NaN]];
end

figure(2); clf; hold on;

% Extract occupied cells and inflate them for safe path planning
occVisRaw = map > 0.65; 
map_inflated = imdilate(occVisRaw, se);
occVisFinal = occVisRaw | map_inflated;

% Convert final pixel coordinates back to meters for the summary plot
[rowm_final, colm_final] = find(occVisFinal);
xObsFinal = (rowm_final ./ mscale) - offset_m;
yObsFinal = (colm_final ./ mscale) - offset_m;

% Plot the final obstacles and the full trajectory taken by the robot
plot(xObsFinal, yObsFinal, 'k.', 'MarkerSize', 6, 'DisplayName', 'Obstacles');
plot(path_m(:,1), path_m(:,2), '--', 'Color', [0.6 0.6 0.6], ...
     'LineWidth', 1, 'DisplayName', 'A* path');
plot(allTrajectory(:,1), allTrajectory(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');
plot(t(1:end-1, 1), t(1:end-1, 2), 'rx', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Waypoints'); 
plot(xri, yri, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');                             
plot(t(end, 1), t(end, 2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal');                   


axis([-10, 10, -10, 10]); 
grid on;
xlabel('x [m]');
ylabel('y [m]');
title('Final Result');
legend('show', 'Location', 'best');


% Stop robot 
tbot.stop();

% % Save the exact binary map and scale variables directly to a .mat file
% filename = 'turtlebot_map.mat';
% save(filename, 'map', 'occVisFinal', 'mscale', 'worldSize');