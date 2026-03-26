clearvars -except tbot

% Start TurtleBot connection (tbot object)
if ( ~exist("tbot", "var") )
    IP_TURTLEBOT = "192.168.21.135";         
    IP_HOST_COMPUTER = "192.168.21.1";       
    tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER);   
    if( tbot.getVersion() < 0.9 ) error ('TurtleBot v09 required'); end 
    %tbot.gazeboDeleteAllModels();
    %tbot.gazeboPlace3DCylinder(2, 2, 0.5, 0.5, 'r');        
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


% Target waypoints
t = [-6.5, 4];%; -2.5, 4];% -2.5, 0.5; 3, 0.5; 3, 4.5; 1, 4.5; 6.5, 4.5; 6.5, -1; 5.5, -1; 5.5, -3];
n = size(t, 1);
allTrajectory = []; 

% Map Setup 
mscale = 20;              
worldSize = 20;             
hm = worldSize*mscale; wm = worldSize*mscale;
map = 0.5 * ones(hm, wm);

% Use an offset to shift negative coordinates into positive matrix indices
offset_m = 10; 

% Setup the robot safety radius and inflation disk
r_robot_m = 0.12; % Safety radius
se = strel('disk', ceil(r_robot_m * mscale));

% Initialize log odds values and control gains
l0 = 0; l_occ = 0.65; l_free = -0.65;
kp = 2; ks = 1.2; Eps = 0.05;
v=0; w=0;

% Graphics Setup 
fig = figure(1); clf(fig);
mainAx  = axes('Parent', fig, 'Position', [0.1, 0.1, 0.8, 0.8]); 
insetAx = axes('Parent', fig, 'Position', [0.7, 0.7, 0.2, 0.2]);

% Init ratecontrol obj 
r = rateControl(5);     

% Main Control Loop
for i=1:n
    % Reset local trajectory buffer for each waypoint
    Trajectory = zeros(maxIterations, 3); 
    
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
        
        % Control Law
        dist = sqrt((x-t(i, 1))^2 + (y-t(i, 2))^2); 
        if dist <= Eps, break; end
        
        %v = min(kp * dist, 0.22); % Saturation added
        v = kp * dist;
        thetaTarget = atan2(t(i, 2)-y, t(i, 1)-x);
        w = ks * normalizeAngle(thetaTarget - theta);
        
        % Extract occupied cells for the main plot display
        occVis = map > 0.65;
        [rowm, colm] = find(occVis);
        xObs = (rowm ./ mscale) - offset_m;
        yObs = (colm ./ mscale) - offset_m;
        
       
        figure(1); clf; hold on;
        
        % Main plot
        plot(xObs, yObs, 'ks', 'MarkerSize', 4, 'MarkerFaceColor', 'k');
        plot(Trajectory(1:it, 1), Trajectory(1:it, 2), 'b');
        plot(t(i, 1), t(i, 2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);     
        drawTurtleBot(x, y, theta);

        % Draw the coordinate axes and set the display limits
        quiver(0,0,1,0,0.5,'r'); 
        quiver(0,0,0,1,0.5,'g');
        axis([-10, 10, -10, 10]); 
        grid on; 
        xlabel('x'); ylabel('y'); title('Map Building');
        
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

% Save the exact binary map and scale variables directly to a .mat file
filename = 'turtlebot_map.mat';
save(filename, 'map', 'occVisFinal', 'mscale', 'worldSize');