%
% Lab #01 - VFF (template v02)
%
% IR / FRB 2025, DEM-UC  
% 
% NOTE: Identify the names of the authors of the labwork


% clear memory 
clearvars -except tbot

%% Start TurtleBot connection (tbot object)
if ( ~exist("tbot", "var") )
    IP_TURTLEBOT = "192.168.21.136";         
    IP_HOST_COMPUTER = "192.168.21.1";       
    tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER);   
    if( tbot.getVersion() < 0.9 ) error ('TurtleBot v09 required'); end 
    %tbot.gazeboDeleteAllModels();
    %tbot.gazeboPlace3DCylinder(2, 2, 0.5, 0.5, 'r');        
    %tbot.gazeboPlace3DCardboardBox(2, 3, 0);
end 

% normalize angle anonymous function: in range [-pi, pi]
normalizeAngle = @(angle) atan2( sin(angle), cos(angle) );

% ---------------------------
% define control constants 
% ---------------------------
cStar = 1.0;              % far ahead path point (constant)
dStar = 0.5;       
kv = 0.055;               % linear velocity proportional control constant
ki = 0.011;               % linear velocity integral control constant
ks = 0.7;                 % angular velocity proportional control constant
R = 0.25;                 % (in meters) active window radius (influence region of replusive forces)
v_max = 0.22;             % Turtlebot max linear velocity m/s
w_max = 2.84;             % Turtlebot max angular velocity rad/s
Fca=1;
Fcr=0.5;
slow_radius = 0.2;
r_robot_m   = 0.08;               % radius of the robot

% initial 2D pose (x,y,theta)
xri = 0.25; yri = 1; theta = 0;   
tbot.setPose(xri, yri, theta);

% set target location
t = [0.6 2;
     2 1.5;
     2 0.5;
     %3.7 2;
     1 3.5];
n= size(t, 1);
allTrajectory = []; %vector to save overall trajectory

% Load Map --------------------------------------
%image = imread('maps/csqmap_grid1.png');
image = imread('maps/ymap_grid1.png');
% set map scale ratio (number of pixels that represent 1 meter) 
mscale = 100;   % grid 1x1 
% Convert map into occupation grid format (obstacles = 1)
map = 1 - double( image(:,:,1) ) ./ 255; 
% get map size (rows, cols) 
[hm, wm] = size(map);
[rowm, colm] = find(map == 1);


%Obstacle augmentation
rad_px = ceil(r_robot_m * mscale);    % number of pixel to add
se = strel('disk', rad_px, 0);
%obstacle inflation
map_infl = imdilate(map > 0.5, se);   
map_infl = double(map_infl);          

% cell update
[rowm2, colm2] = find(map_infl == 1); % virtual occupied cells locations



maxIterations = 400;        % Max number of iterations
distEps = 0.05;             % termination error (5cm)            
previousError=0;            %previous error initialized as zero

% trajectory buffer
trajectory = zeros(maxIterations, 3);

% init ratecontrol obj (loop time control)
r = rateControl(5);    
dt = r.DesiredPeriod;   % loop time period: dt = 1/5 Hz = 0.2s 

% -----------------------------------------------
% main control loop
% -----------------------------------------------
for i=1:n                       %loop for every target point
    IntegralError = 0;          %integral error initialized as zero before every new target
 for it=1:1:maxIterations

    % read TurtleBot's pose (x, y, theta + timestamp) 
    [xr, yr, theta, timestamp] = tbot.readPose();
    trajectory(it, :) = [xr, yr, theta];

    %Attractive force
    dx = t(i, 1) - xr;
    dy = t(i, 2) - yr;
    dGoal = sqrt(dx^2 + dy^2);            % distance between the robot and the target
    Fa = Fca * [dx, dy] / dGoal;          % [Fax, Fay]



    %Repulsive force
    Fr = [0,0];

    for k = 1:numel(rowm2)
       r = rowm2(k);                                    % row index for the obstacle
       c = colm2(k);                                    % column index for the obstacle
       %position of the center of the cell
       xij = (r - 0.5) / mscale;                        % y axis grows going right
       yij = (c - 0.5) / mscale;                        % y axis grows going down

       
       dx = xr - xij; 
       dy = yr - yij;
       d2 = dx*dx + dy*dy;                        % square of the distance between cell and the robot
       if d2 <= R^2                               %if an occupied cell is within the active radius  
        d = sqrt(d2);
        Cij = map_infl(r, c);                     %Cell occupation weight

        % repulsive contribute of the cell
        Fr = Fr + Fcr * (Cij / d2) * [dx/d, dy/d];  % Repulsive force that stacks with every occupied cell within the active radius
       end 
    end
 

    % compute the overall force 
    F = Fa + Fr;
    
    % et (xStar, yStar) -> pursuit path point
    Fn=norm(F);
    dir_hat = F / Fn;          %Unit vector of VFF(direction)                 
    L = min(cStar, dGoal);
    xStar = xr + L * dir_hat(1);
    yStar = yr + L * dir_hat(2);


    % ---------------------------------
    % PI controler 
    % ---------------------------------
    % compute pursuit distance error
    if dGoal>dStar
        Error = sqrt( (xStar - xr).^2 + (yStar - yr).^2 ) - dStar;
    else
        Error=dGoal;  % near the goal the error is the distance from it since the pure pursuit points exceeds the target
    end
    Error = max(0, Error);     %assures that the error doesn't become negative
    if Error > 0
        IntegralError = IntegralError + (Error + previousError) * dt / 2; %trapezoidal integration method
    end
    previousError = Error;  % error update for the next integration
    % linear velocity proportional-integral (PI) control
     v = kv .* Error + ki .* IntegralError;
     v = max(-v_max, min(v, v_max)); % ensures that the physical limits of the robot are not surpassed
     if dGoal <= distEps
         v=0; w=0; tbot.setVelocity(v,w); break; %if the robot reaches the target it stops and exits the cicle
     end
     if i==n
       v = v * min(1, max(0, (dGoal)/slow_radius)); %near the final target it slows down gradually
     end

    %target orientation
    thetaStar= atan2(yStar-yr, xStar-xr);

    % angular velocity (proportional control)
    w = ks* normalizeAngle(thetaStar-theta);
    w = max(-w_max, min(w, w_max)); % ensures that the physical limits of the robot are not surpassed



    % ------------------ 
    % Display   
    % ------------------   
    figure(1); clf; hold on;
    %set(gcf,'Position',[170,700,800,600])  % modify the figure position and size
    plot( rowm2./mscale, colm2./mscale,'k.')  % plot obstacles (as a collection of black dots)
    th = linspace(0, 2*pi, 200);
    xc = xr + R*cos(th);
    yc = yr + R*sin(th);
    plot(xc, yc, 'b--', 'LineWidth', 1);
    plot(trajectory(1:it,1), trajectory(1:it,2),'b')     % plot trajectory
    drawTurtleBot(xr, yr, theta);             % draw Robot
    quiver(xr,yr,Fa(1),Fa(2),'b')             % Atractive Force 
    quiver(xr,yr,Fr(1),Fr(2),'g')             % Repulsive Force 
    quiver(xr,yr,F(1),F(2),'r')               % Total Force

    plot( xStar, yStar, 'r*');  
    plot( t(i, 1), t(i, 2), 'bx');      % target
    quiver(0,0,1,0,0.5,'r')             % draw arrow for x-axis 
    quiver(0,0,0,1,0.5,'g')             % draw arrow for y-axis 
    axis([-0.1, hm/mscale, -0.1, wm/mscale]) % set limits for the current axes
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')
    title('VFF Navigation')

    % adaptive pause
    waitfor(r);

    % send velocity (linear + angular) commands
    tbot.setVelocity(v, w);
    
 end   %  main control loop end 
    traj_i = trajectory(1:it, :);              % solo righe usate
    if ~isempty(allTrajectory)
        allTrajectory = [allTrajectory; NaN NaN NaN];
    end
    allTrajectory = [allTrajectory; traj_i];
    XY = allTrajectory(:,1:2);
    mask = [true; sqrt(sum(diff(XY).^2,2)) < 0.25];   % soglia 25 cm
    XY_fix = XY;
    XY_fix(~mask,:) = NaN;
    allTrajectory(:,1:2) = XY_fix;
end
figure;
plot(allTrajectory(:,1), allTrajectory(:,2), 'b');
hold on;
plot(t(1:end-1,1), t(1:end-1,2), 'rx', 'MarkerSize', 8, 'LineWidth', 2); % obiettivi
plot(xri, yri, 'go', 'MarkerFaceColor','g');                    % primo (start)
plot(t(end,1), t(end,2), 'ro', 'MarkerFaceColor','r');          % ultimo (finale)
grid on;
hold on;
plot( rowm./mscale, colm./mscale,'k.')
xlabel('x');
ylabel('y');
title('TurtleBot3 trajectory');
legend('Trajectory', 'Waypoints', 'Start', 'End', 'Location', 'best');
% -----------------------------------------------

% stop robot 
tbot.stop()

% check statistics of past execution periods
rStats = statistics(r);



