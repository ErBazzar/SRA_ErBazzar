% clear memory 
clearvars -except tbot

% start TurtleBot connection (tbot object), only if required
if ( ~exist("tbot") )
    % Init by assigning the IPs directly to the TurtleBot constructor
    IP_TURTLEBOT = "192.168.21.130";         % Virtual machine IP (or robot IP)
    IP_HOST_COMPUTER = "192.168.21.1";       % Local machine IP
    tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER);   % Init TurtleBot
    % Check version
    if( tbot.getVersion() < 0.9 ) error ('TurtleBot v09 required'); end 
end 

% Normalize angle in range [-pi, pi]
normalizeAngle= @(angle) atan2(sin(angle), cos(angle));

% # of max iterations
maxIterations = 500;

% Buffer for the path of the bot
Trajectory = zeros(maxIterations, 3); 

% Set TurtleBot's 2D pose (x,y,theta)
tbot.setPose(0, 0, pi);

% Target coordinates
xTarget= 3;
yTarget = 3.5;

Eps = 0.05; % Tollerance to consider that the target has been reached by the platform

% Proportional gains for velocity control
kp = 1; % Proportional linear velocity constant
ks = 1;   % Proportional angular velocity constant

% init ratecontrol obj (enables to run a loop at a fixed frequency) 
r = rateControl(5);     % run at 5Hz

for it = 1:1:maxIterations  
    
    % Read TurtleBot pose
    [x, y, theta, timestamp] = tbot.readPose();
    Trajectory(it, 1) = x;     % X axis position
    Trajectory(it, 2) = y;     % Y axis position
    Trajectory(it, 3) = theta; % Orientation

    % Distance between current bot pose and target
    distance = sqrt((x-xTarget)^2 + (y-yTarget)^2); 
    if distance <= Eps
         v=0; w=0; tbot.setVelocity(v,w); break; %if the robot reaches the target it stops and exits the cicle
    end
    % Control law (linear velocity)
    v = kp*distance;

    % Target orientation
    thetaTarget= atan2(yTarget-y, xTarget-x);
    % Control law (angular velocity)
    w = ks* normalizeAngle(thetaTarget-theta);
    

    % Display
    figure(1); clf; hold on;            
    drawTurtleBot(x, y, theta);         % Draw robot  
    plot(xTarget, yTarget, 'b*');
    plot(Trajectory(1:it,1), Trajectory(1:it,2), 'blue')
    % draw x-y axis
    quiver(0,0,1,0,'r')                 % draw arrow for x-axis 
    quiver(0,0,0,1,'g')                 % draw arrow for y-axis 
    axis([-0.5, 4, -0.5, 4])            % the limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')
    title('E2')                         % add title 
  
    % Adaptive pause
    waitfor(r);

    % Send velocity commands to bot
    tbot.setVelocity(v, w);
    
end

% check statistics of past execution periods
stats = statistics(r);

% stop robot 
tbot.stop();