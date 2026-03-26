
% 
% Demo - Read from TurtleBot3 Lidar
%
% Launch the virtual machine.
% Open terminal, and start the house Gazebo world: ./start-gazebo-house.sh.
%


% clear memory 
clearvars -except tbot

% start TurtleBot connection (tbot object), only if required
if ( ~exist("tbot") )
    % init by assigning the IPs directly to the TurtleBot constructor
    IP_TURTLEBOT = "192.168.21.130";         % virtual machine IP (or robot IP)
    IP_HOST_COMPUTER = "192.168.21.1";     % local machine IP
    tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER); % init TurtleBot
    % check version
    if( tbot.getVersion() < 0.9 ) error('TurtleBot v09 required'); end 
end 


% set turtlebot's 2D pose (x,y,theta) -> (-3,1,0): [default for this world]
tbot.setPose(-3,1,0);

% start by moving the robot forward
tbot.setVelocity(0.075, 0)


% -------------------------------------
%  read lidar data w/ fixed timming
% -------------------------------------

% init ratecontrol obj (enables to run a loop at a fixed frequency) 
r = rateControl(5);     % run at 5Hz

% run for 15 seconds: 75 iterations (75 x 1/5 = 15s) 
for i = 1:1:75

    % load LIDAR data from robot 
    [scanMsg, lddata, ldtimestamp] = tbot.readLidar();

    % Get in range LIDAR data indexs (valid data - obstacles).
    [vidx] = tbot.getInRangeLidarDataIdx(lddata);

    % load valid X/Y cartesian lidar readings          
    xydata = lddata.Cartesian(vidx,:);


    % Get out of range lidar data indexs (too near or too far readings).
    % [nidx] = tbot.getOutRangeLidarDataIdx(lddata);
    % load obstacle free orientations (angles)  
    % freeAng = lddata.Angles(nidx);


    % display data (using regular plots w/ cartesian data)
    figure(1); clf;
    plot(xydata(:,1), xydata(:,2), 'b.', 'MarkerSize', 8)   % draw laser readings
    axis([-4, 4, -4, 4])                % the limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')

    % display data (using ROS toolbox)
    %figure(2); clf;
    %rosPlot( scanMsg ) 

    % adaptive pause
    waitfor(r);
end

% check statistics of past execution periods
stats = statistics(r);

% stop robot 
tbot.stop()




