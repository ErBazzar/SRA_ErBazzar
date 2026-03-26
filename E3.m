% clear memory 
clearvars -except tbot

% start TurtleBot connection (tbot object), only if required
if ( ~exist("tbot") )
    % init by assigning the IPs directly to the TurtleBot constructor
    %IP_TURTLEBOT = "192.168.21.132";         % virtual machine IP (or robot IP)
    IP_TURTLEBOT = "192.168.1.200";
    %IP_HOST_COMPUTER = "192.168.21.1";       % local machine IP
    IP_HOST_COMPUTER = "192.168.1.115"; 
    tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER);   % init TurtleBot
    % check version
    if( tbot.getVersion() < 0.9 ) error ('TurtleBot v09 required'); end 
end 

%Normalize angle in range [-pi, pi]
normalizeAngle= @(angle) atan2(sin(angle), cos(angle));

% # of max iterations
maxIterations = 1000;

%buffer for the path of the bot
Trajectory = zeros(maxIterations, 3); 

% set TurtleBot's 2D pose (x,y,theta)
tbot.setPose(2, 2, 0);

%Target coordinates
xTarget= 2;
yTarget = 2;
thetaTarget = 3*pi/2;

Eps =0.35; % Tollerance to consider the target reached by the platform
Eps_ang = 0.4; 
d_star = 0.005;
IntegralError = 0;          %integral error initialized as zero
previousError=0;            %previous error initialized as zero
IntegralError2 = 0;          %integral error initialized as zero
previousError2 = 0;            %previous error initialized as zero

%Proportional gains for velocity control
kp = 0.055;
%kp = 1;
ks = 0.7;
ki = 0.011;

krho = 0.11;
kalpha = 0.8;
kbeta = -0.2;

% init ratecontrol obj (enables to run a loop at a fixed frequency) 
r = rateControl(5);     % run at 5Hz
dt = r.DesiredPeriod;   % loop time period: dt = 1/5 Hz = 0.2s

% Parameters for trajectory
R = 1;  % Radius of the circle
f = 0.03; % Frequency
%f = 0.05;
T = 1/f;  % Period
omega = 2 * pi * f; % Angular velocity (rad/s)

for it = 1:1:maxIterations  
    
    % Read TurtleBot pose (and timestamp) 
    [x, y, theta, timestamp] = tbot.readPose();
    Trajectory(it, 1) = x;
    Trajectory(it, 2) = y;
    Trajectory(it, 3) = theta;

    % Calculate current elapsed time
    currentTime = it * dt;
    if currentTime <= T
        % Circle reference
        ref_x = 2 + R * cos(omega * currentTime);
        ref_y = 2 + R * sin(omega * currentTime);
    
        % Error between current bot pose and target point
        Error = sqrt((x-ref_x)^2 + (y-ref_y)^2) - d_star; 
        IntegralError = IntegralError + (Error + previousError) * dt / 2; %trapezoidal integration method
        previousError = Error;

        v = kp * Error + ki * IntegralError;
    
        % Target orientation
        theta_ref= atan2(ref_y-y, ref_x-x);
        % Control law (angular velocity)
        w = ks* normalizeAngle(theta_ref-theta);
    else 
        rho = sqrt((x-xTarget)^2 + (y-yTarget)^2);
        IntegralError2 = IntegralError2 + (rho + previousError2) * dt / 2; %trapezoidal integration method
        previousError2 = rho;
      
        alpha = normalizeAngle(atan2(yTarget-y, xTarget-x) - theta);
        beta = normalizeAngle(-theta - alpha + thetaTarget);
        delta_ang = abs(normalizeAngle(thetaTarget - theta));

        % Check for backward motion requirement
        if abs(alpha) > pi/2
            % Target is behind the robot, drive backwards
            v = -krho * rho - ki * IntegralError2;
            
            % Adjust alpha and beta for backward driving
            alpha = normalizeAngle(alpha - pi);
            beta = normalizeAngle(beta - pi);
        else
            % Target is in front, drive forward
            v = krho * rho + ki * IntegralError2;
        end
        
        %control law (angular velocity)
        w = kalpha * alpha + kbeta * beta;

        if rho <= Eps && delta_ang <= Eps_ang
         v=0; w=0; tbot.setVelocity(v,w); break; %if the robot reaches the target it stops and exits the cicle
        end
    end
    

    % display
    figure(1); clf; hold on;            % clear figure, hold plots
    drawTurtleBot(x, y, theta);         % draw Robot  
    plot(xTarget, yTarget, 'b*');
    plot(Trajectory(1:it,1), Trajectory(1:it,2), 'blue')
    if currentTime <= T
        plot(ref_x, ref_y, 'ro'); % Red circle for the moving targe
    end
    % draw x-y axis
    quiver(0,0,1,0,'r')                 % draw arrow for x-axis 
    quiver(0,0,0,1,'g')                 % draw arrow for y-axis 
    axis([-0.5, 4, -0.5, 4])            % the limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')
    title('E3')                         % add title 
  
    % adaptive pause
    waitfor(r);

    %send commands to bot
    tbot.setVelocity(v, w);
    
end

% check statistics of past execution periods
stats = statistics(r);

% stop robot 
tbot.stop();