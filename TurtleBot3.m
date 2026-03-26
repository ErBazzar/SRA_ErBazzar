
%
% TurtleBot3 Control Class 
% - ROS based interface w/ the TurtleBot 3 robot. 
% - Supports both the Gazebo simulator and real mobile robot. 
% 
%       Autonomous Robotic Systems 2023
%       Department of Electrical and Computer Engineering
%       University of Coimbra, Portugal 
%       Pedro Martins, pedromartins@isr.uc.pt
% 
% v09d version - 17/04/2023 - see Release Notes.txt for details.    
%

classdef TurtleBot3 < handle

    properties (Access = private)
        rinit  = false  % ROS init flag (default value = false)
        version = 0.93  % version number 
        isSimulation    % simulation flag: (1) gazebo (0) real robot
        velPub          % robot's velocity publisher 
        velMsg          % velocity message
        odomSub         % odometry subscriber 
        lidarSub        % lidar subscriber 
        gzPub           % gazebo set model state publisher
        gzSub           % gazebo set model state subscriber
        gzBotname       % robot name in gazebo
        gzMapName       % 3D map loaded in gazebo
        rstPub          % reset odometry subscriber [robot]
        imuSub          % IMU subscriber 
        battSub         % battery subscriber [robot]
        sensorstSub     % sensor_state subscriber [robot]
        poset           % 2D pose + time buffer (4D vector)
        isPoseOverwrite % pose overwrite mode flag [robot] 
        poseOffset      % pose offset value [robot] 
        wheelbaseline = 0.16    % wheels separation (distance between wheels) [meters]
        wheelradius = 0.033     % wheel radius [meters]
        ticks_per_rev = 4096    % number of ticks per revolution (encoder w/ 12 bits)
        gzObjectCounter = 0     % number of gazebo objects (counter)  
    end


    methods

    % Constructor (two main options)  
    function obj = TurtleBot3( varargin )
        % Constructor (two main options)  
        % obj = TurtleBot3();
        % obj = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER);  % init ROS connection
        %
        % -- Call constructor w/ single argument to setup sensor-state custom ROS message ---
        % -> support only in Linux64 and MacOS systems
        % obj = TurtleBot3('check-versions');                % see package versions 
        % obj = TurtleBot3('setup-path');                    % setup PATH locations  
        % obj = TurtleBot3('setup-sensors-msg');             % setup sensors-state custom message 
        % obj = TurtleBot3('check-sensors-msg');             % check if message is available
        % 
        % note: deleting the class obj will stop the robot and disconnect ROS.


        % check input args 
        if (nargin == 0)                                % no input args

            % -> REQUIRES MANUALLY SETTING THE IP ADDRESSES 
            IP_TURTLEBOT = "192.168.21.130";             % VIRTUAL MACHINE IP 
            %IP_TURTLEBOT = "192.168.1.200";            % TURTLE ROBOT IP (1st ROBOT)
            %IP_TURTLEBOT = "192.168.1.201";            % TURTLE ROBOT IP (2nd ROBOT)
            IP_HOST_COMPUTER = "192.168.1.100";         % LOCAL IP

        elseif (nargin == 1 )                           % one input args

            % switch by input argument   
            switch lower( varargin{1} )
                case 'check-versions'
                    % echo package versions
                    ! python --version  
                    ! cmake --version
                case 'setup-path'
                    setupPath();                        % run setup path function
                case 'setup-sensors-msg'
                    setupSensorStateMessage();           % run message setup function 
                case 'check-sensors-msg'
                    msgInfo = rosmsg('show','turtlebot3_msgs/SensorState')  % check message
                otherwise
                    fprintf('unknown TurtleBot3() setup option\n\n'); 
                    fprintf('- available options:\t[only in linux64 and mac64 systems]\n');
                    fprintf('  TurtleBot3(''check-versions'') -> see package versions\n');
                    fprintf('  TurtleBot3(''setup-path'') -> adds to PATH updated settings\n');
                    fprintf('  TurtleBot3(''setup-sensors-msg'') -> generates custom ROS SensorState message\n');
                    fprintf('  TurtleBot3(''check-sensors-msg'') -> check if SensorState message is installed\n');
            end

            return                                      % terminate setup mode... 

        elseif (nargin == 2 )                           % two input args
            IP_TURTLEBOT = varargin{1};                 % Assign 1st input arg
            IP_HOST_COMPUTER = varargin{2};             % Assign 2nd input arg
        else                                            % invalid input args
            error("Usage: obj = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER)");
        end 

        % -------------------
        % Init ROS 
        % -------------------
        rosinit(IP_TURTLEBOT, "NodeHost", IP_HOST_COMPUTER);

        % set ROS init flag
        obj.rinit = true; 

        % create a publisher for the /cmd_vel topic
        obj.velPub = rospublisher("/cmd_vel", "DataFormat", "struct");
        % create a velocity message 
        obj.velMsg = rosmessage(obj.velPub);


        % subscribe for the odometry messages
        obj.odomSub = rossubscriber("/odom", "DataFormat", "struct");

        % subscribe to the lidar topic
        obj.lidarSub = rossubscriber("/scan", "DataFormat", "struct");
        
        % subscribe to the imu topic
        obj.imuSub = rossubscriber("/imu", "DataFormat", "struct");

        
        % Switch between simulation / real robots ### DO NOT CHANGE THESE VALUES 
        if ( strcmp( IP_TURTLEBOT, "192.168.1.200") || strcmp( IP_TURTLEBOT, "192.168.1.201") )  
            obj.isSimulation = 0;           % clear simulation flag (real robot)
            %fprintf('real robot\n');
        else
            obj.isSimulation = 1;           % set simulation flag (gazebo)
        end  
       
        % [Gazebo]
        % -> only initialize gazebo model state topic in simulation mode 
        if ( obj.isSimulation )

            % query gazebo's objects: 
            % subscriber for the model_states messages
            obj.gzSub = rossubscriber("/gazebo/model_states", "DataFormat", "struct");
            % read data from /gazebo/model_states topic 
            gzMsg = receive(obj.gzSub, 3); 


            % list of turtlebot models reference names 
            nList = ["turtlebot3_burger", "turtlebot3"];
            % look for reference names
            idx = matches(gzMsg.Name, nList);

            if ( sum(idx) == 1 )
                % get gazebo's turtlebot name       
                obj.gzBotname = gzMsg.Name{idx};
                %fprintf("tbot name: %s\n",obj.gzBotname);
            else
                % turtlebot not found 
                error("TurtleBot3 not found in Gazeboo world."); 
            end

            % List of known Gazebo maps
            mList = ["bxmap", "csqmap", "fmap", "lcmap", "rmap", "scmap", "stmap", "umap", "xmap", "ymap"]; 
            % look for reference names (query the 3D Map)
            idx = matches(gzMsg.Name, mList);

            if ( sum(idx) == 1 )
                % get gazebo's map name       
                obj.gzMapName = gzMsg.Name{idx};
                %fprintf("map name: %s\n", obj.gzMapName);
            else
                % unknown map detected
                obj.gzMapName = 'empty';   
            end

            % create a publisher for the /gazebo/set_model_state topic
            obj.gzPub = rospublisher("/gazebo/set_model_state", "DataFormat", "struct");
            % create a /gazebo/set_model_state message 
            %obj.gzStateMsg = rosmessage(obj.gzPub);    % removed from private data

        % [Real TurtleBot]
        % -> init reset publisher and battery state subscriber
        else  
            % create a publisher for the /reset topic (reset odometry)
            obj.rstPub = rospublisher("/reset", "DataFormat", "struct"); 
            
            % subscriber for battery state messages
            % obj.battSub = rossubscriber("/battery_state","DataFormat","struct");
            % -> dummy init (moved to getBatteryLevel() function) 
            obj.battSub = 0;

            % subscriber for sensor state messages (Left Encoder/Right Encoder))
            %obj.sensorstSub = rossubscriber("/sensor_state","DataFormat","struct");
            % -> dummy init (moved to readEncodersTicks() function) 
            obj.sensorstSub = 0; 

            % assign gazebo map to invalid   
            obj.gzMapName = 'none'; 
        end 

        % -----------------------------
        % Generic settings 
        % -----------------------------

        % init 2D pose (x,y,theta) + time vector
        obj.poset = zeros(4,1);

        % init Pose overwrite flag [used only in real TurtleBot]
        obj.isPoseOverwrite = false;

        % init poseOffset vector [real TurtleBot - pose overwrite mode]
        obj.poseOffset = [0, 0, 0]';

    end     % end constructor 

    

    % get version number
    function vs = getVersion(obj)
        % Get the version number.
        %
        % vs = obj.getVersion();
        % 
        % - returns a scalar value.  

        vs = obj.version; 
    end


    % get wheel baseline (distance between wheels) [in meters]
    function b = getWheelBaseline(obj)
        % Get the wheel baseline (distance between wheels) [in meters].
        %
        % b = obj.getWheelBaseline();
        % 
        % - returns a scalar value. 

        b = obj.wheelbaseline; 
    end


    % get wheel radius [in meters]
    function r = getWheelRadius(obj)
        % Get the wheel radius [in meters].
        %
        % r = obj.getWheelRadius();
        % 
        % - returns a scalar value.  

        r = obj.wheelradius; 
    end


    % get the 3D Gazebo Map basename
    function mapName = getGazeboMapName(obj)
        % Get the 3D Gazebo Map basename (map loaded at simulator startup).
        %
        % mapName = obj.getGazeboMapName();
        %
        % - returns a string w/ the map name. 
        % - supported maps: empty, bxmap, csqmap, fmap, lcmap, rmap, scmap, stmap, umap, xmap and ymap.
        %
        % note: 
        % -> returns 'none' when connected to the real TurtleBot robot. 
        % 
        % Tip (usage example): Read the map image (w/ grid cell size 5x5):       
        %   image = imread( sprintf('maps/%s_grid5.png', obj.getGazeboMapName()) ); 

        % return MapName
        mapName = obj.gzMapName; 
    end


    % get number of encoder ticks per revolution
    function tk = getNumberTicksPerRevolution(obj)
        % Get the number of encoder ticks per revolution.
        %
        % tk = obj.getNumberTicksPerRevolution();
        % 
        % - returns a scalar value. 

        tk = obj.ticks_per_rev; 
    end


    % real/fake TurtleBot flag
    function flag = isRealRobot(obj)
        % Check if a real TurtleBot is connected.
        %
        % flag = obj.isRealRobot();
        % 
        % Real/fake TurtleBot flag: 
        % - return true if TurtleBot is connected to a real robot.
        % - return false otherwise (when connected to the Gazebo simulator).

        if( ~obj.isSimulation ) 
            flag = true; 
        else 
            flag = false;
        end 
    end


    % get Battery Level (in percentage: [0, 1])
    function percentage = getBatteryLevel(obj)
        % Get battery level (in percentage: [0, 1])
        %
        % percentage = obj.getBatteryLevel();
        % 
        % return values:
        % - current battery level in percentage [0, 1]
        % - if no battery present -> return 0
        % - if virtual TurtleBot -> return 1 
        % note: 
        % -> subscribes to /battery_state topic on the first run      


        % [Turtlebot]
        if( ~obj.isSimulation )

            % init ROS subscriber, if required
            if( ~isobject(obj.battSub) )
                % subscriber for battery state messages
                obj.battSub = rossubscriber("/battery_state","DataFormat","struct");
            end

            % read data from /battery_state topic 
            battMsg = receive(obj.battSub, 3);          % read data w/ 3s timeout 

            % check if battery is present
            if( battMsg.Present )
                percentage = battMsg.Percentage;        % return battery percentage
            else
                percentage = 0;                         % no battery present
            end 

        % [Gazebo]
        else
            percentage = 1.0;                           % full 'virtual' battery
        end  
    end


    % Send linear and angular velocity commands. 
    function setVelocity(obj, v, w)
        % Send linear (v) and angular (w) velocity commands.
        % 
        % obj.setVelocity(v, w);
        % 
        % (v,w) are scalars with units in [m/s] and [rads/s], respectively.
        %
        % notes:
        % -> negative linear velocity values move the robot backwards.
        % -> negative angular velocity values rotate the robot clockwise.     
        % -> both velocity values are limited to a maximum [vmax = 0.22 m/s, wmax = 2.84 rad/s].


        % clip (saturate) velocity to maximum values [see turtlebot manual]
        if( abs(v) > 0.22) v = sign(v) * 0.22; end      % [max v = 0.22 m/s]
        if( abs(w) > 2.84) w = sign(w) * 2.84; end      % [max w = 2.84 rad/s]

        % generate ROS velocity message
        obj.velMsg.Linear.X = v(1);
        obj.velMsg.Linear.Y = 0;
        obj.velMsg.Linear.Z = 0;
        obj.velMsg.Angular.X = 0;
        obj.velMsg.Angular.Y = 0;
        obj.velMsg.Angular.Z = w(1);

        % send ROS velocity message
        send(obj.velPub, obj.velMsg)
    end


    % Stop robot - send (zero) velocity commands
    function stop(obj)
        % Stop robot. Send (v = 0, w = 0) velocity commands.
        %
        % obj.stop();
        %


        obj.setVelocity( 0, 0);
    end


    % Reads the 2D pose 
    function [x, y, theta, ptimestamp] = readPose(obj)
        % Reads the 2D pose of the robot.
        %
        % [x, y, theta, ptimestamp] = obj.readPose();
        %  
        % returns: 
        % - (x, y) positon [in meters].
        % - (theta) orientation [rad].
        % - ptimestamp reading timestamp (in seconds).


        % read data from odom topic 
        odomMsg = receive(obj.odomSub, 3);      % read data w/ 3s timeout 
        pose = odomMsg.Pose.Pose;
        x = pose.Position.X;
        y = pose.Position.Y;
        %z = pose.Position.Z;
        
        % read pose (returns a quaternion)
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);      
        % angles in Euler format: ZYX
        %theta = rad2deg( angles(1) ); 
        theta = angles(1);                      % return theta in radians 

        % return pose reading timestamp (ROS time converted in seconds) 
        ptimestamp = double(odomMsg.Header.Stamp.Sec) + double(odomMsg.Header.Stamp.Nsec) * 1e-9;

        
        % Pose overwrite mode [real robot]
        if( obj.isPoseOverwrite )
            % fprintf('offset read\n');
            x = x + obj.poseOffset(1);
            y = y + obj.poseOffset(2);
            theta = atan2( sin(theta), cos(theta) );        % normalize theta [-pi, pi]  
            theta = atan2( sin(theta + obj.poseOffset(3)), cos(theta + obj.poseOffset(3)) );    % add angles
        end 

        % hold internal copy of 2D pose + timestap 
        obj.poset = [x; y; theta; ptimestamp]; 
    end


    % get 2D pose (by calling readPose(.) function)
    function [x, y, theta, ptimestamp] = getPose(obj)
        % Reads the 2D pose of the robot.
        %
        % [x, y, theta, ptimestamp] = obj.getPose();
        %  
        % returns: 
        % - (x, y) positon [in meters].
        % - (theta) orientation [rad].
        % - ptimestamp reading timestamp (in seconds).


        % call function readPose()
        [x, y, theta, ptimestamp] = obj.readPose(); 
    end


    % Set 2D pose to (x,y,theta)
    function setPose(obj, x, y, theta)
        % Sets (overwrites) the 2D pose to (x, y, theta). 
        %
        % obj.setPose(x, y, theta);
        % 
        % - (x, y) positon [in meters].
        % - (theta) orientation [rad].
        %
        % note: 
        % -> Supported in both the Gazebo simulator and the real TurtleBot robot. 
        

        % [Gazebo]  
        if( obj.isSimulation )
            
            obj.stop()                      % stop robot 
            pause(0.5);                     % force delay

            % create a /gazebo/set_model_state message 
            gzStateMsg = rosmessage(obj.gzPub);

            % set model name
            gzStateMsg.ModelName = obj.gzBotname;
            % set position 
            gzStateMsg.Pose.Position.X = x;
            gzStateMsg.Pose.Position.Y = y;
            %gzStateMsg.Pose.Position.Z = 0;
            
            % convert Euler angles (ZYX) to quaternion qt=[W,X,Y,Z]
            qt = eul2quat( [theta, 0, 0] );
            % set orientation 
            gzStateMsg.Pose.Orientation.X = qt(2);
            gzStateMsg.Pose.Orientation.Y = qt(3);
            gzStateMsg.Pose.Orientation.Z = qt(4);
            gzStateMsg.Pose.Orientation.W = qt(1);

            % send gz state message
            send(obj.gzPub, gzStateMsg);

            % disable pose overwrite mode (enforce this setting)
            obj.isPoseOverwrite = false;

        % [Turtlebot]
        else
            %fprintf("setting pose is only available in simulation mode\n");
            %fprintf("overwriting pose in (real) TurtleBot\n");

            % set pose offset 
            obj.poseOffset = [x, y, atan2( sin(theta), cos(theta) )]';

            % reset pose (also disables pose overwrite mode)
            obj.resetPose();

            % enable pose overwrite mode
            obj.isPoseOverwrite = true;
            
        end 
    end 


    % Reset turtlebot's pose to origin: (x,y,theta) = (0,0,0) 
    function resetPose(obj)
        % Resets the 2D pose to the origin (x = 0, y = 0, theta = 0).
        %
        % obj.resetPose();
        %
        % note: 
        % -> Supported in both the Gazebo simulator and real TurtleBot robot. 


        % [Gazebo]  
        if( obj.isSimulation )

            % set pose to origin (forward oriented to x axis)
            obj.setPose(0.0, 0.0, 0.0);       

        % [Turtlebot]
        else

            % disable pose overwrite mode
            obj.isPoseOverwrite = false;

            % create an empty message
            rstMsg = rosmessage("std_msgs/Empty", "DataFormat", "struct");  

            % send empty message to /reset topic 
            send(obj.rstPub, rstMsg);
        end
    end 


    % Initialize Encoders module
    % - required to startup the simulated encoder data 
    function initEncoders(obj)
        % Initialize the encoders (simulated) module.
        % 
        % obj.initEncoders();
        %
        % notes: 
        % -> required to startup the simulated encoder data module. 
        % -> used in conjunction with obj.readEncoders(.) and obj.readEncodersWithNoise(.)


        % read pose once (update poset buffer)
        [x, y, theta, timestamp] = obj.readPose();
    end


    % Get (simulated) Encoders Data  
    function [dsr, dsl, pose2D, timestamp] = readEncoders(obj)
        % Get (simulated) encoders data.
        %
        % [dsr, dsl, pose2D, timestamp] = obj.readEncoders();
        %
        % returns: 
        %  - (dsr) incremental motion of the right wheel [in meters].
        %  - (dsl) incremental motion of the left wheel [in meters].
        %  - (pose2D) [x,y,theta] 2D pose given by the TurtleBot's odometry model.
        %  - (timestamp) data reading timestamp [seconds].
        %
        % notes:
        % -> Must be initialized by obj.initEncoders() function.
        % -> The readEncoders(), getEncodersData() and readPose() can't be used together in a loop. 


        % kinematic eqs: 
        % x1 = x + ds * cos(theta + dtheta/2)
        % y1 = y + ds * sin(theta + dtheta/2) 
        % theta1 = theta + dtheta
        %
        % solve for dtheta and ds: 
        %: dtheta = theta1 - theta
        %: eq1 + eq2 -> ds = (x1 - x + y1 - y) / ( cos(theta + dtheta/2) + sin(theta + dtheta/2) )   

        % load previous pose data
        prev_pose = obj.poset;

        % read current pose 
        [x, y, theta, timestamp] = obj.readPose();

        % return pose (given by the internal turtlebot's odometry model) 
        pose2D = [x; y; theta];

        % timestamp difference lower than default message timeout
        if( timestamp - prev_pose(4) < 3  )

            % return amount of angular motion: subtract angles: dtheta = theta - prev_pose(3)
            dtheta = atan2( sin(theta - prev_pose(3)), cos(theta - prev_pose(3)) );

            % return incremental linear motion
            ds = (x - prev_pose(1) + y - prev_pose(2) ) / ( cos( prev_pose(3) + dtheta/2 ) + sin( prev_pose(3) + dtheta/2) ); 
            
        else 
            ds = 0;
            dtheta = 0; 
        end 

        % query wheel baseline (b = 0.16)  
        b = obj.wheelbaseline;

        % solve linear system of equations for (dsr and dsl) 
        % ds = ( dsr + dsl) / 2
        % dtheta = ( dsr - dsl) / b 
        e = inv( [1/2, 1/2; 1/b, -1/b]) * [ds; dtheta];

        % return the solutions
        dsr = e(1);
        dsl = e(2);
    end 


    % Get (simulated) Encoders Data (function overload) 
    function [dsr, dsl, pose2D, timestamp] = getEncodersData(obj)
        % Get (simulated) encoders data.
        %
        % [dsr, dsl, pose2D, timestamp] = obj.getEncodersData();
        %
        % returns: 
        %  - (dsr) incremental motion of the right wheel [in meters].
        %  - (dsl) incremental motion of the left wheel [in meters].
        %  - (pose2D) [x,y,theta] 2D pose given by the TurtleBot's odometry model.
        %  - (timestamp) data reading timestamp [seconds].
        %
        % notes:
        % -> Must be initialized by obj.initEncoders() function.
        % -> The readEncoders(), getEncodersData() and readPose() can't be used together in a loop. 


        % call readEncoders(.)
        [dsr, dsl, pose2D, timestamp] = obj.readEncoders();
    end 


    % Get (simulated) Encoders Data with Gaussian Noise
    function [dsr, dsl, pose2D, timestamp] = readEncodersWithNoise(obj, noise_std)
        % Get (simulated) encoders data with Gaussian noise.
        %
        % [dsr, dsl, pose2D, timestamp] = obj.readEncodersWithNoise(noise_std);
        %
        % receives: 
        %  - (noise_std) 2 x 1 vector w/ noise standard deviations of dsr and dsl (in meters).
        %     if noise_std is a scalar value, the same amount of noise is applied in both measures (dsl and dsl). 
        % returns:   
        %  - (dsr) incremental motion of the right wheel (in meters)
        %  - (dsl) incremental motion of the left wheel (in meters)
        %  - (pose2D) [x,y,theta] 2D pose given by the turtlebot's odometry model (3D vector)
        %  - (timestamp) data reading timestamp (in seconds)
        % note: 
        % -> the readEncodersWithNoise(), getEncodersDataWithNoise() and readPose() can't be used together in a loop.


         % check inputs 
        if( nargin < 2 ) noise_std = zeros(2,1); end
        if( isscalar(noise_std) ) noise_std = [noise_std; noise_std]; end     % expand noise_std

        % read encoders data
        [dsr, dsl, pose2D, timestamp] = obj.readEncoders();

        % add Gaussian noise (w/ zero mean) to encoder data
        dsr = dsr + randn(1,1) * noise_std(1); 
        dsl = dsl + randn(1,1) * noise_std(2);
    end


    % Get (simulated) Encoders Data with Gaussian Noise (function overload) 
    function [dsr, dsl, pose2D, timestamp] = getEncodersDataWithNoise(obj, noise_std)
        % Get (simulated) encoders data with Gaussian noise.
        %
        % [dsr, dsl, pose2D, timestamp] = obj.getEncodersDataWithNoise(noise_std);
        %
        % receives: 
        %  - (noise_std) 2 x 1 vector w/ noise standard deviations of dsr and dsl (in meters).
        %     if noise_std is a scalar value, the same amount of noise is applied in both measures (dsl and dsl). 
        % returns:   
        %  - (dsr) incremental motion of the right wheel (in meters)
        %  - (dsl) incremental motion of the left wheel (in meters)
        %  - (pose2D) [x,y,theta] 2D pose given by the turtlebot's odometry model (3D vector)
        %  - (timestamp) data reading timestamp (in seconds)
        % note: 
        % -> the readEncodersWithNoise(), getEncodersDataWithNoise() and readPose() can't be used together in a loop.


        % call readEncodersWithNoise(.)
        [dsr, dsl, pose2D, timestamp] = obj.readEncodersWithNoise(noise_std);
    end


    % Read (real) Encoders Ticks - only available w/ connected to the Real Robot
    function [left_ticks, right_ticks, timestamp] = readEncodersTicks(obj)
        % Read (real) encoders ticks. 
        %
        % [left_ticks, right_ticks, timestamp] = obj.readEncodersTicks();
        % 
        % returns: 
        % - (left_ticks,right_ticks) left and right encoder (absolute) ticks.
        % - (timestamp) timestamp (in seconds).
        %
        % notes: 
        % -> only available w/ connected to the real TurtleBot.
        % -> requires to setup custom ROS message (SensorState.msg).
        % -> subscribes to /sensor_state topic on the first run.


        % [Robot]
        if( ~obj.isSimulation )

            % init ROS subscriber, if required
            if( ~isobject(obj.sensorstSub) )
                % Requires building ROS custom messages
                % subscriber for sensor state messages (Left Encoder/Right Encoder))
                obj.sensorstSub = rossubscriber("/sensor_state", "DataFormat", "struct");
            end
            
            % read data from sensor_state topic 
            sensorstMsg = receive(obj.sensorstSub, 3);      % read data w/ 3s timeout 

            left_ticks = double( sensorstMsg.LeftEncoder );
            right_ticks = double( sensorstMsg.RightEncoder );

            % return timestamp (ROS time converted in seconds) 
            timestamp = double(sensorstMsg.Header.Stamp.Sec) + double(sensorstMsg.Header.Stamp.Nsec) * 1e-9;

        % [Simulation]
        else
            error('readEncodersTicks() function, only supported in real TurtleBot robot'); 
        end 
    end


    % Read (real) Encoders Ticks - only available w/ connected to the Real Robot (function overload) 
    function [left_ticks, right_ticks, timestamp] = getEncodersDataTicks(obj)
        % Read (real) encoders ticks. 
        %
        % [left_ticks, right_ticks, timestamp] = obj.getEncodersDataTicks();
        % 
        % returns: 
        % - (left_ticks,right_ticks) left and right encoder (absolute) ticks.
        % - (timestamp) timestamp (in seconds).
        %
        % notes: 
        % -> only available w/ connected to the real TurtleBot.
        % -> requires to setup custom ROS message (SensorState.msg).
        % -> subscribes to /sensor_state topic on the first run.
        

        % call readEncodersTicks(.)
        [left_ticks, right_ticks, timestamp] = obj.readEncodersTicks();
    end


    % Read lidar data
    function [scanMsg, lddata, ldtimestamp] = readLidar(obj)
        % Read lidar data
        %
        % [scanMsg, lddata, ldtimestamp] = obj.readLidar();
        %
        % returns: 
        % - (scanMsg) laser obj struct (ROS message)
        % - (lddata) lidar data struct
        %           - lddata.Ranges - (radial/polar) distance measurement [meters] (360 x 1) vector
        %           - lddata.Angles - angular measurement [radians] (360 x 1) vector
        %           - lddata.Cartesian - X/Y cartesian data (360 x 2) matrix
        % - (ldtimestamp) laser scan reading timestamp (in seconds)
        % 
        % notes:
        % -> All measurements are given in w.r.t. the LIDAR sensor (placed at top of the robot). 
        % -> The lsdata (struct field) arrays could have 'Inf' (or zero) values to represent no laser reflections (representing too near or too far readings). 
        % -> Use getInRangeLidarDataIdx(.) and getOutRangeLidarDataIdx(.) functions to select the desired data. 
        % -> rosPlot(scanMsg) can be used to display. 


        % read data from lidar topic 
        scanMsg = receive(obj.lidarSub, 3);     % read data w/ 3s timeout 

        % return lidar internal data (Ranges, Angles and Cartesian data)
        lddata = rosReadLidarScan(scanMsg);

        % return laser scan reading time (ROS time converted in seconds) 
        ldtimestamp = double(scanMsg.Header.Stamp.Sec) + double(scanMsg.Header.Stamp.Nsec) * 1e-9; 
    end


    % Read LIDAR data - NEW FORMAT 
    function [lddata, ldtimestamp, scMsg] = readLidar__(obj)
        % Read LIDAR data (NEW FORMAT - CHECK THE ORDER OF THE RETURN VALUES) 
        %
        % [lddata, ldtimestamp, scMsg] = obj.readLidar__();
        %
        % returns: 
        % - (lddata) lidar data struct
        %           - lddata.Ranges - (radial/polar) distance measurement [meters] (360 x 1) vector
        %           - lddata.Angles - angular measurement [radians] (360 x 1) vector
        %           - lddata.Cartesian - X/Y cartesian data (360 x 2) matrix
        % - (ldtimestamp) laser scan reading timestamp (in seconds)
        % - (scanMsg) laser obj struct (ROS message)
        % 
        % notes: 
        % -> The zreadLIDAR(.) function has different outputs than readLidar() / getLidarData(.).
        % -> All measurements are given in w.r.t. the LIDAR sensor (placed at top of the robot).
        % -> The lsdata (struct field) arrays could have 'Inf' (or zero) values to represent no laser reflections (representing too near or too far readings). 
        % -> Use getInRangeLidarDataIdx(.) and getOutRangeLidarDataIdx(.) functions to select the desired data. 
        % -> rosPlot(scanMsg) can be used to display. 


        % call readLidar(.)
        [scMsg, lddata, ldtimestamp] = obj.readLidar();
    end 


    % Overload of Read lidar data - calls readLidar(.) function
    function [scanMsg, lddata, ldtimestamp] = getLidarData(obj)
        % Read lidar data
        %
        % [scanMsg, lddata, ldtimestamp] = obj.getLidar();
        %
        % returns: 
        % - (scanMsg) laser obj struct (ROS message)
        % - (lddata) lidar data struct
        %           - lddata.Ranges - (radial/polar) distance measurement [meters] (360 x 1) vector
        %           - lddata.Angles - angular measurement [radians] (360 x 1) vector
        %           - lddata.Cartesian - X/Y cartesian data (360 x 2) matrix
        % - (ldtimestamp) laser scan reading timestamp (in seconds)
        % 
        % notes: 
        % -> All measurements are given in w.r.t. the LIDAR sensor (placed at top of the robot).
        % -> The lsdata (struct field) arrays could have 'Inf' (or zero) values to represent no laser reflections (representing too near or too far readings). 
        % -> Use getInRangeLidarDataIdx(.) and getOutRangeLidarDataIdx(.) functions to select the desired data. 
        % -> rosPlot(scanMsg) can be used to display. 


        % call readLidar(.)
        [scanMsg, lddata, ldtimestamp] = obj.readLidar();
    end 


    % Get in range lidar data indexs (valid data - obstacles).
    function [vidx] = getInRangeLidarDataIdx(obj, lddata, maxRange)
        % Get in range lidar data indexs (i.e. valid lidar data - obstacles).
        %
        % [vidx] = obj.getInRangeLidarDataIdx(lddata, maxRange = 3.5);
        %
        % maxRange - is the maximum admitted LIDAR range value (absolute max value = 3.5) [meters].  
        % Returns the linear indexs of several related arrays: lsdata.Ranges(vidx), or lsdata.Angles(vidx) or lddata.Cartesian(vidx,:).
        %
        % Tip (usage): see demoLidar.m


        % check inputs
        if(nargin < 3) maxRange = 3.5; end 

        %Define lidar min and max laser ranges (see: scanMsg.RangeMin and scanMsg.RangeMax)
        minRange = 0.12; 
        % maxRange = 3.5; 

        % return lsdata.'DATATYPE' indexs of laser redings within range (valid obstacles)  
        vidx = find( ~isinf( lddata.Ranges ) & lddata.Ranges >= minRange & lddata.Ranges <= maxRange);  
    end 



    % Get the out of range lidar data indexs (too near or too far readings).
    function [nidx] = getOutRangeLidarDataIdx(obj, lddata, maxRange)
        % Get the out of range lidar data indexs (too near or too far readings).
        %
        % [nidx] = obj.getOutRangeLidarDataIdx(lddata, maxRange = 3.5);
        %
        % maxRange - is the maximum admitted LIDAR range value (absolute max value = 3.5) [meters].  
        % Returns the linear indexs of lsdata.Angles(nidx).
        %
        % Tip (usage): see demoLidar.m


        % check inputs
        if(nargin < 3) maxRange = 3.5; end
        
        %Define lidar min and max laser ranges (see: scanMsg.RangeMin and scanMsg.RangeMax)
        minRange = 0.12; 
        % maxRange = 3.5; 

        % return lsdata.'DATATYPE' indexs of out of range readings (too near or too far readings)
        nidx = find( isinf( lddata.Ranges ) | lddata.Ranges < minRange  | lddata.Ranges > maxRange );
    end 


    % Read Inertial Measurement Unit (IMU) data
    function [imuMsg, a, w, angles] = readIMU(obj)
        % Read Inertial Measurement Unit (IMU) data.
        %
        % [imuMsg, a, w, angles] = obj.readIMU();
        %
        % returns: 
        % - (imuMsg) IMU message (ROS message)
        % - (a) Linear acceleration (3x1) vector 
        % - (w) Angular velocity (3x1) vector 
        % - (angles) 3D orientation estimate given by the IMU sensor [in Euler angles format (ZYX)]


        % read data from IMU topic 
        imuMsg = receive(obj.imuSub, 3);        % read data w/ 3s timeout 

        % return linear acceleration
        a = [imuMsg.LinearAcceleration.X, imuMsg.LinearAcceleration.Y, imuMsg.LinearAcceleration.Z]';

        % return angular velocity
        w = [imuMsg.AngularVelocity.X, imuMsg.AngularVelocity.Y, imuMsg.AngularVelocity.Z]';
        

        % 3D orientation
        % read pose from IMU (quaternion format)
        quat = imuMsg.Orientation;

        % return pose in Euler angles (ZYX)
        angles = quat2eul([quat.W quat.X quat.Y quat.Z])'; 
    end 


    % Read Inertial Measurement Unit (IMU) data
    % - overload of readIMU()
    function [imuMsg, a, w, angles] = getIMUData(obj)
        % Read Inertial Measurement Unit (IMU) data.
        %
        % [imuMsg, a, w, angles] = obj.getIMUData();
        %
        % returns: 
        % - (imuMsg) IMU message (ROS message)
        % - (a) Linear acceleration (3x1) vector 
        % - (w) Angular velocity (3x1) vector 
        % - (angles) 3D orientation estimate given by the IMU sensor [in Euler angles format (ZYX)]


        % call readIMU(.)
        [imuMsg, a, w, angles] = obj.readIMU();
    end 


    % Read Compass data (provided by the Inertial Measurement Unit)
    function [phi, angles] = readCompass(obj)
        % Read Compass data (provided by the Inertial Measurement Unit).
        %
        % [phi, angles] = obj.readCompassData();
        %
        % returns:  
        % - (phi) angle [rad]
        % - (angles) 3D orientation estimate given by the IMU sensor [in Euler angles format (ZYX)]


        % Read Inertial Measurement Unit (IMU) data
        [imuMsg, a, w, angles] = obj.readIMU();

        % get z orientation
        phi = angles(1);
    end 


    % Read Compass data (provided by the Inertial Measurement Unit)
    % - overload of readCompassData()
    function [phi, angles] = getCompassData(obj)
        % Read Compass data (provided by the Inertial Measurement Unit).
        %
        % [phi, angles] = obj.getCompassData();
        %
        % returns:  
        % - (phi) angle [rad]
        % - (angles) 3D orientation estimate given by the IMU sensor [in Euler angles format (ZYX)]


        % call readCompassData(.)
        [phi, angles] = obj.readCompass();
    end


    % Place 3D Cardboard Box in Gazebo simulator
    function gazeboPlace3DCardboardBox(obj, x, y, orientation, color)
        % Place 3D Cardboard Box in Gazebo simulator
        % 
        % obj.gazeboPlace3DCardboardBox(x, y, orientation);
        %  
        % - (x, y) placement position in the simulator world [in meters].
        % - (orientation) placement orientation [rad].
        % - (color) display color [red, green, blue] - diffuse material color [3D vector]. 
        %           Default color = [0.99, 0.6, 0.2]. Color char codes suported: 'r' red, 'g' green, 
        %           'b' blue, 'm' magenta, 'c' cyan, 'y' yellow, 'w' white and 'k' black. 
        % 
        % note: 
        % -> The cardboard box size: 33 x 22 x 30 (length x width x height) (cm).


        % check input parameters 
        if (nargin < 5) color = [0.99, 0.6, 0.2]; end 
        if (nargin < 4) orientation = 0; end 
        if (nargin < 3) error('gazeboPlace3DCardboardBox(.) check inputs'); end  

        % decode color by MatLab char code   
        color = decodeColor(color);

        % [Gazebo]
        if( obj.isSimulation )

            % connect to ROS service 
            client = rossvcclient("/gazebo/spawn_sdf_model","DataFormat","struct");

            % create ROS message 
            req = rosmessage(client);
            
            % set req message 
            req.ModelName = sprintf('CardboardBox_%d', obj.gzObjectCounter);
            req.InitialPose.Position.X = x;
            req.InitialPose.Position.Y = y;
            req.InitialPose.Position.Z = 0;
            
            % convert Euler angles (ZYX) to quaternion qt=[W,X,Y,Z]
            qt = eul2quat( [orientation, 0, 0] );

            % set orientation 
            req.InitialPose.Orientation.X = qt(2);
            req.InitialPose.Orientation.Y = qt(3);
            req.InitialPose.Orientation.Z = qt(4);
            req.InitialPose.Orientation.W = qt(1);

            % open CardboardBox sdf file (v1) 
            %fid = fopen('maps/gzb_models/omodels/box/model.sdf');
            %data = fread(fid);
            %fclose(fid);

            % open CardboardBox sdf file  ---------------
            fid = fopen('maps/gzb_models/omodels/box/model.sdf');

            % file data bufffer 
            fdata = [];

            % read 1st line 
            tline = fgetl(fid); 
            while ischar(tline)
                % look for <diffuse> tag 
                if( contains(tline,'<diffuse>') ) 
                    % modify color parameters 
                    tline = sprintf('<diffuse>%g %g %g 1</diffuse>\n', color(1), color(2), color(3)); 
                    %disp(tline)  
                end 
                % add to buffer 
                fdata = [fdata, tline];
            
                %disp(tline)            % show line 
                tline = fgets(fid);     % read line by line (w/ newline)
            end
            % close file ------------------------
            fclose(fid);


            % copy file contents in to ModelXml data (string format)
            %req.ModelXml = char(data);  % v1
            req.ModelXml = fdata;

            % send request 
            resp = call(client, req, "Timeout", 3);

            % show error message in case of error 
            if( ~ resp.Success) error(resp.StatusMessage); end 

            % update gazebo object counter 
            obj.gzObjectCounter = obj.gzObjectCounter + 1; 
            %fprintf('gzObjectCounter %d \n', obj.gzObjectCounter);


        % [real TurtleBot]
        else
            error('Adding 3D objects, is only supported when connected to the Gazebo simulator'); 
        end 
    
    end 


    % Place 3D Cube in Gazebo simulator
    function gazeboPlace3DCube(obj, x, y, orientation, edgeSize, color)
        % Place 3D Cube in Gazebo simulator
        % 
        % obj.gazeboPlace3DCube(x, y, orientation = 0, edgeSize = 1, color = [r,g,b]);
        %  
        % - (x, y) placement position in the simulator world [in meters].
        % - (orientation) placement orientation [rad]. Default orientation = 0. 
        % - (edgeSize) cube edge length [in meters]. Default edgeSize = 1. 
        % - (color) display color [red, green, blue] - diffuse material color [3D vector]. 
        %           Default color = [0.99, 0.6, 0.2]. Color char codes suported: 'r' red, 'g' green, 
        %           'b' blue, 'm' magenta, 'c' cyan, 'y' yellow, 'w' white and 'k' black. 


        % check input parameters 
        if (nargin < 6) color = [0.99, 0.6, 0.2]; end 
        if (nargin < 5) edgeSize = 1; end 
        if (nargin < 4) orientation = 0; end 
        if (nargin < 3) error('gazeboPlace3DCube(.) check inputs'); end  
        if (edgeSize < 0) edgeSize = 1; end 
        
        % decode color by MatLab char code   
        color = decodeColor(color);
        
        % [Gazebo]
        if( obj.isSimulation )

            % connect to ROS service 
            client = rossvcclient("/gazebo/spawn_sdf_model","DataFormat","struct");

            % create ROS message 
            req = rosmessage(client);
            
            % set req message 
            req.ModelName = sprintf('Cube_%d', obj.gzObjectCounter);
            req.InitialPose.Position.X = x;
            req.InitialPose.Position.Y = y;
            req.InitialPose.Position.Z = edgeSize / 2;  % <- place at correct height
            
            % convert Euler angles (ZYX) to quaternion qt=[W,X,Y,Z]
            qt = eul2quat( [orientation, 0, 0] );

            % set orientation 
            req.InitialPose.Orientation.X = qt(2);
            req.InitialPose.Orientation.Y = qt(3);
            req.InitialPose.Orientation.Z = qt(4);
            req.InitialPose.Orientation.W = qt(1);

            % open Cube sdf file  ---------------
            fid = fopen('maps/gzb_models/omodels/cube/model.sdf');  

            % file data bufffer 
            fdata = [];

            % read 1st line 
            tline = fgetl(fid); 
            while ischar(tline)
                % look for <size> tag 
                if( contains(tline,'<size>') ) 
                    % modify size parameters 
                    tline = sprintf('<size>%g %g %g</size>\n', edgeSize, edgeSize, edgeSize); 
                    %disp(tline)  
                end
                % look for <diffuse> tag 
                if( contains(tline,'<diffuse>') ) 
                    % modify color parameters 
                    tline = sprintf('<diffuse>%g %g %g 1</diffuse>\n', color(1), color(2), color(3)); 
                    %disp(tline)  
                end 
                % add to buffer 
                fdata = [fdata, tline];
            
                %disp(tline)             % show line 
                tline = fgets(fid);     % read line by line (w/ newline)
            end
            % close file ------------------------
            fclose(fid);


            % copy file contents in to ModelXml data (string format)
            req.ModelXml = fdata;
            
            % send request 
            resp = call(client, req, "Timeout", 3);

            % show error message in case of error 
            if( ~ resp.Success) error(resp.StatusMessage); end 

            % update gazebo object counter 
            obj.gzObjectCounter = obj.gzObjectCounter + 1; 
            %fprintf('gzObjectCounter %d \n', obj.gzObjectCounter);


        % [real TurtleBot]
        else
            error('Adding 3D objects, is only supported when connected to the Gazebo simulator'); 
        end 
    
    end 


    % Place 3D Cylinder in Gazebo simulator
    function gazeboPlace3DCylinder(obj, x, y, radius, length, color)
        % Place 3D Cylinder in Gazebo simulator
        % 
        % obj.gazeboPlace3DCylinder(x, y, radius = 0.5, length = 1, color = [r,g,b]);
        % 
        % - (x, y) placement position in the simulator world [in meters].
        % - (radius) Cylinder radius [meters]. Default radius = 0.5. 
        % - (length) Cylinder length [meters]. Default length = 1.
        % - (color) display color [red, green, blue] - diffuse material color [3D vector]. 
        %           Default color = [0.99, 0.6, 0.2]. Color char codes suported: 'r' red, 'g' green, 
        %           'b' blue, 'm' magenta, 'c' cyan, 'y' yellow, 'w' white and 'k' black. 
        %
        % note: 
        % -> the cylinder is placed in vertical. 
        

        % check input parameters 
        if (nargin < 6) color = [0.99, 0.6, 0.2]; end 
        if (nargin < 5) length = 1; end 
        if (nargin < 4) radius = 0.5; end 
        if (nargin < 3) error('gazeboPlace3DCylinder(.) check inputs'); end  
        if (radius <= 0) radius = 0.5; end 
        if (length <= 0) length = 1; end 
        
        % decode color by MatLab char code   
        color = decodeColor(color);
        
        % [Gazebo]
        if( obj.isSimulation )

            % connect to ROS service 
            client = rossvcclient("/gazebo/spawn_sdf_model","DataFormat","struct");

            % create ROS message 
            req = rosmessage(client);
            
            % set req message 
            req.ModelName = sprintf('Cylinder_%d', obj.gzObjectCounter);
            req.InitialPose.Position.X = x;
            req.InitialPose.Position.Y = y;
            req.InitialPose.Position.Z = length / 2;    % <- place at correct height

            
            % open Cylinder sdf file  ---------------
            fid = fopen('maps/gzb_models/omodels/cylinder/model.sdf');  
            
            % file data bufffer 
            fdata = [];

            % read 1st line 
            tline = fgetl(fid); 
            while ischar(tline)
                % look for <radius> tag 
                if( contains(tline,'<radius>') ) 
                    % modify size parameters 
                    tline = sprintf('<radius>%g</radius>\n', radius); 
                    %disp(tline)  
                end
                % look for <length> tag 
                if( contains(tline,'<length>') ) 
                    % modify size parameters 
                    tline = sprintf('<length>%g</length>\n', length); 
                    %disp(tline)  
                end
                % look for <diffuse> tag 
                if( contains(tline,'<diffuse>') ) 
                    % modify color parameters 
                    tline = sprintf('<diffuse>%g %g %g 1</diffuse>\n', color(1), color(2), color(3)); 
                    %disp(tline)  
                end 
                % add to buffer 
                fdata = [fdata, tline];
            
                %disp(tline)             % show line 
                tline = fgets(fid);     % read line by line (w/ newline)
            end
            % close file ------------------------
            fclose(fid);


            % copy file contents in to ModelXml data (string format)
            req.ModelXml = fdata;
            
            % send request 
            resp = call(client, req, "Timeout", 3);

            % show error message in case of error 
            if( ~ resp.Success) error(resp.StatusMessage); end 

            % update gazebo object counter 
            obj.gzObjectCounter = obj.gzObjectCounter + 1; 
            %fprintf('gzObjectCounter %d \n', obj.gzObjectCounter);


        % [real TurtleBot]
        else
            error('Adding 3D objects, is only supported when connected to the Gazebo simulator'); 
        end 
    
    end


    % Delete all (supported) 3D objects in Gazebo simulator   
    function gazeboDeleteAllModels(obj)
        % Detete all (supported) 3D model objects in the Gazebo simulator
        % 
        % obj.gazeboDeleteAllModels();
        %  
        % notes: 
        % -> This function only deletes objects previously placed by gazeboPlace3D*(.) functions.
        % -> Manually inserted objects, Maps and the TurtleBot model are kept unchanged.  
        % -> Only removes the supported 3D objects: [CardboardBox, Cube, Cylinder].
        % -> Forces Gazebo physics simulation on (unpauses/resumes Gazebo). 


        % [Gazebo]
        if( obj.isSimulation )

            % UnPause/Resume Gazebo physics simulation (required to receive model_states message)
            obj.gazeboResume();

            % read data from /gazebo/model_states topic 
            gzMsg = receive(obj.gzSub, 3); 

            % define patterns basenames
            nPatterns = ["CardboardBox", "Cube", "Cylinder"];

            % scan for reference basenames
            matches = contains(gzMsg.Name, nPatterns);

            % find indexes of match models 
            idx = find( matches);

            % -> delete matching models 
            if( length(idx) ) 

                % connect to ROS service 
                client = rossvcclient("/gazebo/delete_model","DataFormat","struct");

                % create ROS message 
                req = rosmessage(client);

                for i=1:1:length(idx)

                    % fill 3D model name 
                    req.ModelName = gzMsg.Name{ idx(i) }; 
                    
                    % send request 
                    resp = call(client, req, "Timeout", 3); 

                end 

            end 

            % reset gazebo object counter 
            obj.gzObjectCounter = 0; 


        % [real TurtleBot]
        else
            error('Removing 3D objects, is only supported when connected to the Gazebo simulator'); 
        end 

    end 


    % Pause Gazebo Physics Simulation
    function gazeboPause(obj)
        % Pauses Gazebo Physics Simulation
        % 
        % obj.gazeboPause();
        %  

       % [Gazebo]
       if( obj.isSimulation )

            % connect to ROS service 
            client = rossvcclient("/gazebo/pause_physics","DataFormat","struct");

            % create ROS message 
            req = rosmessage(client);
            
            % send request 
            call(client, req, "Timeout", 3);

        % [real TurtleBot]
        else
            error('Pausing Simulation, is only supported when connected to the Gazebo simulator'); 
        end 

    end 


    % Resume Gazebo Physics Simulation
    function gazeboResume(obj)
        % Resumes/Unpauses Gazebo Physics Simulation
        % 
        % obj.gazeboResume();
        %  


        % [Gazebo]
        if( obj.isSimulation )

            % connect to ROS service 
            client = rossvcclient("/gazebo/unpause_physics","DataFormat","struct");

            % create ROS message 
            req = rosmessage(client);
            
            % send request 
            call(client, req, "Timeout", 3); 

        % [real TurtleBot]
        else
            error('Resuming Simulation, is only supported when connected to the Gazebo simulator'); 
        end 

    end


    % UnPause Gazebo Physics Simulation
    % - overload of gazeboResume()
    function gazeboUnPause(obj)
        % Resumes/Unpauses Gazebo Physics Simulation
        % 
        % obj.gazeboUnPause();
        %  

        % call gazeboResume()
        obj.gazeboResume();
    end


    % Destructor
    function delete(obj)
        % Destructor: clear the workspace of publishers, subscribers, and other ROS-related objects
        %
        % obj.delete(); 
        % 
        % note: 
        % -> Stops the robot and disconnects ROS, when object is deleted.   


        % check ROS connection (test if shutdown is required)
        if( obj.rinit )
            obj.stop();     % stop robot

            fprintf('ROS shutdown\n');
            rosshutdown;    % shutdown ROS
        end 
        %fprintf('delete(.)\n');
    end

    end     % end methods

end




% ---------------------------------------------------------
% Helper functions
% ---------------------------------------------------------

% decode color by MatLab char code
%  - only considers the 1st character of (c)
% suported color codes: 
%   - 'r' red, 'g' green, 'b' blue, 'm' magenta, 'c' cyan, 'y' yellow, 'w' white and 'k' black. 
%   - if (unknown color code) the ouputs is assigned as: color = c; 
function color = decodeColor(c)

    % asssign output (default case)
    color = c; 

    % if c is char 
    if( ischar(c) )

        % decode (the first character) by MatLab 'char' code   
        switch c(1) 
            case 'r'
                color = [1, 0, 0];  % red 
            case 'g'
                color = [0, 1, 0];  % green 
            case 'b'
                color = [0, 0, 1];  % blue 
            case 'm'
                color = [1, 0, 1];  % magenta 
            case 'c'
                color = [0, 1, 1];  % cyan
            case 'y'
                color = [1, 1, 0];  % yellow 
            case 'k'
                color = [0, 0, 0];  % black 
            case 'w'
                color = [1, 1, 1];  % white 
            otherwise
                fprintf('unknown color');
        end
        
    end 

end





% ---------------------------------------------------------
% ROS message setup functions
% ---------------------------------------------------------

% setup PATH aux function (custom ROS message compilation)
% -> modify to add the location of python and cmake executables  
function setupPath()

    fprintf('--- setup Path ---\n'); 

    % get environment variable
    currentPath = getenv('PATH');

    % query systen architecture
    switch computer('arch')

        % MacOS 64bits 
        case 'maci64'

            % add cmake directory to PATH 
            setenv('PATH', [currentPath,':/usr/local/bin']); 

            % add directory PATH using anaconda enviroments 
            %setenv('PATH',[currPath,':/opt/homebrew/bin',':/opt/anaconda3/envs/p39/bin']); 
    
            % -> accept xcode license w/o having code instaled 
            % defaults write com.apple.dt.Xcode IDEXcodeVersionForAgreedToGMLicense 12.0

        % Linux 64bits 
        case 'glnxa64'
            % no extra settings required

        % Windows 64bits 
        % case 'win64'
        % -> unsuported

        otherwise
            error('Unsuported system architecture');    % win64 unsuported
    end

    % echo version numbers
    ! python --version  
    ! cmake --version

end



% setup custom ROS SensorState message
% -> supported only in Linux64 and Mac systems
function setupSensorStateMessage()

    % Message Format: 
    % http://docs.ros.org/en/melodic/api/turtlebot3_msgs/html/msg/SensorState.html
    % "turtlebot3_msgs/SensorState.msg"

    fprintf('--- setup SensorState.msg ---\n'); 

    % check for unsuported systems (i.e. windows 64)
    if( strcmp( computer('arch'), "win64") )
        error('Unsuported compilation of ROS messages in this system architecture'); 
    end 

    % Build Custom Message File Structure
    genDir = fullfile(pwd,'maps/rosTurtleBotMessages');
    mkdir(genDir)

    packagePath = fullfile(genDir,'turtlebot3_msgs');
    mkdir(packagePath,'msg')

 
    % define message structure
    messageDefinition = {
    'uint8 BUMPER_FORWARD=1'
    'uint8 BUMPER_BACKWARD=2'
    'uint8 CLIFF=1'
    'uint8 SONAR=1'
    'uint8 ILLUMINATION=1'
    'uint8 BUTTON0=1'
    'uint8 BUTTON1=2'
    'uint8 ERROR_LEFT_MOTOR=1'
    'uint8 ERROR_RIGHT_MOTOR=2'
    'uint8 TORQUE_ON=1'
    'uint8 TORQUE_OFF=2'
    'std_msgs/Header header'
    'uint8 bumper'
    'float32 cliff'
    'float32 sonar'
    'float32 illumination'
    'uint8 led'
    'uint8 button'
    'bool torque'
    'int32 left_encoder'
    'int32 right_encoder'
    'float32 battery'
    };

    % write message structure to file "SensorState.msg"
    fileID = fopen(fullfile(packagePath,'msg', 'SensorState.msg'),'w');
    fprintf(fileID,'%s\n',messageDefinition{:});
    fclose(fileID);

    % Generate ROS Messages
    rosgenmsg(genDir)

    % query system architecture
    sysarch = computer('arch');

    % add custom message folder to the MATLAB path
    addpath( fullfile(genDir, sprintf('matlab_msg_gen_ros1/%s/install/m', sysarch)) ); 
    %savepath

    clear classes
    rehash toolboxcache


    % DEBUG...
    % list all supported messages
    %rosmsg list

    % generate message
    %msg = rosmessage('turtlebot3_msgs/SensorState');

    % get message definition 
    %msgInfo = rosmsg('show','turtlebot3_msgs/SensorState')

end 

