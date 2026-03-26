
% 
% Demo - Add Gazebo 3D Objects w/ TurtleBot3 interface. 
%
% Launch the virtual machine.
% Open terminal, and start the empty Gazebo world: ./start-gazebo-empty.sh.
%

% clear memory 
clearvars -except tbot

% start ROS connection (tbot object) and add 3D objects, only if required
if ( ~exist("tbot") )

    % init by assigning the IPs directly to the TurtleBot constructor
    IP_TURTLEBOT = "192.168.21.130";         % virtual machine IP (or robot IP)
    IP_HOST_COMPUTER = "192.168.21.1";     % local machine IP

    % Init TurtleBot3 ROS connection
    tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER); 

    % check version
    if( tbot.getVersion() < 0.93 ) error('TurtleBot3 v09d required'); end 


    % -----------------------
    % Add 3D Objects
    % -----------------------

    % Delete all suported 3D objects  
    tbot.gazeboDeleteAllModels();
    
    % Pause Gazebo physics simulation 
    % -> not required, but faster ! - make sure that gazeboResume() is called after placing all objects
    tbot.gazeboPause();


    % Place 3D Objects ------

    % gazeboPlace3DCardboardBox(x, y, orientation, color);
    tbot.gazeboPlace3DCardboardBox(0.5, 0.5, 0);                        % default color 
    tbot.gazeboPlace3DCardboardBox(0.5, 1.5, pi/4, 'w');                % white box 
    tbot.gazeboPlace3DCardboardBox(0.5, 2.5, pi/2, [0.3, 0.3, 0.3]);    % dark grey box - custom [r,g,b] color 
    
    % gazeboPlace3DCube(x, y, orientation, edgeSize, color);
    tbot.gazeboPlace3DCube(2, 0.5, 0, 0.25, 'r');
    tbot.gazeboPlace3DCube(2, 1.5, 0, 0.5, 'g');
    tbot.gazeboPlace3DCube(2, 2.5, pi/4, 1, 'b');
    % tbot.gazeboPlace3DCube(2, 0.25, 0, 0.125, [0.4, 0.25, 0.2]);    % custom [r,g,b] color 

    % gazeboPlace3DCylinder(x, y, radius, length, color);
    tbot.gazeboPlace3DCylinder(3.5, 0.5, 0.1, 0.5, 'm');
    tbot.gazeboPlace3DCylinder(3.5, 1.5, 0.25, 0.75, 'c');
    tbot.gazeboPlace3DCylinder(3.5, 2.5, 0.5, 1, 'y');


    % UnPause/Resume Gazebo physics simulation
    tbot.gazeboResume();                        % or tbot.gazeboUnPause();

end 


% 
% Notes: 
%
% -> All 3D object are placed JUST ONCE after the initial connection. 
% -> There is no need to keep deleting, and inserting, the same objects every time we run the script.
%


% Place TurtleBot at (x = 0,y = 0, theta = 0)
tbot.resetPose();

% Move forward  
tbot.setVelocity(0.1, 0); 

% wait 3s 
pause(3);

% stop 
tbot.stop();














