%
% Draw (2D) TurtleBot Robot at (x,y,theta) with given color
%
%   [hp] = drawTurtleBot(x, y, theta, color = 'k', linewidth = 0.5); 
%
% Inputs: 
%   - (x,y) - 2D location [meters] (scalars)
%   - theta - orientation angle [radians] (scalar)
%   - color - display color, p.e. 'r' (red), 'g' (green), 'b' (blue), ... 
%   - linewidth - width of line 
% Returns the plot handle (hp) for all displayed parts.
% 
% Note: delete(hp); clears the robot plot.     
%  
% Versions notes: 
% v01 - initial release
% v02 - fix out circumference contours, performance improvements 
% v03 - added color input parameter
% v04 - visual code updates
% v05 - added linewidth option (TurtleBot3 v09b)
%