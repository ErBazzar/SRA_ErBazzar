
%
% Draw Polar Histogram (h, bins) at given pose 
%
% [hp] = drawPolarHistogram(x, y, theta, h, bins, radius, color)
% 
% Inputs: 
%   - (x,y) - 2D location [meters] (scalars)
%   - theta - orientation angle [radians] (scalar)
%   - h - histogram values (bin counts) (vector)
%   - bins - bin edges (vector) 
%   - radius - display radius [meters] (scalar) 
%   - color - display color, p.e. 'r' (red), 'g' (green), 'b' (blue), ... 
% Returns the plot handle (hp) for all displayed parts.
%
% -----------------
% Example usage: 
%  
% n = 30;                                       % define number of bins 
% bins = linspace(-pi, pi, n);                  % generate bins in [-pi,pi[
% mu = 0; sigma = 0.75;                         % Gaussian parameters
% h = exp( - (bins-mu).^2 ./ (2*sigma.^2) );    % generate Gaussian histogram
% x = 0.5; y = 0.5; theta = pi/4;               % set robot pose 
% figure(1); clf;                               % init figure & clear graphics 
% drawTurtleBot(x,y,theta);                     % draw robot
% drawPolarHistogram(x, y, theta, h, bins, 0.5, 'r');   % draw polar histogram
% axis([0,1,0,1]); grid on;                     % set plot limits
% title('demo drawPolarHistogram(.)'); 
%
% Versions notes: 
% v01 - initial release (w/ TurteBot3 v09)
% v02 - updated example notation


function [hp] = drawPolarHistogram(x, y, theta, h, bins, radius, color)
    
	% check inputs
	if( nargin < 6 )
		radius = 0.5;   % default plot radius
		color = 'r';    % default color 
	end

    % get number of bins 
    alpha = length(bins); 

    % compute max bin value 
    hmax = max(h); 

    % init plot handle 
    hp = zeros(alpha-1, 1); 

    % draw only non empty histograms  
    if( hmax > 0 )

        % enable plot hold, if disabled 
        if ( ~ishold ) hold on; end 

        % for each bin, except the last 
        for k=1:1:alpha-1

            % scale factor to be drawn with radius = localWindow
            mag = h(k) / hmax * radius;       

            if( mag > 0 )
                % draw 'pie slice' w/ normalized count value within histogram bin edges 
                hp(k) = fill( [x, x + mag.*cos(bins(k) + theta), x + mag.*cos(bins(k+1) + theta)], ... 
                [y, y + mag.*sin(bins(k)+ theta), y + mag.*sin(bins(k+1) + theta)], color, 'FaceAlpha', 0.5);
            end 

        end

    end 


