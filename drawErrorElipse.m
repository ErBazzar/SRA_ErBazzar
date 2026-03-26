
%
% Draw (error) Elipse from a 2D Covariance matrix (Cov) 
%
% [hp] = drawErrorElipse( Cov, mu, level, color, linewidth)
% 
% Inputs: 
%   - Cov - 2D Covariance Matrix (2x2)
%   - mu - 2D mean (2x1 or 1x2)
%   - level - axis length magnitude 
%   - color - display color, p.e. 'r' (red), 'g' (green), 'b' (blue), ... 
% 	- linewidth - width, in pt, of line
% Returns the plot handle (hp) for all displayed parts.
% 
% Versions notes: 
% v01 - initial release (w/ TurteBot3 v09)


function [hp] = drawErrorElipse( Cov, mu, level, color, linewidth)

	% check inputs
	if( nargin < 3 )
		level = 1.0;	% default magnitude  
		color = 'k'; 	% default color 
		linewidth = 1;	% default linewith  
	end

	% apply SVD decomposition from a 2x2 Cov matrix
	[U, S, V] = svd( Cov(1:2,1:2) );

	% compute axis length 
	a = sqrt( S(1,1) ) * level;
	b = sqrt( S(2,2) ) * level;

	% define parametric angle variable, in increments of 0.1  
	theta = [0 : 0.1 : 2*pi + 0.1];

	% set control points of elipse
	xx = a * cos(theta);
	yy = b * sin(theta);

	% apply linear transformation (rotation)
	XY = U * [xx; yy];

	% draw 'transformed' points and add offset (mu)
	hp = plot( XY(1,:) + mu(1), XY(2,:) + mu(2), 'Color', color, 'LineWidth', linewidth);





	