% Load the map data generated from the turtlebot mapping script
load('turtlebot_map.mat'); 

% Assign the inflated binary map to the variable used for pathfinding
map = occVisFinal; 

% Define the starting point and goal coordinates in world meters
x_start_m = -6; y_start_m = -0.5;
x_goal_m  = 6.5;  y_goal_m  = -0.5;

% Set the offset to match the value used during the mapping phase
offset_m = 10; 

% Shift the world coordinates into positive values to match the matrix space
x_start_shifted = x_start_m + offset_m;
y_start_shifted = y_start_m + offset_m;
x_goal_shifted  = x_goal_m  + offset_m;
y_goal_shifted  = y_goal_m  + offset_m;

% Convert the shifted meter coordinates into discrete grid indices
start_row = round(x_start_shifted * mscale);
start_col = round(y_start_shifted * mscale);
goal_row  = round(x_goal_shifted * mscale);
goal_col  = round(y_goal_shifted * mscale);

% Apply a safety check to ensure indices are not lower than one
start = [max(1, start_row), max(1, start_col)];
goal  = [max(1, goal_row),  max(1, goal_col)];

% Execute the a star pathfinding function on the binary map
path = A_star(map, start, goal);

% Setup the figure for pathfinding visualization using real world meters
figure('Color', 'w', 'Name', 'A* Pathfinding Visualization');
hold on;
[size_X, size_Y] = size(map);

% Create real coordinate vectors by removing the matrix offset
x_real = (1:size_X) / mscale - offset_m;
y_real = (1:size_Y) / mscale - offset_m;

% Display the occupancy grid using the real world meter vectors for the axes
imagesc(x_real, y_real, map');
colormap(flipud(gray)); 
grid on;

% Correct the orientation of the vertical axis to point upwards
axis xy; 
axis equal; 

% Set the plot boundaries based on the actual map dimensions
axis([min(x_real) max(x_real) min(y_real) max(y_real)]);

% Configure the grid tick marks to show values every two meters
set(gca, 'XTick', -10:2:20, 'YTick', -10:2:20);

% Adjust the marker size based on the grid resolution for better visibility
markerSize = 12;
if size_X > 50, markerSize = 8; end

% Plot the start and goal positions using original world coordinates
plot(x_start_m, y_start_m, 'gs', 'MarkerSize', markerSize, 'LineWidth', 2, 'DisplayName', 'Start', 'MarkerFaceColor', 'g');
plot(x_goal_m, y_goal_m, 'rs', 'MarkerSize', markerSize, 'LineWidth', 2, 'DisplayName', 'Goal', 'MarkerFaceColor', 'r');

% Plot the computed path if a valid solution was found
if ~isempty(path)
    lineWidth = 2;
    if size_X > 50, lineWidth = 1.5; end
    
    % Convert the pixel path back to meters for drawing purposes
    path_m_plot = path / mscale - offset_m;
    
    % Draw the calculated path as a continuous blue line
    plot(path_m_plot(:,1), path_m_plot(:,2), 'b-', 'LineWidth', lineWidth, 'DisplayName', 'A* Path');
    
    title(sprintf('A* Algorithm: Path Found (%d steps)', size(path, 1)));
    legend('Location', 'northeastoutside');
else
    title('A* Algorithm: No Path Found');
end

% Set the labels for the axes in meters
xlabel('X [meters]'); 
ylabel('Y [meters]');
hold off;

% Save the path and scale variables for use in the tracking script
filename = 'astar_path.mat';
save(filename, 'path', 'mscale');