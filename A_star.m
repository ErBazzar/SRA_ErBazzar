function path = A_star(map, start, goal)
    % Inputs:
    %   map: 2D matrix where 0 is free space and 1 is an obstacle
    %   start_node: [x, y] coordinates of the start.
    %   goal_node:  [x, y] coordinates of the goal.
    
    % Extracting the size of the map
    [rows, cols] = size(map); 
    
    % Initializinf g_score and f_score matrices
    g_score = inf(rows, cols);
    f_score = inf(rows, cols);
    
    % We use a 3D array to store the parent coordinates.
    % came_from(r, c, 1) will store the parent's row.
    % came_from(r, c, 2) will store the parent's column.
    came_from = zeros(rows, cols, 1); 
    
    % We save the coordinates of the start point and the goal
    start_r = start(1);
    start_c = start(2);
    goal_r = goal(1);
    goal_c = goal(2);
    
    % Initializing start node
    g_score(start_r, start_c) = 0;
    f_score(start_r, start_c) = calculate_heuristic(start, goal);
    
    % Open_set is a N x 2 matrix storing [row, col]
    open_set = [start_r, start_c];
    
    % We keep a parallel array of f_scores for the open set to find the minimum
    open_f_scores = f_score(start_r, start_c); 
    
    % Boolean mask to check if a node is in the open_set
    in_open_set = false(rows, cols);
    in_open_set(start_r, start_c) = true;
    
    % 8-connected movement directions (row_offset, col_offset, cost)
    directions = [
        -1,  0, 1;         % Up
         1,  0, 1;         % Down
         0, -1, 1;         % Left
         0,  1, 1;         % Right
        -1, -1, sqrt(2);   % Up-Left
        -1,  1, sqrt(2);   % Up-Right
         1, -1, sqrt(2);   % Down-Left
         1,  1, sqrt(2);   % Down-Right
    ];

    while ~isempty(open_set)
        % Finding the node in open_set with the lowest f_score
        [~, min_idx] = min(open_f_scores);
        current_r = open_set(min_idx, 1);
        current_c = open_set(min_idx, 2);
        
        % Checking if we reached the goal 
        if current_r == goal_r && current_c == goal_c
            path = reconstruct_path(came_from, current_r, current_c);
            return;
        end
        
        % Removing current node from open_set
        open_set(min_idx, :) = []; 
        open_f_scores(min_idx) = [];
        in_open_set(current_r, current_c) = false;
        
        % Exploring neighbors 
        for i = 1:size(directions, 1)
            n_r = current_r + directions(i, 1);
            n_c = current_c + directions(i, 2);
            move_cost = directions(i, 3);
            
            % Check map boundaries
            if n_r < 1 || n_r > rows || n_c < 1 || n_c > cols
                continue; 
            end
            
            % Check for obstacles 
            if map(n_r, n_c) == 1
                continue; 
            end
            
            % Calculate tentative g_score 
            tentative_g = g_score(current_r, current_c) + move_cost;
            
            % Record if a better path is found
            if tentative_g < g_score(n_r, n_c)
                % Store the parent's row and column separately
                came_from(n_r, n_c, 1) = current_r;
                came_from(n_r, n_c, 2) = current_c;
                
                g_score(n_r, n_c) = tentative_g;
                
                % Update f_score using the heuristic
                f_val = tentative_g + calculate_heuristic([n_r, n_c], goal);
                f_score(n_r, n_c) = f_val;
                
                % Add to open_set if not already there 
                if ~in_open_set(n_r, n_c)
                    open_set(end + 1, :) = [n_r, n_c];
                    open_f_scores(end + 1) = f_val;
                    in_open_set(n_r, n_c) = true;
                else
                    % If it is already in the open set update its score in our tracking array
                    idx = find(open_set(:,1) == n_r & open_set(:,2) == n_c);
                    open_f_scores(idx) = f_val;
                end
            end
        end
    end
    
    warning('No path found to the goal!');
    path = [];
end


% Function to calculate the euclidean heuristic
function h = calculate_heuristic(node, goal)
    h = sqrt((node(1) - goal(1))^2 + (node(2) - goal(2))^2);
end

% Function that reconstructs the path from the goal to the start
function path = reconstruct_path(came_from, curr_r, curr_c) 

    path = [curr_r, curr_c];
    
    while true
        parent_r = came_from(curr_r, curr_c, 1);
        parent_c = came_from(curr_r, curr_c, 2);
        
        % Stop if we reach the start node 
        if parent_r == 0 && parent_c == 0
            break;
        end
        
        curr_r = parent_r;
        curr_c = parent_c;
        path = [[curr_r, curr_c]; path]; % Append to front
    end
end