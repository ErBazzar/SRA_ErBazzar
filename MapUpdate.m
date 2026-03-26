function map = MapUpdate(map, pose, lddata, mscale, l_occ, l_free, l0)

    logmap = log( map ./ max(1e-12, 1 - map) );

    %Pose and scan unpacking
    x     = pose(1);              % robot x
    y     = pose(2);              % robot y 
    theta = pose(3);              % robot orientation 
    ranges = lddata.Ranges;
    angles = lddata.Angles;

    xL = (ranges .* cos(angles)).';          % [1 x N]
    yL = (ranges .* sin(angles)).';          % [1 x N]
    ptsL = [xL;
            yL;
            ones(1,numel(ranges))];                 % [3 x N]


    T = [ cos(theta), -sin(theta),  x - 0.0305*cos(theta); ...
          sin(theta),  cos(theta),  y - 0.0305*sin(theta); ...
          0,        0,        1 ];

    % LIDAR scans transformation into world frame
    ptsW = T * ptsL;                    % [3 x N]
    xW   = ptsW(1,:);                   % [1 x N]
    yW   = ptsW(2,:);                   % [1 x N]

    % Sensor's position in the world frame
    x_s = T(1,3);
    y_s = T(2,3);

    % Sensor validity range
    minR = 0.12;       
    maxR = 3.50;     

    %Confidence extremes for dynamic log-odds update
    conf_max = 1.0;
    conf_min = 0.4;

    % Map size 
    [H, W] = size(map);
    x_min = 0; 
    y_min = 0;  

    i_start = round((x_s - x_min) * mscale) + 1;
    j_start = round((y_s - y_min) * mscale) + 1;

    % Beam iteration
    
    for k = 1:numel(ranges)
        r    = ranges(k);    % measured range [m]
        angL = angles(k);    % beam angle in LiDAR frame [rad]
        if r>= minR

            r_eff_mod = min(r, maxR); % Distance clamping to avoid negative values

            % Normalized and inverted range
            norm_dist_inv = (maxR - r_eff_mod) / (maxR - minR);
            norm_dist_inv = max(0, min(1, norm_dist_inv)); % Clamping 0 to 1
            
            % Confidence factor
            conf_factor = conf_min + (conf_max - conf_min) * norm_dist_inv;
            
            % Dynamic modulation
            l_occ_dyn  = l0 + (l_occ - l0) * conf_factor;
            l_free_dyn = l0 + (l_free - l0) * conf_factor;

            % Hit vs No-hit decision & Endpoint Calculation

            % Infinite, NaN and out of range values
            if isinf(r) || r > maxR || isnan(r)
               
                hasObstacle = false;
                r_eff = maxR;

                global_angle = theta + angL; 

                x_end = x_s + r_eff * cos(global_angle);
                y_end = y_s + r_eff * sin(global_angle);
                
            else

                hasObstacle = true;      %obstacle within range
                
                x_end = xW(k);
                y_end = yW(k);
            end
    
            % WORLD to GRID indices 

            i_end   = round((x_end - x_min) * mscale) + 1;
            j_end   = round((y_end - y_min) * mscale) + 1;
    
            %Ray traversal with Bresenham
            [ii, jj] = bresenham(i_start, j_start, i_end, j_end);
    
            %Keep only cells inside the map bounds
            inside = (ii >= 1 & ii <= H & jj >= 1 & jj <= W);
            ii = ii(inside);  jj = jj(inside);
    
            if ~isempty(ii)       
                % Split free cells vs occupied cell
                freeIdx = 1:numel(ii);
                occIdx  = [];
                
                if hasObstacle
                    % If there is a obstacle the last cell is occupied
                    occIdx = freeIdx(end);     
                    freeIdx(end) = [];         % Removes the last one from the free cells
                else
                end
        
                %  Apply log-odds updates
                if ~isempty(freeIdx)
                    indFree = sub2ind([H W], ii(freeIdx), jj(freeIdx));
                    logmap(indFree) = logmap(indFree) + (l_free_dyn - l0);
                end
        
                if ~isempty(occIdx)
                    indOcc = sub2ind([H W], ii(occIdx), jj(occIdx));
                    logmap(indOcc) = logmap(indOcc) + (l_occ_dyn - l0);
                end
            end
        end
    end
    
    %Convert log-odds to probabilities
    map = 1 ./ (1 + exp(-logmap));
end