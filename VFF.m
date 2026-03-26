function [Fa, Fr] = VFF (currentPose, targetPose, map, mscale, searchWindow)

Fca = 1;
Fcr = 1;

%Attractive force
dx = targetPose(1) - currentPose(1);
dy = targetPose(2) - currentPose(2);
dGoal = sqrt(dx^2 + dy^2);            % Distance between the robot and the target
Fa = Fca * [dx, dy] / dGoal;          % [Fax, Fay]


Fr = [0, 0];

r_robot = round(currentPose(1) * mscale); 
c_robot = round(currentPose(2) * mscale);
win_px = round((searchWindow * mscale) / 2); % raggio finestra in pixel

r_min = max(1, r_robot - win_px); 
r_max = min(size(map,1), r_robot + win_px);
c_min = max(1, c_robot - win_px); 
c_max = min(size(map,2), c_robot + win_px);

for r = r_min:r_max
        for c = c_min:c_max
            if map(r, c) == 1
              
               %position of the center of the cell
               xij = (r - 0.5) / mscale;              % y axis grows going right
               yij = (c - 0.5) / mscale;              % y axis grows going down
            
               
               dx = currentPose(1) - xij; 
               dy = currentPose(2) - yij;
               d2 = dx*dx + dy*dy;           % Square of the distance between cell and the robot
               d = sqrt(d2);

               Cij = map(r, c);              %Cell occupation weight
            
               % Repulsive contribute of the cell
               Fr = Fr + Fcr * (Cij / d2) * [dx/d, dy/d];  % Repulsive force that stacks with every occupied cell within the active radius
                
            end
        end
end