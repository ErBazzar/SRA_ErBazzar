function [steerDirection, h_smooth, bin_edges] = VFH(currentPose, targetPose, map, mscale, searchWindow)

% Algorithm parameters 
n      = 30;       % Number of polar histogram sectors
thresh = 0.1;      % Normalized density threshold 
smax   = 5;        % Wide-valley minimum width [bins]
sigma  = 0.75;     % Gaussian smoothing sigma [bins]

% Constants for obstacle magnitude
% a and b chosen such that a - b * d_max = 0 (farthest cell contributes 0)
b_coeff = 1;
a_coeff = b_coeff * searchWindow;

% Define histogram bin edges in [-pi, pi]
bin_edges   = linspace(-pi, pi, n + 1);

% Compute bin center angles (used internally for steering candidates)
bin_centers = (bin_edges(1:end-1) + bin_edges(2:end)) / 2;

% Build polar obstacle density histogram
h = zeros(1, n);

xr = currentPose(1);
yr = currentPose(2);

% Convert robot world position to grid indices
r_robot = round(xr * mscale);
c_robot = round(yr * mscale);
win_px  = round(searchWindow * mscale);

% Define search window boundaries 
r_min = max(1, r_robot - win_px);
r_max = min(size(map, 1), r_robot + win_px);
c_min = max(1, c_robot - win_px);
c_max = min(size(map, 2), c_robot + win_px);

for r = r_min:r_max
    for c = c_min:c_max
        if map(r, c) > 0.5

            % Position of the center of the cell in world coordinates
            xij = (r - 0.5) / mscale;
            yij = (c - 0.5) / mscale;

            dx = xij - xr;
            dy = yij - yr;
            d2 = dx*dx + dy*dy;                    % Squared distance
            d  = sqrt(d2);

            if d > 0 && d <= searchWindow
                Cij = map(r, c);                   % Cell occupation weight

                % Obstacle magnitude
                m = (Cij^2) * (a_coeff - b_coeff * d);

                % Direction from robot to obstacle cell (beta)
                beta = atan2(dy, dx);

                % Map beta to a histogram sector index k
                k = ceil((beta - bin_edges(1)) / (2 * pi) * n);
                k = max(1, min(n, k));

                % Accumulate obstacle density in sector k: h(k) = h(k) + m
                h(k) = h(k) + m;
            end
        end
    end
end


% Smooth the histogram (1D Gaussian filter)

kernel_half = ceil(3 * sigma);
kx          = -kernel_half:kernel_half;
kernel      = exp(-kx.^2 / (2 * sigma^2));
kernel      = kernel / sum(kernel);                % normalise kernel

% Pad circularly, convolve, then trim
h_padded = [h(end - kernel_half + 1:end), h, h(1:kernel_half)];
h_smooth = conv(h_padded, kernel, 'valid');
h_smooth = h_smooth(1:n);

% Normalise to [0, 1] for threshold comparison
h_max = max(h_smooth);
if h_max > 0
    h_smooth = h_smooth / h_max;
else
    % No obstacles detected -> head straight for the target
    steerDirection = atan2(targetPose(2) - yr, targetPose(1) - xr);
    return;
end

% Check if target direction is directly accessible

targetDir  = atan2(targetPose(2) - yr, targetPose(1) - xr);
target_bin = max(1, min(n, ceil((targetDir - bin_edges(1)) / (2*pi) * n)));

% Check if there are enough free sectors around the target direction
check_half = floor(smax / 2);
target_ok  = true;
for dk = -check_half:check_half
    k_chk = mod(target_bin - 1 + dk, n) + 1;      % Circular index wrapping
    if h_smooth(k_chk) >= thresh
        target_ok = false;
        break;
    end
end

% If the target sector is free: steer directly to target
if target_ok
    steerDirection = targetDir;
    return;
end


% Find the two nearest zero-crossing transitions

% Search clockwise from target for first free sector
kn_cw = [];
for offset = 0:n-1
    k = mod(target_bin - 1 + offset, n) + 1;
    if h_smooth(k) < thresh
        kn_cw = k;
        break;
    end
end

% Search counterclockwise from target for first free sector
kn_ccw = [];
for offset = 0:n-1
    k = mod(target_bin - 1 - offset, n) + 1;
    if h_smooth(k) < thresh
        kn_ccw = k;
        break;
    end
end

% If completely blocked robot must stop
if isempty(kn_cw) && isempty(kn_ccw)
    steerDirection = NaN;
    return;
end

% Select closest valley border (kn)

dist_cw  = inf;
dist_ccw = inf;
if ~isempty(kn_cw)
    dist_cw = abs(atan2(sin(bin_centers(kn_cw) - targetDir), ...
                        cos(bin_centers(kn_cw) - targetDir)));
end
if ~isempty(kn_ccw)
    dist_ccw = abs(atan2(sin(bin_centers(kn_ccw) - targetDir), ...
                         cos(bin_centers(kn_ccw) - targetDir)));
end

% Pick the nearest valley border to the target direction
if dist_cw <= dist_ccw
    kn = kn_cw;
    scan_step = 1;         % Scan clockwise into the valley
else
    kn = kn_ccw;
    scan_step = -1;        % Scan counterclockwise into the valley
end


% Measure valley width and compute steering direction


% Scan from kn into the valley to count consecutive free sectors
width  = 0;
k_scan = kn;
while width < n
    if h_smooth(k_scan) >= thresh
        break;
    end
    width  = width + 1;
    k_scan = mod(k_scan - 1 + scan_step, n) + 1;
end

if width >= smax
    % Wide valley: steer to smax/2 bins from the near edge (kn)
    k_steer = mod(kn - 1 + scan_step * floor(smax/2), n) + 1;
    steerDirection = bin_centers(k_steer);
else
    % Narrow valley: steer to the center of the valley
    k_steer = mod(kn - 1 + scan_step * floor(width/2), n) + 1;
    steerDirection = bin_centers(k_steer);
end

end