% E10 - EKF localization with LiDAR + prior occupancy map
% Drops the robot at an unknown pose, recovers it with globalLocalize, then
% runs the EKF on an open-loop rectangle path

clearvars -except tbot

if (~exist("tbot", "var"))
    IP_TURTLEBOT     = "192.168.21.139";
    IP_HOST_COMPUTER = "192.168.21.1";
    tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER);
    if (tbot.getVersion() < 0.9), error('TurtleBot v09 required'); end
    tbot.gazeboDeleteAllModels();
end

if ~isfile('turtlebot_map.mat')
    error('turtlebot_map.mat not found - run the E8 mapping script first');
end
mapData  = load('turtlebot_map.mat');
occGrid  = mapData.map > 0.65;
mscale   = mapData.mscale;
offset_m = mapData.worldSize / 2;

sensor_offset = [-0.0305; 0];
nBeams        = 36;
max_range     = 3.50;

% setPose only sets the Gazebo ground truth used to plot the error
xri = -3;  yri = 3;  thetai = 0;
tbot.setPose(xri, yri, thetai);
pause(0.5);

[~, lddata0, ~] = tbot.readLidar();
[p, loc_score]  = globalLocalize(lddata0, occGrid, mscale, offset_m, ...
                                 max_range, sensor_offset);

fprintf('Global localization:\n');
fprintf('   estimate p0 = [% .2f m, % .2f m, % .1f deg]\n', ...
        p(1), p(2), rad2deg(p(3)));
fprintf('   ground truth = [% .2f m, % .2f m, % .1f deg]\n', ...
        xri, yri, rad2deg(thetai));
fprintf('   match score = %.3f m,  initial position error = %.3f m\n', ...
        loc_score, norm(p(1:2) - [xri; yri]));
if loc_score > 0.15
    warning('globalLocalize: high residual (%.3f m); EKF start may be wrong.', loc_score);
end

% Inflated initial covariance
Sigma = diag([0.20, 0.20, deg2rad(12)].^2);

kr = 1e-3; kl = 1e-3;
Q  = diag([1e-6, 1e-6, 1e-7]);
b  = tbot.getWheelBaseline();

sigma_R_rel  = 0.035;
sigma_R_base = 0.02;
e_mahal      = 2.5;

encoder_noise_std = 0.002;

T_forward = 6.0;
T_rotate  = 3.0;
v_fwd     = 0.15;
w_rot     = (pi/2) / T_rotate;
T_period  = T_forward + T_rotate;
T_run     = 2 * 4 * T_period;

r         = rateControl(5);
maxIt     = ceil(T_run * r.DesiredRate) + 10;

estTraj      = nan(maxIt, 3);
trueTraj     = nan(maxIt, 3);
sigmaLog     = nan(maxIt, 3);
nUsed        = zeros(maxIt, 1);
ellipseSnaps = {};
plotEvery    = 5;

[ii_obs, jj_obs] = find(occGrid);
xObs = (ii_obs - 1) / mscale - offset_m;
yObs = (jj_obs - 1) / mscale - offset_m;

figure(1); clf; hold on; axis equal; grid on;
xlabel('x [m]');  ylabel('y [m]');
title('E10 - EKF Final  (predict + LiDAR correct)');

tbot.initEncoders();
tbot.setVelocity(v_fwd, 0);

tic;  it = 0;
while toc < T_run
    it    = it + 1;
    t_now = toc;

    if mod(t_now, T_period) < T_forward
        tbot.setVelocity(v_fwd, 0);
    else
        tbot.setVelocity(0, w_rot);
    end

    [dsr, dsl, pose2D, ~] = tbot.readEncodersWithNoise(encoder_noise_std);
    [~, lddata, ~]        = tbot.readLidar();

    [p, Sigma] = ekfPredict(p, Sigma, dsr, dsl, b, kr, kl, Q);

    angles   = lddata.Angles;
    ranges   = lddata.Ranges;
    beam_idx = round(linspace(1, numel(ranges), nBeams));

    v_vec  = [];
    Hg     = [];
    R_diag = [];

    for k = 1:numel(beam_idx)
        i_b     = beam_idx(k);
        z_i     = ranges(i_b);
        theta_i = angles(i_b);

        if ~isfinite(z_i) || z_i < 0.12 || z_i > max_range
            continue;
        end

        [g_i, hit_pt, valid] = predictRange(p, theta_i, occGrid, ...
                                            mscale, offset_m, ...
                                            max_range, sensor_offset);
        if ~valid, continue; end

        sigma_i = sigma_R_rel * z_i + sigma_R_base;
        R_i     = sigma_i^2;

        H_i = obsJacobian(p, hit_pt, sensor_offset);
        v_i = z_i - g_i;
        s_i = H_i*Sigma*H_i.' + R_i;

        % Mahalanobis gate: reject beams the current pose cannot explain
        if (v_i * v_i) / s_i <= e_mahal^2
            v_vec  = [v_vec ; v_i];     
            Hg     = [Hg    ; H_i];     
            R_diag = [R_diag; R_i];    
        end
    end

    nUsed(it) = numel(v_vec);

    if ~isempty(v_vec)
        [p, Sigma, ~] = ekfUpdate(p, Sigma, v_vec, Hg, diag(R_diag));
    end

    estTraj(it,  :) = p.';
    trueTraj(it, :) = pose2D(:).';
    sigmaLog(it, :) = sqrt(diag(Sigma)).';

    if mod(it, plotEvery) == 0 || it == 1
        ellipseSnaps{end+1} = struct('mu', p(1:2), 'cov', Sigma(1:2,1:2));

        cla;
        plot(xObs, yObs, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 3);
        plot(trueTraj(1:it,1), trueTraj(1:it,2), 'g-', 'LineWidth', 1.5);
        plot(estTraj(1:it,1),  estTraj(1:it,2),  'b-', 'LineWidth', 1.5);
        plot(p(1), p(2), 'bo', 'MarkerFaceColor', 'b');
        drawErrorElipse(Sigma(1:2,1:2), p(1:2), 2, 'm', 1.2);
        title(sprintf('E10 - EKF Final  (t=%.1fs,  %d/%d beams used,  |err|=%.3f m)', ...
                      t_now, nUsed(it), nBeams, ...
                      norm(estTraj(it,1:2) - trueTraj(it,1:2))));
        drawnow limitrate;
    end

    waitfor(r);
end

tbot.stop();

estTraj  = estTraj(1:it,  :);
trueTraj = trueTraj(1:it, :);
sigmaLog = sigmaLog(1:it, :);
nUsed    = nUsed(1:it);
t_vec    = (0:it-1) / r.DesiredRate;

figure(2); clf; hold on; axis equal; grid on;
plot(xObs, yObs, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 3);
plot(trueTraj(:,1), trueTraj(:,2), 'g-',  'LineWidth', 1.8);
plot(estTraj(:,1),  estTraj(:,2),  'b--', 'LineWidth', 1.5);
for k = 1:numel(ellipseSnaps)
    drawErrorElipse(ellipseSnaps{k}.cov, ellipseSnaps{k}.mu, 2, 'm', 0.6);
end
xlabel('x [m]'); ylabel('y [m]');
title('E10 - rectangle path: ground truth vs EKF (with LiDAR correction)');
legend('map', 'ground truth', 'EKF estimate', '2\sigma ellipses', 'Location', 'best');

normalizeAngle = @(a) atan2(sin(a), cos(a));
err = estTraj - trueTraj;
err(:,3) = arrayfun(normalizeAngle, err(:,3));

figure(3); clf;
subplot(3,1,1);
plot(t_vec, err(:,1), 'b'); hold on; grid on;
plot(t_vec,  2*sigmaLog(:,1), 'r--');
plot(t_vec, -2*sigmaLog(:,1), 'r--');
ylabel('e_x [m]');
title('E10 - Localisation error vs \pm 2\sigma envelope');

subplot(3,1,2);
plot(t_vec, err(:,2), 'b'); hold on; grid on;
plot(t_vec,  2*sigmaLog(:,2), 'r--');
plot(t_vec, -2*sigmaLog(:,2), 'r--');
ylabel('e_y [m]');

subplot(3,1,3);
plot(t_vec, rad2deg(err(:,3)), 'b'); hold on; grid on;
plot(t_vec,  2*rad2deg(sigmaLog(:,3)), 'r--');
plot(t_vec, -2*rad2deg(sigmaLog(:,3)), 'r--');
ylabel('e_\theta [deg]'); xlabel('t [s]');

figure(4); clf;
yyaxis left;
plot(t_vec, sigmaLog(:,1).^2 .* sigmaLog(:,2).^2, 'b', 'LineWidth', 1.5);
ylabel('\sigma_x^2 \cdot \sigma_y^2');
yyaxis right;
plot(t_vec, nUsed, 'r', 'LineWidth', 1);
ylabel('# beams accepted by Mahalanobis');
xlabel('t [s]');
title('E10 - Uncertainty over time');
grid on;
