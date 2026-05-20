% E9 - EKF prediction

clearvars -except tbot

if (~exist("tbot", "var"))
    IP_TURTLEBOT     = "192.168.21.139";
    IP_HOST_COMPUTER = "192.168.21.1";
    tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER);
    if (tbot.getVersion() < 0.9), error('TurtleBot v09 required'); end
    tbot.gazeboDeleteAllModels();
end

xri = 0;  yri = 0;  thetai = 0;
tbot.setPose(xri, yri, thetai);

p     = [xri; yri; thetai];
Sigma = diag([0.01, 0.01, deg2rad(1.0)].^2);

kr = 1e-3;
kl = 1e-3;
Q  = diag([1e-6, 1e-6, 1e-7]);
b  = tbot.getWheelBaseline();

v = 0.12;
w = 0.20;

tbot.initEncoders();
tbot.setVelocity(v, w);

T_run = 40;
r     = rateControl(5);

maxIt        = T_run * r.DesiredRate + 10;
estTraj      = nan(maxIt, 3);
trueTraj     = nan(maxIt, 3);
sigmaLog     = nan(maxIt, 3);
ellipseEvery = 5;
ellipseSnaps = {};

figure(1); clf;
hold on; axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('E9 - EKF Prediction (open loop)');

tic;  it = 0;
while toc < T_run
    it = it + 1;

    [dsr, dsl, pose2D, ~] = tbot.readEncodersWithNoise(0.002);
    [p, Sigma] = ekfPredict(p, Sigma, dsr, dsl, b, kr, kl, Q);

    estTraj(it,  :) = p.';
    trueTraj(it, :) = pose2D(:).';
    sigmaLog(it, :) = sqrt(diag(Sigma)).';

    if mod(it, ellipseEvery) == 0 || it == 1
        ellipseSnaps{end+1} = struct('it', it, 'mu', p(1:2), 'cov', Sigma(1:2,1:2));

        cla;
        plot(trueTraj(1:it,1), trueTraj(1:it,2), 'g-', 'LineWidth', 1.5);
        plot(estTraj(1:it,1),  estTraj(1:it,2),  'b-', 'LineWidth', 1.5);
        plot(p(1), p(2), 'bo', 'MarkerFaceColor', 'b');
        drawErrorElipse(Sigma(1:2,1:2), p(1:2), 2, 'm', 1.2);
        legend({'ground truth', 'EKF estimate', 'estimate (now)', '2\sigma uncertainty'}, ...
               'Location', 'best');
        drawnow limitrate;
    end

    waitfor(r);
end

tbot.stop();

estTraj  = estTraj(1:it,  :);
trueTraj = trueTraj(1:it, :);
sigmaLog = sigmaLog(1:it, :);
t_vec    = (0:it-1) / r.DesiredRate;

figure(2); clf; hold on; axis equal; grid on;
plot(trueTraj(:,1), trueTraj(:,2), 'g-',  'LineWidth', 1.8);
plot(estTraj(:,1),  estTraj(:,2),  'b--', 'LineWidth', 1.5);
for k = 1:numel(ellipseSnaps)
    snap = ellipseSnaps{k};
    drawErrorElipse(snap.cov, snap.mu, 2, 'm', 0.8);
end
xlabel('x [m]'); ylabel('y [m]');
title('Open-loop trajectory: ground truth vs EKF prediction');
legend('ground truth', 'EKF estimate', '2\sigma ellipses');

normalizeAngle = @(a) atan2(sin(a), cos(a));
err = estTraj - trueTraj;
err(:,3) = arrayfun(normalizeAngle, err(:,3));

figure(3); clf;
subplot(3,1,1);
plot(t_vec, err(:,1), 'b'); hold on; grid on;
plot(t_vec,  2*sigmaLog(:,1), 'r--');
plot(t_vec, -2*sigmaLog(:,1), 'r--');
ylabel('e_x [m]'); title('Prediction error vs 2\sigma envelope');

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
detXY = sigmaLog(:,1).^2 .* sigmaLog(:,2).^2;
plot(t_vec, detXY, 'b', 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('\sigma_x^2 \cdot \sigma_y^2');
title('Uncertainty growth');
