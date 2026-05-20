function [p, Sigma, Jp, Jn] = ekfPredict(p, Sigma, dsr, dsl, b, kr, kl, Q)
%   ekfPredict  EKF prediction step for a differential-drive robot.

    x = p(1); y = p(2); theta = p(3);

    D    = (dsr + dsl) / 2;
    dphi = (dsr - dsl) / b;
    a    = theta + dphi/2;

    p(1) = x + D*cos(a);
    p(2) = y + D*sin(a);
    p(3) = atan2(sin(theta + dphi), cos(theta + dphi));

    Jp = [ 1, 0, -D*sin(a);
           0, 1,  D*cos(a);
           0, 0,  1        ];

    Jn = [ 0.5*cos(a) - (D/(2*b))*sin(a),  0.5*cos(a) + (D/(2*b))*sin(a);
           0.5*sin(a) + (D/(2*b))*cos(a),  0.5*sin(a) - (D/(2*b))*cos(a);
           1/b                          , -1/b                          ];

    Cn    = diag([kr*abs(dsr), kl*abs(dsl)]);
    Sigma = Jp*Sigma*Jp.' + Jn*Cn*Jn.' + Q;
    Sigma = 0.5*(Sigma + Sigma.');
end
