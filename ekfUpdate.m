function [p, Sigma, S] = ekfUpdate(p, Sigma, v, Hg, R)
%   ekfUpdate  EKF correction step (batch form).


    if isempty(v)
        S = [];
        return;
    end

    S = Hg*Sigma*Hg.' + R;
    K = (Sigma * Hg.') / S;

    p    = p + K*v;
    p(3) = atan2(sin(p(3)), cos(p(3)));

    Sigma = (eye(3) - K*Hg) * Sigma;
    Sigma = 0.5*(Sigma + Sigma.');
end
