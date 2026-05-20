function H = obsJacobian(p, hit_point, sensor_offset)
%   obsJacobian  1x3 Jacobian of the LiDAR range

    phi = p(3);
    xso = sensor_offset(1); yso = sensor_offset(2);

    xs = p(1) + xso*cos(phi) - yso*sin(phi);
    ys = p(2) + xso*sin(phi) + yso*cos(phi);
    xt = hit_point(1); yt = hit_point(2);

    g = sqrt((xs - xt)^2 + (ys - yt)^2);
    if g < 1e-9, g = 1e-9; end

    H = [ (xs - xt)/g, ...
          (ys - yt)/g, ...
          ((xt - xs)*( xso*sin(phi) + yso*cos(phi)) + ...
           (yt - ys)*(-xso*cos(phi) + yso*sin(phi))) / g ];
end
