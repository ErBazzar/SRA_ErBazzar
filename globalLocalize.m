function [p0, score] = globalLocalize(lddata, occMap, mscale, offset_m, ...
                                      max_range, sensor_offset)
%   globalLocalize  One-shot pose recovery by LiDAR-to-map scan matching
%   Coarse-to-fine grid search over (x, y, theta)
%   Returns the best pose and its mean end-point-to-wall distance

    nMatch    = 60;
    cap       = 0.50;
    coarse_xy = 0.20;
    coarse_th = 10;
    fine_xy   = 0.04;
    fine_th   = 2;

    if ~any(occMap(:))
        error('globalLocalize: occMap has no occupied cells.');
    end
    dt           = double(bwdist(occMap)) / mscale;
    dt(dt > cap) = cap;
    [H, W]       = size(occMap);

    ranges = lddata.Ranges(:);
    angles = lddata.Angles(:);
    keep   = isfinite(ranges) & ranges >= 0.12 & ranges <= max_range;
    ranges = ranges(keep);
    angles = angles(keep);
    if numel(ranges) < 10
        error('globalLocalize: too few valid beams (%d).', numel(ranges));
    end
    sel    = round(linspace(1, numel(ranges), min(numel(ranges), nMatch)));
    ranges = ranges(sel);
    angles = angles(sel);
    nB     = numel(ranges);

    % search domain = obstacle bounding box + margin
    [io, jo] = find(occMap);
    margin   = round(0.30 * mscale);
    i_lo = max(1, min(io) - margin); i_hi = min(H, max(io) + margin);
    j_lo = max(1, min(jo) - margin); j_hi = min(W, max(jo) + margin);
    x_lo = (i_lo-1)/mscale - offset_m; x_hi = (i_hi-1)/mscale - offset_m;
    y_lo = (j_lo-1)/mscale - offset_m; y_hi = (j_hi-1)/mscale - offset_m;

    % coarse pass
    xs_c = x_lo : coarse_xy : x_hi;
    ys_c = y_lo : coarse_xy : y_hi;
    th_c = (0 : coarse_th : 360 - coarse_th) * pi/180;
    p0   = searchGrid(xs_c, ys_c, th_c);

    % fine pass 
    xs_f = p0(1) + (-coarse_xy : fine_xy : coarse_xy);
    ys_f = p0(2) + (-coarse_xy : fine_xy : coarse_xy);
    th_f = p0(3) + (-coarse_th : fine_th : coarse_th) * pi/180;
    [p0, score] = searchGrid(xs_f, ys_f, th_f);

    p0    = p0(:);
    p0(3) = atan2(sin(p0(3)), cos(p0(3)));

    function [best_p, best_s] = searchGrid(xv, yv, thv)
        best_s = inf;
        best_p = [xv(1); yv(1); thv(1)];
        xso = sensor_offset(1); yso = sensor_offset(2);

        for th = thv
            cb  = cos(th + angles);
            sb  = sin(th + angles);
            rcb = ranges .* cb;
            rsb = ranges .* sb;
            dxs = xso*cos(th) - yso*sin(th);
            dys = xso*sin(th) + yso*cos(th);

            for x = xv
                ri = round((x + offset_m)*mscale) + 1;
                if ri < 1 || ri > H, continue; end
                xs  = x + dxs;
                ex  = xs + rcb;
                gi  = round((ex + offset_m)*mscale) + 1;
                iok = gi >= 1 & gi <= H;

                for y = yv
                    rj = round((y + offset_m)*mscale) + 1;
                    if rj < 1 || rj > W, continue; end
                    if occMap(ri, rj), continue; end

                    ys = y + dys;
                    ey = ys + rsb;
                    gj = round((ey + offset_m)*mscale) + 1;
                    in = iok & gj >= 1 & gj <= W;

                    s = cap * (nB - sum(in));
                    if any(in)
                        idx = gi(in) + (gj(in) - 1)*H;
                        s   = s + sum(dt(idx));
                    end
                    s = s / nB;

                    if s < best_s
                        best_s = s;
                        best_p = [x; y; th];
                    end
                end
            end
        end
    end
end
