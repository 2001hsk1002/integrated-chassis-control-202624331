function out = kpi_lateral_path_deviation(t, x_pos, y_pos, refPath)
%KPI_LATERAL_PATH_DEVIATION 차량 궤적과 reference path의 lateral 거리 KPI (A.5)
%
%   refPath: N×2 [x, y] reference center line
%
%   각 시점의 차량 위치 (x_pos, y_pos)에서 refPath 위 가장 가까운 점까지의
%   euclidean 거리를 lateral deviation으로 사용.

    if isempty(refPath) || size(refPath,1) < 2
        out.devMax = NaN; out.devRMS = NaN; return;
    end

    % refPath segment 별 점-선 perpendicular 거리. 트랙 길이 밖 (s<0 or s>L)는 평가 제외.
    rx = refPath(:,1); ry = refPath(:,2);
    n = numel(t);
    d = nan(n,1);
    for k = 1:n
        d(k) = local_min_perp_distance(rx, ry, x_pos(k), y_pos(k));
    end

    valid = ~isnan(d);
    if any(valid)
        out.devMax = max(d(valid));
        out.devRMS = sqrt(mean(d(valid).^2));
    else
        out.devMax = NaN; out.devRMS = NaN;
    end
    out.devTimeSeries = d;
end

function dmin = local_min_perp_distance(rx, ry, px, py)
% refPath의 각 segment 에 대해 perpendicular projection 후 segment 내부 (0≤s≤1) 인 것만 평가.
% 트랙 시작 전/끝 후의 점은 NaN.
    dmin = inf;
    insideAny = false;
    for i = 1:numel(rx)-1
        x1 = rx(i);   y1 = ry(i);
        x2 = rx(i+1); y2 = ry(i+1);
        vx = x2 - x1; vy = y2 - y1;
        wx = px - x1; wy = py - y1;
        L2 = vx*vx + vy*vy;
        if L2 < 1e-9; continue; end
        s = (wx*vx + wy*vy) / L2;
        if s < 0 || s > 1; continue; end
        insideAny = true;
        % perp 거리 계산
        projx = x1 + s*vx;
        projy = y1 + s*vy;
        d = sqrt((px - projx)^2 + (py - projy)^2);
        if d < dmin; dmin = d; end
    end
    if ~insideAny; dmin = NaN; end
end
