function [delta, drvState] = driver_path_follow(scenario, pose, vx, t, drvState)
%DRIVER_PATH_FOLLOW 경로 추종 closed-loop driver (Stanley 또는 Pure Pursuit)
%
%   [delta, drvState] = DRIVER_PATH_FOLLOW(scenario, pose, vx, t, drvState)
%
%   Inputs:
%       scenario.refPath  - N×2 [x_ref, y_ref] 목표 경로 (필수)
%       scenario.driverType - 'path_follow_stanley' 또는 'path_follow_purepursuit'
%       pose - struct(x, y, psi)
%       vx, t - 현재 종속도, 시간
%       drvState - 누적 상태 (이번 구현에는 무사용)
%
%   Output:
%       delta - 노면휠 조향각 [rad]
%
%   Stanley:
%       δ = θ_e + atan(k · e_lat / max(vx, v_min))
%       전축 중심 기준의 cross-track + heading 오차 합산.
%
%   Pure Pursuit:
%       δ = atan(2 · L · sin(α) / Ld)
%       L = wheelbase, Ld = lookahead, α = bearing from rear axle to lookahead point.

    if ~isfield(scenario, 'refPath') || isempty(scenario.refPath)
        error('[driver_path_follow] scenario.refPath required');
    end
    if ~isfield(drvState, 'L'); drvState.L = 2.7; end                 % wheelbase
    if ~isfield(drvState, 'k_stanley');     drvState.k_stanley = 1.5; end
    if ~isfield(drvState, 'Ld_purepursuit'); drvState.Ld_purepursuit = max(0.5*vx, 5); end
    if ~isfield(drvState, 'v_min'); drvState.v_min = 1.0; end

    % refPath dense spline (1회 캐시: heading 점프 제거 — DLC waypoint 사이 부드러운 곡선)
    if ~isfield(drvState, 'refX_dense')
        rx = scenario.refPath(:,1); ry = scenario.refPath(:,2);
        s_in = [0; cumsum(sqrt(diff(rx).^2 + diff(ry).^2))];
        s_dense = linspace(0, s_in(end), max(200, 5*numel(rx)))';
        drvState.refX_dense = pchip(s_in, rx, s_dense);
        drvState.refY_dense = pchip(s_in, ry, s_dense);
    end
    refX = drvState.refX_dense;
    refY = drvState.refY_dense;

    px = pose.x; py = pose.y; psi = pose.psi;

    dtype = 'path_follow_stanley';
    if isfield(scenario, 'driverType'); dtype = scenario.driverType; end

    switch lower(dtype)
        case {'path_follow', 'path_follow_stanley'}
            delta = local_stanley(px, py, psi, vx, refX, refY, ...
                                  drvState.L, drvState.k_stanley, drvState.v_min);
        case 'path_follow_purepursuit'
            % Lookahead 길이 = max(2 m, 0.5·vx) (속도 비례)
            Ld = max(2.0, 0.5 * vx);
            delta = local_pure_pursuit(px, py, psi, refX, refY, drvState.L, Ld);
        otherwise
            error('[driver_path_follow] Unknown driverType: %s', dtype);
    end
end

%% ============================================================
function delta = local_stanley(px, py, psi, vx, refX, refY, L, k, v_min)
% 전축 중심 좌표
    fx = px + L * cos(psi);
    fy = py + L * sin(psi);

    % 가장 가까운 경로 점
    d2 = (refX - fx).^2 + (refY - fy).^2;
    [~, idx] = min(d2);
    idxNext = min(idx+1, numel(refX));

    % 경로 heading (tangent)
    if idxNext > idx
        psi_path = atan2(refY(idxNext) - refY(idx), refX(idxNext) - refX(idx));
    else
        psi_path = atan2(refY(idx) - refY(max(idx-1,1)), refX(idx) - refX(max(idx-1,1)));
    end

    % heading error
    theta_e = local_wrap_to_pi(psi_path - psi);

    % cross-track error (signed: left positive)
    dx = fx - refX(idx);
    dy = fy - refY(idx);
    e_lat = -sin(psi_path)*dx + cos(psi_path)*dy;
    e_lat = -e_lat;   % 좌측 양수 → 우조향 양수 convention

    delta = theta_e + atan2(k * e_lat, max(vx, v_min));
    delta = max(-deg2rad(40), min(deg2rad(40), delta));
end

%% ============================================================
function delta = local_pure_pursuit(px, py, psi, refX, refY, L, Ld)
% rear axle 기준 (간략화: pose.x/y가 CG 또는 rear axle 가정)
    % lookahead 점: 현재 위치에서 forward 방향으로 Ld 거리 이상 떨어진 첫 ref 점
    dx = refX - px; dy = refY - py;
    s = sqrt(dx.^2 + dy.^2);
    % forward only (path direction과 차량 heading 사이 각도 < 90°)
    fwdMask = (cos(psi)*dx + sin(psi)*dy) > 0;
    cand = find(fwdMask & s >= Ld, 1, 'first');
    if isempty(cand)
        cand = numel(refX);
    end
    Lx = refX(cand) - px;
    Ly = refY(cand) - py;
    % vehicle frame
    Lx_v =  cos(psi)*Lx + sin(psi)*Ly;
    Ly_v = -sin(psi)*Lx + cos(psi)*Ly;
    alpha = atan2(Ly_v, Lx_v);
    Ld_actual = sqrt(Lx_v^2 + Ly_v^2);
    if Ld_actual < 1e-3
        delta = 0; return;
    end
    delta = atan2(2 * L * sin(alpha), Ld_actual);
    delta = max(-deg2rad(40), min(deg2rad(40), delta));
end

%% ============================================================
function a = local_wrap_to_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end
