function [steerFn, brakeFn, info] = scn_trajectory_to_input(t, x, y, opts)
%SCN_TRAJECTORY_TO_INPUT 차량 trajectory (x,y,t) → 운전자 입력 (steer, brake) 역산
%
%   [steerFn, brakeFn, info] = SCN_TRAJECTORY_TO_INPUT(t, x, y, opts)
%
%   Bicycle 역기구학으로 trajectory에서 운전자 입력 재구성:
%     vx, vy        = d(x,y)/dt (저역통과)
%     psi           = atan2(dy/dt, dx/dt)
%     r (yaw rate)  = d(psi)/dt
%     delta         = atan(L * r / vx)             (bicycle 가정)
%     ax            = d(vx)/dt
%     brake_torque  = -m * ax * r_w (단순 가정: ax<0일 때만 brake)
%
%   Inputs:
%       t, x, y - trajectory time + position (N×1 each)
%       opts - struct:
%           .wheelbase  (default 2.7 m)
%           .mass       (default 1600 kg)
%           .rw         (default 0.33 m)
%           .smooth_dt  (default 0.05 s, low-pass filter window)
%           .brakeBias  (default [0.55 0.55 0.45 0.45] / 2 — F:R = 55:45 분배)
%
%   Outputs:
%       steerFn  - function handle @(t) → roadwheel angle [rad]
%       brakeFn  - function handle @(t) → 4×1 brake torque [Nm]
%       info     - struct with internal time series for inspection

    if nargin < 4; opts = struct(); end
    if ~isfield(opts,'wheelbase'); opts.wheelbase = 2.7; end
    if ~isfield(opts,'mass');      opts.mass = 1600; end
    if ~isfield(opts,'rw');        opts.rw = 0.33; end
    if ~isfield(opts,'smooth_dt'); opts.smooth_dt = 0.05; end
    if ~isfield(opts,'brakeBias'); opts.brakeBias = [0.275 0.275 0.225 0.225]; end

    t = t(:); x = x(:); y = y(:);

    % drivingScenario.export inserts duplicate timestamps at action boundaries.
    % interp1 demands strictly monotonic X, so dedupe while preserving order.
    [t, iu] = unique(t, 'stable');
    x = x(iu);
    y = y(iu);
    [t, isrt] = sort(t);
    x = x(isrt);
    y = y(isrt);

    n = numel(t);
    if n < 5
        error('[scn_trajectory_to_input] need at least 5 waypoints');
    end
    dt = mean(diff(t));

    %% Low-pass smoothing window (moving average)
    win = max(round(opts.smooth_dt / dt), 3);
    smooth = @(v) movmean(v, win, 'Endpoints', 'shrink');

    %% Derivative chain
    vx_raw = gradient(x, t);     vx = smooth(vx_raw);
    vy_raw = gradient(y, t);     vy = smooth(vy_raw);
    speed  = sqrt(vx.^2 + vy.^2);
    psi    = unwrap(atan2(vy, vx));
    r_raw  = gradient(psi, t);   r = smooth(r_raw);
    ax_raw = gradient(speed, t); ax = smooth(ax_raw);

    %% Bicycle inverse: delta = atan(L · r / max(speed, ε))
    speed_safe = max(speed, 1.0);
    delta = atan(opts.wheelbase * r ./ speed_safe);

    %% Brake torque (ax < 0 only)
    Fx_total = opts.mass * ax;
    brakeTorque_total = zeros(n, 1);
    decel_mask = Fx_total < 0;
    brakeTorque_total(decel_mask) = -Fx_total(decel_mask) * opts.rw;
    brakeMtx = brakeTorque_total .* opts.brakeBias;

    %% Time-function handles (linear interpolation, extrap=0)
    steerFn = @(tq) local_interp(t, delta, tq, 0);
    brakeFn = @(tq) local_brake_interp(t, brakeMtx, tq);

    %% Info struct
    info.t = t;
    info.vx = vx; info.vy = vy; info.speed = speed;
    info.psi = psi; info.r = r;
    info.ax = ax;
    info.delta = delta;
    info.brakeTorque = brakeMtx;
    info.smooth_dt = opts.smooth_dt;
end

function y = local_interp(t, v, tq, exVal)
    t = t(:); v = v(:);
    if ~isscalar(tq) || isnan(tq) || tq < t(1) || tq > t(end)
        y = exVal; return;
    end
    y = interp1(t, v, tq, 'linear', exVal);
    if isempty(y) || ~isscalar(y) || isnan(y); y = exVal; end
end

function bk = local_brake_interp(t, B, tq)
    bk = zeros(4,1);
    t = t(:);
    if ~isscalar(tq) || isnan(tq) || tq < t(1) || tq > t(end)
        return;
    end
    for w = 1:4
        v = interp1(t, B(:,w), tq, 'linear', 0);
        if ~isempty(v) && isscalar(v) && ~isnan(v); bk(w) = v; end
    end
end
