function [plantOut, plantState] = plant_3dof(plantState, actuatorCmd, steerDriver, params, dt)
%PLANT_3DOF 비선형 3-DOF 차량 모델 (selectable solver)
%
%   3자유도: 종방향(vx), 횡방향(vy), 요(yawRate)
%   비선형 Magic Formula 타이어 횡력 + 마찰원 saturation.
%   Bicycle Model 대비: 비선형 타이어 포화, 종방향 동역학(제동/공력) 추가.
%
%   상태: x = [vx; vy; yawRate] (3 states)
%   적분: params.SIM.solver 토글로 선택 ('ode45'(default)/'ode23'/'ode15s'/'rk4'/'euler')

    VEH   = params.VEH;
    TIRE  = params.TIRE;
    CONST = params.CONST;
    x     = plantState.x;

    %% 조향각
    delta = steerDriver + actuatorCmd.steerAngle;

    %% 정지 영역 brake 토크 점진 감쇠 (vx<0.5 m/s) — substep 시작 전 한 번
    actCmd = actuatorCmd;
    vxAbs = abs(x(1));
    if vxAbs < 0.5
        scale = max(0, (vxAbs - 0.1) / 0.4);
        actCmd.brakeTorque = actuatorCmd.brakeTorque * scale;
    end

    %% substep 동안 동일하게 사용할 ax_prev
    if isfield(plantState, 'ax_prev')
        ax_prev = plantState.ax_prev;
    else
        ax_prev = 0;
    end

    %% 적분기 dispatch
    SIM = getfield_default(params, 'SIM', struct());
    solver  = lower(getfield_default(SIM, 'solver', 'ode45'));
    relTol  = getfield_default(SIM, 'solver_RelTol', 1e-3);
    absTol  = getfield_default(SIM, 'solver_AbsTol', 1e-5);

    odefun = @(tt, xx) calc_derivatives_3dof(xx, delta, actCmd, VEH, TIRE, CONST, ax_prev);

    switch solver
        case 'rk4'
            k1 = calc_derivatives_3dof(x,             delta, actCmd, VEH, TIRE, CONST, ax_prev);
            k2 = calc_derivatives_3dof(x + 0.5*dt*k1, delta, actCmd, VEH, TIRE, CONST, ax_prev);
            k3 = calc_derivatives_3dof(x + 0.5*dt*k2, delta, actCmd, VEH, TIRE, CONST, ax_prev);
            k4 = calc_derivatives_3dof(x + dt*k3,     delta, actCmd, VEH, TIRE, CONST, ax_prev);
            x = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
        case 'euler'
            dxk = calc_derivatives_3dof(x, delta, actCmd, VEH, TIRE, CONST, ax_prev);
            x = x + dt * dxk;
        case {'ode45','ode23','ode15s','ode23t','ode23s','ode113'}
            opts = odeset('RelTol', relTol, 'AbsTol', absTol, 'MaxStep', dt);
            [~, X] = feval(solver, odefun, [0 dt], x, opts);
            x = X(end, :).';
        otherwise
            error('plant_3dof:unknownSolver', 'Unknown SIM.solver: %s', solver);
    end

    % 속도 보호: vx > 0
    x(1) = max(x(1), 0.1);

    plantState.x  = x;
    plantState.vx = x(1);

    %% 출력 매핑 (최종 상태에서 한 번 더 미분/힘 계산)
    [dx_final, out] = calc_derivatives_3dof(x, delta, actCmd, VEH, TIRE, CONST, ax_prev);

    % 다음 스텝 load transfer 입력 갱신
    plantState.ax_prev = dx_final(1) - x(2) * x(3);

    plantOut.vx        = x(1);
    plantOut.vy        = x(2);
    plantOut.yawRate   = x(3);
    plantOut.ax        = dx_final(1) - x(2) * x(3);   % 관성계 종가속도
    plantOut.ay        = dx_final(2) + x(1) * x(3);   % 관성계 횡가속도
    plantOut.slipAngle = atan2(x(2), x(1));
    plantOut.roll      = 0;
    plantOut.pitch     = 0;
    plantOut.suspVel   = zeros(4, 1);
    plantOut.bodyVel   = zeros(4, 1);

    %% 타이어/서스 출력 (axle → per-wheel 균등 배분, saturation 반영)
    wheels = {'FL','FR','RL','RR'};
    Fx_pw = [out.Fx_f/2; out.Fx_f/2; out.Fx_r/2; out.Fx_r/2];
    Fy_pw = [out.Fy_f/2; out.Fy_f/2; out.Fy_r/2; out.Fy_r/2];
    Fz_pw = [out.Fz_f/2; out.Fz_f/2; out.Fz_r/2; out.Fz_r/2];
    sa_pw = [out.alpha_f; out.alpha_f; out.alpha_r; out.alpha_r];
    for w = 1:4
        plantOut.tire.(wheels{w}).Fx        = Fx_pw(w);
        plantOut.tire.(wheels{w}).Fy        = Fy_pw(w);
        plantOut.tire.(wheels{w}).Fz        = Fz_pw(w);
        plantOut.tire.(wheels{w}).slipAngle = sa_pw(w);
        plantOut.tire.(wheels{w}).slipRatio = 0;
    end
    for w = 1:4
        plantOut.susp.(wheels{w}).springFrc  = 0;
        plantOut.susp.(wheels{w}).damperFrc  = 0;
    end

end

%% ============================================================
function [dx, out] = calc_derivatives_3dof(x, delta, actCmd, VEH, TIRE, CONST, ax_prev)
% 3-DOF 상태 미분 계산 (RK4 substep용).
%   x = [vx; vy; yawRate]
%   ax_prev: 외부에서 제공되는 이전 스텝 종가속도 (load transfer 입력, substep 내 동일)

    vx = x(1);
    vy = x(2);
    r  = x(3);

    %% 슬립 앵글
    vx_safe = max(abs(vx), 1.0) * sign(vx + eps);
    alpha_f = delta - atan2(vy + VEH.lf * r, vx_safe);
    alpha_r = -atan2(vy - VEH.lr * r, vx_safe);

    %% 수직 하중 (정적 + 종방향 동적 하중 이동)
    Fz_f_static = VEH.mass * CONST.g * VEH.lr / VEH.L;
    Fz_r_static = VEH.mass * CONST.g * VEH.lf / VEH.L;
    dFz_lon     = -VEH.mass * ax_prev * VEH.h_cog / VEH.L;
    Fz_f = max(Fz_f_static + dFz_lon, 10);
    Fz_r = max(Fz_r_static - dFz_lon, 10);

    %% 타이어 횡력 (axle-level, 단일 슬립 → 디스패처에서 횡력만 사용)
    [~, Fy_f] = tire_force(0, alpha_f, Fz_f, 0, TIRE);
    [~, Fy_r] = tire_force(0, alpha_r, Fz_r, 0, TIRE);

    %% 종방향 힘 + 마찰원 saturation (축별)
    brk = actCmd.brakeTorque;
    Fx_f_cmd = -(brk(1) + brk(2)) / VEH.rw;
    Fx_r_cmd = -(brk(3) + brk(4)) / VEH.rw;
    muFz_f = TIRE.mu_peak * Fz_f;
    muFz_r = TIRE.mu_peak * Fz_r;
    Fx_f = local_clip_friction_ellipse(Fx_f_cmd, Fy_f, muFz_f);
    Fx_r = local_clip_friction_ellipse(Fx_r_cmd, Fy_r, muFz_r);
    Fx_brk = Fx_f + Fx_r;

    %% 공기 저항
    Faero = 0.5 * CONST.rho_air * VEH.Cd * VEH.Af * vx^2 * sign(vx);

    %% 차동 브레이크 요 모멘트
    Mz_brk = (brk(2) - brk(1)) / VEH.rw * VEH.track_f / 2 + ...
             (brk(4) - brk(3)) / VEH.rw * VEH.track_r / 2;

    %% 운동 방정식 (차체 고정 좌표계)
    dvx = (Fx_brk - Faero) / VEH.mass + vy * r;
    dvy = (Fy_f + Fy_r) / VEH.mass - vx * r;
    dr  = (VEH.lf * Fy_f - VEH.lr * Fy_r + Mz_brk) / VEH.Iz;

    dx = [dvx; dvy; dr];

    if nargout > 1
        out.alpha_f = alpha_f;
        out.alpha_r = alpha_r;
        out.Fy_f    = Fy_f;
        out.Fy_r    = Fy_r;
        out.Fx_f    = Fx_f;
        out.Fx_r    = Fx_r;
        out.Fz_f    = Fz_f;
        out.Fz_r    = Fz_r;
    end
end

%% --------------------------------------------------------------
function Fx_out = local_clip_friction_ellipse(Fx_in, Fy_in, muFz)
% 마찰원 제약: sqrt(Fx² + Fy²) <= muFz. Fx의 magnitude만 줄임(부호 보존).
    if muFz <= 0
        Fx_out = 0; return;
    end
    Fy_sq = Fy_in^2;
    if Fy_sq >= muFz^2
        Fx_out = 0;
        return;
    end
    Fx_avail = sqrt(muFz^2 - Fy_sq);
    if abs(Fx_in) <= Fx_avail
        Fx_out = Fx_in;
    else
        Fx_out = sign(Fx_in) * Fx_avail;
    end
end
