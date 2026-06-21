function [plantOut, plantState] = plant_7dof(plantState, actuatorCmd, steerDriver, params, dt)
%PLANT_7DOF 7-DOF 차량 모델 (3DOF 차체 + 4 바퀴 회전, selectable solver)
%
%   3DOF 차체에 4개 바퀴 회전 자유도를 추가하여 타이어 슬립 비를 계산한다.
%   ABS/TCS 제어기 검증에 필요한 바퀴 잠김/공전 현상을 재현할 수 있다.
%   복합 슬립 Magic Formula (tire_combined_slip.m)를 사용한다.
%
%   상태: x = [vx; vy; yawRate; omega_FL; omega_FR; omega_RL; omega_RR] (7 states)
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

    odefun = @(tt, xx) calc_derivatives_7dof(xx, delta, actCmd, VEH, TIRE, CONST, ax_prev);

    switch solver
        case 'rk4'
            k1 = calc_derivatives_7dof(x,             delta, actCmd, VEH, TIRE, CONST, ax_prev);
            k2 = calc_derivatives_7dof(x + 0.5*dt*k1, delta, actCmd, VEH, TIRE, CONST, ax_prev);
            k3 = calc_derivatives_7dof(x + 0.5*dt*k2, delta, actCmd, VEH, TIRE, CONST, ax_prev);
            k4 = calc_derivatives_7dof(x + dt*k3,     delta, actCmd, VEH, TIRE, CONST, ax_prev);
            x = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
        case 'euler'
            dxk = calc_derivatives_7dof(x, delta, actCmd, VEH, TIRE, CONST, ax_prev);
            x = x + dt * dxk;
        case {'ode45','ode23','ode15s','ode23t','ode23s','ode113'}
            opts = odeset('RelTol', relTol, 'AbsTol', absTol, 'MaxStep', dt);
            [~, X] = feval(solver, odefun, [0 dt], x, opts);
            x = X(end, :).';
        otherwise
            error('plant_7dof:unknownSolver', 'Unknown SIM.solver: %s', solver);
    end

    % 보호
    x(1)   = max(x(1), 0.1);     % vx > 0
    x(4:7) = max(x(4:7), 0);     % omega >= 0

    plantState.x  = x;
    plantState.vx = x(1);

    %% 출력 매핑 (최종 상태에서 한 번 더 계산)
    [dx_final, out] = calc_derivatives_7dof(x, delta, actCmd, VEH, TIRE, CONST, ax_prev);

    % 다음 스텝 load transfer 입력 갱신
    plantState.ax_prev = dx_final(1) - x(2) * x(3);

    plantOut.vx        = x(1);
    plantOut.vy        = x(2);
    plantOut.yawRate   = x(3);
    plantOut.ax        = dx_final(1) - x(2) * x(3);
    plantOut.ay        = dx_final(2) + x(1) * x(3);
    plantOut.slipAngle = atan2(x(2), x(1));
    plantOut.roll      = 0;
    plantOut.pitch     = 0;
    plantOut.suspVel   = zeros(4, 1);
    plantOut.bodyVel   = zeros(4, 1);

    wheels = {'FL','FR','RL','RR'};
    for w = 1:4
        plantOut.tire.(wheels{w}).Fx        = out.Fx_tire(w);
        plantOut.tire.(wheels{w}).Fy        = out.Fy_tire(w);
        plantOut.tire.(wheels{w}).Fz        = out.Fz(w);
        plantOut.tire.(wheels{w}).slipAngle = out.alpha(w);
        plantOut.tire.(wheels{w}).slipRatio = out.kappa(w);
    end
    for w = 1:4
        plantOut.susp.(wheels{w}).springFrc  = 0;
        plantOut.susp.(wheels{w}).damperFrc  = 0;
    end

end

%% ============================================================
function [dx, out] = calc_derivatives_7dof(x, delta, actCmd, VEH, TIRE, CONST, ax_prev)
% 7-DOF 상태 미분 계산 (RK4 substep용).
%   x = [vx; vy; r; omega_FL; omega_FR; omega_RL; omega_RR]

    vx = x(1);
    vy = x(2);
    r  = x(3);
    omega = x(4:7);

    htf = VEH.track_f / 2;
    htr = VEH.track_r / 2;

    %% 바퀴별 종/횡 속도
    vx_w = [vx - r*htf;  vx + r*htf;  vx - r*htr;  vx + r*htr];
    vy_w = [vy + VEH.lf*r;  vy + VEH.lf*r;  vy - VEH.lr*r;  vy - VEH.lr*r];
    vx_w_safe = max(abs(vx_w), 0.5) .* sign(vx_w + eps);

    %% 슬립
    kappa = (omega * VEH.rw - vx_w) ./ abs(vx_w_safe);
    alpha = zeros(4, 1);
    alpha(1) = delta - atan2(vy_w(1), vx_w_safe(1));
    alpha(2) = delta - atan2(vy_w(2), vx_w_safe(2));
    alpha(3) = -atan2(vy_w(3), vx_w_safe(3));
    alpha(4) = -atan2(vy_w(4), vx_w_safe(4));

    %% 수직 하중 (정적 + 하중 이동)
    ms = VEH.ms;
    mu_w = VEH.mu_w;
    Fz_static_f = (ms * VEH.lr / VEH.L / 2 + mu_w) * CONST.g;
    Fz_static_r = (ms * VEH.lf / VEH.L / 2 + mu_w) * CONST.g;

    ay_approx = vx * r;
    dFz_lat_f = ms * ay_approx * VEH.h_cog / VEH.track_f;
    dFz_lat_r = ms * ay_approx * VEH.h_cog / VEH.track_r;
    dFz_lon   = -ms * ax_prev * VEH.h_cog / VEH.L;

    Fz = zeros(4, 1);
    Fz(1) = Fz_static_f - dFz_lat_f/2 + dFz_lon/2;
    Fz(2) = Fz_static_f + dFz_lat_f/2 + dFz_lon/2;
    Fz(3) = Fz_static_r - dFz_lat_r/2 - dFz_lon/2;
    Fz(4) = Fz_static_r + dFz_lat_r/2 - dFz_lon/2;
    Fz = max(Fz, 10);

    %% 타이어 힘 (combined slip)
    Fx_tire = zeros(4, 1);
    Fy_tire = zeros(4, 1);
    for i = 1:4
        [Fx_tire(i), Fy_tire(i)] = tire_force(kappa(i), alpha(i), Fz(i), 0, TIRE);
    end

    %% 전륜 → 차체 좌표계
    Fx_body = zeros(4, 1);
    Fy_body = zeros(4, 1);
    cd = cos(delta); sd = sin(delta);
    Fx_body(1) = Fx_tire(1)*cd - Fy_tire(1)*sd;
    Fy_body(1) = Fx_tire(1)*sd + Fy_tire(1)*cd;
    Fx_body(2) = Fx_tire(2)*cd - Fy_tire(2)*sd;
    Fy_body(2) = Fx_tire(2)*sd + Fy_tire(2)*cd;
    Fx_body(3:4) = Fx_tire(3:4);
    Fy_body(3:4) = Fy_tire(3:4);

    %% 공기 저항
    Faero = 0.5 * CONST.rho_air * VEH.Cd * VEH.Af * vx^2 * sign(vx);

    %% 차체 운동
    Fx_sum = sum(Fx_body) - Faero;
    Fy_sum = sum(Fy_body);
    Mz_sum = VEH.lf * (Fy_body(1) + Fy_body(2)) ...
           - VEH.lr * (Fy_body(3) + Fy_body(4)) ...
           + htf * (Fx_body(2) - Fx_body(1)) ...
           + htr * (Fx_body(4) - Fx_body(3));

    dvx = Fx_sum / VEH.mass + vy * r;
    dvy = Fy_sum / VEH.mass - vx * r;
    dr  = Mz_sum / VEH.Iz;

    %% 바퀴 회전
    brk = actCmd.brakeTorque;
    domega = zeros(4, 1);
    for i = 1:4
        domega(i) = (-brk(i) - Fx_tire(i) * VEH.rw) / VEH.Iw;
    end

    dx = [dvx; dvy; dr; domega];

    if nargout > 1
        out.Fx_tire = Fx_tire;
        out.Fy_tire = Fy_tire;
        out.Fz      = Fz;
        out.alpha   = alpha;
        out.kappa   = kappa;
    end
end
