function [plantOut, plantState] = plant_bicycle(plantState, actuatorCmd, steerDriver, params, dt)
%PLANT_BICYCLE 2-DOF Bicycle Model 플랜트 래퍼 (selectable solver)
%
%   기존 calc_bicycle_model.m을 사용하여 선형 2-DOF 모델을 실행한다.
%   가장 단순한 플랜트로, 횡방향 동역학만 모델링한다.
%   종방향 속도는 일정(상수), 롤/피치/서스펜션은 zero-fill.
%
%   상태: x = [vy; yawRate] (2 states)
%   적분: params.SIM.solver 토글로 선택 ('ode45'(default)/'ode23'/'ode15s'/'rk4'/'euler')

    VEH = params.VEH;
    vx  = plantState.vx;  % 일정 속도
    x   = plantState.x;

    %% 상태공간 행렬 (vx 가 substep 내 constant → A,B 도 constant)
    [A, B, ~, ~] = calc_bicycle_model(vx, VEH);

    %% 총 조향각 = 운전자 + AFS 보조
    u_total = steerDriver + actuatorCmd.steerAngle;

    %% 차동 브레이크로부터 요 모멘트 역산
    % coordinator가 yawMoment → differential brake로 변환했으므로 역으로 복원
    brk = actuatorCmd.brakeTorque;
    Mz = (brk(2) - brk(1)) * VEH.track_f / (2 * VEH.rw) + ...
         (brk(4) - brk(3)) * VEH.track_r / (2 * VEH.rw);
    % 부호: brk(FR) > brk(FL) → 좌회전 모멘트(+)

    %% 적분기 dispatch
    SIM = getfield_default(params, 'SIM', struct());
    solver  = lower(getfield_default(SIM, 'solver', 'ode45'));
    relTol  = getfield_default(SIM, 'solver_RelTol', 1e-3);
    absTol  = getfield_default(SIM, 'solver_AbsTol', 1e-5);

    odefun = @(tt, xx) calc_derivatives_bicycle(xx, u_total, Mz, A, B, VEH.Iz);

    switch solver
        case 'euler'
            dxk = calc_derivatives_bicycle(x, u_total, Mz, A, B, VEH.Iz);
            x = x + dxk * dt;
        case 'rk4'
            k1 = calc_derivatives_bicycle(x,             u_total, Mz, A, B, VEH.Iz);
            k2 = calc_derivatives_bicycle(x + 0.5*dt*k1, u_total, Mz, A, B, VEH.Iz);
            k3 = calc_derivatives_bicycle(x + 0.5*dt*k2, u_total, Mz, A, B, VEH.Iz);
            k4 = calc_derivatives_bicycle(x + dt*k3,     u_total, Mz, A, B, VEH.Iz);
            x = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
        case {'ode45','ode23','ode15s','ode23t','ode23s','ode113'}
            opts = odeset('RelTol', relTol, 'AbsTol', absTol, 'MaxStep', dt);
            [~, X] = feval(solver, odefun, [0 dt], x, opts);
            x = X(end, :).';
        otherwise
            error('plant_bicycle:unknownSolver', 'Unknown SIM.solver: %s', solver);
    end

    plantState.x = x;

    %% 출력 매핑
    vy = x(1);
    yr = x(2);

    plantOut.vx        = vx;
    plantOut.vy        = vy;
    plantOut.ax        = 0;
    plantOut.ay        = vx * yr;  % 근사: ay ≈ vx * r
    plantOut.yawRate   = yr;
    plantOut.slipAngle = atan2(vy, vx);
    plantOut.roll      = 0;
    plantOut.pitch     = 0;
    plantOut.suspVel   = zeros(4, 1);
    plantOut.bodyVel   = zeros(4, 1);

    % 타이어/서스펜션 zero-fill
    plantOut.tire = fill_zero_tire();
    plantOut.susp = fill_zero_susp();

end

%% ---- Local Functions ----

function dx = calc_derivatives_bicycle(x, u_total, Mz, A, B, Iz)
% 2-DOF Bicycle Model 상태 미분.
%   x = [vy; yawRate], u_total = 총 조향각 [rad], Mz = 차동 브레이크 요 모멘트 [Nm]
%   A, B 는 선형 상태공간 행렬 (vx 고정 → substep 내 constant)
    dx = A * x + B * u_total;
    dx(2) = dx(2) + Mz / Iz;
end

function tire = fill_zero_tire()
    wheels = {'FL','FR','RL','RR'};
    for w = 1:4
        tire.(wheels{w}).Fx        = 0;
        tire.(wheels{w}).Fy        = 0;
        tire.(wheels{w}).Fz        = 0;
        tire.(wheels{w}).slipAngle = 0;
        tire.(wheels{w}).slipRatio = 0;
    end
end

function susp = fill_zero_susp()
    wheels = {'FL','FR','RL','RR'};
    for w = 1:4
        susp.(wheels{w}).springFrc  = 0;
        susp.(wheels{w}).damperFrc  = 0;
    end
end
