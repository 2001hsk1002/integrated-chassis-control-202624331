function [plantOut, plantState] = plant_14dof(plantState, actuatorCmd, steerDriver, params, dt)
%PLANT_14DOF 14-DOF 차량 모델 (6DOF 차체 + 4 서스펜션 + 4 바퀴 회전)
%
%   전체 차량 동역학 모델: 차체 6자유도(병진3+회전3)에서 heave를 서스펜션으로 대체,
%   4코너 서스펜션 수직 동역학, 4바퀴 회전 동역학.
%   CDC 반능동 서스펜션 제어기(ctrl_vertical)에 실제 데이터를 공급한다.
%
%   상태 (19): [vx; vy; yawRate; roll; rollRate; pitch; pitchRate;
%               zs_FL..RR(4); zsdot_FL..RR(4); omega_FL..RR(4)]
%
%   적분: params.SIM.solver 토글로 선택 ('ode45'(default)/'ode23'/'ode15s'/'rk4'/'euler').
%        서스펜션 고주파 동역학 안정성 위해 'rk4' 또는 'ode15s' 권장.

    VEH   = params.VEH;
    TIRE  = params.TIRE;
    CONST = params.CONST;

    x = plantState.x;
    actCmd = actuatorCmd;
    delta = steerDriver + actCmd.steerAngle;

    % 차량 정지 영역 (vx < 0.5 m/s) — brake 토크 그라데이션 다운,
    % 결과적으로 정지 후 추가 가속도 발생 방지 (validation/제어 의도 보호)
    vxAbs = abs(x(1));
    if vxAbs < 0.5
        scale = max(0, (vxAbs - 0.1) / 0.4);  % 0.1 m/s → 0, 0.5 m/s → 1
        actCmd.brakeTorque = actCmd.brakeTorque * scale;
    end

    % 종방향 하중이동용 ax 추정값 (이전 스텝 기준 — 양의 피드백 동역학 활성화)
    if isfield(plantState, 'ax_prev')
        ax_prev = plantState.ax_prev;
    else
        ax_prev = 0;
    end

    % 노면 입력 z_road (4×1, [m]) — Phase 3 추가. 없으면 0 (평지)
    if isfield(actCmd, 'z_road') && ~isempty(actCmd.z_road)
        z_road = actCmd.z_road(:);
        if numel(z_road) == 1; z_road = z_road * ones(4,1); end
    else
        z_road = zeros(4,1);
    end

    %% 적분기 dispatch
    SIM = getfield_default(params, 'SIM', struct());
    solver  = lower(getfield_default(SIM, 'solver', 'ode45'));
    relTol  = getfield_default(SIM, 'solver_RelTol', 1e-3);
    absTol  = getfield_default(SIM, 'solver_AbsTol', 1e-5);

    odefun = @(tt, xx) calc_derivatives(xx, delta, actCmd, VEH, TIRE, CONST, ax_prev, z_road);

    switch solver
        case 'rk4'
            k1 = calc_derivatives(x,             delta, actCmd, VEH, TIRE, CONST, ax_prev, z_road);
            k2 = calc_derivatives(x + 0.5*dt*k1, delta, actCmd, VEH, TIRE, CONST, ax_prev, z_road);
            k3 = calc_derivatives(x + 0.5*dt*k2, delta, actCmd, VEH, TIRE, CONST, ax_prev, z_road);
            k4 = calc_derivatives(x + dt*k3,     delta, actCmd, VEH, TIRE, CONST, ax_prev, z_road);
            x = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
        case 'euler'
            dxk = calc_derivatives(x, delta, actCmd, VEH, TIRE, CONST, ax_prev, z_road);
            x = x + dt * dxk;
        case {'ode45','ode23','ode15s','ode23t','ode23s','ode113'}
            opts = odeset('RelTol', relTol, 'AbsTol', absTol, 'MaxStep', dt);
            [~, X] = feval(solver, odefun, [0 dt], x, opts);
            x = X(end, :).';
        otherwise
            error('plant_14dof:unknownSolver', 'Unknown SIM.solver: %s', solver);
    end

    % 보호
    x(1) = max(x(1), 0.1);           % vx > 0
    x(16:19) = max(x(16:19), 0);     % omega >= 0

    plantState.x  = x;
    plantState.vx = x(1);

    %% 출력 매핑 (최종 상태에서 한 번 더 힘 계산)
    [~, plantOut] = calc_derivatives(x, delta, actCmd, VEH, TIRE, CONST, ax_prev, z_road);

    % 다음 스텝용 ax_prev 갱신 (관성계 종방향 가속도)
    plantState.ax_prev = plantOut.ax;

end

%% ============================================================
function [dx, out] = calc_derivatives(x, delta, actCmd, VEH, TIRE, CONST, ax_for_loadtransfer, z_road)
%CALC_DERIVATIVES 14DOF 상태 미분 계산
%   z_road (4×1): 각 휠 노면 입력 [m]. 없으면 0 (평지)

    %% 상태 추출
    vx       = max(x(1), 0.1);
    vy       = x(2);
    r        = x(3);   % yawRate
    phi      = x(4);   % roll
    dphi     = x(5);   % rollRate
    theta    = x(6);   % pitch
    dtheta   = x(7);   % pitchRate
    zs       = x(8:11);   % 서스펜션 변위 [FL,FR,RL,RR]
    zsdot    = x(12:15);  % 서스펜션 속도
    omega    = x(16:19);  % 바퀴 회전 속도

    g   = CONST.g;
    ms  = VEH.ms;
    mu_w = VEH.mu_w;
    htf = VEH.track_f / 2;
    htr = VEH.track_r / 2;

    %% 스프링/댐퍼 강성 (코너별)
    ks = [VEH.ks_f; VEH.ks_f; VEH.ks_r; VEH.ks_r];
    cs = actCmd.dampingCoeff;  % CDC 명령값

    %% 서스펜션 힘
    Fspring = zeros(4, 1);
    Fdamper = zeros(4, 1);
    for i = 1:4
        [Fspring(i), Fdamper(i)] = susp_force(zs(i), zsdot(i), ks(i), cs(i));
    end
    Fsusp = Fspring + Fdamper;  % 스프렁 매스에 작용하는 총 서스펜션 힘

    %% 동적 수직 하중
    % 정적 하중 (per wheel) — sprung mass의 lever 분배 + unsprung mass 자체 무게
    Fz_static_f = (ms * VEH.lr / VEH.L / 2 + mu_w) * g;
    Fz_static_r = (ms * VEH.lf / VEH.L / 2 + mu_w) * g;

    % 관성계 가속도 근사 — 이전 스텝의 종/횡 가속도를 하중이동 입력으로 사용
    % (RK4 substep 내에서는 동일값 유지 — 양의 피드백 활성화로 brake decel 정상 동작)
    ay_approx = vx * r;
    ax_approx = ax_for_loadtransfer;

    % 하중 이동
    %   dFz_lat: 횡가속도 양수 → 우측 휠 하중 증가 (좌측 - / 우측 +)
    %   dFz_lon: 종가속도 음수(감속) → 전륜 하중 증가
    %            convention: 본 변수는 전륜으로 이동하는 하중량(양수 = 전륜 증가)
    dFz_lat_f = ms * ay_approx * VEH.h_cog / VEH.track_f;
    dFz_lat_r = ms * ay_approx * VEH.h_cog / VEH.track_r;
    dFz_lon   = -ms * ax_approx * VEH.h_cog / VEH.L;  % 음의 부호: 제동→전륜+

    % 서스펜션 기여 (스프링+댐퍼)
    Fz_tire = zeros(4, 1);
    Fz_tire(1) = Fz_static_f - dFz_lat_f/2 + dFz_lon/2 + VEH.kt * zs(1);  % FL
    Fz_tire(2) = Fz_static_f + dFz_lat_f/2 + dFz_lon/2 + VEH.kt * zs(2);  % FR
    Fz_tire(3) = Fz_static_r - dFz_lat_r/2 - dFz_lon/2 + VEH.kt * zs(3);  % RL
    Fz_tire(4) = Fz_static_r + dFz_lat_r/2 - dFz_lon/2 + VEH.kt * zs(4);  % RR
    Fz_tire = max(Fz_tire, 10);  % 하중 > 0 보호

    %% 바퀴별 속도
    vx_w = [vx - r*htf; vx + r*htf; vx - r*htr; vx + r*htr];
    vy_w = [vy + VEH.lf*r; vy + VEH.lf*r; vy - VEH.lr*r; vy - VEH.lr*r];
    vx_w_safe = max(abs(vx_w), 0.5) .* sign(vx_w + eps);

    %% 타이어 슬립
    kappa = (omega .* VEH.rw - vx_w) ./ abs(vx_w_safe);
    alpha = zeros(4, 1);
    alpha(1) = delta - atan2(vy_w(1), vx_w_safe(1));
    alpha(2) = delta - atan2(vy_w(2), vx_w_safe(2));
    alpha(3) = -atan2(vy_w(3), vx_w_safe(3));
    alpha(4) = -atan2(vy_w(4), vx_w_safe(4));

    %% 타이어 힘 (복합 슬립)
    Fx_tire = zeros(4, 1);
    Fy_tire = zeros(4, 1);
    for i = 1:4
        [Fx_tire(i), Fy_tire(i)] = tire_force(kappa(i), alpha(i), Fz_tire(i), 0, TIRE);
    end

    %% 차체 좌표계 변환 (전륜 조향)
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
    Faero = 0.5 * CONST.rho_air * VEH.Cd * VEH.Af * vx^2;

    %% 차체 병진/요 운동 방정식
    dvx = (sum(Fx_body) - Faero) / VEH.mass + vy * r;
    dvy = sum(Fy_body) / VEH.mass - vx * r;
    dr  = (VEH.lf*(Fy_body(1)+Fy_body(2)) - VEH.lr*(Fy_body(3)+Fy_body(4)) ...
          + htf*(Fx_body(2)-Fx_body(1)) + htr*(Fx_body(4)-Fx_body(3))) / VEH.Iz;

    %% 롤 동역학
    % 유효 롤 강성/감쇠 (좌우 스프링/댐퍼 기여)
    ks_roll = (VEH.ks_f * htf^2 + VEH.ks_r * htr^2) * 2;  % 유효 롤 강성
    cs_roll = (VEH.cs_f * htf^2 + VEH.cs_r * htr^2) * 2;   % 기본 롤 감쇠

    ddphi = (-ks_roll * phi - cs_roll * dphi ...
             + ms * g * VEH.hrc * sin(phi) ...
             + ms * (dvy + vx*r) * VEH.hrc * cos(phi)) / VEH.Ix;

    %% 피치 동역학 (안티다이브/안티스쿼트 보정 포함)
    ks_pitch = (VEH.ks_f * VEH.lf^2 + VEH.ks_r * VEH.lr^2) * 2;
    cs_pitch = (VEH.cs_f * VEH.lf^2 + VEH.cs_r * VEH.lr^2) * 2;

    ax_long = dvx - vy*r;  % 종방향 관성 가속도
    if ax_long < 0
        antiFactor = VEH.antiDive;   % 제동: nose-dive 억제
    else
        antiFactor = VEH.antiSquat;  % 가속: nose-up 억제
    end
    ddtheta = (-ks_pitch * theta - cs_pitch * dtheta ...
               - (1 - antiFactor) * ms * ax_long * VEH.h_cog) / VEH.Iy;

    %% 서스펜션 수직 동역학 (언스프렁 매스, zs=0이 적재 평형인 perturbation 좌표)
    % mu_w * dzsdot = -(ks + kt)*zs - cs*zsdot + kt*z_road
    %   spring   : -ks*zs           (복원)
    %   tire     : -kt*(zs - z_road)  (z_road > 0 ⇒ 노면이 올라옴 ⇒ 휠을 위로 밀어올림)
    %   damper   : -cs*zsdot
    if nargin < 8 || isempty(z_road); z_road = zeros(4,1); end
    dzsdot = (-(ks + VEH.kt) .* zs - cs .* zsdot + VEH.kt .* z_road) / mu_w;

    %% 바퀴 회전 동역학
    brk = actCmd.brakeTorque;
    domega = (-brk - Fx_tire * VEH.rw) / VEH.Iw;

    %% 상태 미분 벡터
    dx = [dvx; dvy; dr; dphi; ddphi; dtheta; ddtheta; ...
          zsdot; dzsdot; domega];

    %% 출력 구조체 (nargout > 1일 때만)
    if nargout > 1
        out.vx        = vx;
        out.vy        = vy;
        out.yawRate   = r;
        out.ax        = dvx - vy*r;
        out.ay        = dvy + vx*r;
        out.slipAngle = atan2(vy, vx);
        out.roll      = phi;
        out.pitch     = theta;

        % 서스펜션 속도 → ctrl_vertical용
        out.suspVel = zsdot;

        % 차체 수직 속도 (각 코너: 롤/피치 기여)
        out.bodyVel = [dphi * htf + dtheta * VEH.lf;    % FL
                      -dphi * htf + dtheta * VEH.lf;    % FR
                       dphi * htr - dtheta * VEH.lr;    % RL
                      -dphi * htr - dtheta * VEH.lr];   % RR

        wheels = {'FL','FR','RL','RR'};
        for w = 1:4
            out.tire.(wheels{w}).Fx        = Fx_tire(w);
            out.tire.(wheels{w}).Fy        = Fy_tire(w);
            out.tire.(wheels{w}).Fz        = Fz_tire(w);
            out.tire.(wheels{w}).slipAngle = alpha(w);
            out.tire.(wheels{w}).slipRatio = kappa(w);
            out.susp.(wheels{w}).springFrc  = Fspring(w);
            out.susp.(wheels{w}).damperFrc  = Fdamper(w);
        end
    end

end
