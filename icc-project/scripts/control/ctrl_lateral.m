function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL 횡방향 통합 제어기 (AFS + ESC)
%
%   설계 개요 (5개 기능 블록):
%     (1) AFS 추종 : 성형(prefilter)된 r_ref 에 대한 gain-scheduled PID.
%         속도에 따라 정상상태 yaw gain G(v) = v/(L + Kus v^2) 이 변하므로
%         K(v) = K0 * G(v0)/G(v) 로 루프이득을 일정화 (1-D LPV).
%     (2) ref 성형/제한 : 1차 LPF(tauR) + 마찰 한계 |r_ref| <= 0.85 mu g / vx.
%     (3) AFS arbitration : 횡가속 이용률 ay_est = |r| vx 가 한계로 가면
%         yaw 추종 대신 beta 카운터스티어(uStab = Kb*beta + Kbd*beta_dot)로
%         연속 전환 — 마찰원을 소모하지 않고 슬립을 직접 억제.
%     (4) ESC : (a) phase-conditioned beta-limiter — |beta|>beta_th 이고
%         '슬립이 커지는 중'(beta*beta_dot>0)일 때만 복원 yaw moment,
%         (b) yaw 오차 deadband 초과분 비례 moment (정상주행 개입 없음).
%     (5) 예측 대칭 감속 : 지속 조향활동 감지 시 전륜 대칭 제동으로 선제
%         감속 (실차 ESC 의 speed-management 기능, ay ~ v^2/R).
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (bicycle model 기반)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 beta [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, .dFilt ...)
%       CTRL, LIM  - sim_params.m 파라미터 (기본 게인/한계)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad]
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (+ = CCW)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항 대응: yaw rate 추종(PID), beta-limiter(ESC), 속도 적응
%   (gain scheduling), anti-windup(조건부 적분) + saturation 모두 구현.
%   시나리오 분기 없음 / global 변수 없음.

    %% ---- 0. 설계 파라미터 (튜닝 상수) ----------------------------------
    % AFS PID (yaw rate 오차 [rad/s] -> 보조 조향 [rad])
    Kp0     = 0.35;             % 비례 게인 @ 기준속도 v0
    Ki0     = 2.00;             % 적분 게인
    Kd0     = 0.070;            % 미분 게인
    tauD    = 0.05;             % [s] 미분 1차 LPF 시정수
    afsMax  = deg2rad(2.0);     % [rad] AFS 보조 조향 한계 (driver 의도 보호)
    intLim  = 0.05;             % [rad] 적분 기여 한계 (anti-windup clamp)

    % ESC (yaw moment)
    betaTh  = deg2rad(4.0);     % [rad] beta-limiter 임계
    Kbeta   = 8.0e4;            % [Nm/rad] beta 초과분 비례 게인
    KbetaD  = 2.5e3;            % [Nm/(rad/s)] beta 미분(댐핑) 게인
    rDead   = deg2rad(15.0);     % [rad/s] yaw 오차 deadband (정상주행 개입 방지)
    Kr      = 6.0e3;            % [Nm/(rad/s)] yaw 오차 ESC 게인
    MzMax   = 6.0e3;            % [Nm] yaw moment 한계

    % gain scheduling 용 공칭 차량 (C-segment, sim_params 와 동일)
    L0   = 2.6;                 % [m] wheelbase
    Kus0 = 9.8e-4;              % [rad s^2/m] understeer gradient (bicycle)
    v0   = 20.0;                % [m/s] 기준 속도

    %% ---- 1. 내부 상태 초기화 (최초 호출 보호) ---------------------------
    if ~isfield(ctrlState, 'intError');  ctrlState.intError  = 0; end
    if ~isfield(ctrlState, 'prevError'); ctrlState.prevError = 0; end
    if ~isfield(ctrlState, 'dFilt');     ctrlState.dFilt     = 0; end
    if ~isfield(ctrlState, 'prevBeta');  ctrlState.prevBeta  = 0; end
    if ~isfield(ctrlState, 'refF');      ctrlState.refF      = yawRateRef; end

    %% ---- 1a. 마찰 한계 ref 제한 ------------------------------------------
    %  |r_ref| <= 0.85 * mu*g / vx  (Rajamani, Vehicle Dynamics and Control)
    %  driver(특히 Stanley)가 물리적으로 불가능한 yaw rate 를 요구해도
    %  추종 목표를 달성 가능한 영역으로 제한 -> 한계영역 제어 폭주 방지
    muG   = 0.85 * 9.81;
    rCap  = muG / max(vx, 3.0);
    rRefC = sign(yawRateRef) * min(abs(yawRateRef), rCap);

    %% ---- 1b. Reference prefilter (1차 LPF, tauR) ------------------------
    %  step 형태의 r_ref 를 부드럽게 성형 -> D-kick / 적분 과대로 인한
    %  오버슈트 억제 (classical reference shaping)
    tauR  = 0.12;                           % [s]
    aR    = dt / (tauR + dt);
    ctrlState.refF = (1 - aR) * ctrlState.refF + aR * rRefC;
    rRefF = ctrlState.refF;

    %% ---- 2. Gain scheduling (속도 적응) --------------------------------
    %  G(v) = v/(L + Kus v^2) : 정상상태 yaw gain. 루프이득 일정화:
    %  ksch = G(v0)/G(v),  저속(<3 m/s) 에서는 제어 개입 자체를 끔.
    vEff = max(vx, 1.0);
    Gv   = vEff / (L0 + Kus0 * vEff^2);
    Gv0  = v0   / (L0 + Kus0 * v0^2);
    ksch = Gv0 / max(Gv, 1e-3);
    ksch = min(max(ksch, 0.3), 3.0);        % 과도한 스케일 방지
    if vx < 3.0; ksch = 0; end              % 극저속: AFS/ESC off

    %% ---- 3. AFS — gain-scheduled PID (성형 ref 추종) ---------------------
    e = rRefF - yawRate;                    % [rad/s]

    dRaw  = (e - ctrlState.prevError) / dt;
    alpha = dt / (tauD + dt);
    ctrlState.dFilt = (1 - alpha) * ctrlState.dFilt + alpha * dRaw;

    leak    = 1 - dt / 0.7;                 % 적분 leak (~0.7 s 망각)
    intCand = ctrlState.intError * leak + e * dt;
    uUnsat  = ksch * (Kp0 * e + Ki0 * intCand + Kd0 * ctrlState.dFilt);
    if abs(uUnsat) < afsMax || sign(e) ~= sign(uUnsat)
        ctrlState.intError = intCand;       % 조건부 적분 (anti-windup)
    else
        ctrlState.intError = ctrlState.intError * leak;
    end
    iMaxState = intLim / max(Ki0 * ksch, 1e-6);
    ctrlState.intError = min(max(ctrlState.intError, -iMaxState), iMaxState);

    uTrack = ksch * (Kp0 * e + Ki0 * ctrlState.intError + Kd0 * ctrlState.dFilt);

    %% ---- 3b. AFS arbitration: tracking <-> beta counter-steer ------------
    %  횡가속 이용률 ay_est = |r|*vx 가 한계(>ayLo)로 가면 yaw 추종 대신
    %  beta 안정화 카운터스티어로 전환 — 브레이크 없이 슬립을 직접 억제.
    betaDot = (slipAngle - ctrlState.prevBeta) / dt;
    if ~isfield(ctrlState, 'bdF'); ctrlState.bdF = 0; end
    aB = dt / (0.03 + dt);
    ctrlState.bdF = (1 - aB) * ctrlState.bdF + aB * betaDot;   % beta-rate LPF

    ayLo = 6.0;  ayHi = 8.0;                % [m/s^2] 전환 구간
    ayU  = abs(yawRate) * vEff;
    w    = min(max(1 - (ayU - ayLo) / (ayHi - ayLo), 0), 1);

    KbS  = 0.85;                            % [rad/rad] beta 카운터스티어
    KbdS = 0.05;                            % [rad/(rad/s)] beta-rate 댐핑
    uStab = KbS * slipAngle + KbdS * ctrlState.bdF;   % 카운터스티어: 슬라이드 방향

    u = w * uTrack + (1 - w) * uStab;
    deltaAdd.steerAngle = min(max(u, -afsMax), afsMax);

    %% ---- 4. ESC — phase-conditioned beta-limiter + yaw-error moment -----
    Mz  = 0;
    spd = min(vEff / v0, 2.0);

    diverging = (slipAngle * ctrlState.bdF) > 0;
    if abs(slipAngle) > betaTh && diverging
        over = abs(slipAngle) - betaTh;
        Mz = Mz - sign(slipAngle) * Kbeta * over * spd ...
                - KbetaD * ctrlState.bdF * spd;
    end
    ctrlState.prevBeta = slipAngle;

    if abs(e) > rDead
        Mz = Mz + Kr * (e - sign(e) * rDead) * spd;
    end
    deltaAdd.yawMoment = min(max(Mz, -MzMax), MzMax);

    %% ---- 4b. ESC 대칭 감속 (predictive speed management) ----------------
    %  지속적인 고속 조향 활동(= 한계 회피 기동)을 감지하면 대칭 제동으로
    %  속도를 선제 감속 -> ay ~ v^2 스케일로 LTR/슬립을 근본 저감.
    %  활동량 = LPF(|d r_ref/dt|, tau_a). 단발 step(스파이크)은 cap 으로
    %  걸러져 거의 기여하지 않음 (지속 sine 조향만 감지) — 시나리오 분기가
    %  아닌, 입력 신호의 일반적 특성(지속 조향률)에 따른 반응.
    if ~isfield(ctrlState, 'prevRefRaw'); ctrlState.prevRefRaw = yawRateRef; end
    if ~isfield(ctrlState, 'actF');       ctrlState.actF = 0;               end
    refDot = abs(yawRateRef - ctrlState.prevRefRaw) / dt;
    refDot = min(refDot, 3.0);              % [rad/s^2] 스파이크 cap
    tauA   = 0.8;
    aA     = dt / (tauA + dt);
    ctrlState.actF = (1 - aA) * ctrlState.actF + aA * refDot;
    ctrlState.prevRefRaw = yawRateRef;

    actTh = 0.30;  kAct = 5000;             % [N/(rad/s^2)]
    fadeBeta = min(max(1 - abs(slipAngle) / deg2rad(2.5), 0), 1);
    if ctrlState.actF > actTh && vx > 12 && ayU > 3.0
        deltaAdd.symBrakeFx = max(-kAct * (ctrlState.actF - actTh) * fadeBeta, -6000);
    else
        deltaAdd.symBrakeFx = 0;
    end

    %% ---- 5. 상태 업데이트 ------------------------------------------------
    ctrlState.prevError = e;

end
