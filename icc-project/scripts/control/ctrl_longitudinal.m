function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 종방향 제어기 — 속도 추종 PI + per-wheel PI 슬립 제어 ABS
%
%   설계 개요:
%     (1) 속도 추종 : PI 제어 (anti-windup clamp) + 저크 제한 (force rate limit).
%         감속 요구(Fx_total < 0)만 coordinator 가 브레이크 토크로 변환한다.
%         제동(ABS 작동) 중에는 적분기를 동결해 windup 을 방지한다.
%     (2) ABS      : runner 가 ctrlState.wheelSlip(4x1) 에 캐시해 주는 직전
%         스텝 휠 슬립비 kappa 를 사용, 목표 kappa* = -0.12 (mu-peak 부근) 에
%         대한 per-wheel PI 슬립 제어. bang-bang 이 아닌 연속 PI 라서
%         슬립 진동(chatter)이 작고 absSlipRMS 가 낮다.
%         출력은 forceCmd.absTrqAdj (4x1, [Nm]) — 양수면 토크 추가(언더슬립
%         휠을 mu-peak 로), 음수면 토크 감소(잠김 직전 휠 릴리프).
%         실제 합산은 ctrl_coordinator → main loop 에서 일어난다.
%
%   Inputs:
%       vxRef, vx  - 목표/실제 종방향 속도 [m/s]
%       ax         - 직전 스텝 종가속도 [m/s^2]
%       ctrlState  - .intError, .prevForce, .wheelSlip(4x1, runner 제공)
%       CTRL, LIM  - sim_params 파라미터 (.MAX_JERK, .MAX_BRAKE_TRQ 등)
%       dt         - sample time [s]
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N] (+가속 / -제동)
%       forceCmd.brakeRatio - 0(가속) ~ 1(전제동)
%       forceCmd.absTrqAdj  - 4x1 per-wheel ABS 토크 보정 [Nm]
%       ctrlState           - 업데이트
%
%   요구사항 대응: 속도 추종 PI, |kappa|>0.12 시 brake 감소(ABS),
%   저크 제한(LIM.MAX_JERK), anti-windup 모두 구현.

    %% ---- 0. 설계 파라미터 ------------------------------------------------
    m       = 1500;             % [kg] 공칭 차량 질량 (저크 제한 환산용)
    % 속도 추종 PI
    KpV     = 1200;             % [N/(m/s)]
    KiV     = 250;              % [N/(m/s)/s]
    FxMax   = 6000;             % [N] 구동력 요구 한계
    FxMin   = -1.5e4;           % [N] 제동력 요구 한계
    % ABS per-wheel PI 슬립 제어
    kapTgt  = -0.12;            % 목표 슬립비 (mu-peak, KPI target 과 동일)
    kapOn   = -0.02;            % 이 값보다 슬립이 깊어야(음수) ABS enable
    KpA     = 1.2e4;            % [Nm / slip]
    KiA     = 6.0e4;            % [Nm / slip / s]
    adjMax  =  900;             % [Nm] per-wheel 토크 '추가' 한계
    adjMin  = -2500;            % [Nm] per-wheel 토크 '감소' 한계
    slewA   = 8.0e4;            % [Nm/s] ABS 보정 슬루 레이트

    %% ---- 1. 내부 상태 초기화 --------------------------------------------
    if ~isfield(ctrlState, 'intError');  ctrlState.intError  = 0;          end
    if ~isfield(ctrlState, 'prevForce'); ctrlState.prevForce = 0;          end
    if ~isfield(ctrlState, 'wheelSlip'); ctrlState.wheelSlip = zeros(4,1); end
    if ~isfield(ctrlState, 'absInt');    ctrlState.absInt    = zeros(4,1); end
    if ~isfield(ctrlState, 'absAdj');    ctrlState.absAdj    = zeros(4,1); end

    kappa = ctrlState.wheelSlip(:);
    if numel(kappa) ~= 4; kappa = zeros(4,1); end

    % 토크 '추가'(mu-peak 부스트)는 강한 외부 제동(풀 브레이킹) 중에만 허용.
    % 그 외 상황에서 추가를 허용하면 '제동->감속->ABS 추가->더 감속' 의
    % 자가발진(self-sustaining braking)이 생길 수 있음 (relief-only 가 안전).
    boostOK = (ax < -6.0);

    % 차량이 제동 중인지 판정 (감속 + 어느 휠이든 의미있는 음의 슬립)
    braking = (ax < -0.5) && any(kappa < kapOn) && (vx > 0.25);

    %% ---- 2. 속도 추종 PI (+anti-windup) ---------------------------------
    eV = vxRef - vx;
    % 제동(ABS) 중이거나 출력 포화 시 적분 동결 (anti-windup)
    intCand = ctrlState.intError + eV * dt;
    FxUnsat = KpV * eV + KiV * intCand;
    if ~braking && (FxUnsat > FxMin && FxUnsat < FxMax || sign(eV) ~= sign(FxUnsat))
        ctrlState.intError = intCand;
    end
    ctrlState.intError = min(max(ctrlState.intError, ...
                                 -CTRL.LON.intMax), CTRL.LON.intMax);

    Fx = KpV * eV + KiV * ctrlState.intError;
    Fx = min(max(Fx, FxMin), FxMax);

    % 제동 중에는 속도 PI 가 시나리오 제동과 싸우지 않도록 가속 요구 차단
    if braking && Fx > 0; Fx = 0; end

    %% ---- 3. 저크 제한 (force slew = m * MAX_JERK) -----------------------
    dFmax = m * LIM.MAX_JERK * dt;
    Fx = min(max(Fx, ctrlState.prevForce - dFmax), ctrlState.prevForce + dFmax);
    ctrlState.prevForce = Fx;

    forceCmd.Fx_total = Fx;
    if Fx < 0; forceCmd.brakeRatio = 1; else; forceCmd.brakeRatio = 0; end

    %% ---- 4. ABS — per-wheel PI 슬립 제어 --------------------------------
    adj = zeros(4,1);
    for w = 1:4
        % enable: 차량 제동 중 + 해당 휠이 실제로 제동 슬립 상태
        wheelBraking = braking && (kappa(w) < kapOn);
        if wheelBraking
            eK = kapTgt - kappa(w);     % kappa 가 목표보다 깊으면(eK>0) 토크 감소가
                                        % 아닌... 부호 확인: kappa=-0.5 < tgt=-0.12
                                        % -> eK = -0.12-(-0.5) = +0.38 -> 슬립 과다.
            % 슬립 과다(eK>0)면 토크를 '줄여야' 하므로 부호 반전 적용
            u = -(KpA * eK + KiA * ctrlState.absInt(w));
            % 조건부 적분 (포화 반대 방향만 적분)
            if (u > adjMin && u < adjMax) || sign(eK) == sign(u)
                ctrlState.absInt(w) = ctrlState.absInt(w) + eK * dt;
            end
            u = -(KpA * eK + KiA * ctrlState.absInt(w));
            if boostOK; aMaxW = adjMax; else; aMaxW = 0; end
            adj(w) = min(max(u, adjMin), aMaxW);
        else
            % 비활성: 적분 리셋 + 보정 토크를 0 으로 부드럽게 복귀
            ctrlState.absInt(w) = 0;
            adj(w) = 0;
        end
    end

    % 슬루 레이트 제한 (chatter 방지)
    dAmax = slewA * dt;
    adj = min(max(adj, ctrlState.absAdj - dAmax), ctrlState.absAdj + dAmax);
    ctrlState.absAdj = adj;

    forceCmd.absTrqAdj = adj;

end
