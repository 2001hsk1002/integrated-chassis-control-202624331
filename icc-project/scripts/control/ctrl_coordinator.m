function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator Allocation — WLS 기반 yaw moment 분배 + 마찰원 제한
%
%   설계 개요:
%     (1) 종방향: lonCmd.Fx_total < 0 (제동 요구) 를 전:후 60:40 으로 4륜 분배.
%         (양수 요구는 본 plant 에 구동 액추에이터가 없어 무시.)
%     (2) ESC   : latCmd.yawMoment (+CCW) 를 같은 쪽(좌/우) 전·후륜 브레이크
%         '증가' 로 구현. 분배는 weighted least squares:
%             min  sum w_i dT_i^2   s.t.  sum a_i dT_i = Mz,
%             a_i = (t_i/2)/rw  (lever arm),  w_i ∝ 1/Fz_i_est
%         -> dT_i = (Mz / sum(a_j^2/w_j)) * a_i / w_i
%         수직하중이 큰 휠에 더 많은 토크 (잠김 방지 + effort 최소화).
%     (3) ABS   : lonCmd.absTrqAdj (per-wheel ±[Nm]) 를 합산. 음수는 시나리오
%         제동 토크에 대한 릴리프 — 최종 [0, MAX] 클립은 main loop 에서 수행
%         되므로 본 함수 출력은 음수 허용 (±MAX 로 클립).
%     (4) 마찰원: 정적 하중 + 종방향 하중이동 추정으로 Fz_i 를 계산하고,
%         '추가' 토크 합이 mu*Fz_i*rw 를 넘지 않게 per-wheel cap.
%     (5) 수직  : verCmd 그대로 통과 + [cMin, cMax] saturation.
%
%   Inputs : latCmd(.steerAngle,.yawMoment), lonCmd(.Fx_total,.brakeRatio,
%            .absTrqAdj 옵션), verCmd(4x1), vx, VEH, CTRL, LIM
%   Output : actuatorCmd.{steerAngle, brakeTorque(4x1), dampingCoeff(4x1)}

    g  = 9.81;
    rw = VEH.rw;
    mu = 1.0;                               % 공칭 노면 마찰
    frontShare = 0.60;                      % 종제동 전:후 분배

    %% ---- 1. AFS steer pass-through + saturation -------------------------
    sa = latCmd.steerAngle;
    actuatorCmd.steerAngle = min(max(sa, -LIM.MAX_STEER_ANGLE), ...
                                       LIM.MAX_STEER_ANGLE);

    %% ---- 2. 종방향 제동 분배 (60:40) ------------------------------------
    T = zeros(4,1);
    Fx = lonCmd.Fx_total;
    if Fx < 0
        Ttot = -Fx * rw;                    % 총 제동 토크 [Nm]
        T = T + Ttot * [frontShare/2; frontShare/2; ...
                        (1-frontShare)/2; (1-frontShare)/2];
    end

    %% ---- 2b. ESC 대칭 감속 요청 (ctrl_lateral.symBrakeFx) ---------------
    FxSym = 0;
    if isstruct(latCmd) && isfield(latCmd, 'symBrakeFx')
        FxSym = min(max(latCmd.symBrakeFx, -8000), 0);
    end
    if FxSym < 0
        % 예측 감속은 일반 제동과 동일한 60:40 분배 (시험 결과 전륜 전용
        % 분배보다 슬립-LTR 트레이드오프가 우수했음 — report §5 참조)
        Ts = -FxSym * rw;
        T = T + Ts * [frontShare/2; frontShare/2; ...
                      (1-frontShare)/2; (1-frontShare)/2];
    end

    %% ---- 3. per-wheel 수직하중 추정 (마찰원/WLS 가중치용) ----------------
    %  정적 하중 + 종방향 하중이동 (ax_est 는 제동 요구 기반 근사)
    m  = VEH.mass; L = VEH.lf + VEH.lr; h = VEH.h_cog;
    axEst = min((Fx + FxSym) / m, 0);       % 제동 요구 기반 감속 추정 (<=0)
    dFz   = -m * axEst * h / L;             % 전륜으로의 이동량 (axle 합)
    FzF = (m * g * VEH.lr / L + dFz) / 2;   % per-wheel front
    FzR = (m * g * VEH.lf / L - dFz) / 2;   % per-wheel rear
    Fz  = [FzF; FzF; max(FzR, 500); max(FzR, 500)];

    %% ---- 4. ESC yaw moment — WLS 분배 -----------------------------------
    Mz = latCmd.yawMoment;
    MzMax = 6000;
    Mz = min(max(Mz, -MzMax), MzMax);
    if abs(Mz) > 1e-3
        if Mz > 0
            idx = [1; 3];                   % +CCW -> 좌측(FL, RL) 제동
        else
            idx = [2; 4];                   % -CW  -> 우측(FR, RR) 제동
        end
        tHalf = [VEH.track_f/2; VEH.track_r/2];
        aLev  = tHalf / rw;                 % [m/m] dMz = aLev .* dT
        wInv  = Fz(idx);                    % w_i = 1/Fz -> 1/w = Fz
        denom = sum(aLev.^2 .* wInv);
        dT    = abs(Mz) * (aLev .* wInv) / max(denom, 1e-6);
        T(idx) = T(idx) + dT;
    end

    %% ---- 5. ABS per-wheel 보정 (음수 = 시나리오 제동 릴리프) -------------
    if isstruct(lonCmd) && isfield(lonCmd, 'absTrqAdj') ...
            && numel(lonCmd.absTrqAdj) == 4
        T = T + lonCmd.absTrqAdj(:);
    end

    %% ---- 6. 마찰원 제한: '추가' 토크를 mu*Fz*rw 이내로 cap ----------------
    Tcap = mu * Fz * rw;                    % per-wheel 잠김 한계 토크 근사
    pos  = T > 0;
    T(pos) = min(T(pos), Tcap(pos));

    %% ---- 7. 최종 saturation ----------------------------------------------
    %  음수(릴리프) 허용: 최종 brake_total = brk_scenario + T 가 main loop
    %  에서 [0, MAX_BRAKE_TRQ] 로 클립됨.
    actuatorCmd.brakeTorque = min(max(T, -LIM.MAX_BRAKE_TRQ), ...
                                        LIM.MAX_BRAKE_TRQ);

    %% ---- 8. 수직 감쇠 pass-through + saturation --------------------------
    v = verCmd(:);
    if numel(v) ~= 4; v = 1500 * ones(4,1); end
    actuatorCmd.dampingCoeff = min(max(v, CTRL.VER.cMin), CTRL.VER.cMax);

end
