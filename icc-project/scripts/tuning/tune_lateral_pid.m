function [bestParams, tuneLog] = tune_lateral_pid(VEH, CTRL, LIM, opts)
%TUNE_LATERAL_PID 횡방향 PID 게인 자동 튜닝 (그리드 서치)
%
%   [bestParams, tuneLog] = TUNE_LATERAL_PID(VEH, CTRL, LIM)
%   [bestParams, tuneLog] = TUNE_LATERAL_PID(VEH, CTRL, LIM, opts)
%
%   Bicycle Model + Step Steer 시나리오에서 요 레이트 추종 성능을
%   최적화하는 PID 게인을 그리드 서치로 탐색한다.
%
%   Inputs:
%       VEH  - (struct) 차량 파라미터
%       CTRL - (struct) 기본 제어 파라미터
%       LIM  - (struct) 제한값
%       opts - (struct, optional)
%           .KpRange - Kp 탐색 범위 (default: [0.5 1.0 1.5 2.0])
%           .KiRange - Ki 탐색 범위 (default: [0.05 0.1 0.2])
%           .KdRange - Kd 탐색 범위 (default: [0.01 0.05 0.1])
%           .vx      - 시뮬레이션 속도 [m/s] (default: 20)
%           .tEnd    - 시뮬레이션 시간 [s] (default: 5)
%
%   Outputs:
%       bestParams - (struct) 최적 PID 게인 (.Kp, .Ki, .Kd)
%       tuneLog    - (table) 전체 탐색 로그

    arguments
        VEH struct
        CTRL struct
        LIM struct
        opts.KpRange (1,:) double = [0.5, 1.0, 1.5, 2.0, 3.0]
        opts.KiRange (1,:) double = [0.01, 0.05, 0.1, 0.2]
        opts.KdRange (1,:) double = [0.01, 0.03, 0.05, 0.1]
        opts.vx (1,1) double = 20
        opts.tEnd (1,1) double = 5
    end

    dt = 0.001;
    t  = (0:dt:opts.tEnd)';
    n  = numel(t);

    % Step Steer 입력
    steerInput = zeros(n, 1);
    steerInput(t >= 0.5) = deg2rad(2);  % 0.5초에 2도 스텝

    % 목표 요 레이트
    yawRateRef = calc_ref_yaw_rate(opts.vx, steerInput, VEH);

    % Bicycle Model 행렬
    [A, B, ~, ~] = calc_bicycle_model(opts.vx, VEH);

    fprintf('[tune_lateral_pid] Grid search: %d x %d x %d = %d combinations\n', ...
        numel(opts.KpRange), numel(opts.KiRange), numel(opts.KdRange), ...
        numel(opts.KpRange) * numel(opts.KiRange) * numel(opts.KdRange));

    %% 그리드 서치
    results = [];
    bestCost = Inf;

    for Kp = opts.KpRange
        for Ki = opts.KiRange
            for Kd = opts.KdRange
                % 시뮬레이션
                x = [0; 0];  % [vy; yawRate]
                ctrlState.intError  = 0;
                ctrlState.prevError = 0;
                cost = 0;
                stable = true;

                for k = 1:n
                    yrRef = yawRateRef(k);
                    yr    = x(2);
                    beta  = atan2(x(1), opts.vx);

                    % PID 제어
                    yrErr = yrRef - yr;
                    ctrlState.intError = ctrlState.intError + yrErr * dt;
                    ctrlState.intError = max(-5, min(5, ctrlState.intError));
                    derErr = (yrErr - ctrlState.prevError) / dt;
                    ctrlState.prevError = yrErr;

                    deltaAdd = Kp * yrErr + Ki * ctrlState.intError + Kd * derErr;
                    deltaAdd = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, deltaAdd));

                    % 상태 적분 (Euler)
                    u_total = steerInput(k) + deltaAdd;
                    dx = A * x + B * u_total;
                    x = x + dx * dt;

                    % 안정성 확인
                    if any(isnan(x)) || any(isinf(x)) || abs(rad2deg(beta)) > 15
                        stable = false;
                        break;
                    end

                    % 비용 누적 (요 레이트 오차 + 제어 노력)
                    cost = cost + (yrErr^2 + 0.01 * deltaAdd^2) * dt;
                end

                if ~stable
                    cost = Inf;
                end

                results = [results; Kp, Ki, Kd, cost, stable]; %#ok<AGROW>

                if cost < bestCost
                    bestCost = cost;
                    bestParams.Kp = Kp;
                    bestParams.Ki = Ki;
                    bestParams.Kd = Kd;
                end
            end
        end
    end

    %% 로그 테이블 생성
    tuneLog = array2table(results, ...
        'VariableNames', {'Kp', 'Ki', 'Kd', 'Cost', 'Stable'});
    tuneLog = sortrows(tuneLog, 'Cost');

    fprintf('[tune_lateral_pid] Best: Kp=%.2f, Ki=%.3f, Kd=%.3f (cost=%.4f)\n', ...
        bestParams.Kp, bestParams.Ki, bestParams.Kd, bestCost);

end
