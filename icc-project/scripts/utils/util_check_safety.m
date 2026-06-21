function [safe, alerts, details] = util_check_safety(result)
%UTIL_CHECK_SAFETY 시뮬레이션 결과 안전성 검증
%
%   [safe, alerts, details] = UTIL_CHECK_SAFETY(result)
%
%   safety_check_agent.md에 정의된 검증 파이프라인을 실행한다:
%   Level 1: Signal Integrity, Level 2: Safety Envelope, Level 3: Stability
%
%   Inputs:
%       result - (struct) 시뮬레이션 결과 (cm_read_results 출력)
%
%   Outputs:
%       safe    - (logical) 전체 안전 판정 (true = safe)
%       alerts  - (cell) 경고/위험 메시지 목록
%       details - (struct) 상세 검증 결과

    alerts  = {};
    safe    = true;
    details = struct();

    %% Level 1: Signal Integrity
    details.signalIntegrity = true;
    fieldsToCheck = {'vx','vy','ax','ay','yawRate','slipAngle','roll'};
    for i = 1:numel(fieldsToCheck)
        fn = fieldsToCheck{i};
        if ~isfield(result, fn), continue; end
        sig = result.(fn);
        if any(isnan(sig))
            alerts{end+1} = sprintf('CRITICAL: %s contains NaN', fn);
            details.signalIntegrity = false;
            safe = false;
        end
        if any(isinf(sig))
            alerts{end+1} = sprintf('CRITICAL: %s contains Inf', fn);
            details.signalIntegrity = false;
            safe = false;
        end
    end

    %% Level 2: Safety Envelope

    % Rollover: LTR
    ltrMax = max(abs(result.LTR));
    details.LTR_max = ltrMax;
    if ltrMax > 0.9
        alerts{end+1} = sprintf('CRITICAL: Rollover risk — LTR = %.3f (> 0.9)', ltrMax);
        safe = false;
    elseif ltrMax > 0.7
        alerts{end+1} = sprintf('WARNING: High rollover risk — LTR = %.3f (> 0.7)', ltrMax);
    end

    % Spin-out: Slip Angle
    betaMaxDeg = rad2deg(max(abs(result.slipAngle)));
    details.slipAngle_max_deg = betaMaxDeg;
    if betaMaxDeg > 12
        alerts{end+1} = sprintf('CRITICAL: Spin-out risk — beta = %.1f deg (> 12)', betaMaxDeg);
        safe = false;
    elseif betaMaxDeg > 5
        alerts{end+1} = sprintf('WARNING: High slip angle — beta = %.1f deg (> 5)', betaMaxDeg);
    end

    % Excessive Yaw Rate
    yrMaxDeg = rad2deg(max(abs(result.yawRate)));
    details.yawRate_max_deg = yrMaxDeg;
    if yrMaxDeg > 60
        alerts{end+1} = sprintf('WARNING: High yaw rate — %.1f deg/s (> 60)', yrMaxDeg);
    end

    %% Level 3: Oscillation Detection
    % 요 레이트 영교차 횟수로 진동 감지
    yr = result.yawRate;
    zeroCrossings = sum(abs(diff(sign(yr))) > 0);
    duration = result.time(end) - result.time(1);
    oscFreq = zeroCrossings / (2 * duration);  % 대략적 진동 주파수
    details.yawRate_oscFreq = oscFreq;

    if zeroCrossings > 6 && oscFreq > 0.5  % 0.5 Hz 이상 진동, 3 cycle 이상
        alerts{end+1} = sprintf('WARNING: Yaw rate oscillation detected — %.1f Hz, %d crossings', ...
                                oscFreq, zeroCrossings);
    end

    %% Level 4: Tire Saturation
    if isfield(result, 'tire')
        wheels = {'FL','FR','RL','RR'};
        for w = 1:4
            wn = wheels{w};
            if ~isfield(result.tire, wn), continue; end
            Fy = result.tire.(wn).Fy;
            Fz = result.tire.(wn).Fz;
            Fz(Fz < 100) = 100;  % 최소 하중 보호
            muUsed = abs(Fy) ./ Fz;
            muMax = max(muUsed);
            details.(['tireSat_' wn]) = muMax;

            if muMax > 0.95
                alerts{end+1} = sprintf('WARNING: Tire %s saturated — mu = %.2f', wn, muMax);
            end
        end
    end

    %% 결과 출력
    nCritical = sum(contains(alerts, 'CRITICAL'));
    nWarning  = sum(contains(alerts, 'WARNING'));
    details.nCritical = nCritical;
    details.nWarning  = nWarning;

    fprintf('\n--- Safety Check ---\n');
    if isempty(alerts)
        fprintf('  All checks PASSED.\n');
    else
        for i = 1:numel(alerts)
            fprintf('  [%d] %s\n', i, alerts{i});
        end
    end
    fprintf('  Verdict: %s (%d critical, %d warnings)\n', ...
        iif(safe, 'SAFE', 'UNSAFE'), nCritical, nWarning);
    fprintf('---\n\n');

end

function out = iif(cond, trueVal, falseVal)
    if cond, out = trueVal; else, out = falseVal; end
end
