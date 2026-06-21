function kpi = util_calc_kpi(result, ref, thresholds)
%UTIL_CALC_KPI 시뮬레이션 결과로부터 전체 KPI 계산 및 판정
%
%   kpi = UTIL_CALC_KPI(result)
%   kpi = UTIL_CALC_KPI(result, ref)
%   kpi = UTIL_CALC_KPI(result, ref, thresholds)
%
%   Inputs:
%       result     - (struct) 시뮬레이션 결과 (cm_read_results 출력)
%       ref        - (struct, optional) 레퍼런스 신호 (.yawRate, .vx)
%       thresholds - (struct, optional) KPI 기준값 (kpi_thresholds.m)
%
%   Outputs:
%       kpi - (struct) KPI 결과 및 판정

    %% 기본값 처리
    if nargin < 2 || isempty(ref)
        ref = [];
    end
    if nargin < 3 || isempty(thresholds)
        run('kpi_thresholds.m');
        thresholds = KPI_TH;
    end

    %% 횡방향 KPI
    if ~isempty(ref) && isfield(ref, 'yawRate')
        yrErr = rad2deg(ref.yawRate - result.yawRate);
        kpi.yawRateError_rms = rms(yrErr);
        kpi.yawRateError_max = max(abs(yrErr));
    else
        kpi.yawRateError_rms = NaN;
        kpi.yawRateError_max = NaN;
    end

    if isfield(result, 'lateralDeviation')
        kpi.lateralDev_max = max(abs(result.lateralDeviation));
    else
        kpi.lateralDev_max = NaN;
    end

    %% 종방향 KPI
    if ~isempty(ref) && isfield(ref, 'vx')
        spdErr = (ref.vx - result.vx) * 3.6;  % m/s → km/h
        kpi.speedError_rms = rms(spdErr);
    else
        kpi.speedError_rms = NaN;
    end

    kpi.jerk_max = max(abs(result.jerk));

    %% 수직 KPI
    if isfield(result, 'az_body')
        kpi.bodyAccel_rms = rms(result.az_body);
    else
        kpi.bodyAccel_rms = NaN;
    end

    kpi.rollAngle_max = rad2deg(max(abs(result.roll)));

    %% 안정성 KPI
    kpi.slipAngle_max = rad2deg(max(abs(result.slipAngle)));
    kpi.LTR_max       = max(abs(result.LTR));
    kpi.yawRate_max   = rad2deg(max(abs(result.yawRate)));

    %% 오버슈트 / 정정시간 (스텝 응답 시)
    kpi.overshoot    = calc_overshoot_internal(result.yawRate);
    kpi.settlingTime = calc_settling_time_internal(result.time, result.yawRate);

    %% 판정
    kpi.verdicts = judge_kpi(kpi, thresholds);
    kpi.overall  = judge_overall(kpi.verdicts);

end

%% ---- 내부 함수 ----

function os = calc_overshoot_internal(signal)
    % 단순 오버슈트 계산 (마지막 10% 구간 평균 대비)
    n = numel(signal);
    steadyVal = mean(signal(round(0.9*n):n));
    if abs(steadyVal) < 1e-6
        os = 0;
        return;
    end
    peakVal = max(abs(signal));
    os = (peakVal - abs(steadyVal)) / abs(steadyVal) * 100;
    os = max(os, 0);
end

function ts = calc_settling_time_internal(time, signal)
    % 정정시간: 최종값 ±5% 이내 도달 시간
    n = numel(signal);
    steadyVal = mean(signal(round(0.9*n):n));
    if abs(steadyVal) < 1e-6
        ts = NaN;
        return;
    end
    band = 0.05 * abs(steadyVal);
    settled = abs(signal - steadyVal) < band;

    % 마지막으로 벗어난 인덱스 찾기
    lastOut = find(~settled, 1, 'last');
    if isempty(lastOut)
        ts = 0;
    elseif lastOut >= n
        ts = NaN;
    else
        ts = time(lastOut + 1) - time(1);
    end
end

function verdicts = judge_kpi(kpi, th)
    verdicts = struct();
    kpiNames = fieldnames(th);
    for i = 1:numel(kpiNames)
        name = kpiNames{i};
        if ~isfield(kpi, name) || isnan(kpi.(name))
            verdicts.(name) = 'N/A';
            continue;
        end
        val = kpi.(name);
        if val <= th.(name).pass
            verdicts.(name) = 'PASS';
        elseif val <= th.(name).marginal
            verdicts.(name) = 'MARGINAL';
        else
            verdicts.(name) = 'FAIL';
        end
    end
end

function overall = judge_overall(verdicts)
    v = struct2cell(verdicts);
    nFail     = sum(strcmp(v, 'FAIL'));
    nMarginal = sum(strcmp(v, 'MARGINAL'));

    if nFail > 0
        overall = 'FAIL';
    elseif nMarginal > 2
        overall = 'CONDITIONAL_PASS';
    elseif nMarginal > 0
        overall = 'PASS_WITH_NOTES';
    else
        overall = 'PASS';
    end
end
