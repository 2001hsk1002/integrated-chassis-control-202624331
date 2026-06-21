function out = kpi_abs_slip(t, slipRatios, brakeActive, target)
%KPI_ABS_SLIP ABS 슬립률 추종 RMS 오차 (B.3)
%
%   브레이크 작동 중 각 휠의 실제 슬립률과 목표 슬립률 사이의 RMS 오차.
%   목표는 통상 -μ_peak slip (~-0.10 ~ -0.15, 부호: 제동 음수)
%
%   Inputs:
%       t           시간 [s]
%       slipRatios  N×4 (per-wheel kappa)
%       brakeActive logical, brake가 active인 구간
%       target      목표 슬립 (default -0.12)
%
%   Outputs:
%       out.absSlipRMS  per-wheel RMS error (4×1)
%       out.maxSlipRMS  max of per-wheel RMS
%       out.meanSlipRMS mean

    if nargin < 4 || isempty(target); target = -0.12; end

    if ~any(brakeActive)
        out.absSlipRMS = NaN(4,1);
        out.maxSlipRMS = NaN;
        out.meanSlipRMS = NaN;
        return;
    end

    err = slipRatios(brakeActive, :) - target;
    rmsPerWheel = sqrt(mean(err.^2, 1))';
    out.absSlipRMS = rmsPerWheel;
    out.maxSlipRMS = max(rmsPerWheel);
    out.meanSlipRMS = mean(rmsPerWheel);
end
