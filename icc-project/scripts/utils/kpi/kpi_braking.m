function out = kpi_braking(t, vx, ax, mu_eff)
%KPI_BRAKING 종방향 제동 KPI (stopping distance, MFDD, μ utilization, jerk)
%
%   UN-R 13H Annex 3 정의:
%     MFDD = (0.8·v₀)² / (25.92 · (s₂ − s₁))      [m/s²]
%     where s₁ = 0.8·v₀ 도달 시 누적거리, s₂ = 0.1·v₀ 도달 시 누적거리
%
%   Inputs:
%       t      - 시간 [s]
%       vx     - 종방향 속도 [m/s]
%       ax     - 종방향 가속도 [m/s²]
%       mu_eff - 효과적 μ (기본 1.0)
%
%   Outputs:
%       out.v0                초기 속도 [m/s]
%       out.stoppingDistance  v_x = 0 도달까지 거리 [m]
%       out.MFDD              Mean Fully-Developed Deceleration [m/s²]
%       out.muUtilization     MFDD / (μ·g)
%       out.jerkMax           |d(ax)/dt|_max [m/s³]
%       out.timeToStop        브레이크 인가부터 정지까지 시간 [s]

    if nargin < 4 || isempty(mu_eff); mu_eff = 1.0; end

    v0 = vx(1);
    out.v0 = v0;

    % 누적 거리
    s = cumtrapz(t, vx);

    % 정지 시점 (vx < 0.5 m/s로 정의)
    iStop = find(vx < 0.5, 1, 'first');
    if isempty(iStop)
        iStop = numel(vx);
        out.stoppingDistance = NaN;
        out.timeToStop = NaN;
    else
        out.stoppingDistance = s(iStop) - s(1);
        out.timeToStop = t(iStop) - t(1);
    end

    % MFDD (UN-R 13H)
    v_80 = 0.8 * v0; v_10 = 0.1 * v0;
    i1 = find(vx <= v_80, 1, 'first');
    i2 = find(vx <= v_10, 1, 'first');
    if ~isempty(i1) && ~isempty(i2) && i2 > i1
        ds = s(i2) - s(i1);
        out.MFDD = (v_80^2 - v_10^2) / (2 * ds);   % positive number
        out.muUtilization = out.MFDD / (mu_eff * 9.81);
    else
        out.MFDD = NaN;
        out.muUtilization = NaN;
    end

    % Max jerk
    dt = mean(diff(t));
    jerk = [0; diff(ax)] / dt;
    out.jerkMax = max(abs(jerk));
end
