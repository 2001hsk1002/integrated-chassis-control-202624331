function out = kpi_sine_with_dwell(t, r, ay, BOS)
%KPI_SINE_WITH_DWELL FMVSS 126 sine-with-dwell KPI
%
%   FMVSS 126 §S5.2.2.1:
%     R1.0  = |r(BOS+1.0s)| / r_peak  ≤ 0.35
%     R1.75 = |r(BOS+1.75s)| / r_peak ≤ 0.20
%   FMVSS 126 §S5.2.2.3:
%     Lateral displacement at BOS+1.07s ≥ 1.83 m
%
%   Inputs:
%       t   - 시간 [s]
%       r   - yaw rate [rad/s]
%       ay  - lateral acceleration [m/s²]
%       BOS - Begin-of-Steer time [s]
%
%   Outputs:
%       out.r_peak             peak yaw rate after BOS
%       out.r_at_1p0           yaw rate at BOS+1.0s
%       out.r_at_1p75          yaw rate at BOS+1.75s
%       out.R1p0               ratio at 1.0s
%       out.R1p75              ratio at 1.75s
%       out.lateralDispBOS1p07 lateral displacement at BOS+1.07s (m, integrated ay)
%       out.pass_R1p0          true if ≤0.35
%       out.pass_R1p75         true if ≤0.20
%       out.pass_disp          true if ≥1.83 m

    % Peak yaw rate (절대값)
    idxPost = t >= BOS;
    rPost   = r(idxPost);
    tPost   = t(idxPost);
    out.r_peak = max(abs(rPost));

    % BOS+1.0, BOS+1.75 시점 yaw rate
    out.r_at_1p0  = local_interp(t, r, BOS + 1.0);
    out.r_at_1p75 = local_interp(t, r, BOS + 1.75);

    if out.r_peak > 1e-3
        out.R1p0  = abs(out.r_at_1p0)  / out.r_peak;
        out.R1p75 = abs(out.r_at_1p75) / out.r_peak;
    else
        out.R1p0 = NaN; out.R1p75 = NaN;
    end

    % Lateral displacement: 적분 ay (2회 적분 — Y = ∫∫ ay dt²)
    % FMVSS 126는 measured y position 기준; 여기서는 ay 적분 근사
    iEnd_disp = find(t >= BOS + 1.07, 1, 'first');
    if isempty(iEnd_disp)
        out.lateralDispBOS1p07 = NaN;
    else
        idxR = (t >= BOS) & (t <= BOS + 1.07);
        tR = t(idxR); ayR = ay(idxR);
        vy_int = cumtrapz(tR, ayR);
        y_int  = cumtrapz(tR, vy_int);
        out.lateralDispBOS1p07 = abs(y_int(end));
    end

    out.pass_R1p0  = out.R1p0  <= 0.35;
    out.pass_R1p75 = out.R1p75 <= 0.20;
    out.pass_disp  = out.lateralDispBOS1p07 >= 1.83;
    out.pass_all   = out.pass_R1p0 && out.pass_R1p75 && out.pass_disp;
end

function y = local_interp(t, x, tq)
    if tq < t(1) || tq > t(end)
        y = NaN;
    else
        y = interp1(t, x, tq, 'linear');
    end
end
