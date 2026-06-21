function out = kpi_yawrate_response(t, r, t_step)
%KPI_YAWRATE_RESPONSE Step 입력에 대한 yaw rate 응답 KPI (rise/overshoot/settling)
%
%   out = KPI_YAWRATE_RESPONSE(t, r, t_step)
%
%   Inputs:
%       t      - 시간 벡터 [s]
%       r      - yaw rate 시계열 [rad/s]
%       t_step - step 인가 시점 [s]
%
%   Outputs:
%       out.r_ss            정상상태 yaw rate [rad/s]
%       out.r_peak          peak yaw rate [rad/s]
%       out.overshoot_pct   overshoot [%]
%       out.riseTime        10→90% rise time [s]
%       out.settlingTime    ±2% settling time [s]

    % Step 이후 데이터만 사용
    idx = t >= t_step;
    tt = t(idx) - t_step;
    rr = r(idx);
    if isempty(rr) || numel(rr) < 10
        out.r_ss=NaN; out.r_peak=NaN; out.overshoot_pct=NaN;
        out.riseTime=NaN; out.settlingTime=NaN;
        return;
    end

    % 정상상태: 마지막 20% 평균
    nss = max(round(numel(rr)*0.2), 1);
    out.r_ss = mean(rr(end-nss+1:end));

    % Peak
    [~, ipk] = max(abs(rr));
    out.r_peak = rr(ipk);

    % Overshoot
    if abs(out.r_ss) > 1e-6
        out.overshoot_pct = max(0, (abs(out.r_peak) - abs(out.r_ss)) / abs(out.r_ss)) * 100;
    else
        out.overshoot_pct = NaN;
    end

    % Rise time 10–90%
    target10 = 0.1 * out.r_ss;
    target90 = 0.9 * out.r_ss;
    if abs(out.r_ss) > 1e-6
        sgn = sign(out.r_ss);
        i10 = find(sgn*rr >= sgn*target10, 1, 'first');
        i90 = find(sgn*rr >= sgn*target90, 1, 'first');
        if ~isempty(i10) && ~isempty(i90) && i90 > i10
            out.riseTime = tt(i90) - tt(i10);
        else
            out.riseTime = NaN;
        end
    else
        out.riseTime = NaN;
    end

    % Settling time ±2%
    band = 0.02 * abs(out.r_ss);
    if abs(out.r_ss) > 1e-6
        outBand = abs(rr - out.r_ss) > band;
        % last out-of-band index → next index is settled
        iSettle = find(outBand, 1, 'last');
        if isempty(iSettle)
            out.settlingTime = 0;
        elseif iSettle < numel(rr)
            out.settlingTime = tt(iSettle + 1);
        else
            out.settlingTime = NaN;
        end
    else
        out.settlingTime = NaN;
    end
end
