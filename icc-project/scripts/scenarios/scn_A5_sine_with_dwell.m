function scenario = scn_A5_sine_with_dwell(SIM)
%SCN_A5_SINE_WITH_DWELL FMVSS 126 / ISO 19365 Sine-with-Dwell
%
%   0.7 Hz steering sine + 0.5 s dwell at 2nd peak.
%   진폭 = 6.5 × A_{0.3g}  (FMVSS 126의 reference amplitude, 보통 약 80~270 deg @ 핸들)
%   본 구현은 노면휠 각 기준으로 A = 6.5° (≈ 핸들 ratio 13:1 → 핸들 84°)
%
%   FMVSS 126 PASS:
%     - R1.0 = |r(BOS+1.0s)| / r_peak  ≤ 0.35
%     - R1.75 = |r(BOS+1.75s)| / r_peak ≤ 0.20
%     - Lateral displacement at BOS+1.07s ≥ 1.83 m

    scenario.id          = 'A5';
    scenario.name        = 'FMVSS 126 Sine-with-Dwell (0.7 Hz + 0.5 s dwell)';
    scenario.refStandard = 'NHTSA FMVSS 126 / ISO 19365:2016';
    scenario.tEnd        = 5.0;
    scenario.vx0         = 80 / 3.6;     % 80 km/h ± 2 km/h per FMVSS 126

    scenario.steerDriver = @(t) local_sine_with_dwell(t);
    scenario.brakeCmd    = @(t) zeros(4, 1);

    scenario.kpis = {'sineDwellR1p0','sineDwellR1p75','lateralDispBOS1p07', ...
                     'sideSlipMax','LTR_max'};

    % Meta: BOS time (Begin of Steer) — 1.0 s
    scenario.BOS = 1.0;
end

function delta = local_sine_with_dwell(t)
    A = deg2rad(6.5);    % roadwheel angle amplitude
    f = 0.7;             % Hz
    T = 1/f;             % 1.428 s
    t_start = 1.0;       % BOS

    if t < t_start
        delta = 0;
    elseif t < t_start + T/2
        delta = A * sin(2*pi*f*(t - t_start));
    elseif t < t_start + T/2 + 0.5
        delta = -A;       % 0.5 s dwell at 2nd peak (negative)
    elseif t < t_start + T/2 + 0.5 + T/4
        tau = t - t_start - T/2 - 0.5;
        delta = -A * cos(2*pi*f*tau);
    else
        delta = 0;
    end
end
