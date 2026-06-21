function scenario = scn_A6_sine_sweep(SIM)
%SCN_A6_SINE_SWEEP ISO 7401 Sinusoidal Steer Sweep — frequency response (Bode)
%
%   ISO 7401:2011 §6.2 — random/sinusoidal steer input으로 frequency response 측정.
%   본 구현은 linear chirp 0.1 → 1.1 Hz를 8 s 에 걸쳐 인가 → yaw rate / lateral acc
%   gain·phase 곡선 (handling Bode plot) 추출.

    scenario.id          = 'A6';
    scenario.name        = 'ISO 7401 Sinusoidal Steer Sweep (0.1 → 1.1 Hz)';
    scenario.refStandard = 'ISO 7401:2011';
    scenario.tEnd        = 10.0;          % 8 s sweep + 2 s settle
    scenario.vx0         = 80 / 3.6;      % 22.22 m/s

    % 진폭 peak 1° (Bode 영역 유지 위해 작게)
    scenario.steerDriver = @(t) local_chirp_steer(t);
    scenario.brakeCmd    = @(t) zeros(4, 1);
    scenario.z_road      = @(t, w) 0;
    scenario.mu_wheel    = @(t, w) 1.0;

    scenario.kpis = {'yawRateGain_lf','yawRateBandwidth','yawRatePhaseLag90', ...
                     'lateralAccGain_lf','lateralAccPhaseLag90'};
end

function delta = local_chirp_steer(t)
    % Linear chirp 0.1 → 1.1 Hz over 0 ≤ t ≤ 8 s
    f0 = 0.1; f1 = 1.1; T = 8.0;
    AMP = deg2rad(1.0);
    if t >= 0 && t <= T
        k = (f1 - f0) / T;
        % phase: 2π · ∫₀ᵗ (f0 + k·τ) dτ = 2π · (f0·t + 0.5·k·t²)
        phi = 2*pi * (f0 * t + 0.5 * k * t^2);
        delta = AMP * sin(phi);
    else
        delta = 0;
    end
end
