function scenario = scn_C2_sweep(SIM)
%SCN_C2_SWEEP Vertical Sinusoidal Sweep (0.1 → 25 Hz) — ride frequency response
%
%   OEM ride sweep — 4-wheel heave 입력의 주파수 sweep으로 body-bounce (1–2 Hz) 와
%   wheel-hop (10–15 Hz) 모드 분리. CDC/Active Suspension의 frequency-domain 평가.
%
%   본 구현은 4-wheel 동일 입력 (pure heave) 의 linear chirp.

    scenario.id          = 'C2';
    scenario.name        = 'Vertical Sweep 0.1 → 25 Hz (Ride frequency response)';
    scenario.refStandard = 'OEM ride sweep / ISO 8608 derived';
    scenario.tEnd        = 20.0;          % 20 s sweep — 저주파 영역 확보
    scenario.vx0         = 30 / 3.6;      % 8.33 m/s

    scenario.steerDriver = @(t) 0;
    scenario.brakeCmd    = @(t) zeros(4, 1);

    % Linear chirp 0.1 → 25 Hz, amplitude 10 mm (선형 영역 유지)
    scenario.z_road   = @(t, w) local_chirp_z(t);
    scenario.mu_wheel = @(t, w) 1.0;

    scenario.kpis = {'peakSuspTravel','wheelHopAmp','bodyBounceAmp', ...
                     'sprungRMSAcc','unsprungRMSAcc','rideComfortVDV'};
end

function z = local_chirp_z(t)
    f0 = 0.1; f1 = 25.0; T = 20.0;
    AMP = 0.010;                          % 10 mm
    if t >= 0 && t <= T
        k = (f1 - f0) / T;
        phi = 2*pi * (f0 * t + 0.5 * k * t^2);
        z = AMP * sin(phi);
    else
        z = 0;
    end
end
