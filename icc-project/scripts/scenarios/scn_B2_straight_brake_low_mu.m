function scenario = scn_B2_straight_brake_low_mu(SIM)
%SCN_B2_STRAIGHT_BRAKE_LOW_MU ISO 21994 Straight-Line Braking on Low-μ (Ice)
%
%   ISO 21994:2007 — 균일한 저-μ (μ ≈ 0.3, 빙판/포장 결빙) 노면에서 직진 제동.
%   ABS 효율과 μ 활용도 평가. B1 (고-μ asphalt μ≈1.0) 의 짝.
%   80 km/h → 0 km/h, brake step t=1 s 인가.

    scenario.id          = 'B2';
    scenario.name        = 'ISO 21994 Straight-Line Braking on Low-μ (μ=0.3)';
    scenario.refStandard = 'ISO 21994:2007';
    scenario.tEnd        = 8.0;
    scenario.vx0         = 80 / 3.6;     % 22.22 m/s — ABS 시험 표준속도

    scenario.steerDriver = @(t) 0;

    % B1과 동일한 brake 토크 인가 (ABS가 알아서 슬립 제한)
    Tf = 1500; Tr = 800;
    scenario.brakeCmd = @(t) (t >= 1.0) * [Tf; Tf; Tr; Tr];

    % 균질 저-μ
    scenario.mu_wheel = @(t, w) 0.3;
    scenario.z_road   = @(t, w) 0;

    scenario.kpis = {'stoppingDistance','MFDD','muUtilization', ...
                     'absSlipRMS','jerkMax','yawDevDuringABS'};
end
