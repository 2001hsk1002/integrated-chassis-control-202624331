function scenario = scn_B1_straight_brake(SIM)
%SCN_B1_STRAIGHT_BRAKE ISO 21994 Straight-line braking (Type-0, 100→0 km/h, dry)
%
%   100 km/h 정상 주행 → t=1s에 brake step → 차량 정지

    scenario.id          = 'B1';
    scenario.name        = 'ISO 21994 Straight Braking 100→0 km/h (dry, μ≈1.0)';
    scenario.refStandard = 'ISO 21994:2007 / UN-R 13H';
    scenario.tEnd        = 5.0;
    scenario.vx0         = 100 / 3.6;       % 27.78 m/s

    scenario.steerDriver = @(t) 0;

    % 최대 brake 토크 (per wheel) 인가, t≥1s
    Tf = 1500;  % 1500 Nm/wheel front
    Tr =  800;  % 800 Nm/wheel rear
    scenario.brakeCmd = @(t) (t >= 1.0) * [Tf; Tf; Tr; Tr];

    scenario.kpis = {'stoppingDistance','MFDD','muUtilization','jerkMax', ...
                     'absSlipRMS','yawDevDuringABS'};
end
