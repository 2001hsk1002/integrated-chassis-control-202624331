function scenario = scn_A7_brake_in_turn(SIM)
%SCN_A7_BRAKE_IN_TURN ISO 7975 Brake-in-Turn
%
%   정상 선회 (R=100m, ay ≈ 0.4g) 중 t=3s에 0.4g brake step 인가.
%   AFS + ABS 통합 안정성 평가.

    scenario.id          = 'A7';
    scenario.name        = 'ISO 7975 Brake-in-Turn (R=100m, 0.4g brake step)';
    scenario.refStandard = 'ISO 7975:2019';
    scenario.tEnd        = 6.0;
    scenario.vx0         = 100 / 3.6;       % 27.78 m/s → ay = 7.7 m/s² @ R=100

    R = 100; L = 2.047;
    delta_const = L/R + 0.001 * scenario.vx0^2 / (9.81 * R);  % approx
    scenario.steerDriver = @(t) delta_const;

    % t=3s에 0.4g 감속을 위한 brake torque
    % Fx_total = m·a = 1600·3.924 ≈ 6280 N → T = Fx·rw ≈ 2070 Nm 총
    % per-wheel: front 60%, rear 40%
    Tf = 6280 * 0.6 / 2 * 0.3295;   % 621 Nm/wheel front
    Tr = 6280 * 0.4 / 2 * 0.3295;   % 414 Nm/wheel rear
    scenario.brakeCmd = @(t) (t >= 3.0) * [Tf; Tf; Tr; Tr];

    scenario.kpis = {'yawDevDuringBrake','sideSlipMax','lateralDevMax', ...
                     'LTR_max','tireUtilizationMax'};
end
