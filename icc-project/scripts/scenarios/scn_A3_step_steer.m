function scenario = scn_A3_step_steer(SIM)
%SCN_A3_STEP_STEER ISO 7401 Step steer (open-loop)
%
%   t=1s에 δ=2° step 입력 후 yaw rate transient (rise/overshoot/settling) 평가

    scenario.id          = 'A3';
    scenario.name        = 'ISO 7401 Step Steer (δ=2°, 80 km/h)';
    scenario.refStandard = 'ISO 7401:2011';
    scenario.tEnd        = 4.0;
    scenario.vx0         = 80 / 3.6;

    scenario.steerDriver = @(t) deg2rad(2.0) * (t >= 1.0);
    scenario.brakeCmd    = @(t) zeros(4, 1);

    scenario.kpis = {'yawRateRiseTime','yawRateOvershoot','yawRateSettling', ...
                     'sideSlipMax','tireUtilizationMax'};
end
