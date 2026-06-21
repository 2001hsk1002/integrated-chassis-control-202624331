function scenario = scn_A4_ss_circular(SIM)
%SCN_A4_SS_CIRCULAR ISO 4138 Steady-state circular driving
%
%   Method 2 (constant radius, speed ramp). R = 50 m, vx 5 → 25 m/s (8s ramp + 4s hold)
%   목표 핸들 각은 ay = vx²/R 추종을 위한 정상상태 운전자 입력
%   본 구현은 open-loop으로 ay 4 m/s² 도달까지 ramping

    scenario.id          = 'A4';
    scenario.name        = 'ISO 4138 Steady-State Circular (R=50 m, constant radius)';
    scenario.refStandard = 'ISO 4138:2021';
    scenario.tEnd        = 12.0;
    scenario.vx0         = 5.0;      % start slow

    R = 50;
    L = 2.047;   % wheelbase (BMW_5)
    scenario.steerDriver = @(t) local_steer_for_R(t, R, L);
    scenario.brakeCmd    = @(t) zeros(4, 1);

    scenario.kpis = {'yawRateSteadyState','aySteadyState','understeerGradient', ...
                     'LTR_max','tireUtilizationMax'};

    % HD scenario — banking 2° (ISO 4138 권장 high-speed circular 평가)
    scenario.hd.banking = struct('s0', 0, 'length', 200, 'angle_deg', 2);
end

function delta = local_steer_for_R(t, R, L)
    % vx ramps linearly 5 → 25 m/s over 8 s, then hold
    if t < 8
        vx = 5 + (25 - 5) * t / 8;
    else
        vx = 25;
    end
    % Ackerman + Kus·ay  근사: δ ≈ L/R + Kus·vx²/(g·R)
    Kus = 0.001;   % near-neutral BMW_5
    delta = L/R + Kus * vx^2 / (9.81 * R);
end
