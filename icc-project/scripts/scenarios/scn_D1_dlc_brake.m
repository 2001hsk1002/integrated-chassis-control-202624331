function scenario = scn_D1_dlc_brake(SIM)
%SCN_D1_DLC_BRAKE ICC 통합 — DLC + 0.3g 제동
%
%   ISO 3888-1 DLC 조향 + 동시에 0.3g 일정 제동.
%   AFS + ABS + Coordinator 통합 검증.

    scenario.id          = 'D1';
    scenario.name        = 'DLC + 0.3g Braking (Integration test)';
    scenario.refStandard = 'OEM combined / ISO 3888-1 + ISO 21994 derived';
    scenario.tEnd        = 8.0;
    scenario.vx0         = 80 / 3.6;

    scenario.steerDriver = @(t) local_dlc_steer(t);

    % 0.3g 제동 (m=1600 kg → 4.7 kN total, per-wheel torque)
    Tf = 470;   % Nm/wheel front  (60% bias)
    Tr = 320;   % Nm/wheel rear  (40% bias)
    scenario.brakeCmd = @(t) (t >= 2.0 && t < 5.5) * [Tf; Tf; Tr; Tr];

    scenario.kpis = {'yawRateOvershoot','sideSlipMax','lateralDevMax', ...
                     'tireUtilizationMax','LTR_max','stoppingDistance', ...
                     'jerkMax','yawDevDuringBrake'};

    % A1 동일 DLC path (driver model 추종 대상) + 0.3g 제동 병행
    scenario.refPath = [0 0; 12 0; 25 3.5; 36 3.5; 61 0; 80 0];
    scenario.driverType = 'path_follow_stanley';

    % HD scenario — ISO 3888-1 cones (A1과 동일)
    gates = [0 0 1; 12 0 1; 25 3.5 1; 36 3.5 1; 61 0 1; 80 0 1];
    cones = {};
    for k = 1:size(gates,1)
        s = gates(k,1); tCtr = gates(k,2); hw = gates(k,3);
        cones{end+1} = struct('s',s,'t',tCtr+hw,'name',sprintf('cone_L_%d',k),'type','obstacle','height',0.5); %#ok<AGROW>
        cones{end+1} = struct('s',s,'t',tCtr-hw,'name',sprintf('cone_R_%d',k),'type','obstacle','height',0.5); %#ok<AGROW>
    end
    scenario.hd.objects = cones;
end

function delta = local_dlc_steer(t)
    if t >= 2.0 && t < 3.0
        delta = deg2rad(3.0) * sin(pi * (t - 2.0));
    elseif t >= 3.0 && t < 4.5
        delta = -deg2rad(3.0) * sin(pi * (t - 3.0) / 1.5);
    elseif t >= 4.5 && t < 6.0
        delta = deg2rad(3.0) * sin(pi * (t - 4.5) / 1.5);
    else
        delta = 0;
    end
end
