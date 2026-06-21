function scenario = scn_A2_dlc_severe(SIM)
%SCN_A2_DLC_SEVERE ISO 3888-2 Severe Double Lane Change (Moose test)
%
%   ISO 3888-2:2011 — Passenger cars — Test track for severe lane-change.
%   ISO 3888-1 대비 lane width가 좁고 (entry 1.1·b m, sec.3 1.3·b+0.25 m), 시험 속도는
%   feasibility 한계 (max stable entry speed)를 측정. 본 구현은 typical 60 km/h 회피기동.
%   ICC 평가에서는 ESC/AFS의 한계 핸들링 응답 (sideSlip, ay 한계) 검증.

    scenario.id          = 'A2';
    scenario.name        = 'ISO 3888-2 Severe DLC (Moose test) @ 60 km/h';
    scenario.refStandard = 'ISO 3888-2:2011';
    scenario.tEnd        = 8.0;
    scenario.vx0         = 60 / 3.6;     % 16.67 m/s

    % A1보다 짧고 가파른 sine — peak 5°, 0.7 s pulse (회피 강도 증가)
    scenario.steerDriver = @(t) local_steer(t);
    scenario.brakeCmd    = @(t) zeros(4, 1);
    scenario.z_road      = @(t, w) 0;
    scenario.mu_wheel    = @(t, w) 1.0;

    scenario.kpis = {'yawRateOvershoot','sideSlipMax','lateralDevMax', ...
                     'tireUtilizationMax','LTR_max'};

    scenario.refPath = local_dlc_path();
    scenario.driverType = 'path_follow_stanley';

    % HD scenario — ISO 3888-2 cone gates (좁은 lane: width 0.9 m, 더 짧은 section)
    %   Section 1 (entry): s=0–12 m, 폭 1.1·b ≈ 1.95 m
    %   Section 2 (offset): s=13.5–25 m, 폭 0.9·b+0.25 ≈ 1.85 m, lateral offset 1 m
    %   Section 3 (exit): s=37.5–49.5 m, 폭 1.3·b+0.25 ≈ 2.55 m
    gates = [ ...
        0,    0.0, 0.97; ...
       12,    0.0, 0.97; ...
       13.5,  1.0, 0.92; ...
       25,    1.0, 0.92; ...
       37.5,  0.0, 1.27; ...
       49.5,  0.0, 1.27];
    cones = {};
    for k = 1:size(gates,1)
        s = gates(k,1); tCtr = gates(k,2); hw = gates(k,3);
        cones{end+1} = struct('s',s,'t',tCtr+hw,'name',sprintf('cone_L_%d',k),'type','obstacle','height',0.5); %#ok<AGROW>
        cones{end+1} = struct('s',s,'t',tCtr-hw,'name',sprintf('cone_R_%d',k),'type','obstacle','height',0.5); %#ok<AGROW>
    end
    scenario.hd.objects = cones;
end

function delta = local_steer(t)
    % Moose test는 더 큰 진폭/짧은 dwell (peak 5°)
    if t >= 1.5 && t < 2.2
        delta = deg2rad(5.0) * sin(pi * (t - 1.5) / 0.7);
    elseif t >= 2.2 && t < 3.2
        delta = -deg2rad(5.0) * sin(pi * (t - 2.2) / 1.0);
    elseif t >= 3.2 && t < 4.0
        delta = deg2rad(5.0) * sin(pi * (t - 3.2) / 0.8);
    else
        delta = 0;
    end
end

function path = local_dlc_path()
    % ISO 3888-2 ideal center line (앞 section 더 짧음)
    x = [0  12  18  25  37.5  49.5];
    y = [0   0  1.0 1.0   0    0];
    path = [x', y'];
end
