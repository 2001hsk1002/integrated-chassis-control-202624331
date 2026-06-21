function scenario = scn_A1_dlc_80(SIM)
%SCN_A1_DLC_80 ISO 3888-1 Double Lane Change @ 80 km/h
%
%   ISO 3888-1:2018 — Passenger cars — Test track for a severe lane-change manoeuvre
%   80 km/h 진입, 3-section 트랙 (12 m + 13.5 m offset + 12 m)
%   본 구현은 운전자 모델 없이 open-loop 조향 sine 시퀀스 — 정확한 경로 추적은 Phase 4 (driver model) 이후

    scenario.id          = 'A1';
    scenario.name        = 'ISO 3888-1 DLC @ 80 km/h';
    scenario.refStandard = 'ISO 3888-1:2018';
    scenario.tEnd        = 8.0;
    scenario.vx0         = 80 / 3.6;        % 22.22 m/s

    % 운전자 조향 — 3-section sine 시퀀스 (노면휠 각, [rad])
    scenario.steerDriver = @(t) local_steer(t);

    % brake 무인가
    scenario.brakeCmd = @(t) zeros(4, 1);

    % 평탄로 + 균일 마찰
    scenario.z_road   = @(t, w) 0;
    scenario.mu_wheel = @(t, w) 1.0;

    % 평가 KPI
    scenario.kpis = {'yawRateOvershoot','yawRateSettling','sideSlipMax', ...
                     'lateralDevMax','LTR_max','tireUtilizationMax'};

    % 목표 경로 (driver model 추종 대상)
    scenario.refPath = local_dlc_path();
    scenario.driverType = 'path_follow_stanley';

    % HD scenario — ISO 3888-1 cone gates (좌·우 5 m × 2 줄, gate별 위치)
    scenario.hd.objects = local_iso3888_cones();
end

function cones = local_iso3888_cones()
    % ISO 3888-1 cone gate positions (Section 1: entry, 2: lane change, 3: exit)
    %   gate centers (s [m], lateralOffset [m], halfWidth [m])
    gates = [ ...
        0,    0.0, 1.0;   % Section 1 entry
       12,    0.0, 1.0;   % Section 1 exit
       25,    3.5, 1.0;   % Section 2 entry (left lane)
       36,    3.5, 1.0;   % Section 2 exit
       61,    0.0, 1.0;   % Section 3 entry
       80,    0.0, 1.0];  % Section 3 exit
    cones = {};
    for k = 1:size(gates,1)
        s = gates(k,1); tCtr = gates(k,2); hw = gates(k,3);
        cones{end+1} = struct('s',s,'t',tCtr+hw,'name',sprintf('cone_L_%d',k),'type','obstacle','height',0.5);
        cones{end+1} = struct('s',s,'t',tCtr-hw,'name',sprintf('cone_R_%d',k),'type','obstacle','height',0.5);
    end
end

function delta = local_steer(t)
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

function path = local_dlc_path()
    % ISO 3888-1 ideal center line
    x = [0  12  25  36  61  73  80];
    y = [0   0  3.5 3.5   0   0   0];
    path = [x', y'];
end
