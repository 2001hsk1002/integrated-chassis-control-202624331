function scenario = scn_B3_split_mu(SIM)
%SCN_B3_SPLIT_MU ISO 14512 Split-μ Braking
%
%   ISO 14512:1999 — 좌/우 μ가 다른 노면에서 직진 제동
%   좌측: high-μ (1.0), 우측: low-μ (0.3)
%   100 km/h → 0 km/h, brake t=1s 인가.

    scenario.id          = 'B3';
    scenario.name        = 'ISO 14512 Split-μ Braking (left μ=1.0 / right μ=0.3)';
    scenario.refStandard = 'ISO 14512:1999';
    scenario.tEnd        = 5.0;
    scenario.vx0         = 100 / 3.6;        % 27.78 m/s

    scenario.steerDriver = @(t) 0;

    % B1과 동일한 brake 토크 인가
    Tf = 1500; Tr = 800;
    scenario.brakeCmd = @(t) (t >= 1.0) * [Tf; Tf; Tr; Tr];

    % Per-wheel μ — 좌측 휠 (FL, RL) 고μ, 우측 (FR, RR) 저μ
    scenario.mu_wheel = @(t, w) local_mu_split(t, w);

    scenario.kpis = {'stoppingDistance','MFDD','muUtilization','jerkMax', ...
                     'absSlipRMS','yawDevDuringABS','lateralDevMax'};

    % B3 시나리오는 RoadFrictionAction을 OSC에서 사용해야 함
    scenario.muSplit.left  = 1.0;
    scenario.muSplit.right = 0.3;

    % HD scenario — OpenDRIVE lane material (XODR 우측 lane 기준; 차량은 lane -1/-2 사이 주행)
    %   lane -1: 중심선에 가까운 쪽 → 좌측 타이어 통과 → μ=1.0
    %   lane -2: 더 바깥쪽         → 우측 타이어 통과 → μ=0.3
    scenario.hd.laneMaterial = { ...
        struct('laneId', -1, 'friction', 1.0), ...
        struct('laneId', -2, 'friction', 0.3) };
end

function mu = local_mu_split(~, w)
    % w ∈ {1=FL, 2=FR, 3=RL, 4=RR}
    if w == 1 || w == 3   % left side
        mu = 1.0;
    else                  % right side
        mu = 0.3;
    end
end
