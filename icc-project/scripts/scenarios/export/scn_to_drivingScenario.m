function ds = scn_to_drivingScenario(scenario, opts)
%SCN_TO_DRIVINGSCENARIO 우리 scenario struct → MATLAB ADT drivingScenario 객체
%
%   ds = SCN_TO_DRIVINGSCENARIO(scenario, opts)
%
%   변환 내용:
%     1. Straight road (200 m, 2-lane each direction) — A1/A3/B1 모두 직선
%        DLC 트랙은 lane center를 따라가므로 path-following으로 표현 가능
%     2. Ego vehicle — BMW 5 size, scenario.vx0 초기 속도
%     3. Steering / Brake / Throttle 시계열 → SmoothTrajectory (parametric)
%
%   Inputs:
%       scenario - scenario_dispatcher 출력
%       opts - struct (옵션):
%           .roadLength (default 200 m)
%           .roadLaneWidth (default 3.5 m)
%           .roadNumLanes (default 4, two each direction)
%           .scenarioLen (default scenario.tEnd)
%
%   Outputs:
%       ds - drivingScenario object (ADT)
%
%   See also: drivingScenario, export

    if nargin < 2; opts = struct(); end
    if ~isfield(opts,'roadLength');    opts.roadLength = 200; end
    if ~isfield(opts,'roadLaneWidth'); opts.roadLaneWidth = 3.5; end
    if ~isfield(opts,'roadNumLanes');  opts.roadNumLanes = 4; end
    if ~isfield(opts,'scenarioLen');   opts.scenarioLen = scenario.tEnd; end

    %% 1. Driving scenario 생성
    ds = drivingScenario;
    ds.SampleTime = 0.01;
    ds.StopTime = opts.scenarioLen;

    %% 2. 도로 추가 (직선 200 m, 4-lane)
    roadCenters = [0 0 0; opts.roadLength 0 0];
    ls = lanespec(opts.roadNumLanes, 'Width', opts.roadLaneWidth);
    road(ds, roadCenters, 'Lanes', ls);

    %% 3. Ego 차량 생성
    egoVeh = vehicle(ds, 'ClassID', 1, ...                       % 1 = passenger car
        'Length', 4.94, 'Width', 1.87, 'Height', 1.48, ...       % BMW 5 dims
        'PlotColor', [0.85 0.2 0.2]);

    %% 4. 시간 격자에서 trajectory 적분 (ego 운동학)
    dt = ds.SampleTime;
    t = (0:dt:opts.scenarioLen)';
    n = numel(t);

    % open-loop 운동학 시뮬레이션 (Bicycle 근사)
    L = 2.7;  % wheelbase 근사
    vx = zeros(n,1); psi = zeros(n,1); x = zeros(n,1); y = zeros(n,1);
    vx(1) = scenario.vx0;
    % lane 1 (right driving) center: y = -1.75 (negative because of standard convention)
    y(1) = -opts.roadLaneWidth/2;

    for k = 2:n
        delta = scenario.steerDriver(t(k-1));
        bk = scenario.brakeCmd(t(k-1));
        if isscalar(bk); bk = bk*ones(4,1); end
        % 매우 단순한 종방향: brake → ax = -sum(T)/r_w/mass
        Tbrake = sum(bk);
        Fbrake = -Tbrake / 0.33;          % per-wheel rw≈0.33
        m_veh  = 1600;
        ax = Fbrake/m_veh;
        % aero drag
        ax = ax - 0.5*1.225*0.30*2.2*vx(k-1)^2/m_veh;
        vx(k) = max(vx(k-1) + ax*dt, 0);
        % yaw integration (bicycle): r = vx*delta/L
        r = vx(k-1) * delta / L;
        psi(k) = psi(k-1) + r * dt;
        x(k) = x(k-1) + vx(k-1)*cos(psi(k-1))*dt;
        y(k) = y(k-1) + vx(k-1)*sin(psi(k-1))*dt;
    end

    waypoints = [x, y, zeros(n,1)];
    speeds    = vx;

    %% 5. Ego trajectory 등록
    trajectory(egoVeh, waypoints, speeds);

    % Note: drivingScenario는 UserData 속성을 지원하지 않으므로 외부에 메타 저장 불가
end
