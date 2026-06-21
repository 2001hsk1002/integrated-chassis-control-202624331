function scenario = scenario_dispatcher(id, SIM, weatherVariant)
%SCENARIO_DISPATCHER ID 기반 ICC 표준 시나리오 빌더
%
%   scenario = SCENARIO_DISPATCHER(id, SIM, weatherVariant)
%
%   ICC 표준 시나리오 ID (A1, A3, A4, A5, A7, B1, B3, C1, D1, D4 등)에 대해
%   driver/brake/road 입력 + 평가 KPI 목록을 포함한 struct 반환.
%
%   각 시나리오는 다음 필드 포함:
%       id, name, refStandard      — 시나리오 메타
%       tEnd, vx0                  — 시간/속도 초기값
%       steerDriver(t)             — function handle [rad], 노면휠 각
%       brakeCmd(t)                — function handle [Nm 4×1], per-wheel brake torque
%       z_road(t, wheel)           — function handle [m], per-wheel 노면 입력 (default 0)
%       mu_wheel(t, wheel)         — function handle [-], per-wheel 마찰계수 (default 1.0)
%       kpis                       — cell array, 평가할 KPI 이름 목록
%       refPath                    — (옵션) N×2 [x,y] 목표 경로
%       weather                    — struct(name, precipitation, intensity, mu_scale)
%       hd                         — struct(objects, banking, laneMaterial, vehicleCatalog)
%
%   weatherVariant — 'dry' (default) | 'wet' | 'snow'
%      mu_wheel을 weather.mu_scale로 자동 스케일.
%
%   사용 예:
%       scenario = scenario_dispatcher('A1', SIM);
%       scenario = scenario_dispatcher('B1', SIM, 'wet');

    if nargin < 2
        SIM = struct('dt', 0.001);
    end
    if nargin < 3 || isempty(weatherVariant)
        weatherVariant = 'dry';
    end

    switch upper(id)
        case 'A1';  scenario = scn_A1_dlc_80(SIM);
        case 'A2';  scenario = scn_A2_dlc_severe(SIM);
        case 'A3';  scenario = scn_A3_step_steer(SIM);
        case 'A4';  scenario = scn_A4_ss_circular(SIM);
        case 'A5';  scenario = scn_A5_sine_with_dwell(SIM);
        case 'A6';  scenario = scn_A6_sine_sweep(SIM);
        case 'A7';  scenario = scn_A7_brake_in_turn(SIM);
        case 'B1';  scenario = scn_B1_straight_brake(SIM);
        case 'B2';  scenario = scn_B2_straight_brake_low_mu(SIM);
        case 'B3';  scenario = scn_B3_split_mu(SIM);
        case 'C1';  scenario = scn_C1_single_bump(SIM);
        case 'C2';  scenario = scn_C2_sweep(SIM);
        case 'D1';  scenario = scn_D1_dlc_brake(SIM);
        otherwise
            error('[scenario_dispatcher] Unknown scenario id: %s', id);
    end

    % 누락 필드 기본값 채움
    if ~isfield(scenario, 'z_road') || isempty(scenario.z_road)
        scenario.z_road = @(t, wheel) 0;
    end
    if ~isfield(scenario, 'mu_wheel') || isempty(scenario.mu_wheel)
        scenario.mu_wheel = @(t, wheel) 1.0;
    end
    if ~isfield(scenario, 'kpis') || isempty(scenario.kpis)
        scenario.kpis = {'yawRateError_rms','slipAngle_max','LTR_max'};
    end

    % Driver model 분류 (시나리오에 명시 안 했으면 추론)
    if ~isfield(scenario, 'driverType') || isempty(scenario.driverType)
        % brake 위주 + 조향 무인가 시나리오 (B*, C*) 는 open_loop
        % steer 입력이 있으나 path-following 의도가 없는 (A3/A4/A5/A6/A7) 시나리오는 robot
        sid = upper(scenario.id);
        if any(strcmp(sid, {'B1','B2','B3','C1','C2'}))
            scenario.driverType = 'open_loop';
        else
            scenario.driverType = 'robot';
        end
    end

    % HD scenario default 필드
    if ~isfield(scenario, 'hd')
        scenario.hd = struct();
    end
    if ~isfield(scenario.hd, 'objects');       scenario.hd.objects = {}; end
    if ~isfield(scenario.hd, 'banking');       scenario.hd.banking = []; end
    if ~isfield(scenario.hd, 'laneMaterial');  scenario.hd.laneMaterial = []; end
    if ~isfield(scenario.hd, 'vehicleCatalog')
        scenario.hd.vehicleCatalog = struct('name','BMW_5','infoFile','BMW_5_15_030326');
    end

    % Weather variant 적용 (mu_wheel 자동 스케일)
    scenario = local_apply_weather(scenario, weatherVariant);
end

function scenario = local_apply_weather(scenario, variant)
    switch lower(variant)
        case 'dry'
            w = struct('name','dry', 'precipitation','none', 'intensity',0.0, 'mu_scale',1.0);
        case 'wet'
            w = struct('name','wet', 'precipitation','rain', 'intensity',0.6, 'mu_scale',0.7);
        case 'snow'
            w = struct('name','snow','precipitation','snow', 'intensity',0.5, 'mu_scale',0.3);
        otherwise
            error('[scenario_dispatcher] Unknown weather variant: %s', variant);
    end
    scenario.weather = w;
    if w.mu_scale ~= 1.0
        mu0 = scenario.mu_wheel;
        scenario.mu_wheel = @(t,wh) mu0(t,wh) * w.mu_scale;
    end
end
