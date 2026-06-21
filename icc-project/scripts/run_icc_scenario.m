function [result, kpi] = run_icc_scenario(scenarioId, plantModel, varargin)
%RUN_ICC_SCENARIO ICC 표준 시나리오 1개를 plant + driver model + (옵션) ICC 제어기로 실행
%
%   [result, kpi] = RUN_ICC_SCENARIO(scenarioId, plantModel, ...)
%
%   Inputs:
%       scenarioId  - (char) 'A1', 'A3', ..., 'D1' 등
%       plantModel  - (char) 'bicycle' | '3dof' | '7dof' | '14dof'
%       옵션 (Name-Value):
%           'Controller' (default 'off') — 'off' 베이스라인, 'on' ICC 제어기 활성
%           'WeatherVariant' (default 'dry') — 'dry' | 'wet' | 'snow'
%           'SavePlot'   (default true)
%
%   Outputs:
%       result - 시계열 + per-wheel forces + 제어기 출력 로그
%       kpi    - 시나리오별 KPI struct

    p = inputParser;
    addParameter(p, 'Controller',     'off', @(x) any(strcmpi(x,{'on','off'})));
    addParameter(p, 'WeatherVariant', 'dry', @ischar);
    addParameter(p, 'SavePlot',       true,  @islogical);
    addParameter(p, 'Solver',         '',    @ischar);  % '' = sim_params 의 SIM.solver 기본값 사용
    parse(p, varargin{:});
    ctrlOn  = strcmpi(p.Results.Controller, 'on');

    %% 초기화
    thisDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(thisDir);
    if ~exist('SIM','var') || ~isstruct(SIM) || ~isfield(SIM,'cm_vehicleFile')
        run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));
    end
    addpath(fullfile(projectRoot, 'scripts', 'driver'));
    SIM.plantModel = plantModel;
    if ~isempty(p.Results.Solver)
        SIM.solver = p.Results.Solver;   % override sim_params default
    end
    params = struct('VEH', VEH, 'TIRE', TIRE, 'CONST', CONST, 'SIM', SIM);

    %% 시나리오 로드 (weather variant 적용)
    scenario = scenario_dispatcher(scenarioId, SIM, p.Results.WeatherVariant);
    dt = SIM.dt;
    n  = round(scenario.tEnd / dt) + 1;
    t  = (0:n-1)' * dt;

    fprintf('\n=== %s [%s | controller=%s | weather=%s] : %s ===\n', ...
        scenario.id, plantModel, p.Results.Controller, scenario.weather.name, scenario.name);
    fprintf('  Driver: %s | Standard: %s\n', scenario.driverType, scenario.refStandard);

    %% Plant + driver + controller 상태 초기화
    plantState = plant_init_state(plantModel, scenario.vx0, params);
    drvState   = struct('L', VEH.lf + VEH.lr, 'k_stanley', 1.5, 'v_min', 1.0);
    ctrlState_lat = struct('intError', 0, 'prevError', 0);
    ctrlState_lon = struct('intError', 0, 'prevForce', 0);
    ctrlState_ver = struct();                                  % [FIX] ctrl_vertical state

    %% 로그 사전 할당
    wheels = {'FL','FR','RL','RR'};
    log.t = t;
    fields = {'vx','vy','ax','ay','yawRate','slipAngle','roll','pitch'};
    for f = fields; log.(f{1}) = zeros(n,1); end
    log.steerDriver = zeros(n,1);
    log.steerAFS    = zeros(n,1);
    log.yawMomentESC = zeros(n,1);
    log.yawRateRef  = zeros(n,1);
    log.x_pos = zeros(n,1); log.y_pos = zeros(n,1); log.psi = 0;
    log.brakeTotal = zeros(n,1);
    log.zs = zeros(n,4);
    for w=1:4
        log.tire.(wheels{w}).Fx = zeros(n,1);
        log.tire.(wheels{w}).Fy = zeros(n,1);
        log.tire.(wheels{w}).Fz = zeros(n,1);
        log.tire.(wheels{w}).slipRatio = zeros(n,1);
        log.tire.(wheels{w}).slipAngle = zeros(n,1);
    end

    %% 시뮬레이션 루프
    pose = struct('x',0,'y',0,'psi',0);
    for k = 1:n
        tk = t(k);
        vx_now = plantState.x(1);
        if numel(plantState.x) >= 2; vy_now = plantState.x(2); else; vy_now = 0; end
        if numel(plantState.x) >= 3; yawRate_now = plantState.x(3); else; yawRate_now = 0; end
        beta_now = atan2(vy_now, max(vx_now, 0.1));

        % ----- 1. Driver (open or closed-loop) -----
        [delta_driver, drvState] = driver_dispatch(scenario, pose, vx_now, tk, drvState);

        % ----- 2. Brake command (시나리오 forced) -----
        brk_scenario = scenario.brakeCmd(tk);
        if isscalar(brk_scenario); brk_scenario = brk_scenario * ones(4,1); end

        % ----- 3. ICC controllers -----
        yawRateRef = calc_ref_yaw_rate(vx_now, delta_driver, VEH);
        if ctrlOn
            % (a) ctrl_lateral
            [latCmd, ctrlState_lat] = ctrl_lateral(yawRateRef, yawRate_now, beta_now, ...
                                                  vx_now, ctrlState_lat, CTRL, LIM, dt);

            % (b) ctrl_longitudinal — [FIX] ABS 활성화
            if k > 1
                ctrlState_lon.wheelSlip = [log.tire.FL.slipRatio(k-1);
                                           log.tire.FR.slipRatio(k-1);
                                           log.tire.RL.slipRatio(k-1);
                                           log.tire.RR.slipRatio(k-1)];
                ax_prev = log.ax(k-1);
            else
                ctrlState_lon.wheelSlip = zeros(4,1);
                ax_prev = 0;
            end
            [lonCmd, ctrlState_lon] = ctrl_longitudinal(scenario.vx0, vx_now, ax_prev, ...
                                                        ctrlState_lon, CTRL, LIM, dt);

            % (c) ctrl_vertical — [FIX2] CDC 활성화, 4 필드 (zs, zs_dot, zu, zu_dot) 모두 제공
            %   plant_14dof.m L209 주석에 따라 x(8:11), x(12:15) 는 언스프렁(wheel) 좌표.
            %   스프렁(body) 좌표는 roll/pitch 로부터 각 코너에서 small-angle 근사로 계산.
            suspState = struct();
            if isfield(plantState, 'x') && numel(plantState.x) >= 15
                % 언스프렁 (wheel) 변위/속도 — plant state 에서 직접
                suspState.zu     = plantState.x(8:11);
                suspState.zu_dot = plantState.x(12:15);

                % 스프렁 (body) 변위/속도 — roll/pitch + 각 corner 위치로부터 계산
                %   small-angle: z_sprung_i ≈ y_i · φ - x_i · θ  (+z up, +y left)
                phi       = plantState.x(4);    % roll angle
                phi_dot   = plantState.x(5);    % roll rate
                theta     = plantState.x(6);    % pitch angle
                theta_dot = plantState.x(7);    % pitch rate
                lf  = VEH.lf;  lr  = VEH.lr;
                htf = VEH.track_f / 2;  htr = VEH.track_r / 2;
                % Corner (xc, yc) from CoG: FL(+lf,+htf), FR(+lf,-htf), RL(-lr,+htr), RR(-lr,-htr)
                suspState.zs = [ htf*phi - lf*theta;       % FL
                                -htf*phi - lf*theta;       % FR
                                 htr*phi + lr*theta;       % RL
                                -htr*phi + lr*theta];      % RR
                suspState.zs_dot = [ htf*phi_dot - lf*theta_dot;
                                    -htf*phi_dot - lf*theta_dot;
                                     htr*phi_dot + lr*theta_dot;
                                    -htr*phi_dot + lr*theta_dot];
            end
            [verCmd, ctrlState_ver] = ctrl_vertical(suspState, ctrlState_ver, CTRL, dt);

            % (d) ctrl_coordinator
            actAdd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx_now, VEH, CTRL, LIM);
            delta_AFS  = actAdd.steerAngle;
            brakeESC   = actAdd.brakeTorque;                           % 차동 part
            dampCoeff  = actAdd.dampingCoeff;
        else
            delta_AFS = 0;
            brakeESC  = zeros(4,1);
            dampCoeff = 1500 * ones(4,1);
        end

        % ----- 4. Final actuator command -----
        steer_total = delta_driver + delta_AFS;
        steer_total = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steer_total));
        brake_total = brk_scenario + brakeESC;
        brake_total = max(0, min(LIM.MAX_BRAKE_TRQ, brake_total));

        actCmd.steerAngle    = 0;                                     % steer는 별도 인가
        actCmd.brakeTorque   = brake_total;
        actCmd.dampingCoeff  = dampCoeff;

        % ----- 5. Road input -----
        zr = zeros(4,1);
        for w = 1:4
            zr(w) = scenario.z_road(tk, w);
        end
        actCmd.z_road = zr;

        % ----- 6. Plant step -----
        [out, plantState] = plant_step(plantState, actCmd, steer_total, params, dt);

        % ----- 7. Log -----
        log.vx(k)=out.vx; log.vy(k)=out.vy; log.ax(k)=out.ax; log.ay(k)=out.ay;
        log.yawRate(k)=out.yawRate; log.slipAngle(k)=out.slipAngle;
        log.roll(k)=out.roll; log.pitch(k)=out.pitch;
        log.steerDriver(k) = delta_driver;
        log.steerAFS(k)    = delta_AFS;
        log.yawRateRef(k)  = yawRateRef;
        if ctrlOn; log.yawMomentESC(k) = latCmd.yawMoment; end
        log.brakeTotal(k)  = sum(brake_total);
        if isfield(plantState, 'x') && numel(plantState.x) >= 11
            log.zs(k, :) = plantState.x(8:11)';
        end
        for w=1:4
            wn = wheels{w};
            log.tire.(wn).Fx(k) = out.tire.(wn).Fx;
            log.tire.(wn).Fy(k) = out.tire.(wn).Fy;
            log.tire.(wn).Fz(k) = out.tire.(wn).Fz;
            log.tire.(wn).slipRatio(k) = out.tire.(wn).slipRatio;
            log.tire.(wn).slipAngle(k) = out.tire.(wn).slipAngle;
        end

        % ----- 8. Pose 적분 (driver model이 다음 step에 사용) -----
        if k > 1
            log.psi = log.psi + out.yawRate * dt;
            log.x_pos(k) = log.x_pos(k-1) + (out.vx*cos(log.psi) - out.vy*sin(log.psi)) * dt;
            log.y_pos(k) = log.y_pos(k-1) + (out.vx*sin(log.psi) + out.vy*cos(log.psi)) * dt;
        end
        pose.x = log.x_pos(k); pose.y = log.y_pos(k); pose.psi = log.psi;
    end

    %% Result struct
    result = log;
    result.scenario = scenario;
    result.plantModel = plantModel;
    result.controller = p.Results.Controller;

    %% KPI 계산
    kpi = local_compute_kpis(result, scenario, params);

    %% 출력 요약
    fprintf('  KPI Summary:\n');
    fn = fieldnames(kpi);
    for i = 1:numel(fn)
        v = kpi.(fn{i});
        if isnumeric(v) && isscalar(v)
            fprintf('    %s = %.4f\n', fn{i}, v);
        end
    end

    %% 저장
    if p.Results.SavePlot
        outDir = fullfile(projectRoot, 'data', 'scenarios');
        if ~exist(outDir,'dir'); mkdir(outDir); end
        saveFile = fullfile(outDir, sprintf('result_%s_%s_%s_%s.mat', ...
            scenarioId, plantModel, p.Results.Controller, datestr(now,'yyyymmdd_HHMMSS')));
        save(saveFile, 'result', 'kpi');
        fprintf('  Saved: %s\n', saveFile);
    end
end

%% ============================================================
function kpi = local_compute_kpis(result, scenario, params)
% 시나리오별 KPI 자동 분기 — scenario.kpis 목록 따라 helper 호출
    t = result.t; r = result.yawRate; ay = result.ay; ax = result.ax;
    vx = result.vx; sa = result.slipAngle;
    kpi = struct();

    kpi.sideSlipMax  = rad2deg(max(abs(sa)));
    kpi.ayMax        = max(abs(ay));
    kpi.LTR_max      = local_calcLTR_max(result, params);

    requested = scenario.kpis;

    if any(strcmp(requested,'yawRateOvershoot')) || any(strcmp(requested,'yawRateRiseTime')) || ...
       any(strcmp(requested,'yawRateSettling'))
        sr = kpi_yawrate_response(t, r, 1.0);
        kpi.yawRateOvershoot = sr.overshoot_pct;
        kpi.yawRateRiseTime  = sr.riseTime;
        kpi.yawRateSettling  = sr.settlingTime;
        kpi.r_ss_degps       = rad2deg(sr.r_ss);
    end

    if any(strcmp(requested,'sineDwellR1p0')) || any(strcmp(requested,'sineDwellR1p75')) || ...
       any(strcmp(requested,'lateralDispBOS1p07'))
        BOS = 1.0;
        if isfield(scenario, 'BOS'); BOS = scenario.BOS; end
        sd = kpi_sine_with_dwell(t, r, ay, BOS);
        kpi.sineDwellR1p0      = sd.R1p0;
        kpi.sineDwellR1p75     = sd.R1p75;
        kpi.lateralDispBOS1p07 = sd.lateralDispBOS1p07;
        kpi.sineDwellPass      = sd.pass_all;
    end

    if any(strcmp(requested,'stoppingDistance')) || any(strcmp(requested,'MFDD')) || ...
       any(strcmp(requested,'muUtilization')) || any(strcmp(requested,'jerkMax'))
        b = kpi_braking(t, vx, ax, 1.0);
        kpi.stoppingDistance = b.stoppingDistance;
        kpi.MFDD             = b.MFDD;
        kpi.muUtilization    = b.muUtilization;
        kpi.jerkMax          = b.jerkMax;
        kpi.timeToStop       = b.timeToStop;
    end

    if any(strcmp(requested,'absSlipRMS'))
        slipMtx = [result.tire.FL.slipRatio, result.tire.FR.slipRatio, ...
                   result.tire.RL.slipRatio, result.tire.RR.slipRatio];
        brakeActive = result.brakeTotal > 50;
        a = kpi_abs_slip(t, slipMtx, brakeActive, -0.12);
        kpi.absSlipRMS = a.maxSlipRMS;
    end

    if any(strcmp(requested,'tireUtilizationMax'))
        tu = kpi_tire_utilization(result.tire, 1.0);
        kpi.tireUtilizationMax = tu.uMaxOverall;
    end

    if any(strcmp(requested,'lateralDevMax')) && isfield(scenario, 'refPath')
        pd = kpi_lateral_path_deviation(t, result.x_pos, result.y_pos, scenario.refPath);
        kpi.lateralDevMax = pd.devMax;
        kpi.lateralDevRMS = pd.devRMS;
    end

    if any(strcmp(requested,'yawRateSteadyState'))
        idxSS = t > t(end) - 1.0;
        kpi.yawRateSteadyState = rad2deg(mean(r(idxSS)));
        kpi.aySteadyState      = mean(ay(idxSS));
    end

    if any(strcmp(requested,'understeerGradient'))
        L = params.VEH.lf + params.VEH.lr;
        idxSS = t > t(end) - 1.0;
        ay_ss = mean(ay(idxSS));
        delta_ss = mean(result.steerDriver(idxSS) + result.steerAFS(idxSS));
        if abs(ay_ss) > 0.1
            R = mean(vx(idxSS))^2 / ay_ss;
            kpi.understeerGradient = (delta_ss - L/R) / (ay_ss / 9.81);
        else
            kpi.understeerGradient = NaN;
        end
    end

    if any(strcmp(requested,'yawDevDuringBrake')) || any(strcmp(requested,'yawDevDuringABS'))
        brakeActive = result.brakeTotal > 50;
        if any(brakeActive)
            yaw_at_brake_start = result.psi;
            kpi.yawDevDuringBrake = rad2deg(yaw_at_brake_start);
        else
            kpi.yawDevDuringBrake = NaN;
        end
    end
end

function LTR = local_calcLTR_max(result, ~)
    if isfield(result.tire, 'FL') && any(result.tire.FL.Fz ~= 0)
        FzL = result.tire.FL.Fz + result.tire.RL.Fz;
        FzR = result.tire.FR.Fz + result.tire.RR.Fz;
        Fz_tot = FzL + FzR; Fz_tot(Fz_tot < 1) = 1;
        LTR = max(abs((FzR - FzL) ./ Fz_tot));
    else
        LTR = NaN;
    end
end
