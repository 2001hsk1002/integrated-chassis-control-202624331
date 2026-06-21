function result = vdb_run_sim(scenario, opts)
%VDB_RUN_SIM VDB PassVeh14DOF.slx wrapper — 시나리오 입력으로 sim 실행 + 결과 추출
%
%   result = VDB_RUN_SIM(scenario, opts)
%
%   Inputs:
%       scenario - struct from scenario_dispatcher.m:
%           .id, .name, .tEnd, .vx0, .steerDriver(t), .brakeCmd(t), .z_road(t,w)
%       opts - struct (옵션):
%           .dt_log    (default 0.01)  로그 sampling interval
%           .axlTrq    (default 0)     base axle torque (cruise 보상 안 함)
%           .modelPath (default 'models/simulink/vdb_ref') VDB 모델 디렉토리
%           .verbose   (default true)
%
%   Outputs:
%       result - struct with logged signals (t, vx, vy, ax, ay, yawRate, slipAngle, ...)
%
%   주의:
%     - VDB는 brake input이 BrkPrs (Pa), 우리는 Nm. 간이 변환 (G=15.94 Nm/bar 전제)
%     - VDB AxlTrq를 0으로 두면 자유굴림 → 공기항력으로 감속. cruise 유지 시 별도 PID 필요
%     - 초기 vx는 VEH.InitialLongVel을 scenario.vx0로 override

    if nargin < 2; opts = struct(); end
    if ~isfield(opts,'dt_log');     opts.dt_log = 0.01; end
    if ~isfield(opts,'axlTrq');     opts.axlTrq = 0; end
    if ~isfield(opts,'modelPath');  opts.modelPath = fullfile(pwd,'models','simulink','vdb_ref'); end
    if ~isfield(opts,'verbose');    opts.verbose = true; end
    if ~isfield(opts,'overrideVEH');opts.overrideVEH = []; end       % 사용자 VEH 주입
    if ~isfield(opts,'settleTime'); opts.settleTime = 0.5; end       % 휠 ω 정착용 prepend (s)
    if ~isfield(opts,'tireVariant');opts.tireVariant = 'mf_vec'; end % 'mf'|'mf_vec'|'fiala'|'fiala_vec'|'dugoff_vec'

    addpath(opts.modelPath);
    origDir = pwd;
    cleanup = onCleanup(@() cd(origDir));
    cd(opts.modelPath);

    mdl = 'PassVeh14DOF';
    if ~bdIsLoaded(mdl)
        load_system(mdl);
    end

    %% Tire variant 선택 (5가지 model 중 1개)
    tireVar = [mdl '/Wheels and Tires/VDBS/Tires'];
    variantMap = containers.Map(...
        {'mf','fiala','mf_vec','fiala_vec','dugoff_vec'}, ...
        {'0','1','2','3','4'});
    if isKey(variantMap, opts.tireVariant)
        set_param(tireVar, 'OverrideUsingVariant', variantMap(opts.tireVariant));
        if opts.verbose
            fprintf('[vdb_run_sim] Tire variant: %s (LabelMode=%s)\n', ...
                opts.tireVariant, variantMap(opts.tireVariant));
        end
    else
        warning('[vdb_run_sim] Unknown tireVariant: %s. Using default.', opts.tireVariant);
    end

    %% Model workspace VEH 설정 (BMW_5 override 옵션 + 초기 vx)
    ws = get_param(mdl, 'ModelWorkspace');
    VEH = ws.evalin('VEH');

    % BMW_5 override (사용자가 opts.overrideVEH 제공한 경우)
    if ~isempty(opts.overrideVEH) && isstruct(opts.overrideVEH)
        ovr = opts.overrideVEH;
        % VDB 필드명 ↔ 우리 VEH 필드명 매핑
        if isfield(ovr,'mass');         VEH.Mass                    = ovr.mass; end
        if isfield(ovr,'ms');           VEH.SprungMass              = ovr.ms; end
        if isfield(ovr,'L');            VEH.WheelBase               = ovr.L; end
        if isfield(ovr,'lf');           VEH.FrontAxlePositionfromCG = ovr.lf; end
        if isfield(ovr,'lr');           VEH.RearAxlePositionfromCG  = ovr.lr; end
        if isfield(ovr,'h_cog');        VEH.HeightCG                = ovr.h_cog; end
        if isfield(ovr,'Iz');           VEH.YawMomentInertia        = ovr.Iz; end
        if isfield(ovr,'Iy');           VEH.PitchMomentInertia      = ovr.Iy; end
        if isfield(ovr,'Ix');           VEH.RollMomentInertia       = ovr.Ix; end
        if isfield(ovr,'Cd');           VEH.DragCoefficient         = ovr.Cd; end
        if isfield(ovr,'Af');           VEH.FrontalArea             = ovr.Af; end
        if isfield(ovr,'track_f');      VEH.TrackWidth              = ovr.track_f; end
        if isfield(ovr,'mu_w');
            VEH.UnsprungMassFrontAxle = 2 * ovr.mu_w;
            VEH.UnsprungMassRearAxle  = 2 * ovr.mu_w;
        end
        if opts.verbose
            fprintf('[vdb_run_sim] VEH override applied: mass=%.0f, L=%.3f, Iz=%.1f, h_cog=%.3f\n', ...
                VEH.Mass, VEH.WheelBase, VEH.YawMomentInertia, VEH.HeightCG);
        end
    end

    VEH.InitialLongVel = max(scenario.vx0, 0.5);
    ws.assignin('VEH', VEH);

    %% 입력 시계열 생성 (settling phase 앞에 prepend)
    settleT = max(opts.settleTime, 0);
    t = (0:opts.dt_log:(scenario.tEnd + settleT))';
    n = numel(t);
    t_sim_start = settleT;  % scenario 시작 시점 (sim 절대시간 기준)

    % 운전자 조향각 → per-wheel WhlAng (front만). settling 동안 0.
    deltaT = zeros(n, 1);
    for k = 1:n
        if t(k) >= settleT
            deltaT(k) = scenario.steerDriver(t(k) - settleT);
        end
    end
    WhlAng = [deltaT, deltaT, zeros(n,1), zeros(n,1)];

    % 운전자 brake torque (4×1 per-wheel) → BrkPrs (Pa). settling 동안 0.
    brakeT = zeros(n, 4);
    for k = 1:n
        if t(k) >= settleT
            bk = scenario.brakeCmd(t(k) - settleT);
            if isscalar(bk); bk = bk * ones(4,1); end
            brakeT(k, :) = bk(:)';
        end
    end
    % conversion: T [Nm] = G [Nm/bar] * p [bar] = G/1e5 [Nm/Pa] * p [Pa]
    % BMW_5: G_F=15.94 Nm/bar, G_R=8.02 Nm/bar
    G_F = 15.94; G_R = 8.02;
    BrkPrs = zeros(n, 4);
    BrkPrs(:,1:2) = brakeT(:,1:2) / G_F * 1e5;   % front (Pa)
    BrkPrs(:,3:4) = brakeT(:,3:4) / G_R * 1e5;   % rear
    BrkPrs = max(BrkPrs, 0);

    % Axle drive torque
    AxlTrq = opts.axlTrq * ones(n, 4);

    % 노면 (평지 default + scenario.z_road). settling 동안 0.
    Ground = zeros(n, 4);
    if isfield(scenario,'z_road') && ~isempty(scenario.z_road)
        for k = 1:n
            if t(k) >= settleT
                for w = 1:4
                    Ground(k, w) = scenario.z_road(t(k) - settleT, w);
                end
            end
        end
    end

    % per-wheel friction (settling 동안 = scenario 시작값)
    if isfield(scenario,'mu_wheel') && ~isempty(scenario.mu_wheel)
        Friction = zeros(n, 4);
        for k = 1:n
            tau = max(t(k) - settleT, 0);
            for w = 1:4
                Friction(k, w) = scenario.mu_wheel(tau, w);
            end
        end
    else
        Friction = ones(n, 4);
    end

    WindXYZ = zeros(n, 3);
    DCM_data = zeros(3, 3, 4, n);
    for k = 1:n; for w = 1:4; DCM_data(:,:,w,k) = eye(3); end; end

    %% Dataset 구성
    ds = Simulink.SimulationData.Dataset();
    ds = ds.addElement(timeseries(WhlAng,   t, 'Name', 'WhlAng'));
    ds = ds.addElement(timeseries(AxlTrq,   t, 'Name', 'AxlTrq'));
    ds = ds.addElement(timeseries(BrkPrs,   t, 'Name', 'BrkPrs'));
    ds = ds.addElement(timeseries(WindXYZ,  t, 'Name', 'WindXYZ'));
    ds = ds.addElement(timeseries(Ground,   t, 'Name', 'Ground'));
    ds = ds.addElement(timeseries(Friction, t, 'Name', 'Friction'));
    ds = ds.addElement(timeseries(DCM_data, t, 'Name', 'DCM'));

    %% Sim
    sim_tEnd = scenario.tEnd + settleT;
    si = Simulink.SimulationInput(mdl);
    si = si.setExternalInput(ds);
    si = si.setModelParameter('StopTime', num2str(sim_tEnd));
    si = si.setModelParameter('SaveOutput', 'on', 'SaveFormat', 'Dataset', ...
                              'SaveTime', 'on', 'TimeSaveName', 'tout', ...
                              'OutputSaveName', 'yout');

    if opts.verbose
        fprintf('[vdb_run_sim] %s — running sim (%.1fs incl %.2fs settling)...\n', ...
            scenario.id, sim_tEnd, settleT);
    end
    tStart = tic;
    out = sim(si);
    if opts.verbose
        fprintf('[vdb_run_sim] %s done (%.1fs wall)\n', scenario.id, toc(tStart));
    end

    %% 결과 추출 (settling phase 제외, scenario 시계로 변환)
    result = local_extract_outputs(out, t);
    result.scenario = scenario;
    result.inputs.t        = t - settleT;   % scenario 0 기준
    result.inputs.WhlAng   = WhlAng;
    result.inputs.AxlTrq   = AxlTrq;
    result.inputs.BrkPrs   = BrkPrs;

    % settling phase 잘라내고 scenario 시계로 재정렬
    result = local_strip_settling(result, settleT);
end

function r = local_strip_settling(r, settleT)
    if settleT <= 0; return; end
    fields = {'vx','vy','ax','ay','yawRate','rollRate','pitchRate','slipAngle', ...
              'roll','pitch','yaw','x_pos','y_pos'};
    keep = r.t >= settleT;
    r.t = r.t(keep) - settleT;
    for k = 1:numel(fields)
        if isfield(r, fields{k}) && numel(r.(fields{k})) == numel(keep)
            r.(fields{k}) = r.(fields{k})(keep);
        end
    end
end

%% ============================================================
function r = local_extract_outputs(out, t_in)
% Bus signal에서 핵심 채널 추출. yout.Veh, yout.Wheels의 nested struct를 탐색.
    r.t = t_in;
    r.vx = NaN(numel(t_in),1); r.vy = r.vx; r.ax = r.vx; r.ay = r.vx;
    r.yawRate = r.vx; r.slipAngle = r.vx; r.roll = r.vx; r.pitch = r.vx;
    r.raw = struct();   % full structure for inspection

    if ~isa(out,'Simulink.SimulationOutput')
        return;
    end

    try
        yout = out.get('yout');
        if isa(yout, 'Simulink.SimulationData.Dataset')
            for k = 1:yout.numElements
                el = yout.getElement(k);
                nm = el.Name;
                if isempty(nm); nm = sprintf('Element%d', k); end
                r.raw.(nm) = el.Values;
            end
        end

        % Extract common signals from Veh substructure
        if isfield(r.raw, 'Veh')
            r = local_assign_from_veh(r, r.raw.Veh);
        end
    catch ME
        fprintf('[vdb_run_sim] output extract warn: %s\n', ME.message);
    end
end

function r = local_assign_from_veh(r, vehStruct)
% Walk Veh struct recursively (unlimited depth) → flat path → timeseries map
    flat = struct();
    flat = local_walk(vehStruct, '', flat);
    r.raw.veh_signals = flat;

    % VDB PassVeh14DOF.slx output bus paths
    % Acc_xddot/yddot/zddot 는 kinematic accel; Acc_ax/ay/az 는 specific force (g 영향)
    % 우리 M-file의 ax/ay와 매칭하려면 xddot/yddot 사용
    candidate_map = {
        'BdyFrm_Cg_Vel_xdot',    'vx';
        'BdyFrm_Cg_Vel_ydot',    'vy';
        'BdyFrm_Cg_Acc_xddot',   'ax';
        'BdyFrm_Cg_Acc_yddot',   'ay';
        'BdyFrm_Cg_AngVel_r',    'yawRate';
        'BdyFrm_Cg_AngVel_p',    'rollRate';
        'BdyFrm_Cg_AngVel_q',    'pitchRate';
        'BdyFrm_Cg_Ang_Beta',    'slipAngle';
        'InertFrm_Cg_Ang_phi',   'roll';
        'InertFrm_Cg_Ang_theta', 'pitch';
        'InertFrm_Cg_Ang_psi',   'yaw';
        'InertFrm_Geom_Disp_X',  'x_pos';
        'InertFrm_Geom_Disp_Y',  'y_pos';
    };
    sigs = fieldnames(flat);
    for m = 1:size(candidate_map,1)
        for s = 1:numel(sigs)
            if endsWith(sigs{s}, candidate_map{m,1})
                v = flat.(sigs{s});
                if isa(v,'timeseries')
                    d = v.Data;
                    if isvector(d); d = d(:); end
                    tSig = v.Time(:);
                    % 시간 보간 (variable-step solver 대응)
                    try
                        r.(candidate_map{m,2}) = interp1(tSig, d(:,1), r.t, 'linear', 'extrap');
                    catch
                        if numel(d) == numel(r.t)
                            r.(candidate_map{m,2}) = d(:,1);
                        end
                    end
                    break;
                end
            end
        end
    end
end

function flat = local_walk(node, prefix, flat)
    fn = fieldnames(node);
    for k = 1:numel(fn)
        nm = fn{k};
        full = [prefix nm];
        v = node.(nm);
        if isa(v, 'timeseries')
            flat.(full) = v;
        elseif isstruct(v)
            flat = local_walk(v, [full '_'], flat);
        end
    end
end

% local_map_veh_name removed — replaced by suffix matching in local_assign_from_veh
