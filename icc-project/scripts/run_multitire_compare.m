%RUN_MULTITIRE_COMPARE M-file tire models × VDB tire variants 정합성 검증
%
%   A1 DLC 시나리오 + BMW_5 동일 파라미터에서 다음 6개 조합 실행:
%     M-file plant_14dof + tire_force {simple_mf, full_mf, rt_proxy}
%     VDB PassVeh14DOF + variant {mf_vec, fiala_vec, dugoff_vec}
%
%   3-way KPI 표로 어느 조합이 가장 정합되는지 확인.

clear; clc;

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));

% 시나리오 (고정)
scenario = scenario_dispatcher('A1', SIM);
dt_log = 0.01;
% 절대경로 보장
cd(projectRoot);

% BMW_5 override (VDB용)
veh_override = struct();
ovr_fields = {'mass','ms','L','lf','lr','h_cog','Iz','Iy','Ix','Cd','Af','track_f','mu_w'};
for k = 1:numel(ovr_fields)
    if isfield(VEH, ovr_fields{k})
        veh_override.(ovr_fields{k}) = VEH.(ovr_fields{k});
    end
end

%% --- M-file 3 tire models (직접 plant loop — run_icc_scenario는 init 재실행하여 TIRE override 무시) ---
mfileTires = {'simple_mf','full_mf','rt_proxy'};
mfile_runs = struct();
dt = SIM.dt;
t_arr = (0:dt:scenario.tEnd)';
n = numel(t_arr);
wheels = {'FL','FR','RL','RR'};

for ti = 1:numel(mfileTires)
    tname = mfileTires{ti};
    fprintf('\n===== M-file [%s] =====\n', tname);

    % Build TIRE struct for this variant
    TIRE_use = TIRE;
    TIRE_use.model = tname;
    if strcmp(tname,'full_mf') && isfile(SIM.tire_tir_file)
        TIRE_use = tire_full_mf_parse_tir(SIM.tire_tir_file, TIRE_use);
    end
    params = struct('VEH', VEH, 'TIRE', TIRE_use, 'CONST', CONST, 'SIM', SIM);

    % Manually run plant_14dof
    plantState = plant_init_state('14dof', scenario.vx0, params);
    log = struct();
    log.t = t_arr;
    flds = {'vx','vy','ax','ay','yawRate','slipAngle','roll','pitch'};
    for f = flds; log.(f{1}) = zeros(n,1); end

    for k = 1:n
        tk = t_arr(k);
        steer = scenario.steerDriver(tk);
        brk = scenario.brakeCmd(tk);
        if isscalar(brk); brk = brk * ones(4,1); end
        actCmd = struct('steerAngle', 0, 'brakeTorque', brk, ...
                        'dampingCoeff', 1500*ones(4,1));
        zr = zeros(4,1);
        if isfield(scenario,'z_road') && ~isempty(scenario.z_road)
            for w=1:4; zr(w) = scenario.z_road(tk, w); end
        end
        actCmd.z_road = zr;
        [out, plantState] = plant_step(plantState, actCmd, steer, params, dt);
        for f = flds; log.(f{1})(k) = out.(f{1}); end
    end
    mfile_runs.(tname) = log;
    fprintf('  yawR_pk=%.2f deg/s, ay_pk=%.2f, slip_pk=%.2f deg\n', ...
        rad2deg(max(abs(log.yawRate))), max(abs(log.ay)), rad2deg(max(abs(log.slipAngle))));
end

%% --- VDB 3 tire variants ---
vdbVariants = {'mf_vec','fiala_vec','dugoff_vec'};
vdb_runs = struct();
for vi = 1:numel(vdbVariants)
    vname = vdbVariants{vi};
    fprintf('\n===== VDB [%s] =====\n', vname);
    cd(projectRoot);   % ensure path consistency
    opts = struct('dt_log', dt_log, 'overrideVEH', veh_override, ...
                  'settleTime', 0.5, 'verbose', true, 'tireVariant', vname);
    try
        vdbRes = vdb_run_sim(scenario, opts);
        vdb_runs.(vname) = vdbRes;
    catch ME
        fprintf('  FAIL: %s\n', ME.message);
        vdb_runs.(vname) = [];
    end
end

%% CarMaker ref (lateral 시나리오)
cmRes = [];
try
    raw = cm_read_erg('C:/Users/VIC/Projects/carmaker_data/SimOutput/bsong/20260506/RT_CGSL2R_IN_CM15_104327.erg');
    cmRes.t = raw.Time - raw.Time(1);
    cmRes.vx = raw.Car_v;
    cmRes.ay = raw.Car_ay;
    cmRes.yawRate = raw.Car_YawRate;
    cmRes.slipAngle = raw.Car_SideSlipAngle;
catch
end

%% KPI 표 (6 model × 4 metric)
fprintf('\n\n=========== Multi-Tire KPI Matrix (A1 DLC) ===========\n');
fprintf('%-30s | yawR_pk[deg/s] | ay_pk[m/s²] | slip_pk[deg] | vx_end[m/s]\n', 'Model');
fprintf('%s\n', repmat('-', 1, 95));
allNames = [strcat('M-file: ', mfileTires), strcat('VDB: ', vdbVariants)];
allRes = struct('vx',[],'ay',[],'yawRate',[],'slipAngle',[]);
allRes(1) = []; %#ok<NASGU>
allResults = {};
for ti = 1:numel(mfileTires)
    r = mfile_runs.(mfileTires{ti});
    fprintf('%-30s | %14.2f | %11.2f | %12.2f | %10.2f\n', ...
        ['M-file: ' mfileTires{ti}], ...
        rad2deg(max(abs(r.yawRate))), max(abs(r.ay)), ...
        rad2deg(max(abs(r.slipAngle))), r.vx(end));
    allResults{end+1} = struct('name', ['M-file ' mfileTires{ti}], 'data', r);
end
for vi = 1:numel(vdbVariants)
    r = vdb_runs.(vdbVariants{vi});
    if isempty(r)
        fprintf('%-30s | %14s | %11s | %12s | %10s\n', ...
            ['VDB: ' vdbVariants{vi}], 'FAIL', '-', '-', '-');
        continue;
    end
    fprintf('%-30s | %14.2f | %11.2f | %12.2f | %10.2f\n', ...
        ['VDB: ' vdbVariants{vi}], ...
        rad2deg(max(abs(r.yawRate))), max(abs(r.ay)), ...
        rad2deg(max(abs(r.slipAngle))), r.vx(end));
    allResults{end+1} = struct('name', ['VDB ' vdbVariants{vi}], 'data', r);
end

if ~isempty(cmRes)
    fprintf('%-30s | %14.2f | %11.2f | %12.2f | %10.2f\n', ...
        'CarMaker (RT_CGSL2R ref)', ...
        rad2deg(max(abs(cmRes.yawRate))), max(abs(cmRes.ay)), ...
        rad2deg(max(abs(cmRes.slipAngle))), cmRes.vx(end));
end

%% 6-way overlay plot
figDir = fullfile(projectRoot, 'docs', 'figures', '3way');
if ~exist(figDir,'dir'); mkdir(figDir); end
figPath = fullfile(figDir, sprintf('multitire_compare_A1_%s.png', datestr(now,'yyyymmdd_HHMMSS')));

fig = figure('Position',[60 60 1300 850],'Visible','off','Color','w');
colors = lines(numel(allResults));
chans = {'vx','ay','yawRate','slipAngle'};
labels = {'v_x [m/s]','a_y [m/s²]','Yaw rate [deg/s]','\beta [deg]'};
for ci = 1:4
    subplot(2,2,ci); hold on; grid on;
    for k = 1:numel(allResults)
        r = allResults{k}.data;
        if strcmp(chans{ci},'yawRate') || strcmp(chans{ci},'slipAngle')
            y = rad2deg(r.(chans{ci}));
        else
            y = r.(chans{ci});
        end
        plot(r.t, y, 'LineWidth', 1.4, 'Color', colors(k,:), ...
            'DisplayName', allResults{k}.name);
    end
    if ~isempty(cmRes)
        if strcmp(chans{ci},'yawRate') || strcmp(chans{ci},'slipAngle')
            cmY = rad2deg(cmRes.(chans{ci}));
        else
            cmY = cmRes.(chans{ci});
        end
        plot(cmRes.t, cmY, 'k--', 'LineWidth', 1.6, 'DisplayName','CarMaker ref');
    end
    xlabel('Time [s]'); ylabel(labels{ci}); title(labels{ci});
    if ci==1; legend('Location','best','FontSize',8); end
end
sgtitle('Multi-tire comparison — A1 DLC, BMW_5 params','FontWeight','bold','Interpreter','none');
exportgraphics(fig, figPath, 'Resolution', 150);
close(fig);
fprintf('\nSaved: %s\n', figPath);

%% Save results
matPath = fullfile(projectRoot, 'data', sprintf('multitire_compare_%s.mat', datestr(now,'yyyymmdd_HHMMSS')));
save(matPath, 'allResults', 'cmRes', 'mfile_runs', 'vdb_runs');
fprintf('Saved: %s\n', matPath);
