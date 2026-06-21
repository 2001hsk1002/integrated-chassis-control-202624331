%RUN_3WAY_COMPARE M-file 14-DOF ↔ VDB 14-DOF ↔ CarMaker .erg 3-way 비교
%
%   대상 시나리오: A1 DLC, A4 SS Circular, B1 Brake
%   VDB는 BMW_5 파라미터로 override + 0.5s settling phase (휠 ω 동기)
%   결과: docs/figures/vdb_3way_*.png, data/3way_*.mat

clear; clc;

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));

dt_log = 0.01;
ids = {'A1', 'A4', 'B1'};
results = struct();

% BMW_5 override 구조체 (VDB의 VEH mask에 주입)
veh_override = struct();
ovr_fields = {'mass','ms','L','lf','lr','h_cog','Iz','Iy','Ix','Cd','Af','track_f','mu_w'};
for k = 1:numel(ovr_fields)
    if isfield(VEH, ovr_fields{k})
        veh_override.(ovr_fields{k}) = VEH.(ovr_fields{k});
    end
end
vdb_opts = struct('dt_log', dt_log, 'overrideVEH', veh_override, ...
                  'settleTime', 0.5, 'verbose', false);

for s = 1:numel(ids)
    sid = ids{s};
    fprintf('\n=========== Scenario %s ===========\n', sid);
    scenario = scenario_dispatcher(sid, SIM);

    % 1) M-file 14-DOF
    fprintf('[run_3way] M-file 14-DOF...\n');
    [mfRes, mfKpi] = run_icc_scenario(sid, '14dof', false, false);

    % 2) VDB 14-DOF (Simulink batch sim with BMW_5 override + settling)
    fprintf('[run_3way] VDB 14-DOF (BMW_5 override)...\n');
    vdbRes = vdb_run_sim(scenario, vdb_opts);

    % 3) CarMaker — 시나리오에 매핑할 .erg가 있는 경우만
    cmRes = local_load_cm_reference(sid);

    % 비교 plot
    figDir = fullfile(projectRoot, 'docs', 'figures', '3way');
    figPath = fullfile(figDir, sprintf('vdb_3way_%s_%s.png', sid, datestr(now,'yyyymmdd_HHMMSS')));
    util_plot_3way_compare(mfRes.t, mfRes, vdbRes.t, vdbRes, cmRes, scenario.name, figPath);
    fprintf('[run_3way] Saved: %s\n', figPath);

    % KPI 비교
    kpi_compare = local_kpi_table(mfRes, vdbRes, cmRes);
    fprintf('\n%s KPI:\n', sid);
    disp(kpi_compare);

    % 결과 저장
    matPath = fullfile(projectRoot, 'data', sprintf('3way_%s_%s.mat', sid, datestr(now,'yyyymmdd_HHMMSS')));
    if ~exist(fileparts(matPath),'dir'); mkdir(fileparts(matPath)); end
    save(matPath, 'mfRes', 'vdbRes', 'cmRes', 'kpi_compare');

    results.(sid).mfile = mfRes;
    results.(sid).vdb   = vdbRes;
    results.(sid).cm    = cmRes;
    results.(sid).kpi   = kpi_compare;
end

fprintf('\n=== 3-way comparison complete ===\n');

%% ============================================================
function cm = local_load_cm_reference(sid)
    cm = [];
    cmFiles.A1 = 'C:/Users/VIC/Projects/carmaker_data/SimOutput/bsong/20260506/RT_CGSL2R_IN_CM15_104327.erg';   % 곡선 — 부분 매핑
    cmFiles.A4 = 'C:/Users/VIC/Projects/carmaker_data/SimOutput/bsong/20260506/RT_CGSL2R_IN_CM15_104327.erg';   % 곡선
    cmFiles.B1 = 'C:/Users/VIC/Projects/carmaker_data/SimOutput/bsong/20260428/LK_CCIR_ST_CM15_124910.erg';     % AEB brake
    if ~isfield(cmFiles, sid); return; end
    f = cmFiles.(sid);
    if ~isfile(f); fprintf('  CM ref not found: %s\n', f); return; end
    try
        raw = cm_read_erg(f);
        cm.t = raw.Time - raw.Time(1);
        cm.vx = raw.Car_v;
        cm.ay = raw.Car_ay;
        cm.yawRate = raw.Car_YawRate;
        cm.slipAngle = raw.Car_SideSlipAngle;
    catch ME
        fprintf('  CM load fail: %s\n', ME.message);
    end
end

function T = local_kpi_table(mf, vdb, cm)
    rmsErr = @(a,b) sqrt(mean((a(:) - b(:)).^2, 'omitnan'));
    % VDB resample to mf time grid
    if isfield(vdb,'t') && ~isempty(vdb.t)
        vdb_vx_at_mf = interp1(vdb.t, vdb.vx, mf.t, 'linear','extrap');
        vdb_yr_at_mf = interp1(vdb.t, vdb.yawRate, mf.t, 'linear','extrap');
        vdb_ay_at_mf = interp1(vdb.t, vdb.ay, mf.t, 'linear','extrap');
        vdb_sa_at_mf = interp1(vdb.t, vdb.slipAngle, mf.t, 'linear','extrap');
    else
        vdb_vx_at_mf = NaN; vdb_yr_at_mf = NaN; vdb_ay_at_mf = NaN; vdb_sa_at_mf = NaN;
    end
    metrics = {'vx_peak', 'ay_peak', 'yawRate_peak_deg', 'slipAngle_peak_deg'};
    mfile_val = [max(abs(mf.vx)), max(abs(mf.ay)), rad2deg(max(abs(mf.yawRate))), rad2deg(max(abs(mf.slipAngle)))];
    vdb_val   = [max(abs(vdb.vx)), max(abs(vdb.ay)), rad2deg(max(abs(vdb.yawRate))), rad2deg(max(abs(vdb.slipAngle)))];
    mf_vdb_rms = [rmsErr(mf.vx, vdb_vx_at_mf), rmsErr(mf.ay, vdb_ay_at_mf), ...
                  rad2deg(rmsErr(mf.yawRate, vdb_yr_at_mf)), rad2deg(rmsErr(mf.slipAngle, vdb_sa_at_mf))];

    cm_val = NaN(1,4);
    if ~isempty(cm) && isfield(cm,'vx')
        cm_val(1) = max(abs(cm.vx));
        cm_val(2) = max(abs(cm.ay));
        cm_val(3) = rad2deg(max(abs(cm.yawRate)));
        cm_val(4) = rad2deg(max(abs(cm.slipAngle)));
    end

    T = table(metrics', mfile_val(:), vdb_val(:), cm_val(:), mf_vdb_rms(:), ...
              'VariableNames', {'Metric','M_file','VDB','CarMaker','MF_VDB_RMS'});
end
