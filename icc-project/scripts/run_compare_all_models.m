%RUN_COMPARE_ALL_MODELS 4개 plant 모델 (bicycle, 3-DOF, 7-DOF, 14-DOF) 결과 통합 비교
%
%   동일 CarMaker .erg 시나리오로 4개 모델을 모두 실행 → vx/ax/pitch/yr/ay/slip 6 채널 overlay 그림
%   결과 PNG는 docs/figures/에 저장.

thisFile = mfilename('fullpath');
projectRoot = fileparts(fileparts(thisFile));
run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));

ergLong = 'C:/Users/VIC/Projects/carmaker_data/SimOutput/bsong/20260428/LK_CCIR_ST_CM15_124910.erg';
ergLat  = 'C:/Users/VIC/Projects/carmaker_data/SimOutput/bsong/20260506/RT_CGSL2R_IN_CM15_104327.erg';

models = {'bicycle','3dof','7dof','14dof'};
modelLabels = {'bicycle','3-DOF','7-DOF','14-DOF'};
longResults = struct('model',{},'t',{},'vx_cm',{},'vx_sim',{},'ax_cm',{},'ax_sim',{},'pitch_cm',{},'pitch_sim',{});
latResults  = struct('model',{},'t',{},'yr_cm',{},'yr_sim',{},'ay_cm',{},'ay_sim',{},'slip_cm',{},'slip_sim',{});

dt = SIM.dt;
params = struct('VEH',VEH,'TIRE',TIRE,'CONST',CONST,'SIM',SIM);

%% Longitudinal phase (3/7/14 only — bicycle vx 고정)
raw = cm_read_erg(ergLong);
pMC = raw.Brake_Hyd_Sys_pMC;
i0 = find(pMC > 1, 1) - 5;
t_cm = raw.Time(i0:end) - raw.Time(i0);
vx_cm = raw.Car_v(i0:end); ax_cm = raw.Car_ax(i0:end); pitch_cm = raw.Car_Pitch(i0:end);
pWB_raw = [raw.Brake_Hyd_Sys_pWB_FL(i0:end), raw.Brake_Hyd_Sys_pWB_FR(i0:end), raw.Brake_Hyd_Sys_pWB_RL(i0:end), raw.Brake_Hyd_Sys_pWB_RR(i0:end)];
pMC_seg = pMC(i0:end);
alpha = 0.8;
ratF = median(pWB_raw(pMC_seg>40,1)./max(pMC_seg(pMC_seg>40),1));
ratR = median(pWB_raw(pMC_seg>40,3)./max(pMC_seg(pMC_seg>40),1));
pWB = pWB_raw;
pWB(:,1) = (1-alpha)*pWB_raw(:,1) + alpha*pMC_seg*ratF;
pWB(:,2) = (1-alpha)*pWB_raw(:,2) + alpha*pMC_seg*ratF;
pWB(:,3) = (1-alpha)*pWB_raw(:,3) + alpha*pMC_seg*ratR;
pWB(:,4) = (1-alpha)*pWB_raw(:,4) + alpha*pMC_seg*ratR;
bt_seq = [pWB(:,1)*15.94, pWB(:,2)*15.94, pWB(:,3)*8.02, pWB(:,4)*8.02];

for m = 2:4   % skip bicycle for longitudinal
    SIM.plantModel = models{m};
    ps = plant_init_state(SIM.plantModel, max(vx_cm(1),0.5), params);
    N = round(t_cm(end)/dt) + 1;
    tSim = (0:N-1)'*dt;
    log_vx = zeros(N,1); log_ax = zeros(N,1); log_pitch = zeros(N,1);
    for k = 1:N
        bt = interp1(t_cm, bt_seq, tSim(k), 'linear','extrap')';
        cmd.steerAngle = 0; cmd.brakeTorque = bt; cmd.dampingCoeff = 1500*ones(4,1);
        [out, ps] = plant_step(ps, cmd, 0, params, dt);
        log_vx(k) = out.vx; log_ax(k) = out.ax; log_pitch(k) = out.pitch;
    end
    R = struct();
    R.model = modelLabels{m}; R.t = t_cm;
    R.vx_cm = vx_cm; R.vx_sim = interp1(tSim, log_vx, t_cm, 'linear','extrap');
    R.ax_cm = ax_cm; R.ax_sim = interp1(tSim, log_ax, t_cm, 'linear','extrap');
    R.pitch_cm = pitch_cm; R.pitch_sim = interp1(tSim, log_pitch, t_cm, 'linear','extrap');
    longResults(end+1) = R; %#ok<SAGROW>
end

%% Lateral phase (all 4 models)
raw = cm_read_erg(ergLat);
yrC = raw.Car_YawRate;
iAct = find(abs(yrC) > deg2rad(1), 1, 'first');
iStart = max(1, iAct-50);
iLast = find(abs(yrC) > deg2rad(1), 1, 'last');
iEnd = min(numel(raw.Time), iLast+100);
t_cm = raw.Time(iStart:iEnd) - raw.Time(iStart);
vx_cm = raw.Car_v(iStart:iEnd);
yr_cm = raw.Car_YawRate(iStart:iEnd);
ay_cm = raw.Car_ay(iStart:iEnd);
slip_cm = raw.Car_SideSlipAngle(iStart:iEnd);
steer_cm = (raw.Car_CFL_rz(iStart:iEnd) + raw.Car_CFR_rz(iStart:iEnd))/2;

for m = 1:4
    SIM.plantModel = models{m};
    ps = plant_init_state(SIM.plantModel, vx_cm(1), params);
    N = round(t_cm(end)/dt) + 1;
    tSim = (0:N-1)'*dt;
    log_yr = zeros(N,1); log_ay = zeros(N,1); log_slip = zeros(N,1);

    switch SIM.plantModel
        case 'bicycle', omegaIdx = [];
        case '3dof',    omegaIdx = [];
        case '7dof',    omegaIdx = 4:7;
        case '14dof',   omegaIdx = 16:19;
    end

    for k = 1:N
        tk = tSim(k);
        sw = interp1(t_cm, steer_cm, tk, 'linear','extrap');
        vxK = interp1(t_cm, vx_cm, tk, 'linear','extrap');
        if ~strcmp(SIM.plantModel,'bicycle')
            ps.x(1) = max(vxK, 0.5);
        end
        if ~isempty(omegaIdx)
            ps.x(omegaIdx) = vxK / VEH.rw;
        end
        ps.vx = vxK;
        cmd.steerAngle = 0; cmd.brakeTorque = zeros(4,1); cmd.dampingCoeff = 1500*ones(4,1);
        [out, ps] = plant_step(ps, cmd, sw, params, dt);
        log_yr(k) = out.yawRate; log_ay(k) = out.ay; log_slip(k) = out.slipAngle;
    end
    R = struct();
    R.model = modelLabels{m}; R.t = t_cm;
    R.yr_cm = yr_cm; R.yr_sim = interp1(tSim, log_yr, t_cm, 'linear','extrap');
    R.ay_cm = ay_cm; R.ay_sim = interp1(tSim, log_ay, t_cm, 'linear','extrap');
    R.slip_cm = slip_cm; R.slip_sim = interp1(tSim, log_slip, t_cm, 'linear','extrap');
    latResults(end+1) = R; %#ok<SAGROW>
end

%% 비교 그림 + 결과 .mat 저장
figDir = fullfile(projectRoot, 'docs', 'figures');
if ~exist(figDir,'dir'), mkdir(figDir); end
figPath = fullfile(figDir, sprintf('compare_4models_%s.png', datestr(now,'yyyymmdd_HHMMSS')));
util_plot_4model_compare(longResults, latResults, figPath);
fprintf('Saved: %s\n', figPath);

matPath = fullfile(figDir, sprintf('compare_4models_%s.mat', datestr(now,'yyyymmdd_HHMMSS')));
save(matPath, 'longResults', 'latResults', 'VEH', 'TIRE');
fprintf('Saved: %s\n', matPath);
