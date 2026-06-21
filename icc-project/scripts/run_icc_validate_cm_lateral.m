%RUN_ICC_VALIDATE_CM_LATERAL CarMaker 곡선/턴 시나리오로 14-DOF 횡방향 정확성 검증
%
%   오픈루프 횡방향 검증:
%     1. CarMaker .erg에서 steering, vx, yawRate, ay, slipAngle, roll 추출
%     2. 같은 steer 시계열을 plant_14dof에 주입
%     3. vx는 CarMaker 값으로 매 스텝 강제 (드라이브 토크 입력 부재 우회 — 횡방향만 평가)
%     4. yaw rate / ay / slip / roll RMS error + 첨두 일치 KPI 산출
%
%   대상: RT_/LT_ 시나리오 (peak yaw rate 18~25 deg/s, peak ay 3~4 m/s²)

%% 초기화 (호출자가 SIM.plantModel 설정 가능)
thisFile = mfilename('fullpath');
projectRoot = fileparts(fileparts(thisFile));
if ~exist('SIM','var') || ~isstruct(SIM) || ~isfield(SIM, 'cm_vehicleFile')
    run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));
end
if ~isfield(SIM, 'plantModel') || isempty(SIM.plantModel)
    SIM.plantModel = '14dof';
end

%% 대상 시나리오
ergFiles = {
    'C:/Users/VIC/Projects/carmaker_data/SimOutput/bsong/20260506/RT_CGSL2R_IN_CM15_104327.erg';   % pure turn, no brake
    'C:/Users/VIC/Projects/carmaker_data/SimOutput/bsong/20260506/LT_CGSR2L_IN_CM15_104201.erg';   % left curve, no brake
};

params = struct('VEH', VEH, 'TIRE', TIRE, 'CONST', CONST, 'SIM', SIM);
dt = SIM.dt;

results = struct();

for s = 1:numel(ergFiles)
    ergFile = ergFiles{s};
    if ~isfile(ergFile)
        warning('[validate_cm_lat] Skip (missing): %s', ergFile);
        continue;
    end
    [~, base] = fileparts(ergFile);
    if ~isfield(SIM,'plantModel'), SIM.plantModel='14dof'; end
    fprintf('\n=== Lateral %d/%d: %s (plant=%s) ===\n', s, numel(ergFiles), base, SIM.plantModel);

    raw = cm_read_erg(ergFile);

    %% 활성 횡방향 phase 검출 (yaw rate > 1 deg/s)
    yrCm  = raw.Car_YawRate;
    iAct = find(abs(yrCm) > deg2rad(1), 1, 'first');
    if isempty(iAct)
        warning('[validate_cm_lat] No lateral activity, skip: %s', base);
        continue;
    end
    iStart = max(1, iAct - 50);   % 0.5s margin
    % 끝: 마지막 활성 + 1s
    iLastAct = find(abs(yrCm) > deg2rad(1), 1, 'last');
    iEnd     = min(numel(raw.Time), iLastAct + 100);

    t_cm     = raw.Time(iStart:iEnd) - raw.Time(iStart);
    vx_cm    = raw.Car_v(iStart:iEnd);
    yr_cm    = raw.Car_YawRate(iStart:iEnd);
    ay_cm    = raw.Car_ay(iStart:iEnd);
    slip_cm  = raw.Car_SideSlipAngle(iStart:iEnd);
    roll_cm  = raw.Car_Roll(iStart:iEnd);
    % steer 입력: Car.CFL.rz / Car.CFR.rz = 실제 노면휠 조향각 (rad).
    % Steer.WhlAng은 핸들각이며 ~13:1 비율로 노면휠보다 큼.
    steer_cm = (raw.Car_CFL_rz(iStart:iEnd) + raw.Car_CFR_rz(iStart:iEnd)) / 2;

    fprintf('  Phase %.2fs ~ %.2fs (%.2fs), vx0=%.2f m/s, peak yr=%.1f deg/s, peak ay=%.2f m/s^2\n', ...
            raw.Time(iStart), raw.Time(iEnd), t_cm(end), vx_cm(1), ...
            rad2deg(max(abs(yr_cm))), max(abs(ay_cm)));

    %% Plant 시뮬레이션 (vx는 CM 값으로 매 스텝 강제)
    if ~isfield(SIM, 'plantModel') || isempty(SIM.plantModel)
        SIM.plantModel = '14dof';
    end
    plantState = plant_init_state(SIM.plantModel, vx_cm(1), params);

    tEnd = t_cm(end);
    tSim = (0:dt:tEnd)';
    nSim = numel(tSim);
    log = struct();
    log.t  = tSim;
    log.yr = zeros(nSim, 1);
    log.ay = zeros(nSim, 1);
    log.slip = zeros(nSim, 1);
    log.roll = zeros(nSim, 1);

    cs_const = (VEH.cs_f + VEH.cs_r) / 2;

    % plantModel별 vx/omega 상태 인덱스
    switch SIM.plantModel
        case 'bicycle', omegaIdx = [];
        case '3dof',    omegaIdx = [];
        case '7dof',    omegaIdx = 4:7;
        case '14dof',   omegaIdx = 16:19;
        otherwise,      omegaIdx = [];
    end

    for k = 1:nSim
        tk = tSim(k);
        steerK = interp1(t_cm, steer_cm, tk, 'linear', 'extrap');
        vxK    = interp1(t_cm, vx_cm,    tk, 'linear', 'extrap');

        % vx 강제 (드라이브 토크 부재 우회 — 횡방향만 평가)
        if ~strcmp(SIM.plantModel, 'bicycle')
            plantState.x(1) = max(vxK, 0.5);
        end
        if ~isempty(omegaIdx)
            plantState.x(omegaIdx) = vxK / VEH.rw;
        end
        plantState.vx = vxK;

        actCmd.steerAngle   = 0;
        actCmd.brakeTorque  = zeros(4, 1);
        actCmd.dampingCoeff = cs_const * ones(4, 1);

        [out, plantState] = plant_step(plantState, actCmd, steerK, params, dt);

        log.yr(k)   = out.yawRate;
        log.ay(k)   = out.ay;
        log.slip(k) = out.slipAngle;
        log.roll(k) = out.roll;
    end

    %% 비교 채널 정렬
    cmp.t       = t_cm;
    cmp.yr_cm   = yr_cm;       cmp.yr_sim   = interp1(tSim, log.yr,   t_cm, 'linear', 'extrap');
    cmp.ay_cm   = ay_cm;       cmp.ay_sim   = interp1(tSim, log.ay,   t_cm, 'linear', 'extrap');
    cmp.slip_cm = slip_cm;     cmp.slip_sim = interp1(tSim, log.slip, t_cm, 'linear', 'extrap');
    cmp.roll_cm = roll_cm;     cmp.roll_sim = interp1(tSim, log.roll, t_cm, 'linear', 'extrap');
    cmp.steer   = steer_cm;
    cmp.vx      = vx_cm;

    %% KPI (각도는 deg 단위)
    kpi.yawRate_rms_err_deg = rad2deg(sqrt(mean((cmp.yr_sim - cmp.yr_cm).^2)));
    kpi.yawRate_max_err_deg = rad2deg(max(abs(cmp.yr_sim - cmp.yr_cm)));
    kpi.ay_rms_err          = sqrt(mean((cmp.ay_sim - cmp.ay_cm).^2));
    kpi.ay_max_err          = max(abs(cmp.ay_sim - cmp.ay_cm));
    kpi.slip_rms_err_deg    = rad2deg(sqrt(mean((cmp.slip_sim - cmp.slip_cm).^2)));
    kpi.slip_max_err_deg    = rad2deg(max(abs(cmp.slip_sim - cmp.slip_cm)));
    kpi.roll_rms_err_deg    = rad2deg(sqrt(mean((cmp.roll_sim - cmp.roll_cm).^2)));
    kpi.roll_max_err_deg    = rad2deg(max(abs(cmp.roll_sim - cmp.roll_cm)));

    fprintf('  KPI: yawR_rms=%.2f deg/s, ay_rms=%.2f m/s^2, slip_rms=%.2f deg, roll_rms=%.2f deg\n', ...
            kpi.yawRate_rms_err_deg, kpi.ay_rms_err, kpi.slip_rms_err_deg, kpi.roll_rms_err_deg);

    %% 판정
    th = KPI_TH.modelFidelityLat;
    verdict.yr   = local_verdict(kpi.yawRate_rms_err_deg, th.yawRate_rms_err);
    verdict.ay   = local_verdict(kpi.ay_rms_err,          th.ay_rms_err);
    verdict.slip = local_verdict(kpi.slip_rms_err_deg,    th.slip_rms_err);
    verdict.roll = local_verdict(kpi.roll_rms_err_deg,    th.roll_rms_err);
    fprintf('  Verdict: yr=%s ay=%s slip=%s roll=%s\n', ...
            verdict.yr, verdict.ay, verdict.slip, verdict.roll);

    %% 그림 저장
    figPath = fullfile(projectRoot, 'data', sprintf('validate_cm_lat_%s_%s_%s.png', ...
                       SIM.plantModel, base, datestr(now, 'yyyymmdd_HHMMSS')));
    util_plot_cm_lateral(cmp, kpi, base, figPath);
    fprintf('  Saved: %s\n', figPath);

    results(s).scenario = base;
    results(s).kpi      = kpi;
    results(s).verdict  = verdict;
end

outFile = fullfile(projectRoot, 'data', sprintf('validate_cm_lat_summary_%s_%s.mat', SIM.plantModel, datestr(now,'yyyymmdd_HHMMSS')));
save(outFile, 'results', 'VEH', 'TIRE');
fprintf('\n[validate_cm_lat] Summary saved to %s\n', outFile);

%% ============================================================
function v = local_verdict(value, th)
    if value < th.pass
        v = 'PASS';
    elseif value < th.marginal
        v = 'MARGINAL';
    else
        v = 'FAIL';
    end
end
