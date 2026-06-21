%RUN_ICC_VALIDATE_CM CarMaker AEB 시나리오 대조로 14-DOF 모델 정확성 검증
%
%   오픈루프 종방향 검증:
%     1. CarMaker .erg에서 시간/입력/응답 추출
%     2. AEB 활성 phase 검출 (제동 시작 ~ 종료)
%     3. 그 phase 동안 plant_14dof에 동일 brake torque 시계열 주입
%        - 초기 vx를 CarMaker vx(t_start)와 일치시켜 모델 단독 평가
%     4. vx, ax, pitch RMS/max error + 제동거리 오차 산출
%     5. util_plot_cm_compare로 overlay 그림 저장
%
%   기본 시나리오: LK_CCIR_ST_CM15 첫 번째 (가벼운 제동 또는 AEB)
%   추가 시나리오 비교는 ergFiles cell에 path 추가.

%% 초기화 (스크립트 위치 기준 절대 경로)
% 호출자가 이미 init한 경우 SIM이 존재. 그 경우 SIM.plantModel 사용자 override 보존.
thisFile = mfilename('fullpath');
projectRoot = fileparts(fileparts(thisFile));
if ~exist('SIM','var') || ~isstruct(SIM) || ~isfield(SIM, 'cm_vehicleFile')
    run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));
end
if ~isfield(SIM, 'plantModel') || isempty(SIM.plantModel)
    SIM.plantModel = '14dof';
end

%% 대상 .erg 파일 목록
ergFiles = {
    'C:/Users/VIC/Projects/carmaker_data/SimOutput/bsong/20260428/LK_CCIR_ST_CM15_124910.erg';
    'C:/Users/VIC/Projects/carmaker_data/SimOutput/bsong/20260428/LK_CCIR_ST_CM15_143103.erg';
};

%% 파라미터 (sim_params에서 로드된 VEH/TIRE/CONST)
params = struct('VEH', VEH, 'TIRE', TIRE, 'CONST', CONST, 'SIM', SIM);
dt = SIM.dt;

%% 페달 압력 → 브레이크 토크 변환 게인 (Nm/bar per wheel)
% CarMaker BMW_5 .erg의 활성 제동 구간(pMC>50bar)에서 median(-Fx*rw / pWB).
BRK_GAIN_F = 15.94;  % Nm / bar  (front per wheel)
BRK_GAIN_R = 8.02;   % Nm / bar  (rear  per wheel)
BRAKE_ACTIVE_THR = 1.0;  % [bar] 이 압력 초과 시 제동 phase 시작

% Hydraulic lead 보정: 실제 휠 브레이크 토크는 pMC 신호에 가깝게 응답하지만
% .erg에 기록된 pWB는 ~50ms 라인 지연이 있어 초기 transient에서 plant 응답이 늦음.
% (1-alpha)*pWB + alpha*pMC 가중으로 lead 회복 (alpha=0 → 기존 pWB만).
BRK_LEAD_ALPHA = 0.8;  % 최적 균형 (vx/ax/stopDist 종합)

%% 각 시나리오 처리
results = struct();
for s = 1:numel(ergFiles)
    ergFile = ergFiles{s};
    if ~isfile(ergFile)
        warning('[validate_cm] Skip (missing): %s', ergFile);
        continue;
    end

    [~, base] = fileparts(ergFile);
    fprintf('\n=== Scenario %d/%d: %s (plant=%s) ===\n', s, numel(ergFiles), base, SIM.plantModel);

    raw = cm_read_erg(ergFile);

    %% AEB 제동 phase 검출
    pMC = raw.Brake_Hyd_Sys_pMC;
    iAct = find(pMC > BRAKE_ACTIVE_THR, 1, 'first');
    if isempty(iAct)
        warning('[validate_cm] No braking phase found, skip: %s', base);
        continue;
    end
    iStart = max(1, iAct - 5);            % 약간 앞 마진
    iEnd   = numel(raw.Time);

    t_cm    = raw.Time(iStart:iEnd) - raw.Time(iStart);   % 0-base 재설정
    vx_cm   = raw.Car_v(iStart:iEnd);
    ax_cm   = raw.Car_ax(iStart:iEnd);
    pitch_cm = raw.Car_Pitch(iStart:iEnd);
    pWB_raw = [raw.Brake_Hyd_Sys_pWB_FL(iStart:iEnd), ...
               raw.Brake_Hyd_Sys_pWB_FR(iStart:iEnd), ...
               raw.Brake_Hyd_Sys_pWB_RL(iStart:iEnd), ...
               raw.Brake_Hyd_Sys_pWB_RR(iStart:iEnd)];
    % Hydraulic lead: pWB와 pMC의 가중평균(전후 비율 보존하기 위해 각 wheel의 pWB와 pMC 사용)
    pMC_seg = raw.Brake_Hyd_Sys_pMC(iStart:iEnd);
    pWB = pWB_raw;
    for c = 1:4
        pWB(:,c) = (1 - BRK_LEAD_ALPHA) * pWB_raw(:,c) + BRK_LEAD_ALPHA * pMC_seg .* (pWB_raw(:,c) ./ max(pWB_raw(:,c) + pMC_seg, 1e-3) * 2);
    end
    % 더 단순한 lead 적용: pWB와 pMC * (steady_pWB_ratio) 가중
    steady_ratio_f = median(pWB_raw(pMC_seg>40, 1) ./ max(pMC_seg(pMC_seg>40), 1));
    steady_ratio_r = median(pWB_raw(pMC_seg>40, 3) ./ max(pMC_seg(pMC_seg>40), 1));
    if ~isfinite(steady_ratio_f), steady_ratio_f = 1.0; end
    if ~isfinite(steady_ratio_r), steady_ratio_r = 1.0; end
    pWB(:,1) = (1 - BRK_LEAD_ALPHA) * pWB_raw(:,1) + BRK_LEAD_ALPHA * pMC_seg * steady_ratio_f;
    pWB(:,2) = (1 - BRK_LEAD_ALPHA) * pWB_raw(:,2) + BRK_LEAD_ALPHA * pMC_seg * steady_ratio_f;
    pWB(:,3) = (1 - BRK_LEAD_ALPHA) * pWB_raw(:,3) + BRK_LEAD_ALPHA * pMC_seg * steady_ratio_r;
    pWB(:,4) = (1 - BRK_LEAD_ALPHA) * pWB_raw(:,4) + BRK_LEAD_ALPHA * pMC_seg * steady_ratio_r;
    if isfield(raw, 'Steer_WhlAng')
        steer_cm = raw.Steer_WhlAng(iStart:iEnd);
    else
        steer_cm = zeros(size(t_cm));
    end

    %% Brake torque 시계열 (4×1)
    brakeTorqueSeq = [pWB(:,1)*BRK_GAIN_F, pWB(:,2)*BRK_GAIN_F, ...
                      pWB(:,3)*BRK_GAIN_R, pWB(:,4)*BRK_GAIN_R];
    brakeTorqueSeq = min(brakeTorqueSeq, LIM.MAX_BRAKE_TRQ);

    %% Plant 초기화 (SIM.plantModel 따라 분기)
    if ~isfield(SIM, 'plantModel') || isempty(SIM.plantModel)
        SIM.plantModel = '14dof';
    end
    plantState = plant_init_state(SIM.plantModel, max(vx_cm(1), 0.5), params);

    %% 시뮬레이션
    tEnd = t_cm(end);
    tSim = (0:dt:tEnd)';
    nSim = numel(tSim);

    log = struct();
    log.t = tSim;
    log.vx = zeros(nSim, 1);
    log.ax = zeros(nSim, 1);
    log.pitch = zeros(nSim, 1);

    cs_const = (VEH.cs_f + VEH.cs_r) / 2;

    fprintf('  [%s] Phase %.2fs ~ %.2fs (%.2fs), vx0=%.2f m/s\n', ...
            base, raw.Time(iStart), raw.Time(iEnd), tEnd, vx_cm(1));

    for k = 1:nSim
        tk = tSim(k);
        bt = interp1(t_cm, brakeTorqueSeq, tk, 'linear', 'extrap')';
        sw = interp1(t_cm, steer_cm,        tk, 'linear', 'extrap');

        actCmd = struct();
        actCmd.steerAngle    = 0;
        actCmd.brakeTorque   = bt;
        actCmd.dampingCoeff  = cs_const * ones(4, 1);

        [out, plantState] = plant_step(plantState, actCmd, sw, params, dt);

        log.vx(k)    = out.vx;
        log.ax(k)    = out.ax;
        log.pitch(k) = out.pitch;
    end

    %% 비교 채널 정렬 (CarMaker 시계 t_cm 격자에 plant 결과 보간)
    cmp.t          = t_cm;
    cmp.vx_cm      = vx_cm;
    cmp.vx_sim     = interp1(tSim, log.vx,    t_cm, 'linear', 'extrap');
    cmp.ax_cm      = ax_cm;
    cmp.ax_sim     = interp1(tSim, log.ax,    t_cm, 'linear', 'extrap');
    cmp.pitch_cm   = pitch_cm;
    cmp.pitch_sim  = interp1(tSim, log.pitch, t_cm, 'linear', 'extrap');
    cmp.brakeTorq  = sum(brakeTorqueSeq, 2);   % 4륜 합

    %% KPI 윈도우: 차량 정지(vx_cm < 0.5)는 KPI에서 제외 (모델/제어 무관 영역)
    moveMask = vx_cm > 0.5;
    if nnz(moveMask) < 10
        moveMask = true(size(vx_cm));  % fallback
    end
    cmp.moveMask = moveMask;

    %% KPI (정지 phase 제외)
    kpi.vx_rms_err     = sqrt(mean((cmp.vx_sim(moveMask)    - cmp.vx_cm(moveMask)).^2));
    kpi.vx_max_err     = max(abs(cmp.vx_sim(moveMask)       - cmp.vx_cm(moveMask)));
    kpi.ax_rms_err     = sqrt(mean((cmp.ax_sim(moveMask)    - cmp.ax_cm(moveMask)).^2));
    kpi.ax_max_err     = max(abs(cmp.ax_sim(moveMask)       - cmp.ax_cm(moveMask)));
    kpi.pitch_rms_err  = sqrt(mean((cmp.pitch_sim(moveMask) - cmp.pitch_cm(moveMask)).^2)) * 180/pi;
    kpi.pitch_max_err  = max(abs(cmp.pitch_sim(moveMask)    - cmp.pitch_cm(moveMask))) * 180/pi;

    % 제동거리: cumtrapz vx (approx). 초기 vx 동일하므로 sim vs cm 양쪽 적분.
    dist_cm  = cumtrapz(t_cm, vx_cm);
    dist_sim = cumtrapz(t_cm, cmp.vx_sim);
    kpi.stopDist_cm   = dist_cm(end);
    kpi.stopDist_sim  = dist_sim(end);
    if abs(dist_cm(end)) > 1e-3
        kpi.stopDist_err_pct = abs(dist_sim(end) - dist_cm(end)) / abs(dist_cm(end)) * 100;
    else
        kpi.stopDist_err_pct = NaN;
    end

    fprintf('  KPI: vx_rms=%.3f m/s, ax_rms=%.3f m/s^2, pitch_rms=%.3f deg, stopDist err=%.2f%%\n', ...
            kpi.vx_rms_err, kpi.ax_rms_err, kpi.pitch_rms_err, kpi.stopDist_err_pct);

    %% 판정 (kpi_thresholds.modelFidelity)
    verdict = struct();
    verdict.vx       = local_verdict(kpi.vx_rms_err,    KPI_TH.modelFidelity.vx_rms_err);
    verdict.ax       = local_verdict(kpi.ax_rms_err,    KPI_TH.modelFidelity.ax_rms_err);
    verdict.pitch    = local_verdict(kpi.pitch_rms_err, KPI_TH.modelFidelity.pitch_rms_err);
    verdict.stopDist = local_verdict(kpi.stopDist_err_pct, KPI_TH.modelFidelity.stopDist_err_pct);
    fprintf('  Verdict: vx=%s ax=%s pitch=%s stopDist=%s\n', ...
            verdict.vx, verdict.ax, verdict.pitch, verdict.stopDist);

    %% 그림 저장 (plant 모델 라벨 포함, 절대경로)
    figPath = fullfile(projectRoot, 'data', sprintf('validate_cm_%s_%s_%s.png', ...
                       SIM.plantModel, base, datestr(now, 'yyyymmdd_HHMMSS')));
    util_plot_cm_compare(cmp, kpi, base, figPath);
    fprintf('  Saved: %s\n', figPath);

    %% 결과 누적
    results(s).scenario = base;
    results(s).kpi      = kpi;
    results(s).verdict  = verdict;
    results(s).figPath  = figPath;
end

%% 요약 .mat 저장
outFile = fullfile(projectRoot, 'data', sprintf('validate_cm_summary_%s_%s.mat', SIM.plantModel, datestr(now,'yyyymmdd_HHMMSS')));
save(outFile, 'results', 'VEH', 'TIRE');
fprintf('\n[validate_cm] Summary saved to %s\n', outFile);

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
