%KPI_THRESHOLDS KPI 판정 기준값 정의
%   각 KPI의 PASS / MARGINAL / FAIL 기준을 구조체로 정의한다.

%% 횡방향 (Lateral)
KPI_TH.yawRateError_rms.pass = 0.5;    % [deg/s]
KPI_TH.yawRateError_rms.marginal = 1.0;

KPI_TH.yawRateError_max.pass = 2.0;    % [deg/s]
KPI_TH.yawRateError_max.marginal = 5.0;

KPI_TH.lateralDev_max.pass = 0.5;      % [m]
KPI_TH.lateralDev_max.marginal = 1.0;

%% 종방향 (Longitudinal)
KPI_TH.speedError_rms.pass = 0.5;      % [km/h]
KPI_TH.speedError_rms.marginal = 1.0;

KPI_TH.jerk_max.pass = 2.0;            % [m/s^3]
KPI_TH.jerk_max.marginal = 5.0;

%% 수직 (Vertical)
KPI_TH.bodyAccel_rms.pass = 1.0;       % [m/s^2]
KPI_TH.bodyAccel_rms.marginal = 2.0;

KPI_TH.rollAngle_max.pass = 3.0;       % [deg]
KPI_TH.rollAngle_max.marginal = 5.0;

%% 안정성 (Stability)
KPI_TH.slipAngle_max.pass = 5.0;       % [deg]
KPI_TH.slipAngle_max.marginal = 8.0;

KPI_TH.LTR_max.pass = 0.7;             % [-]
KPI_TH.LTR_max.marginal = 0.9;

KPI_TH.overshoot.pass = 10;            % [%]
KPI_TH.overshoot.marginal = 20;

KPI_TH.settlingTime.pass = 0.5;        % [s]
KPI_TH.settlingTime.marginal = 1.0;

%% 모델 정확도 (Model Fidelity vs CarMaker reference)
KPI_TH.modelFidelity.vx_rms_err.pass     = 0.3;     % [m/s]
KPI_TH.modelFidelity.vx_rms_err.marginal = 0.7;

KPI_TH.modelFidelity.ax_rms_err.pass     = 0.5;     % [m/s^2]
KPI_TH.modelFidelity.ax_rms_err.marginal = 1.0;

KPI_TH.modelFidelity.pitch_rms_err.pass     = 0.3;  % [deg]
KPI_TH.modelFidelity.pitch_rms_err.marginal = 0.7;

KPI_TH.modelFidelity.stopDist_err_pct.pass     = 5;    % [%]
KPI_TH.modelFidelity.stopDist_err_pct.marginal = 10;

%% 모델 정확도 — 횡방향 (Lateral fidelity vs CarMaker)
KPI_TH.modelFidelityLat.yawRate_rms_err.pass     = 2.0;   % [deg/s]
KPI_TH.modelFidelityLat.yawRate_rms_err.marginal = 5.0;

KPI_TH.modelFidelityLat.ay_rms_err.pass     = 0.5;        % [m/s^2]
KPI_TH.modelFidelityLat.ay_rms_err.marginal = 1.2;

KPI_TH.modelFidelityLat.slip_rms_err.pass     = 1.0;      % [deg]
KPI_TH.modelFidelityLat.slip_rms_err.marginal = 2.5;

KPI_TH.modelFidelityLat.roll_rms_err.pass     = 0.5;      % [deg]
KPI_TH.modelFidelityLat.roll_rms_err.marginal = 1.5;

fprintf('[kpi_thresholds] Thresholds loaded.\n');
