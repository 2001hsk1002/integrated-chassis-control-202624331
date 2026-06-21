%RUN_ICC_STANDALONE ICC 독립 시뮬레이션 (Plant Adapter 기반)
%
%   선택된 플랜트 모델(bicycle/3dof/7dof/14dof)과 전체 제어기 체인으로
%   시뮬레이션을 수행한다. SIM.plantModel로 플랜트를 전환한다.
%
%   Usage:
%       run('scripts/run_icc_standalone.m')
%
%   Plant 전환:
%       config/sim_params.m에서 SIM.plantModel = '3dof' 등으로 변경 후 실행

clear; clc; close all;

%% 초기화 — 이 스크립트 위치로부터 프로젝트 루트를 결정
thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));

fprintf('\n=== ICC Standalone Simulation [Plant: %s] ===\n', SIM.plantModel);

%% 시뮬레이션 설정
dt   = SIM.dt;
tEnd = SIM.t_end;
t    = (0:dt:tEnd)';
n    = numel(t);
vx0  = 80 * CONST.kmh2ms;  % 80 km/h 초기 속도

%% 입력 생성: 이중 차선 변경 (DLC) 조향 패턴
steerDriver = zeros(n, 1);
for k = 1:n
    if t(k) >= 1 && t(k) < 3
        steerDriver(k) = deg2rad(3) * sin(pi * (t(k) - 1) / 2);
    elseif t(k) >= 3 && t(k) < 5
        steerDriver(k) = -deg2rad(3) * sin(pi * (t(k) - 3) / 2);
    end
end

%% 속도 레퍼런스 (현재 일정 속도)
vxRef = vx0;

%% 파라미터 팩
params = struct('VEH', VEH, 'TIRE', TIRE, 'CONST', CONST, 'SIM', SIM);

%% 플랜트 초기화
plantState = plant_init_state(SIM.plantModel, vx0, params);

%% 제어기 상태 초기화
latState.intError  = 0;
latState.prevError = 0;
lonState.intError  = 0;
lonState.prevForce = 0;

%% 초기 제로 액추에이터 명령
zeroCmd.steerAngle   = 0;
zeroCmd.brakeTorque  = zeros(4, 1);
zeroCmd.dampingCoeff = VEH.cs_f * ones(4, 1);  % 기본 감쇠

%% 초기 플랜트 스텝 (k=0 상태 획득)
[plantOut, plantState] = plant_step(plantState, zeroCmd, steerDriver(1), params, dt);

%% 로그 배열 사전 할당
simLog.time        = t;
simLog.vx          = zeros(n, 1);
simLog.vy          = zeros(n, 1);
simLog.ax          = zeros(n, 1);
simLog.ay          = zeros(n, 1);
simLog.yawRate     = zeros(n, 1);
simLog.slipAngle   = zeros(n, 1);
simLog.roll        = zeros(n, 1);
simLog.pitch       = zeros(n, 1);
simLog.steerDriver = steerDriver;
simLog.steerAdd    = zeros(n, 1);
simLog.yawMoment   = zeros(n, 1);

% 타이어/서스펜션 로그 (4바퀴)
wheels = {'FL','FR','RL','RR'};
for w = 1:4
    simLog.tire.(wheels{w}).Fx = zeros(n, 1);
    simLog.tire.(wheels{w}).Fy = zeros(n, 1);
    simLog.tire.(wheels{w}).Fz = zeros(n, 1);
    simLog.tire.(wheels{w}).slipAngle = zeros(n, 1);
    simLog.tire.(wheels{w}).slipRatio = zeros(n, 1);
    simLog.susp.(wheels{w}).springFrc = zeros(n, 1);
    simLog.susp.(wheels{w}).damperFrc = zeros(n, 1);
end

%% 메인 시뮬레이션 루프
for k = 1:n
    %% 목표 요 레이트 (현재 속도 기반)
    yawRateRef_k = calc_ref_yaw_rate(plantOut.vx, steerDriver(k), VEH);

    %% 횡방향 제어
    [latCmd, latState] = ctrl_lateral(yawRateRef_k, plantOut.yawRate, ...
                                       plantOut.slipAngle, plantOut.vx, ...
                                       latState, CTRL, LIM, dt);

    %% 종방향 제어
    [lonCmd, lonState] = ctrl_longitudinal(vxRef, plantOut.vx, plantOut.ax, ...
                                            lonState, CTRL, LIM, dt);

    %% 수직 제어
    verCmd = ctrl_vertical(plantOut.suspVel, plantOut.bodyVel, ...
                           plantOut.ay, plantOut.roll, CTRL);

    %% 통합 조율기
    actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, plantOut.vx, VEH, CTRL, LIM);

    %% 플랜트 스텝
    [plantOut, plantState] = plant_step(plantState, actuatorCmd, steerDriver(k), params, dt);

    %% 로그 저장
    simLog.vx(k)        = plantOut.vx;
    simLog.vy(k)        = plantOut.vy;
    simLog.ax(k)        = plantOut.ax;
    simLog.ay(k)        = plantOut.ay;
    simLog.yawRate(k)   = plantOut.yawRate;
    simLog.slipAngle(k) = plantOut.slipAngle;
    simLog.roll(k)      = plantOut.roll;
    simLog.pitch(k)     = plantOut.pitch;
    simLog.steerAdd(k)  = latCmd.steerAngle;
    simLog.yawMoment(k) = latCmd.yawMoment;

    for w = 1:4
        wn = wheels{w};
        simLog.tire.(wn).Fx(k)        = plantOut.tire.(wn).Fx;
        simLog.tire.(wn).Fy(k)        = plantOut.tire.(wn).Fy;
        simLog.tire.(wn).Fz(k)        = plantOut.tire.(wn).Fz;
        simLog.tire.(wn).slipAngle(k) = plantOut.tire.(wn).slipAngle;
        simLog.tire.(wn).slipRatio(k) = plantOut.tire.(wn).slipRatio;
        simLog.susp.(wn).springFrc(k) = plantOut.susp.(wn).springFrc;
        simLog.susp.(wn).damperFrc(k) = plantOut.susp.(wn).damperFrc;
    end

    %% 발산 체크
    if any(isnan(plantState.x)) || any(isinf(plantState.x))
        warning('Simulation diverged at t = %.3f s', t(k));
        simLog.time = t(1:k);
        fields = {'vx','vy','ax','ay','yawRate','slipAngle','roll','pitch','steerAdd','yawMoment'};
        for f = 1:numel(fields)
            simLog.(fields{f}) = simLog.(fields{f})(1:k);
        end
        simLog.steerDriver = simLog.steerDriver(1:k);
        for w = 1:4
            wn = wheels{w};
            tFields = fieldnames(simLog.tire.(wn));
            for tf = 1:numel(tFields)
                simLog.tire.(wn).(tFields{tf}) = simLog.tire.(wn).(tFields{tf})(1:k);
            end
            sFields = fieldnames(simLog.susp.(wn));
            for sf = 1:numel(sFields)
                simLog.susp.(wn).(sFields{sf}) = simLog.susp.(wn).(sFields{sf})(1:k);
            end
        end
        break;
    end
end

%% 결과 구조체 구성 (util_calc_kpi 호환)
result.time      = simLog.time;
result.vx        = simLog.vx;
result.vy        = simLog.vy;
result.ax        = simLog.ax;
result.ay        = simLog.ay;
result.yawRate   = simLog.yawRate;
result.slipAngle = simLog.slipAngle;
result.roll      = simLog.roll;
result.pitch     = simLog.pitch;
result.steerAngle = simLog.steerDriver + simLog.steerAdd;
result.jerk      = [0; diff(simLog.ax)] / dt;
result.tire      = simLog.tire;
result.susp      = simLog.susp;

% LTR 계산
if isfield(result.tire, 'FL') && any(result.tire.FL.Fz ~= 0)
    Fz_left  = result.tire.FL.Fz + result.tire.RL.Fz;
    Fz_right = result.tire.FR.Fz + result.tire.RR.Fz;
    Fz_total = Fz_left + Fz_right;
    Fz_total(Fz_total == 0) = eps;
    result.LTR = (Fz_right - Fz_left) ./ Fz_total;
else
    % Bicycle 등 Fz가 0인 경우 → 근사 LTR
    result.LTR = result.ay * VEH.h_cog / (CONST.g * VEH.track_f / 2);
end

ref.yawRate = calc_ref_yaw_rate(result.vx, simLog.steerDriver, VEH);
ref.vx      = result.vx;

%% KPI 계산
kpi = util_calc_kpi(result, ref);

fprintf('\n--- KPI Summary [%s] ---\n', SIM.plantModel);
fprintf('  Yaw Rate Error (RMS): %.3f deg/s [%s]\n', kpi.yawRateError_rms, kpi.verdicts.yawRateError_rms);
fprintf('  Yaw Rate Error (max): %.3f deg/s [%s]\n', kpi.yawRateError_max, kpi.verdicts.yawRateError_max);
fprintf('  Slip Angle (max):     %.3f deg   [%s]\n', kpi.slipAngle_max, kpi.verdicts.slipAngle_max);
fprintf('  Roll Angle (max):     %.3f deg   [%s]\n', kpi.rollAngle_max, kpi.verdicts.rollAngle_max);
fprintf('  LTR (max):            %.3f       [%s]\n', kpi.LTR_max, kpi.verdicts.LTR_max);
fprintf('  Overall: %s\n', kpi.overall);

%% 안전성 검증
[safe, alerts] = util_check_safety(result);

%% 플롯
util_plot_results(result, 'ref', ref, ...
    'title', sprintf('ICC [%s] — DLC @ %d km/h', SIM.plantModel, round(vx0 * 3.6)));

%% 결과 저장
saveFile = fullfile('data', sprintf('sim_%s_%s.mat', SIM.plantModel, datestr(now,'yyyymmdd_HHMMSS')));
save(saveFile, 'result', 'ref', 'kpi', 'simLog');
fprintf('[run_icc_standalone] Result saved: %s\n', saveFile);

fprintf('\n=== Simulation Complete [%s] ===\n', SIM.plantModel);
