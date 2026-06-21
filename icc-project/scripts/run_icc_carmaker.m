%RUN_ICC_CARMAKER ICC + CarMaker 통합 시뮬레이션 실행
%
%   CarMaker ScriptControl을 통해 시뮬레이션을 실행하고 결과를 분석한다.
%   사전에 CarMaker가 실행 중이어야 한다.
%
%   Usage:
%       run('scripts/run_icc_carmaker.m')

clear; clc; close all;

%% 초기화 — 이 스크립트 위치로부터 프로젝트 루트를 결정
thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);  % scripts/ 의 상위 = 프로젝트 루트
run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));

fprintf('\n=== ICC + CarMaker Simulation ===\n');

%% 시나리오 설정
scenarios = {
    'DLC_80'
    'DLC_120'
    'StepSteer_80'
    'Slalom_60'
};

% 단일 시나리오 실행 시:
% scenarios = {'DLC_80'};

%% CarMaker 연결
cm = cm_connect('localhost', CM_PORT);

%% 배치 실행
summary = cm_batch_run(cm, scenarios, ...
    'timeout', 120, ...
    'saveResults', true, ...
    'outputDir', 'data');

%% 개별 결과 분석
for i = 1:numel(scenarios)
    if isempty(summary.results{i}), continue; end

    result = summary.results{i};
    scenName = scenarios{i};

    % KPI (이미 계산됨)
    kpi = summary.kpis{i};

    % 안전성 검증
    [safe, alerts] = util_check_safety(result);

    % 플롯 저장
    figPath = fullfile('data', sprintf('fig_%s_%s.png', ...
        scenName, datestr(now,'yyyymmdd')));
    util_plot_results(result, ...
        'title', sprintf('ICC — %s', scenName), ...
        'savePath', figPath, ...
        'visible', 'off');

    % 보고서 내보내기
    util_export_report(result, kpi, scenName, 'format', 'csv');
end

%% 연결 해제
clear cm;

fprintf('\n=== All Simulations Complete ===\n');
disp(summary.table);
