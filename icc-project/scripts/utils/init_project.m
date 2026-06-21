%INIT_PROJECT ICC 프로젝트 초기화
%   MATLAB 경로 설정, 파라미터 로드, CarMaker 경로를 초기화한다.
%   프로젝트 작업 시작 전 반드시 한 번 실행해야 한다.
%
%   Usage:
%       run('scripts/utils/init_project.m')

% 프로젝트 루트 경로 결정
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
fprintf('=== ICC Project Initialization ===\n');
fprintf('Project root: %s\n', projectRoot);

% MATLAB path 추가
addpath(genpath(fullfile(projectRoot, 'scripts')));
addpath(genpath(fullfile(projectRoot, 'config')));
addpath(fullfile(projectRoot, 'models', 'simulink'));
addpath(fullfile(projectRoot, 'tests'));

% data 디렉토리 생성 (없으면)
dataDir = fullfile(projectRoot, 'data');
if ~isfolder(dataDir)
    mkdir(dataDir);
end

% 설정 파일 로드
run(fullfile(projectRoot, 'config', 'sim_params.m'));
run(fullfile(projectRoot, 'config', 'kpi_thresholds.m'));
run(fullfile(projectRoot, 'config', 'carmaker_paths.m'));

% 작업 디렉토리를 프로젝트 루트로 변경
cd(projectRoot);

fprintf('=== Initialization Complete ===\n');
