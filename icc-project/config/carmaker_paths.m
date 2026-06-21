%CARMAKER_PATHS CarMaker 경로 설정
%   IPG CarMaker 연동에 필요한 경로와 포트를 정의한다.
%   사용자 환경에 맞게 수정하세요.

% CarMaker 설치 경로
CM_INSTALL_DIR = 'C:/IPG/carmaker/win64-13.0';

% CarMaker 프로젝트 디렉토리
CM_PROJECT_DIR = fullfile(fileparts(mfilename('fullpath')), '..', 'carmaker_project');
CM_PROJECT_DIR = char(java.io.File(CM_PROJECT_DIR).getCanonicalPath());

% CarMaker 실행 파일
CM_EXE = fullfile(CM_INSTALL_DIR, 'bin', 'CM.exe');

% ScriptControl 포트
CM_PORT = 16660;

% SimOutput 디렉토리
CM_SIM_OUTPUT = fullfile(CM_PROJECT_DIR, 'SimOutput');

% MATLAB 인터페이스 path 추가
cmMatlabPath = fullfile(CM_INSTALL_DIR, 'matlab');
if isfolder(cmMatlabPath)
    addpath(genpath(cmMatlabPath));
    fprintf('[carmaker_paths] CarMaker MATLAB path added: %s\n', cmMatlabPath);
else
    warning('[carmaker_paths] CarMaker MATLAB path not found: %s', cmMatlabPath);
end
