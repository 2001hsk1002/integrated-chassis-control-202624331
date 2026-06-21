function TIRE = tire_full_mf_parse_tir(tirFile, baseTIRE)
%TIRE_FULL_MF_PARSE_TIR ADAMS Tire Property File (.tir) → MATLAB TIRE struct
%
%   TIRE = TIRE_FULL_MF_PARSE_TIR(tirFile, baseTIRE)
%
%   ADAMS .tir 파일 (FILE_VERSION 3.0, PROPERTY_FILE_FORMAT='MF_05')에서
%   Pacejka Magic Formula 5.2 계수를 추출. tire_full_mf 가 요구하는 필드:
%       FZ0, PCX1, PDX1, PDX2, PEX1..4, PKX1..3, PHX1..2, PVX1..2,
%       PCY1, PDY1..3, PEY1..4, PKY1..3, PHY1..3, PVY1..4
%
%   .tir 파일 포맷: '$' 주석, [SECTION] 블록, 'KEY = VALUE' 항목.
%   Lines like "PDY1 =  1.0283   $ comment"
%
%   Inputs:
%       tirFile  - (char) .tir 파일 경로
%       baseTIRE - (struct, 옵션) 누락 필드 fallback. 없으면 NaN 채움
%
%   Outputs:
%       TIRE - (struct) MF 5.2 계수 + .model='full_mf' 설정됨

    if ~isfile(tirFile)
        error('[tire_full_mf_parse_tir] File not found: %s', tirFile);
    end

    if nargin < 2 || isempty(baseTIRE)
        baseTIRE = struct();
    end

    %% 텍스트 파싱 — INFOFILE과 비슷한 KEY=VALUE 라인 추출
    fid = fopen(tirFile, 'r');
    if fid < 0
        error('[tire_full_mf_parse_tir] Cannot open: %s', tirFile);
    end
    cleanup = onCleanup(@() fclose(fid));

    raw = struct();
    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line); break; end
        % '$' 주석 제거
        idxC = find(line == '$', 1, 'first');
        if ~isempty(idxC)
            line = line(1:idxC-1);
        end
        line = strtrim(line);
        if isempty(line); continue; end
        if line(1) == '[' || line(1) == '(' || line(1) == '{'
            continue;  % 섹션/주석 헤더 무시
        end
        eqIdx = find(line == '=', 1, 'first');
        if isempty(eqIdx); continue; end
        key = strtrim(line(1:eqIdx-1));
        val = strtrim(line(eqIdx+1:end));
        % 따옴표 제거
        val = regexprep(val, "^['""](.*)['""]$", '$1');
        % 숫자 변환 시도
        v = str2double(val);
        if ~isnan(v)
            raw.(local_safe_key(key)) = v;
        else
            raw.(local_safe_key(key)) = val;
        end
    end

    %% MF 5.2 계수 매핑 (.tir 표준 명명을 그대로 사용)
    TIRE = baseTIRE;
    TIRE.model = 'full_mf';
    TIRE.source = struct('file', tirFile, 'parsed', datestr(now,'yyyy-mm-dd HH:MM:SS'));

    % ADAMS .tir는 SAE convention (positive alpha → Fy 음수). 본 프로젝트의 plants는
    % ISO convention (positive alpha → positive Fy). 출력 시 Fy 부호 반전.
    TIRE.fy_sign_flip = -1;

    % 공칭 하중
    TIRE.FZ0 = local_pick(raw, {'FNOMIN','FZ0'}, 4000);

    % 종방향 (Long)
    longKeys = {'PCX1','PDX1','PDX2','PEX1','PEX2','PEX3','PEX4', ...
                'PKX1','PKX2','PKX3','PHX1','PHX2','PVX1','PVX2'};
    for k = 1:numel(longKeys)
        TIRE.(longKeys{k}) = local_pick(raw, longKeys(k), NaN);
    end

    % 횡방향 (Lat)
    latKeys = {'PCY1','PDY1','PDY2','PDY3','PEY1','PEY2','PEY3','PEY4', ...
               'PKY1','PKY2','PKY3','PHY1','PHY2','PHY3','PVY1','PVY2','PVY3','PVY4'};
    for k = 1:numel(latKeys)
        TIRE.(latKeys{k}) = local_pick(raw, latKeys(k), NaN);
    end

    % NaN 필드는 기본값으로 채움 (load sensitivity 항이 .tir에 없으면 0)
    fillDefaults = struct( ...
        'PEX4',0, 'PHX1',0, 'PHX2',0, 'PVX1',0, 'PVX2',0, ...
        'PDY3',0, 'PEY3',0, 'PEY4',0, ...
        'PHY1',0, 'PHY2',0, 'PHY3',0, ...
        'PVY1',0, 'PVY2',0, 'PVY3',0, 'PVY4',0);
    fn = fieldnames(fillDefaults);
    for k = 1:numel(fn)
        if isnan(TIRE.(fn{k}))
            TIRE.(fn{k}) = fillDefaults.(fn{k});
        end
    end

    % 미정인 critical 필드 검증
    critical = {'PCX1','PDX1','PKX1','PCY1','PDY1','PKY1'};
    for k = 1:numel(critical)
        if isnan(TIRE.(critical{k}))
            warning('[tire_full_mf_parse_tir] Critical coefficient %s not found in %s', ...
                    critical{k}, tirFile);
        end
    end
end

%% --------------------------------------------------------------
function safe = local_safe_key(key)
    safe = regexprep(key, '[^A-Za-z0-9_]', '_');
    if ~isempty(safe) && ~isstrprop(safe(1), 'alpha') && safe(1) ~= '_'
        safe = ['x', safe];
    end
end

function v = local_pick(s, names, default)
    for i = 1:numel(names)
        if isfield(s, names{i})
            v = s.(names{i});
            return;
        end
    end
    v = default;
end
