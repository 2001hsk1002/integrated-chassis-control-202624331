function params = cm_parse_infofile(filePath)
%CM_PARSE_INFOFILE CarMaker INFOFILE1.1 형식 파서
%
%   params = CM_PARSE_INFOFILE(filePath)
%
%   CarMaker의 INFOFILE1.1 형식 (Vehicle 파라미터, TestRun, .erg.info 등)을
%   struct로 파싱한다. cmread/IPGControl 의존성 없는 자립 파서.
%
%   포맷 규칙:
%     - 첫 줄: '#INFOFILE1.1 ...'
%     - 'Key = value' 형태 (스칼라 또는 공백 구분 벡터 자동 인식)
%     - 들여쓴 줄(탭/스페이스)은 직전 키의 멀티라인 값 연속(cell)
%     - '##'/'#' 주석 라인 및 빈 줄 무시
%     - Key의 '.'은 '_'로 치환 (MATLAB 필드명 호환)
%
%   Inputs:
%       filePath - (char) INFOFILE 경로
%
%   Outputs:
%       params - (struct) Key가 필드, 값은 numeric / char / cell

    if ~isfile(filePath)
        error('[cm_parse_infofile] File not found: %s', filePath);
    end

    fid = fopen(filePath, 'r', 'n', 'UTF-8');
    if fid < 0
        error('[cm_parse_infofile] Cannot open: %s', filePath);
    end
    cleanup = onCleanup(@() fclose(fid));

    params = struct();
    lastKey = '';
    headerSeen = false;

    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end

        % 첫 헤더 검증
        if ~headerSeen
            if startsWith(strtrim(line), '#INFOFILE')
                headerSeen = true;
                continue;
            end
        end

        % 주석/빈 줄
        trimmed = strtrim(line);
        if isempty(trimmed) || startsWith(trimmed, '#')
            continue;
        end

        % 들여쓰기 → 직전 키의 멀티라인 연속
        isIndented = ~isempty(line) && (line(1) == ' ' || line(1) == sprintf('\t'));
        if isIndented && ~isempty(lastKey)
            curVal = params.(lastKey);
            if ~iscell(curVal)
                curVal = {curVal};
            end
            curVal{end+1, 1} = trimmed; %#ok<AGROW>
            params.(lastKey) = curVal;
            continue;
        end

        % 'Key = value' 분해
        eqIdx = find(line == '=', 1, 'first');
        if isempty(eqIdx)
            continue;  % 형식 외 라인 무시
        end
        keyRaw = strtrim(line(1:eqIdx-1));
        valRaw = strtrim(line(eqIdx+1:end));

        if isempty(keyRaw)
            continue;
        end

        % 'Key:' 같은 블록 시작자 처리 (드물게 발생) → 콜론 제거
        keyRaw = regexprep(keyRaw, ':$', '');

        % 필드명 안전화
        fieldName = local_to_field(keyRaw);

        % 값 해석: 우선 numeric vector로 시도, 실패 시 char
        valTrimmed = strtrim(valRaw);
        parsedVal = local_try_numeric(valTrimmed);

        params.(fieldName) = parsedVal;
        lastKey = fieldName;
    end

end

%% --------------------------------------------------------------
function fn = local_to_field(keyRaw)
% '.' → '_', 그 외 비식별자 문자 → '_', 숫자 시작 → 'x' 프리픽스
    fn = regexprep(keyRaw, '[^A-Za-z0-9_]', '_');
    if isempty(fn)
        fn = 'x';
    end
    if ~isstrprop(fn(1), 'alpha') && fn(1) ~= '_'
        fn = ['x' fn];
    end
end

%% --------------------------------------------------------------
function out = local_try_numeric(s)
% 공백 구분 토큰을 모두 numeric으로 변환 시도. 실패 시 원문 char 반환.
    if isempty(s)
        out = '';
        return;
    end

    % 명백한 비숫자 (영문자 시작이고 따옴표/특수문자 포함 등)
    % → 변수 표현 '$vA=60' 같은 형태가 있을 수 있으므로 char 우선 케이스
    if ~isempty(regexp(s, '[A-Za-z\$/\\]', 'once'))
        % 단, 'e' 지수 표기('1.5e-3')는 허용
        % 단순 휴리스틱: 'e' 외 영문자가 있으면 char로 분류
        if isempty(regexp(s, '^[\s\+\-\d\.eE]+$', 'once'))
            out = s;
            return;
        end
    end

    toks = strsplit(s);
    nums = zeros(1, numel(toks));
    ok = true;
    for k = 1:numel(toks)
        v = str2double(toks{k});  % 토큰 전체가 유효한 숫자일 때만 스칼라, 아니면 NaN
        if ~isscalar(v) || ~isfinite(v)
            ok = false;
            break;
        end
        nums(k) = v;
    end
    if ok
        if numel(nums) == 1
            out = nums(1);
        else
            out = nums;
        end
    else
        out = s;
    end
end
