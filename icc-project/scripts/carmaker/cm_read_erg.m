function raw = cm_read_erg(ergFile)
%CM_READ_ERG CarMaker .erg 파일 읽기 — cmread MEX 또는 자체 파서 fallback
%
%   raw = CM_READ_ERG(ergFile)
%
%   읽기 우선순위:
%     1. cmread MEX (IPG 제공) — 라이선스/경로 OK 시 표준 방식
%     2. 자체 .erg.info + 바이너리 파서 — 라이선스 무관, INFOFILE 헤더 기반
%
%   채널 이름의 '.'은 '_'로 치환되어 struct 필드명으로 변환됨
%   (cmread MEX의 관습 따름).
%
%   Inputs:
%       ergFile - (char) .erg 경로 (.erg.info 동반 존재 필수 — 자체 파서 모드)
%
%   Outputs:
%       raw - (struct) 채널명 → 시계열 벡터

    if ~isfile(ergFile)
        error('[cm_read_erg] File not found: %s', ergFile);
    end

    %% ---------- 1차 시도: cmread MEX ----------
    raw = local_try_cmread(ergFile);
    if ~isempty(raw)
        fprintf('[cm_read_erg] Loaded via cmread MEX (%s).\n', ergFile);
        return;
    end

    %% ---------- 2차: 자체 파서 ----------
    raw = local_parse_erg_native(ergFile);
    fprintf('[cm_read_erg] Loaded via native parser (%s, %d samples, %d channels).\n', ...
        ergFile, numel(raw.Time), numel(fieldnames(raw)));
end

%% ============================================================
function raw = local_try_cmread(ergFile)
% cmread MEX 가용성 확인 + 호출 시도
    raw = [];

    % MATLAB 릴리스에 맞는 cmread MEX path 추가
    relCandidates = {'R2024b','R2024a','R2023b','R2023a','R2022b'};
    rel = '';
    try
        rel = ['R' version('-release')];
    catch
    end
    if ~isempty(rel) && ~any(strcmp(relCandidates, rel))
        relCandidates = [{rel}, relCandidates]; %#ok<AGROW>
    elseif ~isempty(rel)
        relCandidates = [{rel}, relCandidates(~strcmp(relCandidates, rel))];
    end

    cmReleaseBase = 'C:/IPG/carmaker/win64-15.0/Matlab';
    if exist('cmread', 'file') ~= 3   % 3 = MEX file
        for k = 1:numel(relCandidates)
            p = fullfile(cmReleaseBase, relCandidates{k});
            if isfolder(p)
                addpath(p);
                if exist('cmread', 'file') == 3
                    break;
                end
            end
        end
    end

    if exist('cmread', 'file') ~= 3
        return;  % cmread 못 찾음 → fallback
    end

    try
        rawIn = cmread(ergFile);
        % cmread는 struct of struct ({.data, .unit}) 형태로 반환할 수 있음
        % 단순 vector struct로 정규화
        fn = fieldnames(rawIn);
        raw = struct();
        for i = 1:numel(fn)
            v = rawIn.(fn{i});
            if isstruct(v) && isfield(v, 'data')
                raw.(fn{i}) = v.data(:);
            elseif isnumeric(v)
                raw.(fn{i}) = v(:);
            end
        end
    catch ME
        warning('[cm_read_erg] cmread call failed (%s) — falling back to native parser.', ...
                ME.message);
        raw = [];
    end
end

%% ============================================================
function raw = local_parse_erg_native(ergFile)
% .erg.info 기반 자체 바이너리 파서
%
% 포맷:
%   - 16-byte header ('CM-ERG\0\0' 매직 + 8-byte version/flags)
%   - 레코드: 각 채널 (Double=8B / Float/Int=4B / "4 Bytes"=4B) 순차
%   - LittleEndian

    infoFile = [ergFile '.info'];
    if ~isfile(infoFile)
        error('[cm_read_erg:native] Companion .info not found: %s', infoFile);
    end

    info = cm_parse_infofile(infoFile);

    % byte order
    bo = local_get_str(info, 'File_ByteOrder', 'LittleEndian');
    if strcmpi(bo, 'BigEndian')
        endian = 'b';
    else
        endian = 'l';
    end

    % 채널 메타 (Name, Type) 순서대로 수집
    channels = local_collect_channels(info);
    if isempty(channels)
        error('[cm_read_erg:native] No channel definitions found in %s', infoFile);
    end

    % 레코드 크기 계산 + 각 채널 오프셋 산출 (struct에 즉시 기록)
    recSize = 0;
    for i = 1:numel(channels)
        channels(i).off = recSize;
        recSize = recSize + channels(i).bytes;
    end

    % 파일 크기로 레코드 수 추정 (header 후보: 16 bytes 기본)
    finfo = dir(ergFile);
    fileBytes = finfo.bytes;
    headerSize = 16;
    bodyBytes  = fileBytes - headerSize;
    if mod(bodyBytes, recSize) ~= 0
        % header 후보 8 / 32 / 0 으로 재시도
        for hs = [8, 32, 0]
            if mod(fileBytes - hs, recSize) == 0 && (fileBytes - hs) > 0
                headerSize = hs; bodyBytes = fileBytes - hs;
                break;
            end
        end
    end
    if mod(bodyBytes, recSize) ~= 0
        error('[cm_read_erg:native] Cannot align record size %d to file (file=%d, header_try=%d).', ...
               recSize, fileBytes, headerSize);
    end
    nRec = bodyBytes / recSize;

    % 전체 본문을 uint8로 한 번에 읽음
    fid = fopen(ergFile, 'r', endian);
    if fid < 0
        error('[cm_read_erg:native] Cannot open %s', ergFile);
    end
    cleanup = onCleanup(@() fclose(fid));
    fseek(fid, headerSize, 'bof');
    buf = fread(fid, bodyBytes, '*uint8');

    if numel(buf) ~= bodyBytes
        error('[cm_read_erg:native] Read short: %d vs %d', numel(buf), bodyBytes);
    end

    % 채널별로 typecast 추출 (column extraction via reshape)
    raw = struct();
    matBuf = reshape(buf, recSize, nRec).';  % nRec × recSize uint8

    for i = 1:numel(channels)
        ch = channels(i);
        if isempty(ch.fld)
            continue;  % $none$ 같은 placeholder 스킵
        end
        colBytes = matBuf(:, ch.off+1 : ch.off+ch.bytes);
        flat = reshape(colBytes.', [], 1);  % bytes in record order

        switch ch.cast
            case 'double'
                vec = typecast(flat, 'double');
            case 'single'
                vec = double(typecast(flat, 'single'));
            case 'int32'
                vec = double(typecast(flat, 'int32'));
            case 'skip'
                continue;
            otherwise
                continue;
        end

        % 엔디안 보정: typecast는 시스템 엔디안. 시스템과 다르면 swap.
        if (endian == 'b') ~= local_is_big_endian()
            vec = swapbytes(vec);
        end

        raw.(ch.fld) = vec;
    end
end

%% --------------------------------------------------------------
function chs = local_collect_channels(info)
% File.At.<N>.Name / Type 메타를 N 순서대로 정렬
    chs = struct('idx',{},'name',{},'type',{},'bytes',{},'cast',{},'fld',{},'off',{});
    fn = fieldnames(info);
    pat = '^File_At_(\d+)_Name$';
    for i = 1:numel(fn)
        tok = regexp(fn{i}, pat, 'tokens', 'once');
        if isempty(tok)
            continue;
        end
        idx = str2double(tok{1});
        nameKey = sprintf('File_At_%d_Name', idx);
        typeKey = sprintf('File_At_%d_Type', idx);
        if ~isfield(info, typeKey)
            continue;
        end
        nameRaw = strtrim(local_to_char(info.(nameKey)));
        typeRaw = strtrim(local_to_char(info.(typeKey)));

        % type → bytes / cast
        switch typeRaw
            case 'Double'
                bytes = 8; cast = 'double';
            case 'Float'
                bytes = 4; cast = 'single';
            case 'Int'
                bytes = 4; cast = 'int32';
            case '4 Bytes'
                bytes = 4; cast = 'skip';
            case '8 Bytes'
                bytes = 8; cast = 'skip';
            otherwise
                bytes = 4; cast = 'skip';  % 모르는 타입은 skip + 4B 가정
        end

        % $none$ 같은 placeholder는 fld 비움
        fld = '';
        if ~strcmp(nameRaw, '$none$') && ~isempty(nameRaw)
            fld = local_name_to_field(nameRaw);
        end

        c.idx = idx; c.name = nameRaw; c.type = typeRaw;
        c.bytes = bytes; c.cast = cast; c.fld = fld; c.off = 0;
        chs(end+1) = c; %#ok<AGROW>
    end

    if isempty(chs)
        return;
    end

    [~, ord] = sort([chs.idx]);
    chs = chs(ord);
end

%% --------------------------------------------------------------
function s = local_get_str(info, fld, def)
    if isfield(info, fld) && ischar(info.(fld))
        s = strtrim(info.(fld));
    elseif isfield(info, fld) && isnumeric(info.(fld))
        s = num2str(info.(fld));
    else
        s = def;
    end
end

function s = local_to_char(v)
    if ischar(v)
        s = v;
    elseif isnumeric(v)
        s = num2str(v);
    elseif iscell(v) && ~isempty(v)
        s = v{1};
        if ~ischar(s); s = num2str(s); end
    else
        s = '';
    end
end

function fn = local_name_to_field(nameRaw)
% 'Car.WFL.rot' → 'Car_WFL_rot'
    fn = regexprep(nameRaw, '[^A-Za-z0-9_]', '_');
    if isempty(fn)
        fn = 'x';
    end
    if ~isstrprop(fn(1), 'alpha') && fn(1) ~= '_'
        fn = ['x' fn];
    end
end

function tf = local_is_big_endian()
    [~,~,ne] = computer;
    tf = (ne == 'B');
end
