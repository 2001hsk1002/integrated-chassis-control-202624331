function sig = util_ctrl_signature(projectRoot)
%UTIL_CTRL_SIGNATURE SHA256 of concatenated ctrl_*.m files (CRLF-normalized).
%
%   sig = UTIL_CTRL_SIGNATURE(projectRoot)
%
%   학생/TA/GitHub Actions 가 OS 무관하게 동일 hash 재현하도록 설계됨.
%   Bash equivalent:
%       cat ctrl_lateral.m ctrl_longitudinal.m ctrl_vertical.m ctrl_coordinator.m \
%         | tr -d '\r' | sha256sum
%
%   파일 4개 순서 (고정): lateral → longitudinal → vertical → coordinator
%
%   Inputs:
%       projectRoot - (optional) icc-project 루트 절대경로. 생략 시 본 파일 위치에서 추정.
%
%   Outputs:
%       sig - 64자 lowercase hex SHA256 문자열

    if nargin < 1 || isempty(projectRoot)
        thisFile = mfilename('fullpath');
        projectRoot = fileparts(fileparts(fileparts(thisFile)));
    end

    files = {
        fullfile(projectRoot, 'scripts', 'control', 'ctrl_lateral.m')
        fullfile(projectRoot, 'scripts', 'control', 'ctrl_longitudinal.m')
        fullfile(projectRoot, 'scripts', 'control', 'ctrl_vertical.m')
        fullfile(projectRoot, 'scripts', 'control', 'ctrl_coordinator.m')
    };

    bytes = uint8([]);
    for i = 1:numel(files)
        fid = fopen(files{i}, 'rb');
        if fid < 0
            error('util_ctrl_signature:fopen', 'Cannot open %s', files{i});
        end
        b = fread(fid, inf, 'uint8=>uint8');
        fclose(fid);
        bytes = [bytes; b];                                                %#ok<AGROW>
    end

    bytes = bytes(bytes ~= uint8(13));   % CRLF → LF (strip CR)

    md = java.security.MessageDigest.getInstance('SHA-256');
    md.update(bytes);
    hashBytes = typecast(md.digest(), 'uint8');
    sig = lower(sprintf('%02x', hashBytes));
end
