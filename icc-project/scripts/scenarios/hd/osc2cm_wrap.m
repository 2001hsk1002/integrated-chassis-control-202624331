function result = osc2cm_wrap(xoscFile, outDir, scnId)
%OSC2CM_WRAP CarMaker osc2cm CLI wrapper — .xosc + .xodr → TestRun/.rd5/InfoFiles
%
%   result = OSC2CM_WRAP(xoscFile, outDir, scnId)
%
%   1. outDir 안에 임시 CM project tree (Data/Vehicle, Data/Road, Data/TestRun) 생성
%   2. BMW_5 InfoFile을 Data/Vehicle로 복사
%   3. osc2cm.exe 호출 — validation + conversion
%
%   result struct:
%     .validated     true if osc2cm schema/feature validation passed
%     .converted     true if Road5 / TestRun 파일 생성 성공
%     .trPath        TestRun 파일 경로 (converted=true일 때)
%     .log           전체 osc2cm log
%
%   참고: OpenDRIVE → Road5 변환은 IPGRoad 라이선스 필요. 라이선스 없으면
%   validation까지만 성공 (즉 .xosc 자체는 OSC 1.3 schema 준수 확인됨).

    cmExe = 'C:\IPG\carmaker\win64-15.0\bin\osc2cm.exe';
    assert(isfile(cmExe), '[osc2cm_wrap] osc2cm not found: %s', cmExe);

    cmProj = fullfile(outDir, 'cm_project');
    local_mk_cm_project(cmProj);

    cmd = sprintf(['"%s" -p "%s" -o "%s" -e Car1 -i BMW_5_15_030326 ' ...
                   '--validate --oscversion 130 --logtoconsole --loglevel 4'], ...
                  cmExe, cmProj, xoscFile);
    [status, log] = system(cmd);

    result = struct();
    result.log       = log;
    result.validated = contains(log, 'Validation succeeded with 0 errors');
    result.trPath    = fullfile(cmProj, 'Data', 'TestRun', scnId);
    result.converted = (status == 0) && isfile(result.trPath);

    if result.converted
        fprintf('  ✓ osc2cm: validated + converted → %s\n', result.trPath);
    elseif result.validated
        if contains(log, 'IPGRoad license error')
            fprintf('  ⚠ osc2cm: validated; conversion blocked by IPGRoad license\n');
        else
            fprintf('  ⚠ osc2cm: validated but conversion failed (status=%d)\n', status);
        end
    else
        error('[osc2cm_wrap] OSC validation failed.\nLog:\n%s', log);
    end
end

%% ------------------------------------------------------------
function local_mk_cm_project(cmProj)
% CarMaker project skeleton 생성 + BMW_5 InfoFile 복사

    subdirs = {'Data','Data/Vehicle','Data/Road','Data/TestRun','Data/Tire','SimInput'};
    for k = 1:numel(subdirs)
        d = fullfile(cmProj, subdirs{k});
        if ~exist(d,'dir'); mkdir(d); end
    end

    % BMW_5 InfoFile 복사 (catalog reference가 'BMW_5_15_030326' 가리킴)
    bmwSrc = 'C:\Users\VIC\Projects\carmaker_data\Data\Vehicle\BMW_5_15_030326';
    bmwDst = fullfile(cmProj, 'Data', 'Vehicle', 'BMW_5_15_030326');
    if isfile(bmwSrc) && ~isfile(bmwDst)
        copyfile(bmwSrc, bmwDst);
    end
end
