function reportPath = util_export_report(result, kpi, scenario, opts)
%UTIL_EXPORT_REPORT KPI 판정 결과를 CSV/텍스트 보고서로 내보내기
%
%   reportPath = UTIL_EXPORT_REPORT(result, kpi, scenario)
%   reportPath = UTIL_EXPORT_REPORT(result, kpi, scenario, opts)
%
%   Inputs:
%       result   - (struct) 시뮬레이션 결과
%       kpi      - (struct) KPI 계산 결과 (util_calc_kpi 출력)
%       scenario - (char) 시나리오 이름
%       opts     - (struct, optional)
%           .outputDir - 출력 디렉토리 (default: 'data')
%           .format    - 'csv' 또는 'txt' (default: 'csv')
%
%   Outputs:
%       reportPath - (char) 생성된 보고서 파일 경로

    arguments
        result struct
        kpi struct
        scenario (1,:) char
        opts.outputDir (1,:) char = 'data'
        opts.format (1,:) char = 'csv'
    end

    if ~isfolder(opts.outputDir), mkdir(opts.outputDir); end
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');

    %% KPI 테이블 생성
    kpiNames  = {};
    kpiValues = [];
    kpiVerdicts = {};

    excludeFields = {'verdicts', 'overall'};
    fields = fieldnames(kpi);
    for i = 1:numel(fields)
        fn = fields{i};
        if any(strcmp(fn, excludeFields)), continue; end
        if ~isnumeric(kpi.(fn)), continue; end
        kpiNames{end+1,1} = fn; %#ok<AGROW>
        kpiValues(end+1,1) = kpi.(fn); %#ok<AGROW>
        if isfield(kpi.verdicts, fn)
            kpiVerdicts{end+1,1} = kpi.verdicts.(fn); %#ok<AGROW>
        else
            kpiVerdicts{end+1,1} = 'N/A'; %#ok<AGROW>
        end
    end

    T = table(kpiNames, kpiValues, kpiVerdicts, ...
        'VariableNames', {'KPI', 'Value', 'Verdict'});

    %% 파일 출력
    switch opts.format
        case 'csv'
            reportPath = fullfile(opts.outputDir, ...
                sprintf('kpi_%s_%s.csv', scenario, timestamp));
            writetable(T, reportPath);

        case 'txt'
            reportPath = fullfile(opts.outputDir, ...
                sprintf('report_%s_%s.txt', scenario, timestamp));
            fid = fopen(reportPath, 'w');
            fprintf(fid, '=== ICC Safety Verification Report ===\n');
            fprintf(fid, 'Scenario : %s\n', scenario);
            fprintf(fid, 'Date     : %s\n', datestr(now));
            fprintf(fid, 'Duration : %.1f s\n', result.time(end));
            fprintf(fid, 'Samples  : %d\n\n', numel(result.time));
            fprintf(fid, '--- KPI Results ---\n');
            fprintf(fid, '%-25s %10s %10s\n', 'KPI', 'Value', 'Verdict');
            fprintf(fid, '%s\n', repmat('-', 1, 47));
            for i = 1:numel(kpiNames)
                fprintf(fid, '%-25s %10.3f %10s\n', ...
                    kpiNames{i}, kpiValues(i), kpiVerdicts{i});
            end
            fprintf(fid, '\n--- Overall Verdict: %s ---\n', kpi.overall);
            fclose(fid);
    end

    fprintf('[util_export_report] Report saved: %s\n', reportPath);

end
