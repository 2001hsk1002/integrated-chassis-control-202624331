function summary = cm_batch_run(cm, scenarioList, opts)
%CM_BATCH_RUN 다수 시나리오 배치 시뮬레이션 실행
%
%   summary = CM_BATCH_RUN(cm, scenarioList)
%   summary = CM_BATCH_RUN(cm, scenarioList, opts)
%
%   여러 TestRun을 순차 실행하고 KPI를 일괄 계산한다.
%
%   Inputs:
%       cm           - (tcpclient) CarMaker TCP 클라이언트
%       scenarioList - (cell) TestRun 이름 목록 (e.g., {'DLC_80','Slalom_60'})
%       opts         - (struct, optional)
%           .timeout      - 시나리오당 타임아웃 [s] (default: 120)
%           .saveResults  - 결과 저장 여부 (default: true)
%           .outputDir    - 결과 저장 디렉토리 (default: 'data')
%
%   Outputs:
%       summary - (struct) 배치 실행 요약
%           .scenarios - (cell) 시나리오 이름 목록
%           .results   - (cell) 각 시나리오 결과 구조체
%           .kpis      - (cell) 각 시나리오 KPI
%           .verdicts  - (cell) 각 시나리오 판정
%           .table     - (table) 요약 테이블

    arguments
        cm
        scenarioList (:,1) cell
        opts.timeout (1,1) double = 120
        opts.saveResults (1,1) logical = true
        opts.outputDir (1,:) char = 'data'
    end

    nScen = numel(scenarioList);
    fprintf('\n========================================\n');
    fprintf(' ICC Batch Simulation: %d scenarios\n', nScen);
    fprintf('========================================\n\n');

    summary.scenarios = scenarioList;
    summary.results   = cell(nScen, 1);
    summary.kpis      = cell(nScen, 1);
    summary.verdicts  = cell(nScen, 1);

    tBatch = tic;

    for i = 1:nScen
        name = scenarioList{i};
        fprintf('[%d/%d] %s ... ', i, nScen, name);

        try
            % 시뮬레이션 실행
            res = cm_run_testrun(cm, name, 'timeout', opts.timeout);

            if res.success && ~isempty(res.data)
                % KPI 계산
                kpi = util_calc_kpi(res.data);
                summary.results{i}  = res.data;
                summary.kpis{i}     = kpi;
                summary.verdicts{i} = kpi.overall;
                fprintf('=> %s (%.1f s)\n', kpi.overall, res.elapsed);
            else
                summary.verdicts{i} = 'ERROR';
                fprintf('=> FAILED\n');
            end

        catch ME
            summary.verdicts{i} = 'ERROR';
            fprintf('=> ERROR: %s\n', ME.message);
        end
    end

    %% 요약 테이블 생성
    scenCol   = scenarioList;
    verdictCol = summary.verdicts;
    summary.table = table(scenCol, verdictCol, ...
        'VariableNames', {'Scenario', 'Verdict'});

    batchTime = toc(tBatch);
    fprintf('\n========================================\n');
    fprintf(' Batch Complete: %.1f s\n', batchTime);
    fprintf('========================================\n');
    disp(summary.table);

    %% 결과 저장
    if opts.saveResults
        if ~isfolder(opts.outputDir), mkdir(opts.outputDir); end
        saveFile = fullfile(opts.outputDir, ...
            sprintf('batch_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
        save(saveFile, 'summary');
        fprintf('[cm_batch_run] Saved: %s\n', saveFile);
    end

end
