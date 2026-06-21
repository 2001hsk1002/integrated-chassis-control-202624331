function result = cm_run_testrun(cm, testRunName, opts)
%CM_RUN_TESTRUN CarMaker TestRun 실행 및 결과 수집
%
%   result = CM_RUN_TESTRUN(cm, testRunName)
%   result = CM_RUN_TESTRUN(cm, testRunName, opts)
%
%   TestRun을 로드하고 시뮬레이션을 실행한 뒤 결과를 수집한다.
%
%   Inputs:
%       cm          - (tcpclient) TCP 클라이언트
%       testRunName - (char) TestRun 이름
%       opts        - (struct, optional) 옵션
%           .timeout         - 시뮬레이션 타임아웃 [s] (default: 120)
%           .collectResults  - 결과 자동 수집 여부 (default: true)
%           .simOutputDir    - SimOutput 디렉토리 경로
%
%   Outputs:
%       result - (struct) 시뮬레이션 결과 (.data, .success, .elapsed)

    arguments
        cm
        testRunName (1,:) char
        opts.timeout (1,1) double = 120
        opts.collectResults (1,1) logical = true
        opts.simOutputDir (1,:) char = 'SimOutput'
    end

    result.testRun = testRunName;
    result.success = false;
    result.data    = [];
    result.elapsed = 0;

    t0 = tic;
    fprintf('[cm_run_testrun] === %s ===\n', testRunName);

    %% TestRun 로드
    resp = cm_send(cm, ['LoadTestRun ' testRunName]);
    if contains(resp, 'Error')
        warning('[cm_run_testrun] LoadTestRun failed: %s', resp);
        return;
    end
    pause(1);
    fprintf('[cm_run_testrun] TestRun loaded.\n');

    %% 시뮬레이션 시작
    resp = cm_send(cm, 'StartSim');
    if contains(resp, 'Error')
        warning('[cm_run_testrun] StartSim failed: %s', resp);
        return;
    end
    fprintf('[cm_run_testrun] Simulation started...\n');

    %% 완료 대기
    success = cm_wait_idle(cm, opts.timeout);
    result.elapsed = toc(t0);

    if ~success
        warning('[cm_run_testrun] Simulation did not complete.');
        return;
    end

    result.success = true;

    %% 결과 수집
    if opts.collectResults
        ergFile = fullfile(opts.simOutputDir, [testRunName '.erg']);
        if isfile(ergFile)
            result.data = cm_read_results(ergFile);
            fprintf('[cm_run_testrun] Results collected. Elapsed: %.1f s\n', result.elapsed);
        else
            warning('[cm_run_testrun] Result file not found: %s', ergFile);
        end
    end

end
