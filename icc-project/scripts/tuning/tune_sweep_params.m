function sweepResult = tune_sweep_params(paramName, paramRange, simFunc, evalFunc)
%TUNE_SWEEP_PARAMS 단일 파라미터 스윕 및 성능 비교
%
%   sweepResult = TUNE_SWEEP_PARAMS(paramName, paramRange, simFunc, evalFunc)
%
%   지정된 파라미터를 범위 내에서 변화시키며 시뮬레이션을 반복 실행하고
%   성능 지표를 비교한다.
%
%   Inputs:
%       paramName  - (char) 파라미터 이름 (표시용)
%       paramRange - (1xN double) 탐색 범위
%       simFunc    - (function_handle) 시뮬레이션 함수
%                    signature: result = simFunc(paramValue)
%       evalFunc   - (function_handle) 평가 함수
%                    signature: score = evalFunc(result)
%
%   Outputs:
%       sweepResult - (struct) 스윕 결과
%           .paramName  - 파라미터 이름
%           .values     - 파라미터 값
%           .scores     - 각 값의 평가 점수
%           .bestValue  - 최적 파라미터 값
%           .bestScore  - 최적 점수
%
%   Example:
%       simFcn  = @(Kp) run_step_steer_sim(Kp);
%       evalFcn = @(res) rms(res.yawRateError);
%       result  = tune_sweep_params('Kp', 0.5:0.5:5.0, simFcn, evalFcn);

    nPoints = numel(paramRange);
    scores  = zeros(1, nPoints);

    fprintf('[tune_sweep_params] Sweeping %s: %d points [%.3f ~ %.3f]\n', ...
        paramName, nPoints, paramRange(1), paramRange(end));

    for i = 1:nPoints
        val = paramRange(i);
        fprintf('  [%d/%d] %s = %.4f ... ', i, nPoints, paramName, val);
        try
            result = simFunc(val);
            scores(i) = evalFunc(result);
            fprintf('score = %.4f\n', scores(i));
        catch ME
            scores(i) = Inf;
            fprintf('ERROR: %s\n', ME.message);
        end
    end

    % 최적값 선택
    [bestScore, bestIdx] = min(scores);
    bestValue = paramRange(bestIdx);

    sweepResult.paramName = paramName;
    sweepResult.values    = paramRange;
    sweepResult.scores    = scores;
    sweepResult.bestValue = bestValue;
    sweepResult.bestScore = bestScore;

    fprintf('[tune_sweep_params] Best %s = %.4f (score = %.4f)\n', ...
        paramName, bestValue, bestScore);

    % 시각화
    figure('Name', ['Sweep: ' paramName]);
    plot(paramRange, scores, 'bo-', 'LineWidth', 1.5, 'MarkerSize', 8);
    hold on;
    plot(bestValue, bestScore, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
    xlabel(paramName); ylabel('Score (lower = better)');
    title(['Parameter Sweep: ' paramName]);
    grid on; legend('All', 'Best', 'Location', 'best');

end
