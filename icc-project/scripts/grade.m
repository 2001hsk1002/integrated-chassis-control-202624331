%GRADE 자동 채점기 — ICC 제어기 설계 과제
%
%   학생이 작성한 ctrl_*.m 을 P1 시나리오 (A1/A3/A4/A7/B1/D1) 에 적용해
%   베이스라인 (제어기 OFF) 대비 KPI 개선을 측정 후 100점 만점 점수 산출.
%
%   산출: grade_report.json (PR 코멘트용)
%   종료 코드: 0 (성공) / 1 (학생 코드 runtime error)
%
%   채점 매트릭스는 ASSIGNMENT.md §5.1 참조.

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
addpath(thisDir);
run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));
cd(projectRoot);

% ----------------------------------------------------------------
% 학생 정보
% ----------------------------------------------------------------
try
    sinfo = student_info();
catch
    sinfo = struct('student_id','UNKNOWN','name','UNKNOWN');
end

fprintf('\n=================================================================\n');
fprintf('  ICC Term Project — Automated Grader\n');
fprintf('  Student: %s (%s)\n', sinfo.name, sinfo.student_id);
fprintf('  Timestamp: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf('=================================================================\n');

% ----------------------------------------------------------------
% 채점 매트릭스 — ASSIGNMENT.md §5.1
% ----------------------------------------------------------------
%   columns: target value, tolerance (±%), max score, direction ('lt'/'gt'/'within')
%
%   compute_score:
%     'lt':     KPI ≤ target → 만점; KPI ≥ target·(1+tol/100) → 0점; linear in between
%     'gt':     반대
%     'within': |KPI - target| ≤ target·tol/100 → 만점, 벗어나면 linear 감점
matrix = { ...
%   sid  KPI                     target    tol%   max   dir
    'A3','yawRateOvershoot',     10.0,     100,   4,   'lt'; ...
    'A3','yawRateRiseTime',       0.30,    100,   4,   'lt'; ...
    'A3','yawRateSettling',       0.80,    100,   4,   'lt'; ...
    'A1','sideSlipMax',           3.0,     100,   6,   'lt'; ...
    'A1','LTR_max',               0.60,    100,   5,   'lt'; ...
    'A1','lateralDevMax',         0.70,    100,   4,   'lt'; ...
    'A4','understeerGradient',    0.003,    80,   5,   'within'; ...
    'A4','sideSlipMax',           2.0,     100,   5,   'lt'; ...
    'A7','sideSlipMax',           5.0,     100,   8,   'lt'; ...
    'A7','LTR_max',               0.70,    100,   7,   'lt'; ...
    'B1','stoppingDistance',     40.0,      50,   5,   'lt'; ...
    'B1','absSlipRMS',            0.10,    150,   5,   'lt'; ...
    'D1','sideSlipMax',           4.0,     100,   4,   'lt'; ...
    'D1','LTR_max',               0.60,    100,   2,   'lt'; ...
    'D1','lateralDevMax',         1.0,     100,   2,   'lt'};

% ----------------------------------------------------------------
% Run benchmark — OFF + ON
% ----------------------------------------------------------------
ids = unique(matrix(:,1), 'stable');
ON  = struct(); OFF = struct(); errs = {};

for k = 1:numel(ids)
    sid = ids{k};
    fprintf('\n----- %s -----\n', sid);
    try
        [~, k_off] = run_icc_scenario(sid, '14dof', 'Controller','off','SavePlot',false);
        [~, k_on ] = run_icc_scenario(sid, '14dof', 'Controller','on', 'SavePlot',false);
        OFF.(sid) = k_off;
        ON.(sid)  = k_on;
    catch ME
        fprintf('  ✗ runtime error: %s\n', ME.message);
        errs{end+1} = sprintf('%s: %s', sid, ME.message);
        OFF.(sid) = struct(); ON.(sid) = struct();
    end
end

% ----------------------------------------------------------------
% Score
% ----------------------------------------------------------------
totalScore = 0; maxScore = 0;
breakdown = {};

fprintf('\n========================= Scoring =========================\n');
fprintf('%-4s | %-22s | %10s | %10s | %5s / %5s\n', 'sid', 'KPI', 'value', 'target', 'score', 'max');
fprintf('%s\n', repmat('-', 1, 78));
for i = 1:size(matrix, 1)
    sid = matrix{i, 1}; kname = matrix{i, 2};
    tgt = matrix{i, 3}; tol = matrix{i, 4};
    mx  = matrix{i, 5}; dir = matrix{i, 6};
    maxScore = maxScore + mx;

    if ~isfield(ON, sid) || ~isfield(ON.(sid), kname) || ~isfield(OFF, sid) || ~isfield(OFF.(sid), kname)
        score = 0; val = NaN;
    else
        val_on  = ON.(sid).(kname);
        val_off = OFF.(sid).(kname);
        if ~isnumeric(val_on) || isnan(val_on)
            score = 0; val = NaN;
        else
            score = local_score(val_on, val_off, tgt, tol, dir, mx);
            val = val_on;
        end
    end
    totalScore = totalScore + score;
    breakdown(end+1, :) = {sid, kname, val, tgt, score, mx}; %#ok<SAGROW>

    if ~isnan(val)
        fprintf('%-4s | %-22s | %10.4f | %10.4f | %5.2f / %5d\n', sid, kname, val, tgt, score, mx);
    else
        fprintf('%-4s | %-22s | %10s | %10.4f | %5.2f / %5d\n', sid, kname, 'N/A', tgt, score, mx);
    end
end

% Quantitative score is 70/100 (per ASSIGNMENT §5.1)
% Normalize: maxScore (currently 70) → 70 points
quantPct = 100 * totalScore / maxScore;
quantPts = 0.70 * quantPct;

% ----------------------------------------------------------------
% Deductions
% ----------------------------------------------------------------
deductions = 0; deductReasons = {};
if contains(sinfo.student_id, 'TODO') || strcmp(sinfo.student_id, 'UNKNOWN')
    deductions = deductions + 5;
    deductReasons{end+1} = '-5: student_info.m 미기입';
end
if ~isempty(errs)
    deductReasons{end+1} = sprintf('runtime errors (자동채점 quantitative 부분 0점 처리): %d 시나리오', numel(errs));
end

% ----------------------------------------------------------------
% Final
% ----------------------------------------------------------------
fprintf('\n========================= Summary =========================\n');
fprintf('Quantitative:  %.2f / 70.00  (%.1f %%)\n', quantPts, quantPct);
fprintf('Deductions:    -%d\n', deductions);
fprintf('Manual (TBD):  / 30.00  (보고서 평가 — 채점자가 직접 매김)\n');
fprintf('-----------------------------------------------------------\n');
fprintf('Auto-graded total: %.2f / 70.00 (보고서 평가 추가 시 max 100)\n', max(0, quantPts - deductions));

% ----------------------------------------------------------------
% JSON report
% ----------------------------------------------------------------
report = struct();
report.student      = sinfo;
report.timestamp    = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
report.quantitative = struct('score', quantPts, 'max', 70, 'pct', quantPct);
report.deductions   = struct('amount', deductions, 'reasons', {deductReasons});
report.errors       = errs;
report.breakdown    = local_breakdown_to_struct(breakdown);
report.final_auto   = max(0, quantPts - deductions);

% Phase D' integrity / reproducibility metadata
try
    report.ctrl_signature = util_ctrl_signature(projectRoot);
catch ME
    report.ctrl_signature = '';
    warning('grade:signature', 'ctrl_signature 계산 실패: %s', ME.message);
end
v = ver('MATLAB');
if ~isempty(v)
    report.matlab_version = sprintf('%s %s', v(1).Version, v(1).Release);
else
    report.matlab_version = version();
end
if exist('SIM','var') && isstruct(SIM) && isfield(SIM,'solver')
    report.solver_used = SIM.solver;
else
    report.solver_used = 'unknown';
end

jsonStr = jsonencode(report, 'PrettyPrint', true);
outFile = fullfile(projectRoot, 'grade_report.json');
fid = fopen(outFile, 'w'); fwrite(fid, jsonStr); fclose(fid);
fprintf('\nSaved: %s\n', outFile);

% Exit code 0 if no runtime errors, 1 otherwise (for CI status)
if ~isempty(errs)
    exit(1);
end

%% ============================================================
function score = local_score(val_on, val_off, target, tolPct, direction, maxPts)
% 베이스라인 보다 악화 시 0점. 만점 조건은 ASSIGNMENT 매트릭스. 그 사이는 linear.
    if ~isfinite(val_on) || ~isfinite(val_off)
        score = 0; return;
    end

    % If didn't improve over baseline → 0
    switch direction
        case 'lt'
            if val_on >= val_off; score = 0; return; end
            ub = target * (1 + tolPct/100);
            if val_on <= target;     score = maxPts;
            elseif val_on >= ub;     score = 0;
            else                     score = maxPts * (ub - val_on) / (ub - target);
            end
        case 'gt'
            if val_on <= val_off; score = 0; return; end
            lb = target * (1 - tolPct/100);
            if val_on >= target;     score = maxPts;
            elseif val_on <= lb;     score = 0;
            else                     score = maxPts * (val_on - lb) / (target - lb);
            end
        case 'within'
            err = abs(val_on - target);
            band = abs(target) * tolPct/100;
            if err <= band;          score = maxPts;
            elseif err >= 2*band;    score = 0;
            else                     score = maxPts * (2*band - err) / band;
            end
        otherwise
            score = 0;
    end
end

function s = local_breakdown_to_struct(br)
    s = struct('sid',{},'kpi',{},'value',{},'target',{},'score',{},'max',{});
    for i = 1:size(br,1)
        s(i).sid    = br{i,1};
        s(i).kpi    = br{i,2};
        s(i).value  = br{i,3};
        s(i).target = br{i,4};
        s(i).score  = br{i,5};
        s(i).max    = br{i,6};
    end
end
