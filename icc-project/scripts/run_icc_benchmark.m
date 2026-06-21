%RUN_ICC_BENCHMARK ICC 제어기 ON/OFF benchmark — P1 시나리오 (A1, A3, A4, A7, B1, D1)
%
%   각 시나리오를 같은 plant (14DOF) + 같은 driver model로 두 번 실행:
%       (1) Controller='off' — baseline (운전자 입력만)
%       (2) Controller='on'  — ICC 제어기 (AFS+ESC+CDC+Coordinator) 활성
%   주요 KPI 의 delta% 표 출력. ICC 의 성능 개선을 정량적으로 확인.

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);
run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));
cd(projectRoot);

% P1 시나리오 — Phase 1+2로 검증 가능한 핵심 6종
ids = {'A1','A3','A4','A7','B1','D1'};

fprintf('\n=================================================================\n');
fprintf('      ICC Controller Benchmark — P1 scenarios (Controller off vs on)\n');
fprintf('=================================================================\n');

results = struct();

for k = 1:numel(ids)
    sid = ids{k};
    fprintf('\n----- %s -----', sid);
    try
        [~, k_off] = run_icc_scenario(sid, '14dof', 'Controller','off', 'SavePlot',false);
        [~, k_on ] = run_icc_scenario(sid, '14dof', 'Controller','on',  'SavePlot',false);
        results.(sid).off = k_off;
        results.(sid).on  = k_on;
    catch ME
        fprintf('\n  ✗ scenario %s failed: %s\n', sid, ME.message);
        results.(sid).off = struct();
        results.(sid).on  = struct();
    end
end

% ---------- 결과표 ----------
fprintf('\n\n========================= KPI Comparison =========================\n');
metrics = {'sideSlipMax','LTR_max','tireUtilizationMax', ...
           'lateralDevMax','stoppingDistance','yawRateOvershoot','jerkMax'};
header = sprintf('%-6s | %-22s | %10s | %10s | %8s', 'scn', 'KPI', 'OFF', 'ON', 'delta%');
fprintf('%s\n', header);
fprintf('%s\n', repmat('-', 1, numel(header)));

for k = 1:numel(ids)
    sid = ids{k};
    if ~isfield(results, sid); continue; end
    r = results.(sid);
    for m = 1:numel(metrics)
        mn = metrics{m};
        if isfield(r.off, mn) && isfield(r.on, mn)
            v_off = r.off.(mn);
            v_on  = r.on.(mn);
            if isnumeric(v_off) && isnumeric(v_on) && ~isnan(v_off) && ~isnan(v_on)
                if abs(v_off) > 1e-9
                    delta = 100 * (v_on - v_off) / abs(v_off);
                else
                    delta = NaN;
                end
                if isnan(delta)
                    dstr = '  n/a  ';
                else
                    dstr = sprintf('%+7.1f%%', delta);
                end
                fprintf('%-6s | %-22s | %10.4f | %10.4f | %s\n', ...
                    sid, mn, v_off, v_on, dstr);
            end
        end
    end
    fprintf('%s\n', repmat('-', 1, numel(header)));
end

% ---------- 저장 ----------
outDir = fullfile(projectRoot, 'data', 'scenarios');
if ~exist(outDir,'dir'); mkdir(outDir); end
saveFile = fullfile(outDir, sprintf('benchmark_P1_%s.mat', datestr(now,'yyyymmdd_HHMMSS')));
save(saveFile, 'results');
fprintf('\nSaved: %s\n', saveFile);
fprintf('======================= Benchmark done =======================\n');
