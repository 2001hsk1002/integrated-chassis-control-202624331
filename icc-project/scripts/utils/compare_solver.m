function [results, summary] = compare_solver(varargin)
%COMPARE_SOLVER 여러 plant 적분기의 KPI 결과 비교
%
%   [results, summary] = COMPARE_SOLVER(Name, Value, ...)
%
%   Options:
%       'Scenarios' - cell array of scenario IDs (default: {'A3','A1','A4','A7','B1','D1'})
%       'Plant'     - plant model (default: '14dof')
%       'Solvers'   - cell array of solver names (default: {'ode45','rk4','ode15s'})
%       'Controller'- 'on' | 'off' (default: 'on')
%       'Verbose'   - true | false (default: true)
%
%   Outputs:
%       results.(sid).(solver) = KPI struct (run_icc_scenario 의 두 번째 출력)
%       summary                = solver 간 KPI delta 표 (table)
%
%   학생/강의자 모두 사용 가능. 기본 ode45 와 다른 solver 의 KPI 차이를 보여
%   solver 선택이 채점/안정성에 미치는 영향을 정량 확인하기 위한 도구.
%
%   Example:
%       results = compare_solver();                              % 기본 비교
%       compare_solver('Scenarios', {'A3','B1'}, 'Plant','7dof');  % 일부만

    %% 옵션 파싱
    p = inputParser();
    p.addParameter('Scenarios', {'A3','A1','A4','A7','B1','D1'}, @iscell);
    p.addParameter('Plant',      '14dof', @ischar);
    p.addParameter('Solvers',    {'ode45','rk4','ode15s'}, @iscell);
    p.addParameter('Controller', 'on', @(s) any(strcmp(s, {'on','off'})));
    p.addParameter('Verbose',    true, @islogical);
    p.parse(varargin{:});
    opt = p.Results;

    %% 환경 초기화
    thisDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(fileparts(thisDir));
    run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));

    %% 각 (scenario, solver) 조합 실행
    results = struct();
    nSid = numel(opt.Scenarios);
    nSlv = numel(opt.Solvers);

    if opt.Verbose
        fprintf('\n=================================================================\n');
        fprintf('  compare_solver — Plant=%s, Controller=%s\n', opt.Plant, opt.Controller);
        fprintf('  Scenarios: %s\n', strjoin(opt.Scenarios, ', '));
        fprintf('  Solvers:   %s\n', strjoin(opt.Solvers, ', '));
        fprintf('=================================================================\n');
    end

    for is = 1:nSlv
        slv = opt.Solvers{is};

        for ks = 1:nSid
            sid = opt.Scenarios{ks};
            try
                tic;
                [~, kpi] = run_icc_scenario(sid, opt.Plant, ...
                    'Controller', opt.Controller, 'SavePlot', false, ...
                    'Solver', slv);
                tElapsed = toc;
                kpi.wallTime = tElapsed;
                results.(sid).(slv) = kpi;
                if opt.Verbose
                    fprintf('  [%s / %s] OK  (%.2fs)\n', slv, sid, tElapsed);
                end
            catch ME
                results.(sid).(slv) = struct('error', ME.message);
                if opt.Verbose
                    fprintf('  [%s / %s] FAIL: %s\n', slv, sid, ME.message);
                end
            end
        end
    end

    %% Summary table — base solver (첫번째) 대비 delta
    baseSlv = opt.Solvers{1};
    rows = {};
    for ks = 1:nSid
        sid = opt.Scenarios{ks};
        if ~isfield(results, sid); continue; end
        baseKpi = results.(sid).(baseSlv);
        if ~isstruct(baseKpi) || isfield(baseKpi, 'error'); continue; end
        kpiNames = fieldnames(baseKpi);
        for kk = 1:numel(kpiNames)
            kn = kpiNames{kk};
            baseVal = baseKpi.(kn);
            if ~isnumeric(baseVal) || ~isscalar(baseVal); continue; end
            row.scenario = sid;
            row.kpi      = kn;
            row.(baseSlv) = baseVal;
            for is = 2:nSlv
                slv = opt.Solvers{is};
                if isfield(results.(sid), slv) && isfield(results.(sid).(slv), kn)
                    v = results.(sid).(slv).(kn);
                    row.(slv) = v;
                    if abs(baseVal) > 1e-9
                        row.([slv '_pctDelta']) = (v - baseVal) / abs(baseVal) * 100;
                    else
                        row.([slv '_pctDelta']) = NaN;
                    end
                else
                    row.(slv) = NaN;
                    row.([slv '_pctDelta']) = NaN;
                end
            end
            rows{end+1} = row;  %#ok<AGROW>
        end
    end

    if ~isempty(rows)
        summary = struct2table([rows{:}]);
        if opt.Verbose
            fprintf('\n----- KPI delta (vs %s) -----\n', baseSlv);
            disp(summary);
        end
    else
        summary = table();
        if opt.Verbose
            fprintf('\n(no comparable KPI rows)\n');
        end
    end
end
