%TEST_HD_EXPORT A1–D1 시나리오 HD scenario asset bundle 산출 + 검증
%
%   각 시나리오에 대해:
%     1. scenario_dispatcher(id, SIM, weather) → scenario struct
%     2. scn_export_hd → .xosc/.xodr/.crg (+ HD patch) + osc2cm 변환 .tr
%     3. 결과 XML 검증 (version=1.3, 필요 요소 존재)
%     4. .tr 산출 확인

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));
run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));

addpath(fullfile(projectRoot, 'scripts', 'scenarios', 'hd'));
addpath(fullfile(projectRoot, 'scripts', 'scenarios', 'export'));
cd(projectRoot);

outRoot = fullfile(projectRoot, 'data', 'scenarios_export');

% (id, weather variants 적용 여부)
%   B3·C1은 시나리오 자체가 μ를 정의 → weather variant 의미 없음 → dry만
%   나머지는 dry + wet 두 가지
plan = { ...
    'A1', {'dry','wet'}; ...
    'A3', {'dry','wet'}; ...
    'A4', {'dry','wet'}; ...
    'A5', {'dry','wet'}; ...
    'A7', {'dry','wet'}; ...
    'B1', {'dry','wet','snow'}; ...
    'B3', {'dry'}; ...
    'C1', {'dry'}; ...
    'D1', {'dry','wet'} };

fprintf('\n========== HD Scenario Export Test ==========\n');
results = {};

for k = 1:size(plan,1)
    sid = plan{k,1};
    variants = plan{k,2};
    for v = 1:numel(variants)
        weather = variants{v};
        bundleId = sprintf('%s_%s', sid, weather);
        outDir = fullfile(outRoot, [bundleId '_HD']);

        fprintf('\n----- %s (%s) -----\n', sid, weather);
        scn = scenario_dispatcher(sid, SIM, weather);
        paths = scn_export_hd(scn, outDir);

        % 검증
        chk = local_verify(paths, scn);
        results(end+1, :) = {bundleId, chk}; %#ok<SAGROW>
    end
end

% 결과표
fprintf('\n========== Verification Summary ==========\n');
fprintf('%-12s | xosc1.3 | OD obj | OD mat | banking | CRG | weather | osc2cm valid | TestRun\n', 'bundle');
fprintf('%s\n', repmat('-', 1, 95));
for k = 1:size(results,1)
    bid = results{k,1};
    c = results{k,2};
    fprintf('%-12s |   %s    |   %s    |   %s    |   %s    |  %s  |   %s    |      %s       |    %s\n', ...
        bid, ...
        check2sym(c.xoscVersion13), check2sym(c.hasObjects), check2sym(c.hasLaneMaterial), ...
        check2sym(c.hasBanking),    check2sym(c.hasCrg),     check2sym(c.hasWeather), ...
        check2sym(c.osc2cmValidated), check2sym(c.osc2cmConverted));
end

fprintf('\n========== HD Export done ==========\n');

%% ========================================================
function chk = local_verify(paths, scn)
    chk = struct('xoscVersion13',false, 'hasObjects',false, 'hasLaneMaterial',false, ...
                 'hasBanking',false, 'hasCrg',false, 'hasWeather',false, ...
                 'osc2cmValidated',false, 'osc2cmConverted',false);
    if isfile(paths.xosc)
        dom = xmlread(paths.xosc);
        rev = dom.getDocumentElement().getElementsByTagName('FileHeader');
        if rev.getLength() > 0
            mj = char(rev.item(0).getAttribute('revMajor'));
            mn = char(rev.item(0).getAttribute('revMinor'));
            chk.xoscVersion13 = strcmp(mj,'1') && strcmp(mn,'3');
        end
        if ~strcmp(scn.weather.name,'dry')
            chk.hasWeather = dom.getElementsByTagName('EnvironmentAction').getLength() > 0;
        else
            chk.hasWeather = true;
        end
    end
    if isfile(paths.xodr)
        dom = xmlread(paths.xodr);
        chk.hasObjects      = dom.getElementsByTagName('object').getLength() > 0;
        chk.hasLaneMaterial = dom.getElementsByTagName('material').getLength() > 0;
        seNodes = dom.getElementsByTagName('superelevation');
        for k = 0:seNodes.getLength()-1
            a = str2double(char(seNodes.item(k).getAttribute('a')));
            if abs(a) > 1e-6; chk.hasBanking = true; break; end
        end
    end
    chk.hasCrg = isfield(paths,'crg') && ~isempty(paths.crg) && isfile(paths.crg);
    if isfield(paths,'osc2cm') && isstruct(paths.osc2cm)
        chk.osc2cmValidated = paths.osc2cm.validated;
        chk.osc2cmConverted = paths.osc2cm.converted;
    end
end

function s = check2sym(b)
    if b; s = '✓'; else; s = ' '; end
end
