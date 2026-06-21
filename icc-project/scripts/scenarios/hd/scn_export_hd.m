function paths = scn_export_hd(scenario, outDir, opts)
%SCN_EXPORT_HD HD scenario asset bundle export
%
%   paths = SCN_EXPORT_HD(scenario, outDir, opts)
%
%   1. scn_export_osc로 baseline (.xosc 1.3 + .xodr + .crg + manifest) 생성
%   2. scn_hd_patch로 HD 자산 in-place inject:
%        .xodr ← lane material (split-μ), superelevation (banking), objects (cones, barriers)
%        .xosc ← EnvironmentAction (weather), VehicleCatalog (BMW_5) reference
%   3. osc2cm_wrap으로 CarMaker TestRun/.rd5/InfoFiles 변환
%
%   사용 예:
%       scn = scenario_dispatcher('B3', SIM);
%       paths = scn_export_hd(scn, 'data/scenarios_export/B3_HD');

    if nargin < 2 || isempty(outDir)
        outDir = fullfile(pwd, 'data', 'scenarios_export', [scenario.id '_HD']);
    end
    if ~exist(outDir, 'dir'); mkdir(outDir); end
    if nargin < 3; opts = struct(); end

    fprintf('\n[scn_export_hd] %s (weather=%s)\n', scenario.id, scenario.weather.name);

    %% 1. Baseline OSC 1.3 + OpenDRIVE 1.6 + OpenCRG (재사용)
    paths = scn_export_osc(scenario, outDir, opts);

    %% 2. HD 자산 in-place patch
    scn_hd_patch(paths.xosc, paths.xodr, scenario);

    %% 3. CarMaker 변환 (validation + conversion)
    paths.osc2cm = osc2cm_wrap(paths.xosc, outDir, scenario.id);
end
