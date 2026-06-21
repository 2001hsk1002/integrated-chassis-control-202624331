function paths = scn_export_osc(scenario, outDir, opts)
%SCN_EXPORT_OSC 우리 scenario → OpenSCENARIO 1.1 + OpenDRIVE 1.6 export (CarMaker / esmini / RoadRunner 호환)
%
%   paths = SCN_EXPORT_OSC(scenario, outDir, opts)
%
%   파일 산출:
%     <outDir>/<scenario.id>.xosc     ASAM OpenSCENARIO 1.1
%     <outDir>/<scenario.id>.xodr     ASAM OpenDRIVE 1.6
%
%   호환:
%     - **CarMaker** 8.0+: TestRun → Open Scenario Import (.xosc + .xodr)
%     - **esmini**: 오픈소스 OSC player — esmini --osc <file.xosc>
%     - **RoadRunner**: File → Import → OpenSCENARIO
%     - **MATLAB ADT**: drivingScenarioDesigner 에서 Import

    if nargin < 2 || isempty(outDir)
        outDir = fullfile(pwd, 'data', 'scenarios_export');
    end
    if ~exist(outDir,'dir'); mkdir(outDir); end
    if nargin < 3; opts = struct(); end

    %% 1. drivingScenario object 빌드
    ds = scn_to_drivingScenario(scenario, opts);

    %% 2. OpenDRIVE 1.6 export (도로 geometry)
    paths.xodr = fullfile(outDir, [scenario.id '.xodr']);
    try
        export(ds, "OpenDRIVE", paths.xodr, "OpenDRIVEVersion", 1.6);
        fprintf('  ✓ OpenDRIVE: %s\n', paths.xodr);
    catch ME
        fprintf('  ✗ OpenDRIVE export failed: %s\n', ME.message);
        paths.xodr = '';
    end

    %% 3. OpenSCENARIO 1.1 export (driver actions)
    %     MATLAB ADT R2024b는 1.0/1.1만 export 가능. CarMaker 15는 1.0~1.3을 모두 import 하므로 OK.
    %     HD export 단계 (scn_hd_patch)에서 1.3 으로 version bump.
    paths.xosc = fullfile(outDir, [scenario.id '.xosc']);
    try
        export(ds, "OpenSCENARIO", paths.xosc, "OpenSCENARIOVersion", 1.1);
        fprintf('  ✓ OpenSCENARIO: %s\n', paths.xosc);
    catch ME
        fprintf('  ✗ OpenSCENARIO export failed: %s\n', ME.message);
        paths.xosc = '';
    end

    %% 4. OpenSCENARIO 2.0 DSL export (esmini/Foretify 등 OSC 2.0 도구용. CM은 OSC 1.x만 지원)
    paths.osc2 = fullfile(outDir, [scenario.id '.osc']);
    try
        scn_export_osc2(scenario, outDir, opts);
        fprintf('  ✓ OpenSCENARIO 2.0 (.osc DSL): %s\n', paths.osc2);
    catch ME
        fprintf('  ✗ OSC 2.0 export failed: %s\n', ME.message);
        paths.osc2 = '';
    end

    %% 5. Surface (OpenCRG) — z_road / mu_wheel이 nontrivial인 경우만
    paths.crg = '';
    if isfield(scenario,'z_road') && ~isempty(scenario.z_road)
        sampleT = linspace(0, scenario.tEnd, 100);
        zSamples = arrayfun(@(t) scenario.z_road(t, 1), sampleT);
        if max(abs(zSamples)) > 1e-4
            paths.crg = scn_export_opencrg(scenario, outDir, opts);
            fprintf('  ✓ OpenCRG (surface): %s\n', paths.crg);
        end
    end

    %% 6. RoadFrictionAction overlay — split-μ (좌/우 비대칭) 시나리오에만 emit
    %     균질 μ (weather scale 등)는 EnvironmentAction.RoadCondition으로 표현 → 별도 overlay 불필요
    paths.frictionXml = '';
    if isfield(scenario,'mu_wheel') && ~isempty(scenario.mu_wheel)
        mu_FL = scenario.mu_wheel(0, 1);
        mu_FR = scenario.mu_wheel(0, 2);
        mu_RL = scenario.mu_wheel(0, 3);
        mu_RR = scenario.mu_wheel(0, 4);
        if abs(mu_FL - mu_FR) > 1e-3 || abs(mu_RL - mu_RR) > 1e-3
            paths.frictionXml = fullfile(outDir, [scenario.id '_friction.xml']);
            local_write_friction_action(scenario, paths.frictionXml);
            fprintf('  ✓ RoadFrictionAction XML: %s (μ_L=%.2f, μ_R=%.2f)\n', ...
                paths.frictionXml, mu_FL, mu_FR);
        end
    end

    %% 5. Manifest
    paths.manifest = fullfile(outDir, [scenario.id '_manifest.txt']);
    fid = fopen(paths.manifest, 'w');
    fprintf(fid, '# Scenario Export Manifest\n');
    fprintf(fid, '# Scenario ID: %s\n', scenario.id);
    fprintf(fid, '# Name: %s\n', scenario.name);
    fprintf(fid, '# Standard: %s\n', scenario.refStandard);
    fprintf(fid, '# Export Time: %s\n\n', datestr(now));
    fprintf(fid, 'OpenDRIVE 1.6:        %s\n', paths.xodr);
    fprintf(fid, 'OpenSCENARIO 1.1 XML: %s\n', paths.xosc);
    fprintf(fid, 'OpenSCENARIO 2.0 DSL: %s\n', paths.osc2);
    fprintf(fid, 'OpenCRG (surface):    %s\n', paths.crg);
    fprintf(fid, '\n# Usage:\n');
    fprintf(fid, '#   CarMaker 13/15:  Tools → TestManager → Import OpenSCENARIO\n');
    fprintf(fid, '#                    - 1.x: use %s + %s\n', paths.xosc, paths.xodr);
    fprintf(fid, '#                    - 2.0: use %s (DSL native, CM 13/15+)\n', paths.osc2);
    fprintf(fid, '#                    - For 3D surface, attach %s (.crg)\n', paths.crg);
    fprintf(fid, '#   esmini (OSS):    esmini --osc %s --window 800 600\n', paths.xosc);
    fprintf(fid, '#   RoadRunner:      File → Import → OpenSCENARIO (XOSC)\n');
    fprintf(fid, '#   MATLAB ADT:      drivingScenarioDesigner; importScenario(''%s'')\n', paths.xosc);
    fclose(fid);
    fprintf('  ✓ Manifest: %s\n', paths.manifest);
end

%% --------------------------------------------------------------
function local_write_friction_action(scenario, xmlPath)
% RoadFrictionAction을 별도 OSC XML로 작성 (CarMaker, RoadRunner가 attach 가능)
% OpenSCENARIO 1.1 EnvironmentAction.RoadCondition.frictionScaleFactor — global
% Lane-별 차별 friction은 OpenDRIVE Object Material로 확장 (여기는 global 표현)
    mu_FL = scenario.mu_wheel(0, 1);
    mu_FR = scenario.mu_wheel(0, 2);
    fid = fopen(xmlPath, 'w');
    fprintf(fid, '<?xml version="1.0" encoding="UTF-8"?>\n');
    fprintf(fid, '<!-- RoadFrictionAction overlay for %s -->\n', scenario.id);
    fprintf(fid, '<!-- Left side μ=%.3f, Right side μ=%.3f -->\n', mu_FL, mu_FR);
    fprintf(fid, '<EnvironmentAction>\n');
    fprintf(fid, '  <Environment>\n');
    fprintf(fid, '    <RoadCondition frictionScaleFactor="%.3f">\n', mean([mu_FL, mu_FR]));
    fprintf(fid, '      <Properties>\n');
    fprintf(fid, '        <Property name="leftSideMu"  value="%.3f"/>\n', mu_FL);
    fprintf(fid, '        <Property name="rightSideMu" value="%.3f"/>\n', mu_FR);
    fprintf(fid, '        <Property name="splitMode"   value="lateral"/>\n');
    fprintf(fid, '      </Properties>\n');
    fprintf(fid, '    </RoadCondition>\n');
    fprintf(fid, '  </Environment>\n');
    fprintf(fid, '</EnvironmentAction>\n');
    fclose(fid);
end
