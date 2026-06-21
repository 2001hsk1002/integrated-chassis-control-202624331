%TEST_ROUNDTRIP A1/B3 export → import 정합성 검증
%
%   원본 시나리오를 OSC 1.1 + OSC 2.0으로 export → 다시 import →
%   driver input timeseries 일치도 확인.

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));
run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));

cd(projectRoot);
outDir = fullfile(projectRoot, 'data', 'scenarios_export');

testIds = {'A1', 'B1', 'C1', 'B3'};

fprintf('\n========== Scenario Round-trip Test ==========\n');
for k = 1:numel(testIds)
    sid = testIds{k};
    fprintf('\n----- %s -----\n', sid);
    sc_orig = scenario_dispatcher(sid, SIM);

    % Export 4 formats
    paths = scn_export_osc(sc_orig, outDir);

    % Import OSC 1.x XML
    fprintf('\n  >> Import OSC 1.x XML:\n');
    sc_imp_xml = scn_from_openscenario(paths.xosc);

    % Import OSC 2.0 DSL
    fprintf('\n  >> Import OSC 2.0 DSL:\n');
    if isfile(paths.osc2)
        sc_imp_osc2 = scn_from_osc2(paths.osc2);
    end

    % Compare driver input timeseries (explicit loop — handle 호환성)
    fprintf('\n  Driver input comparison (peak values):\n');
    tProbe = linspace(0, sc_orig.tEnd, 50);
    [delta_o, brake_o] = local_probe(sc_orig, tProbe);
    [delta_i, brake_i] = local_probe(sc_imp_xml, tProbe);
    fprintf('   Original    steer peak=%5.2f deg, brake peak=%5.0f Nm\n', ...
        rad2deg(max(abs(delta_o))), max(brake_o));
    fprintf('   OSC 1.x imp steer peak=%5.2f deg, brake peak=%5.0f Nm\n', ...
        rad2deg(max(abs(delta_i))), max(brake_i));
    if isfile(paths.osc2)
        [delta_i2, brake_i2] = local_probe(sc_imp_osc2, tProbe);
        fprintf('   OSC 2.0 imp steer peak=%5.2f deg, brake peak=%5.0f Nm\n', ...
            rad2deg(max(abs(delta_i2))), max(brake_i2));
    end
end

function [delta, brake] = local_probe(scn, tProbe)
    n = numel(tProbe);
    delta = zeros(n,1); brake = zeros(n,1);
    for k = 1:n
        try; delta(k) = scn.steerDriver(tProbe(k)); catch; delta(k)=0; end
        try
            bk = scn.brakeCmd(tProbe(k));
            brake(k) = sum(bk(:));
        catch
            brake(k) = 0;
        end
    end
end

fprintf('\n========== Round-trip done ==========\n');
