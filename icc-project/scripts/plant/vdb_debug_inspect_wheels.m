%VDB_DEBUG_INSPECT_WHEELS Wheels bus 신호 전체 dump + Brake/Torque 필터
%
%   B1 시나리오 (직진 제동)을 VDB로 실행한 후 result.raw.Wheels bus의 모든
%   signal name + 데이터 범위 출력. BrkPrs ↔ BrkTrq 관계 디버그용.

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));
run(fullfile(projectRoot,'scripts','utils','init_project.m'));

sc = scenario_dispatcher('B1', SIM);
sc.tEnd = 2.0;
result = vdb_run_sim(sc);

if ~isfield(result.raw,'Wheels')
    fprintf('No Wheels bus in result.raw\n'); return;
end

fprintf('\nWheels top-level: %s\n', strjoin(fieldnames(result.raw.Wheels)', ', '));
flat = local_walk(result.raw.Wheels, '', struct());
sigs = fieldnames(flat);
fprintf('\nTotal Wheels signals: %d\n', numel(sigs));
fprintf('\n=== Brake / Torque / Pressure signals ===\n');
for k = 1:numel(sigs)
    nm = sigs{k};
    if contains(nm,'Brk','IgnoreCase',true) || contains(nm,'Trq','IgnoreCase',true) || ...
       contains(nm,'Pres','IgnoreCase',true) || contains(nm,'Tq','IgnoreCase',true)
        ts = flat.(nm);
        if isa(ts,'timeseries')
            d = ts.Data; if isvector(d); d = d(:); end
            fprintf('  %-55s sz=%-12s  range=[%.3g, %.3g]\n', ...
                nm, mat2str(size(d)), min(d(:)), max(d(:)));
        end
    end
end

fprintf('\n=== Sample of all wheel signals ===\n');
for k = 1:min(40, numel(sigs))
    nm = sigs{k};
    ts = flat.(nm);
    if isa(ts,'timeseries')
        d = ts.Data; if isvector(d); d = d(:); end
        fprintf('  %-55s sz=%-12s  range=[%.3g, %.3g]\n', ...
            nm, mat2str(size(d)), min(d(:)), max(d(:)));
    end
end

%% Also report what we sent in
fprintf('\n=== Our input ===\n');
fprintf('  scenario B1 brake @ t=1s+ = [Tf Tf Tr Tr] = [1500 1500 800 800] Nm/wheel\n');
fprintf('  converted to BrkPrs (Pa) sent to VDB:\n');
fprintf('    Front: 1500/15.94 = %.1f bar = %.2e Pa\n', 1500/15.94, 1500/15.94*1e5);
fprintf('    Rear:   800/8.02  = %.1f bar = %.2e Pa\n', 800/8.02,  800/8.02*1e5);

%% Save full Wheels for inspection
outFile = fullfile(projectRoot,'data','vdb_wheels_inspect.mat');
save(outFile, 'flat', 'result');
fprintf('\nSaved full flat sig dict: %s\n', outFile);

function flat = local_walk(node, prefix, flat)
    fn = fieldnames(node);
    for k = 1:numel(fn)
        nm = fn{k};
        full = [prefix, nm];
        v = node.(nm);
        if isa(v, 'timeseries')
            flat.(full) = v;
        elseif isstruct(v)
            flat = local_walk(v, [full '_'], flat);
        end
    end
end
