%VDB_DEBUG_BRAKE BrkPrs=0 vs BrkPrs=big 비교로 입력 실효성 검증

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));
run(fullfile(projectRoot,'scripts','utils','init_project.m'));

sc = scenario_dispatcher('B1', SIM);
sc.tEnd = 2.5;

% Case A: brake off
scA = sc; scA.brakeCmd = @(t) zeros(4,1);
fprintf('\n=== Case A: BrkPrs = 0 ===\n');
resA = vdb_run_sim(scA);
fprintf('  vx: %.2f → %.2f m/s\n', resA.vx(1), resA.vx(end));
fprintf('  ax peak: %.2f m/s²\n', max(abs(resA.ax)));

% Case B: brake on (our default)
fprintf('\n=== Case B: BrkPrs = ~9.4 MPa @ t>=1s ===\n');
resB = vdb_run_sim(sc);
fprintf('  vx: %.2f → %.2f m/s\n', resB.vx(1), resB.vx(end));
fprintf('  ax peak: %.2f m/s²\n', max(abs(resB.ax)));

% Case C: brake very large
scC = sc;
scC.brakeCmd = @(t) (t>=1.0) * [50000;50000;30000;30000];   % huge torque → 50000/15.94 ≈ 3137 bar
fprintf('\n=== Case C: BrkPrs HUGE (T=50000 Nm/wheel → ~3137 bar) ===\n');
resC = vdb_run_sim(scC);
fprintf('  vx: %.2f → %.2f m/s\n', resC.vx(1), resC.vx(end));
fprintf('  ax peak: %.2f m/s²\n', max(abs(resC.ax)));

% Case D: brake in BAR units (not Pa) — test alternative unit
% Modify scenario to bypass internal Pa conversion - we'd need to edit vdb_run_sim
% For now, just compare time series of vx
fprintf('\n=== vx evolution comparison ===\n');
fprintf('Time(s)  | vx_A (brake=0) | vx_B (Pa)  | vx_C (HUGE Pa)\n');
for t = [0 0.5 1.0 1.5 2.0]
    [~,iA] = min(abs(resA.t - t));
    [~,iB] = min(abs(resB.t - t));
    [~,iC] = min(abs(resC.t - t));
    fprintf('  %.1f   |   %6.2f       |   %6.2f   |   %6.2f\n', ...
        t, resA.vx(iA), resB.vx(iB), resC.vx(iC));
end

save(fullfile(projectRoot,'data','vdb_brake_debug.mat'),'resA','resB','resC');
