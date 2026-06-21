function util_plot_cm_compare(cmp, kpi, scenarioName, figPath)
%UTIL_PLOT_CM_COMPARE CarMaker vs 14-DOF Plant 오버레이 비교 그림
%
%   util_plot_cm_compare(cmp, kpi, scenarioName, figPath)
%
%   Inputs:
%       cmp - struct: t, vx_cm, vx_sim, ax_cm, ax_sim, pitch_cm, pitch_sim, brakeTorq
%       kpi - struct: vx_rms_err, vx_max_err, ax_rms_err, ax_max_err,
%                     pitch_rms_err, pitch_max_err, stopDist_*
%       scenarioName - (char) 시나리오 라벨
%       figPath - (char) 저장 경로 (.png)

    fig = figure('Position', [100 100 1100 800], 'Visible', 'off');

    cm_col  = [0.10 0.35 0.85];   % CarMaker = blue
    sim_col = [0.85 0.20 0.20];   % Plant 14-DOF = red

    %% 1. Longitudinal velocity
    subplot(2, 2, 1);
    plot(cmp.t, cmp.vx_cm, '-', 'Color', cm_col, 'LineWidth', 1.5); hold on;
    plot(cmp.t, cmp.vx_sim, '--', 'Color', sim_col, 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('v_x [m/s]');
    title(sprintf('Longitudinal Velocity (RMS err = %.3f m/s)', kpi.vx_rms_err));
    legend({'CarMaker', '14-DOF'}, 'Location', 'best');

    %% 2. Longitudinal acceleration
    subplot(2, 2, 2);
    plot(cmp.t, cmp.ax_cm, '-', 'Color', cm_col, 'LineWidth', 1.5); hold on;
    plot(cmp.t, cmp.ax_sim, '--', 'Color', sim_col, 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('a_x [m/s^2]');
    title(sprintf('Longitudinal Acceleration (RMS err = %.3f m/s^2)', kpi.ax_rms_err));
    legend({'CarMaker', '14-DOF'}, 'Location', 'best');

    %% 3. Pitch angle
    subplot(2, 2, 3);
    plot(cmp.t, rad2deg(cmp.pitch_cm), '-', 'Color', cm_col, 'LineWidth', 1.5); hold on;
    plot(cmp.t, rad2deg(cmp.pitch_sim), '--', 'Color', sim_col, 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('Pitch [deg]');
    title(sprintf('Pitch Angle (RMS err = %.3f deg)', kpi.pitch_rms_err));
    legend({'CarMaker', '14-DOF'}, 'Location', 'best');

    %% 4. Brake torque (input)
    subplot(2, 2, 4);
    plot(cmp.t, cmp.brakeTorq, '-', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('Brake Torque Total [Nm]');
    title(sprintf('Brake Input (stopDist err = %.2f%%)', kpi.stopDist_err_pct));

    %% 전체 제목
    sgtitle(sprintf('CarMaker vs 14-DOF Plant — %s', scenarioName), ...
            'FontWeight', 'bold', 'Interpreter', 'none');

    %% 저장
    if ~exist(fileparts(figPath), 'dir')
        mkdir(fileparts(figPath));
    end
    exportgraphics(fig, figPath, 'Resolution', 150);
    close(fig);
end
