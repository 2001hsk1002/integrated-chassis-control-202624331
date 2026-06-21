function util_plot_cm_lateral(cmp, kpi, scenarioName, figPath)
%UTIL_PLOT_CM_LATERAL CarMaker vs 14-DOF 횡방향 오버레이 (steer, yawRate, ay, slip, roll)

    fig = figure('Position', [100 100 1200 850], 'Visible', 'off');
    cm_col  = [0.10 0.35 0.85];
    sim_col = [0.85 0.20 0.20];

    %% 1. Steering input
    subplot(2, 3, 1);
    plot(cmp.t, rad2deg(cmp.steer), '-', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('\delta [deg]');
    title('Steering (input)');

    %% 2. Yaw rate
    subplot(2, 3, 2);
    plot(cmp.t, rad2deg(cmp.yr_cm), '-', 'Color', cm_col, 'LineWidth', 1.5); hold on;
    plot(cmp.t, rad2deg(cmp.yr_sim), '--', 'Color', sim_col, 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('Yaw Rate [deg/s]');
    title(sprintf('Yaw Rate (RMS=%.2f deg/s)', kpi.yawRate_rms_err_deg));
    legend({'CarMaker', '14-DOF'}, 'Location', 'best');

    %% 3. Lateral accel
    subplot(2, 3, 3);
    plot(cmp.t, cmp.ay_cm,  '-', 'Color', cm_col, 'LineWidth', 1.5); hold on;
    plot(cmp.t, cmp.ay_sim, '--', 'Color', sim_col, 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('a_y [m/s^2]');
    title(sprintf('Lateral Accel (RMS=%.2f m/s^2)', kpi.ay_rms_err));

    %% 4. Side-slip
    subplot(2, 3, 4);
    plot(cmp.t, rad2deg(cmp.slip_cm),  '-', 'Color', cm_col, 'LineWidth', 1.5); hold on;
    plot(cmp.t, rad2deg(cmp.slip_sim), '--', 'Color', sim_col, 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('\beta [deg]');
    title(sprintf('Side-slip (RMS=%.2f deg)', kpi.slip_rms_err_deg));

    %% 5. Roll
    subplot(2, 3, 5);
    plot(cmp.t, rad2deg(cmp.roll_cm),  '-', 'Color', cm_col, 'LineWidth', 1.5); hold on;
    plot(cmp.t, rad2deg(cmp.roll_sim), '--', 'Color', sim_col, 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('\phi [deg]');
    title(sprintf('Roll (RMS=%.2f deg)', kpi.roll_rms_err_deg));

    %% 6. Speed (CM enforced)
    subplot(2, 3, 6);
    plot(cmp.t, cmp.vx, '-', 'Color', [0.5 0.3 0.7], 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('v_x [m/s]');
    title('Speed (CM, enforced on plant)');

    sgtitle(sprintf('CarMaker vs 14-DOF Lateral — %s', scenarioName), ...
            'FontWeight', 'bold', 'Interpreter', 'none');

    if ~exist(fileparts(figPath), 'dir')
        mkdir(fileparts(figPath));
    end
    exportgraphics(fig, figPath, 'Resolution', 150);
    close(fig);
end
