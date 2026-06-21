function util_plot_3way_compare(t_mfile, mfile, t_vdb, vdb, cm, scenarioName, figPath)
%UTIL_PLOT_3WAY_COMPARE M-file ↔ VDB ↔ CarMaker 3-way overlay
%
%   mfile, vdb : struct with fields .vx, .ay, .yawRate, .slipAngle
%   cm         : (옵션) CarMaker reference, 같은 필드. empty면 2-way plot

    fig = figure('Position',[80 80 1200 800], 'Visible','off','Color','w');
    mcol = [0.85 0.20 0.20];
    vcol = [0.10 0.30 0.85];
    ccol = [0.10 0.10 0.10];

    %% vx
    subplot(2,2,1); hold on; grid on;
    plot(t_mfile, mfile.vx, '--', 'Color', mcol, 'LineWidth', 1.6);
    plot(t_vdb,   vdb.vx,   '-',  'Color', vcol, 'LineWidth', 1.6);
    legendItems = {'M-file 14-DOF', 'VDB 14-DOF'};
    if ~isempty(cm) && isfield(cm,'vx')
        plot(cm.t, cm.vx, '-', 'Color', ccol, 'LineWidth', 1.4);
        legendItems{end+1} = 'CarMaker';
    end
    xlabel('Time [s]'); ylabel('v_x [m/s]'); title('Longitudinal velocity');
    legend(legendItems, 'Location','best');

    %% ay
    subplot(2,2,2); hold on; grid on;
    plot(t_mfile, mfile.ay, '--', 'Color', mcol, 'LineWidth', 1.6);
    plot(t_vdb,   vdb.ay,   '-',  'Color', vcol, 'LineWidth', 1.6);
    if ~isempty(cm) && isfield(cm,'ay'); plot(cm.t, cm.ay, '-', 'Color', ccol, 'LineWidth', 1.4); end
    xlabel('Time [s]'); ylabel('a_y [m/s^2]'); title('Lateral acceleration');

    %% yaw rate
    subplot(2,2,3); hold on; grid on;
    plot(t_mfile, rad2deg(mfile.yawRate), '--', 'Color', mcol, 'LineWidth', 1.6);
    plot(t_vdb,   rad2deg(vdb.yawRate),   '-',  'Color', vcol, 'LineWidth', 1.6);
    if ~isempty(cm) && isfield(cm,'yawRate'); plot(cm.t, rad2deg(cm.yawRate), '-', 'Color', ccol, 'LineWidth', 1.4); end
    xlabel('Time [s]'); ylabel('Yaw rate [deg/s]'); title('Yaw rate');

    %% slip angle
    subplot(2,2,4); hold on; grid on;
    plot(t_mfile, rad2deg(mfile.slipAngle), '--', 'Color', mcol, 'LineWidth', 1.6);
    plot(t_vdb,   rad2deg(vdb.slipAngle),   '-',  'Color', vcol, 'LineWidth', 1.6);
    if ~isempty(cm) && isfield(cm,'slipAngle'); plot(cm.t, rad2deg(cm.slipAngle), '-', 'Color', ccol, 'LineWidth', 1.4); end
    xlabel('Time [s]'); ylabel('\beta [deg]'); title('Side-slip');

    sgtitle(sprintf('3-way comparison — %s', scenarioName), 'FontWeight','bold','FontSize',13);

    if ~exist(fileparts(figPath),'dir'); mkdir(fileparts(figPath)); end
    exportgraphics(fig, figPath, 'Resolution', 150);
    close(fig);
end
