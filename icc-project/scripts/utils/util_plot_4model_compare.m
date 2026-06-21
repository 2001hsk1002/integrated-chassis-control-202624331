function util_plot_4model_compare(longResults, latResults, figPath)
%UTIL_PLOT_4MODEL_COMPARE 4개 plant 모델 비교 그림 (Longitudinal + Lateral)
%
%   longResults: struct array with fields {model, t, vx_cm, vx_sim, ax_cm, ax_sim, pitch_cm, pitch_sim}
%   latResults : struct array with fields {model, t, yr_cm, yr_sim, ay_cm, ay_sim, slip_cm, slip_sim}
%   figPath    : output PNG (절대경로)

    cmColor = [0.10 0.10 0.10];
    modelColors = struct('bicycle',[0.6 0.6 0.6], 'x3dof',[0.20 0.60 0.20], ...
                         'x7dof',[0.20 0.40 0.85], 'x14dof',[0.85 0.20 0.20]);
    modelLabels = {'bicycle','3-DOF','7-DOF','14-DOF'};
    fieldKeys   = {'bicycle','x3dof','x7dof','x14dof'};

    fig = figure('Position', [60 40 1500 950], 'Visible', 'off');

    %% Longitudinal — vx
    subplot(2, 3, 1); hold on; grid on;
    if ~isempty(longResults)
        plot(longResults(1).t, longResults(1).vx_cm, '-', 'Color', cmColor, 'LineWidth', 2.0);
        for i=1:numel(longResults)
            R = longResults(i);
            col = modelColors.(fieldKeys{strcmp(modelLabels, R.model)});
            plot(R.t, R.vx_sim, '-', 'Color', col, 'LineWidth', 1.2);
        end
    end
    xlabel('Time [s]'); ylabel('v_x [m/s]');
    title('Longitudinal velocity (LK_CCIR_ST AEB)','Interpreter','none');
    legend(['CarMaker' modelLabels(arrayfun(@(R) any(strcmp(R.model,modelLabels)), longResults))], 'Location','best');

    %% Longitudinal — ax
    subplot(2, 3, 2); hold on; grid on;
    if ~isempty(longResults)
        plot(longResults(1).t, longResults(1).ax_cm, '-', 'Color', cmColor, 'LineWidth', 2.0);
        for i=1:numel(longResults)
            R = longResults(i);
            col = modelColors.(fieldKeys{strcmp(modelLabels, R.model)});
            plot(R.t, R.ax_sim, '-', 'Color', col, 'LineWidth', 1.2);
        end
    end
    xlabel('Time [s]'); ylabel('a_x [m/s^2]');
    title('Longitudinal acceleration');

    %% Longitudinal — pitch
    subplot(2, 3, 3); hold on; grid on;
    if ~isempty(longResults)
        plot(longResults(1).t, rad2deg(longResults(1).pitch_cm), '-', 'Color', cmColor, 'LineWidth', 2.0);
        for i=1:numel(longResults)
            R = longResults(i);
            col = modelColors.(fieldKeys{strcmp(modelLabels, R.model)});
            plot(R.t, rad2deg(R.pitch_sim), '-', 'Color', col, 'LineWidth', 1.2);
        end
    end
    xlabel('Time [s]'); ylabel('Pitch [deg]');
    title('Pitch angle');

    %% Lateral — yaw rate
    subplot(2, 3, 4); hold on; grid on;
    if ~isempty(latResults)
        plot(latResults(1).t, rad2deg(latResults(1).yr_cm), '-', 'Color', cmColor, 'LineWidth', 2.0);
        for i=1:numel(latResults)
            R = latResults(i);
            col = modelColors.(fieldKeys{strcmp(modelLabels, R.model)});
            plot(R.t, rad2deg(R.yr_sim), '-', 'Color', col, 'LineWidth', 1.2);
        end
    end
    xlabel('Time [s]'); ylabel('Yaw Rate [deg/s]');
    title('Yaw rate (RT_CGSL2R turning)','Interpreter','none');
    legend(['CarMaker' modelLabels(arrayfun(@(R) any(strcmp(R.model,modelLabels)), latResults))], 'Location','best');

    %% Lateral — ay
    subplot(2, 3, 5); hold on; grid on;
    if ~isempty(latResults)
        plot(latResults(1).t, latResults(1).ay_cm, '-', 'Color', cmColor, 'LineWidth', 2.0);
        for i=1:numel(latResults)
            R = latResults(i);
            col = modelColors.(fieldKeys{strcmp(modelLabels, R.model)});
            plot(R.t, R.ay_sim, '-', 'Color', col, 'LineWidth', 1.2);
        end
    end
    xlabel('Time [s]'); ylabel('a_y [m/s^2]');
    title('Lateral acceleration');

    %% Lateral — side-slip
    subplot(2, 3, 6); hold on; grid on;
    if ~isempty(latResults)
        plot(latResults(1).t, rad2deg(latResults(1).slip_cm), '-', 'Color', cmColor, 'LineWidth', 2.0);
        for i=1:numel(latResults)
            R = latResults(i);
            col = modelColors.(fieldKeys{strcmp(modelLabels, R.model)});
            plot(R.t, rad2deg(R.slip_sim), '-', 'Color', col, 'LineWidth', 1.2);
        end
    end
    xlabel('Time [s]'); ylabel('\beta [deg]');
    title('Side-slip angle');

    sgtitle('4-Model Comparison vs CarMaker (BMW 5 Series) — RK4 calibrated', ...
            'FontWeight','bold','FontSize',13);

    if ~exist(fileparts(figPath), 'dir')
        mkdir(fileparts(figPath));
    end
    exportgraphics(fig, figPath, 'Resolution', 150);
    close(fig);
end
