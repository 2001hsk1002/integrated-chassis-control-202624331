function fig = util_plot_results(result, opts)
%UTIL_PLOT_RESULTS ICC 표준 결과 플롯 (9-subplot)
%
%   fig = UTIL_PLOT_RESULTS(result)
%   fig = UTIL_PLOT_RESULTS(result, opts)
%
%   Inputs:
%       result - (struct) 시뮬레이션 결과
%       opts   - (struct, optional)
%           .title    - 플롯 제목 (default: 'ICC Simulation Results')
%           .ref      - 레퍼런스 신호 구조체
%           .savePath - 저장 경로 (지정 시 PNG 저장)
%           .visible  - figure 표시 여부 (default: 'on')
%
%   Outputs:
%       fig - (figure) 생성된 figure 핸들

    arguments
        result struct
        opts.title (1,:) char = 'ICC Simulation Results'
        opts.ref struct = struct()
        opts.savePath (1,:) char = ''
        opts.visible (1,:) char = 'on'
    end

    t = result.time;

    fig = figure('Name', opts.title, ...
        'Position', [50 50 1400 900], 'Visible', opts.visible);

    %% Row 1: Vehicle Motion
    % 1-1: Speed
    subplot(3,3,1);
    plot(t, result.vx * 3.6, 'b-', 'LineWidth', 1.2);
    if isfield(opts.ref, 'vx')
        hold on;
        plot(t, opts.ref.vx * 3.6, 'r--', 'LineWidth', 1.0);
        legend('Actual', 'Ref', 'Location', 'best');
    end
    xlabel('Time [s]'); ylabel('Speed [km/h]');
    title('Vehicle Speed'); grid on;

    % 1-2: Yaw Rate
    subplot(3,3,2);
    plot(t, rad2deg(result.yawRate), 'b-', 'LineWidth', 1.2);
    if isfield(opts.ref, 'yawRate')
        hold on;
        plot(t, rad2deg(opts.ref.yawRate), 'r--', 'LineWidth', 1.0);
        legend('Actual', 'Ref', 'Location', 'best');
    end
    xlabel('Time [s]'); ylabel('Yaw Rate [deg/s]');
    title('Yaw Rate'); grid on;

    % 1-3: Slip Angle
    subplot(3,3,3);
    plot(t, rad2deg(result.slipAngle), 'b-', 'LineWidth', 1.2);
    hold on;
    yline([-5 5], 'r--', 'LineWidth', 1.0);
    xlabel('Time [s]'); ylabel('\beta [deg]');
    title('Body Slip Angle'); grid on;
    legend('Actual', 'Warning', 'Location', 'best');

    %% Row 2: Accelerations & Steering
    % 2-1: Lateral Accel
    subplot(3,3,4);
    plot(t, result.ay, 'b-', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('a_y [m/s^2]');
    title('Lateral Acceleration'); grid on;

    % 2-2: Longitudinal Accel
    subplot(3,3,5);
    plot(t, result.ax, 'b-', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('a_x [m/s^2]');
    title('Longitudinal Acceleration'); grid on;

    % 2-3: Steering
    subplot(3,3,6);
    plot(t, rad2deg(result.steerAngle), 'b-', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Steer [deg]');
    title('Steering Angle'); grid on;

    %% Row 3: Stability Indicators
    % 3-1: Phase Plane
    subplot(3,3,7);
    plot(rad2deg(result.slipAngle), rad2deg(result.yawRate), ...
         'b-', 'LineWidth', 1.2);
    xlabel('\beta [deg]'); ylabel('Yaw Rate [deg/s]');
    title('Phase Plane (\beta - \psi'')'); grid on;

    % 3-2: LTR
    subplot(3,3,8);
    plot(t, result.LTR, 'b-', 'LineWidth', 1.2);
    hold on;
    yline([-0.7 0.7], 'r--', 'LineWidth', 1.0);
    xlabel('Time [s]'); ylabel('LTR [-]');
    title('Load Transfer Ratio'); grid on;

    % 3-3: g-g Diagram
    subplot(3,3,9);
    scatter(result.ay / 9.81, result.ax / 9.81, 4, t, 'filled');
    colorbar; colormap(jet);
    xlabel('Lateral [g]'); ylabel('Longitudinal [g]');
    title('g-g Diagram'); grid on; axis equal;
    % 마찰원 표시
    theta = linspace(0, 2*pi, 100);
    hold on;
    plot(cos(theta), sin(theta), 'r--', 'LineWidth', 1.0);

    sgtitle(opts.title, 'FontSize', 14, 'FontWeight', 'bold');

    %% 저장
    if ~isempty(opts.savePath)
        saveas(fig, opts.savePath);
        fprintf('[util_plot_results] Saved: %s\n', opts.savePath);
    end

end
