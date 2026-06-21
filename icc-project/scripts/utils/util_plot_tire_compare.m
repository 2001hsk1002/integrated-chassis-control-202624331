function util_plot_tire_compare(outPath, TIRE_simple, TIRE_full)
%UTIL_PLOT_TIRE_COMPARE 타이어 모델 (simple_mf vs full_mf) 특성 비교 plot
%
%   여러 Fz에서 Fy vs alpha, Fx vs kappa 곡선을 overlay.
%   복합 슬립 (마찰 타원) 효과도 시각화.

    alphas = deg2rad(-10:0.25:10);
    kappas = -0.20:0.005:0.20;
    Fz_set = [2000 4000 6000 8000];     % [N]

    fig = figure('Position',[60 60 1300 850],'Visible','off','Color','w');

    %% 1. Pure lateral Fy vs alpha (multiple Fz)
    subplot(2,3,1); hold on; grid on;
    cols = lines(numel(Fz_set));
    for i = 1:numel(Fz_set)
        Fy_s = arrayfun(@(a) localSecond(@()tire_simple_mf(0,a,Fz_set(i),0,TIRE_simple)), alphas);
        plot(rad2deg(alphas), Fy_s/1000, '-', 'Color', cols(i,:), 'LineWidth', 1.5);
    end
    xlabel('\alpha [deg]'); ylabel('F_y [kN]');
    title('simple\_mf (4-param) — pure lateral');
    legend(arrayfun(@(z) sprintf('Fz=%d N',z), Fz_set, 'UniformOutput',false), 'Location','best');

    subplot(2,3,2); hold on; grid on;
    for i = 1:numel(Fz_set)
        Fy_f = arrayfun(@(a) localSecond(@()tire_full_mf(0,a,Fz_set(i),0,TIRE_full)), alphas);
        plot(rad2deg(alphas), Fy_f/1000, '-', 'Color', cols(i,:), 'LineWidth', 1.5);
    end
    xlabel('\alpha [deg]'); ylabel('F_y [kN]');
    title('full\_mf (MF 5.2, MF\_205\_60R15) — pure lateral');
    legend(arrayfun(@(z) sprintf('Fz=%d N',z), Fz_set, 'UniformOutput',false), 'Location','best');

    %% 2. Pure longitudinal Fx vs kappa
    subplot(2,3,3); hold on; grid on;
    Fz_demo = 4000;
    Fx_s = arrayfun(@(k) localFirst(@()tire_simple_mf(k,0,Fz_demo,0,TIRE_simple)), kappas);
    Fx_f = arrayfun(@(k) localFirst(@()tire_full_mf(k,0,Fz_demo,0,TIRE_full)), kappas);
    plot(kappas, Fx_s/1000, 'b-', 'LineWidth', 1.8); hold on;
    plot(kappas, Fx_f/1000, 'r--', 'LineWidth', 1.8);
    xlabel('\kappa [-]'); ylabel('F_x [kN]');
    title(sprintf('Pure longitudinal (Fz=%d N)', Fz_demo));
    legend({'simple\_mf','full\_mf'},'Location','best');

    %% 3. Combined slip (κ varying, fixed alpha)
    subplot(2,3,4); hold on; grid on;
    alpha_demo = deg2rad(4);
    Fz_demo = 4000;
    Fx_s = arrayfun(@(k) localFirst(@()tire_simple_mf(k,alpha_demo,Fz_demo,0,TIRE_simple)), kappas);
    Fy_s = arrayfun(@(k) localSecond(@()tire_simple_mf(k,alpha_demo,Fz_demo,0,TIRE_simple)), kappas);
    Fx_f = arrayfun(@(k) localFirst(@()tire_full_mf(k,alpha_demo,Fz_demo,0,TIRE_full)), kappas);
    Fy_f = arrayfun(@(k) localSecond(@()tire_full_mf(k,alpha_demo,Fz_demo,0,TIRE_full)), kappas);
    plot(kappas, Fx_s/1000, 'b-', 'LineWidth', 1.5); hold on;
    plot(kappas, Fy_s/1000, 'b:', 'LineWidth', 1.5);
    plot(kappas, Fx_f/1000, 'r--', 'LineWidth', 1.5);
    plot(kappas, Fy_f/1000, 'r-.', 'LineWidth', 1.5);
    xlabel('\kappa [-]'); ylabel('Force [kN]');
    title(sprintf('Combined slip @ \\alpha=%d°, Fz=%d N', round(rad2deg(alpha_demo)), Fz_demo));
    legend({'F_x simple','F_y simple','F_x full','F_y full'},'Location','best');

    %% 4. Friction ellipse (Fx vs Fy)
    subplot(2,3,5); hold on; grid on; axis equal;
    Fz_demo = 4000;
    kk = -0.15:0.01:0.15;  aa = deg2rad(-8:0.5:8);
    [KK, AA] = meshgrid(kk, aa);
    Fx_s = zeros(size(KK)); Fy_s = zeros(size(KK));
    Fx_f = zeros(size(KK)); Fy_f = zeros(size(KK));
    for i = 1:numel(KK)
        [Fx_s(i), Fy_s(i)] = tire_simple_mf(KK(i), AA(i), Fz_demo, 0, TIRE_simple);
        [Fx_f(i), Fy_f(i)] = tire_full_mf(KK(i), AA(i), Fz_demo, 0, TIRE_full);
    end
    scatter(Fx_s(:)/1000, Fy_s(:)/1000, 4, [0.2 0.4 0.85], 'filled', 'MarkerFaceAlpha',0.4); hold on;
    scatter(Fx_f(:)/1000, Fy_f(:)/1000, 4, [0.85 0.2 0.2], 'MarkerFaceAlpha',0.4);
    theta = linspace(0, 2*pi, 200);
    mu = 1.0;
    plot(mu*Fz_demo/1000*cos(theta), mu*Fz_demo/1000*sin(theta), 'k--', 'LineWidth', 1.0);
    xlabel('F_x [kN]'); ylabel('F_y [kN]');
    title(sprintf('Friction ellipse @ Fz=%d N (\\mu·Fz dashed)', Fz_demo));
    legend({'simple\_mf','full\_mf','\\mu·Fz circle'},'Location','best');

    %% 6. Lateral slip stiffness (Kfy = dFy/dalpha at zero) vs Fz
    subplot(2,3,6); hold on; grid on;
    Fzs = 500:200:9000;
    Kfy_s = zeros(size(Fzs)); Kfy_f = zeros(size(Fzs));
    da = deg2rad(0.5);
    for i=1:numel(Fzs)
        [~, Fyp_s] = tire_simple_mf(0,  da, Fzs(i), 0, TIRE_simple);
        [~, Fyn_s] = tire_simple_mf(0, -da, Fzs(i), 0, TIRE_simple);
        Kfy_s(i) = (Fyp_s - Fyn_s) / (2*da);
        [~, Fyp_f] = tire_full_mf(0,  da, Fzs(i), 0, TIRE_full);
        [~, Fyn_f] = tire_full_mf(0, -da, Fzs(i), 0, TIRE_full);
        Kfy_f(i) = (Fyp_f - Fyn_f) / (2*da);
    end
    plot(Fzs/1000, Kfy_s/1000, 'b-', 'LineWidth', 1.8); hold on;
    plot(Fzs/1000, Kfy_f/1000, 'r--', 'LineWidth', 1.8);
    xlabel('F_z [kN]'); ylabel('K_{f\\alpha} = \\partial F_y/\\partial\\alpha [kN/rad]');
    title('Lateral slip stiffness vs load');
    legend({'simple\_mf','full\_mf'},'Location','best');

    sgtitle('Tire Model Comparison: simple\_mf vs full\_mf (MF 5.2)','FontWeight','bold','FontSize',13);

    if ~exist(fileparts(outPath),'dir'); mkdir(fileparts(outPath)); end
    exportgraphics(fig, outPath, 'Resolution', 150);
    close(fig);
end

function y = localFirst(fn)
    [a, ~] = fn();
    y = a;
end
function y = localSecond(fn)
    [~, b] = fn();
    y = b;
end
