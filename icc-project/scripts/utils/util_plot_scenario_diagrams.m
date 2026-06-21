function util_plot_scenario_diagrams(outDir)
%UTIL_PLOT_SCENARIO_DIAGRAMS ICC 표준 시나리오 다이어그램(PNG) 생성
%
%   outDir에 다음 파일 저장:
%     scn_A1_dlc.png         ISO 3888-1 DLC 트랙 + 조향 입력
%     scn_A2_severe_dlc.png  ISO 3888-2
%     scn_A3_step_steer.png  조향 step + 응답 정성도
%     scn_A4_ss_circular.png 정상상태 원선회
%     scn_A5_sine_dwell.png  FMVSS 126 sine-with-dwell
%     scn_A6_sine_sweep.png  주파수 sweep
%     scn_A7_brake_in_turn.png ISO 7975 BIT
%     scn_B1_straight_brake.png ISO 21994
%     scn_B3_split_mu.png    ISO 14512
%     scn_C1_bump.png        single bump
%     scn_C3_random_road.png ISO 8608 random road
%     scn_D1_dlc_brake.png   DLC + brake 통합
%     scn_D4_fishhook.png    NHTSA fishhook

    if ~exist(outDir,'dir'); mkdir(outDir); end

    %% A1 ISO 3888-1 DLC track (cone layout per ISO 3888-1)
    fig = newFig('scn_A1_dlc'); set(fig,'Position',[80 80 1000 480]);
    subplot(1,2,1); hold on; axis equal; grid on;
    % ISO 3888-1 sections (vehicle width assumed 1.8 m → lane widths 1.1·b+0.25 m)
    drawDLCTrack_ISO3888_1();
    title('ISO 3888-1 — Track (top view)');
    xlabel('x [m]'); ylabel('y [m]');

    subplot(1,2,2); hold on; grid on;
    t = linspace(0, 8, 800);
    steer = zeros(size(t));
    steer(t>=2.0 & t<3.0) = 3*sin(pi*(t(t>=2.0 & t<3.0)-2));
    steer(t>=3.0 & t<4.5) = -3*sin(pi*(t(t>=3.0 & t<4.5)-3)/1.5);
    steer(t>=4.5 & t<6.0) = 3*sin(pi*(t(t>=4.5 & t<6.0)-4.5)/1.5);
    plot(t, steer, 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Driver steer (roadwheel) [deg]');
    title('A1 — Steer input profile @ 80 km/h');
    sgtitle('A1: ISO 3888-1 DLC @ 80 km/h','FontWeight','bold');
    saveFig(fig, outDir, 'scn_A1_dlc');

    %% A2 ISO 3888-2 severe DLC
    fig = newFig('scn_A2_severe'); set(fig,'Position',[80 80 1000 480]);
    subplot(1,2,1); hold on; axis equal; grid on;
    drawDLCTrack_ISO3888_2();
    title('ISO 3888-2 — Severe (narrower lane)');
    xlabel('x [m]'); ylabel('y [m]');
    subplot(1,2,2); hold on; grid on;
    t = linspace(0, 6, 800);
    steer = 5*sin(2*pi*0.7*t) .* (t>1.0 & t<3.5);
    plot(t, steer, 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Driver steer [deg]');
    title('A2 — More aggressive input');
    sgtitle('A2: ISO 3888-2 Severe DLC','FontWeight','bold');
    saveFig(fig, outDir, 'scn_A2_severe_dlc');

    %% A3 Step steer
    fig = newFig('scn_A3'); set(fig,'Position',[80 80 1000 440]);
    subplot(1,2,1); hold on; grid on;
    t = linspace(0, 4, 800);
    steer = 2 * (t >= 1);
    plot(t, steer, 'b-', 'LineWidth', 2);
    xlabel('Time [s]'); ylabel('Steer [deg]');
    title('Step steer input δ=2°');
    ylim([-0.5 3]);
    subplot(1,2,2); hold on; grid on;
    % Idealized yaw rate response (1st order with overshoot)
    r_ss = 5.0;     % deg/s
    wn = 6;
    zeta = 0.5;
    r_resp = r_ss * (1 - exp(-zeta*wn*(t-1)).*cos(wn*sqrt(1-zeta^2)*(t-1)));
    r_resp(t<1) = 0;
    plot(t, r_resp, 'r-', 'LineWidth', 1.5);
    plot([1 4],[r_ss r_ss],'k--');
    plot([1 4],[r_ss*1.02 r_ss*1.02],'k:'); plot([1 4],[r_ss*0.98 r_ss*0.98],'k:');
    text(2.5, r_ss+1.0, 'r_{ss}', 'FontWeight','bold');
    text(2.5, r_ss*0.85, '±2% band', 'FontSize',9);
    xlabel('Time [s]'); ylabel('Yaw rate [deg/s]');
    title('Yaw rate response (rise/overshoot/settling)');
    sgtitle('A3: ISO 7401 Step Steer','FontWeight','bold');
    saveFig(fig, outDir, 'scn_A3_step_steer');

    %% A4 Steady-state circular
    fig = newFig('scn_A4'); set(fig,'Position',[80 80 1000 440]);
    subplot(1,2,1); hold on; axis equal; grid on;
    R = 50;
    theta = linspace(0, 2*pi, 200);
    plot(R*cos(theta), R*sin(theta), 'k-', 'LineWidth', 1.5);
    plot(R*cos(theta(1:40)), R*sin(theta(1:40)), 'b-', 'LineWidth', 3);
    plot(0,0,'k+','MarkerSize',10);
    text(0.5, 0.5, 'center', 'FontSize',9);
    quiver(R, 0, 0, 5, 0, 'Color','b','LineWidth',1.5,'MaxHeadSize',1.5);
    text(R+2, 5, 'v_x', 'FontSize',10,'Color','b');
    xlabel('x [m]'); ylabel('y [m]'); title('R = 50 m circular path');
    subplot(1,2,2); hold on; grid on;
    vx = linspace(0, 25, 100);
    delta_neutral = vx.^2 / (R * 9.81);  % linear-tire neutral steer (no Kus)
    delta_us = vx.^2 / (R * 9.81) .* (1 + 0.002*vx.^2);  % understeer
    plot(vx*3.6, rad2deg(delta_neutral*0.05), 'k--', 'LineWidth', 1.2);
    plot(vx*3.6, rad2deg(delta_us*0.05), 'b-', 'LineWidth', 1.5);
    xlabel('vx [km/h]'); ylabel('Steer δ [deg]');
    title('Required δ vs speed (understeer characteristic)');
    legend({'neutral','understeer'},'Location','best');
    sgtitle('A4: ISO 4138 Steady-State Circular','FontWeight','bold');
    saveFig(fig, outDir, 'scn_A4_ss_circular');

    %% A5 Sine-with-Dwell (FMVSS 126)
    fig = newFig('scn_A5'); set(fig,'Position',[80 80 1000 440]);
    hold on; grid on;
    A_deg = 5 * 6.5;  % steering wheel amplitude ~6.5×A0; here A_deg as plot scale
    f = 0.7;
    T = 1/f;
    t = linspace(0, 4, 1000);
    steer = zeros(size(t));
    for i = 1:numel(t)
        ti = t(i);
        if ti < 0.5            % ramp up to first peak
            steer(i) = 0;
        elseif ti < 0.5 + 0.75 % half sine to +peak
            steer(i) = A_deg * sin(2*pi*f*(ti - 0.5));
        elseif ti < 0.5 + 0.75 + 0.5  % dwell at -peak (after going through 0)
            % sine continues
            steer(i) = A_deg * sin(2*pi*f*(ti - 0.5));
        elseif ti < 0.5 + 1.25 + 0.5
            % 0.5 s dwell at second peak
            t_dwell_start = 0.5 + 1.25;
            if ti < t_dwell_start
                steer(i) = A_deg * sin(2*pi*f*(ti - 0.5));
            else
                steer(i) = A_deg * sin(2*pi*f*(t_dwell_start - 0.5));  % held
            end
        else
            t_end_dwell = 0.5 + 1.25 + 0.5;
            steer(i) = A_deg * sin(2*pi*f*(ti - 0.5 - 0.5));   % continue
        end
    end
    % Simplified plot: standard sine + 0.5s dwell at second peak
    t = linspace(0, 3, 1000);
    steer = zeros(size(t));
    for i=1:numel(t)
        if t(i) < 0.5
            steer(i) = 0;
        elseif t(i) < 0.5 + T/2
            steer(i) = A_deg * sin(2*pi*f*(t(i)-0.5));
        elseif t(i) < 0.5 + T/2 + 0.5
            steer(i) = -A_deg;   % dwell at -A
        else
            tau = t(i) - 0.5 - T/2 - 0.5;
            if tau < T/4
                steer(i) = -A_deg * cos(2*pi*f*tau);
            else
                steer(i) = 0;
            end
        end
    end
    plot(t, steer, 'b-', 'LineWidth', 1.8);
    yline(0,'k:'); xline(0.5,'k:'); xline(0.5+T/2,'k:'); xline(0.5+T/2+0.5,'k:');
    text(0.5+T/4, A_deg*0.5, '1st peak', 'HorizontalAlignment','center','FontSize',9);
    text(0.5+T/2+0.25, -A_deg-2, '0.5 s dwell', 'HorizontalAlignment','center','FontSize',9);
    text(0.5+T/2+0.5+0.2, -A_deg*0.5, 'recovery', 'FontSize',9);
    xlabel('Time [s]'); ylabel('Steering wheel angle [deg]');
    title('A5: FMVSS 126 / ISO 19365 Sine-with-Dwell (0.7 Hz + 0.5 s dwell)');
    saveFig(fig, outDir, 'scn_A5_sine_dwell');

    %% A6 Sine sweep
    fig = newFig('scn_A6'); set(fig,'Position',[80 80 1000 440]);
    subplot(2,1,1); grid on;
    t = linspace(0, 20, 2000);
    f_inst = 0.1 + 1.0*t/20;  % linear sweep 0.1 → 1.1 Hz
    phase = 2*pi*cumsum(f_inst)*(t(2)-t(1));
    steer = 2*sin(phase);
    plot(t, steer, 'b-', 'LineWidth', 0.8);
    xlabel('Time [s]'); ylabel('Steer [deg]'); title('Linear-chirp 0.1 → 1.1 Hz');
    subplot(2,1,2); grid on;
    plot(t, f_inst, 'r-', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Frequency [Hz]'); title('Instantaneous frequency');
    sgtitle('A6: ISO 7401 Random/Sine Sweep','FontWeight','bold');
    saveFig(fig, outDir, 'scn_A6_sine_sweep');

    %% A7 Brake-in-Turn (ISO 7975)
    fig = newFig('scn_A7'); set(fig,'Position',[80 80 1000 440]);
    subplot(1,2,1); hold on; axis equal; grid on;
    R = 100;
    theta = linspace(pi/2, 0, 100);
    x_path = R*cos(theta);
    y_path = R - R*sin(theta);
    plot(x_path, y_path, 'k-', 'LineWidth', 1.5);
    iBrake = 40;
    plot(x_path(1:iBrake), y_path(1:iBrake), 'g-', 'LineWidth', 3);
    plot(x_path(iBrake:end), y_path(iBrake:end), 'r-', 'LineWidth', 3);
    plot(x_path(iBrake), y_path(iBrake), 'ko', 'MarkerSize',10,'MarkerFaceColor','y');
    text(x_path(iBrake)+5, y_path(iBrake)+3, 'Brake apply', 'FontSize',10);
    xlabel('x [m]'); ylabel('y [m]'); title('Constant radius → braking');
    legend({'path','cruise','brake'},'Location','best');
    subplot(1,2,2); hold on; grid on;
    t = linspace(0, 6, 600);
    brake = zeros(size(t));
    brake(t > 2) = 0.4 * 9.81 * (1 - exp(-3*(t(t>2)-2)));   % ax target = 0.4g
    steer = 3*ones(size(t));
    plot(t, brake, 'r-', 'LineWidth', 1.5);
    plot(t, steer, 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Inputs');
    legend({'Brake decel target [m/s²]','Steer [deg]'},'Location','best');
    title('ISO 7975 — constant steer + brake step');
    sgtitle('A7: ISO 7975 Brake-in-Turn','FontWeight','bold');
    saveFig(fig, outDir, 'scn_A7_brake_in_turn');

    %% B1 Straight-line braking
    fig = newFig('scn_B1'); set(fig,'Position',[80 80 1000 420]);
    subplot(1,2,1); hold on; grid on;
    t = linspace(0, 4.5, 1000);
    brakeP = zeros(size(t));
    brakeP(t > 1) = 100 * (1 - exp(-10*(t(t>1)-1)));
    plot(t, brakeP, 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Brake pressure [bar]');
    title('Brake step (100 bar)');
    subplot(1,2,2); hold on; grid on;
    vx = 27.8 * ones(size(t));   % 100 km/h
    decel = 9 * (1 - exp(-8*(t-1)));
    decel(t<1) = 0;
    vx_t = vx(1) - cumtrapz(t, decel);
    vx_t = max(vx_t, 0);
    plot(t, vx_t*3.6, 'b-', 'LineWidth', 1.5);
    plot(t, decel, 'r--', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('vx [km/h] / decel [m/s²]');
    title('vx and decel (expected)');
    legend({'vx [km/h]','decel [m/s²]'},'Location','east');
    sgtitle('B1: ISO 21994 Straight-Line Braking 100→0 km/h','FontWeight','bold');
    saveFig(fig, outDir, 'scn_B1_straight_brake');

    %% B3 Split-μ braking
    fig = newFig('scn_B3'); set(fig,'Position',[80 80 1000 440]);
    subplot(1,2,1); hold on; axis equal; grid on;
    % Road sketch: left half high-μ, right half low-μ
    fill([0 0 60 60],[-2 0 0 -2],[0.5 0.5 0.5],'EdgeColor','none');
    fill([0 0 60 60],[0 2 2 0],[0.95 0.95 0.85],'EdgeColor','none');
    text(30, -1, 'μ=0.9 (dry)','HorizontalAlignment','center','FontSize',10);
    text(30,  1, 'μ=0.3 (wet/ice)','HorizontalAlignment','center','FontSize',10);
    % Car path
    plot([0 60],[0 0],'k--','LineWidth',1.2);
    plot(0, 0, 'ko','MarkerSize',12,'MarkerFaceColor','b');
    text(0, 1.5, 'Start','FontSize',9);
    xlabel('x [m]'); ylabel('y [m]'); title('Split-μ road (left/right different)');
    xlim([-5 65]); ylim([-4 4]);
    subplot(1,2,2); hold on; grid on;
    t = linspace(0, 4, 1000);
    brake = 50 * (1 - exp(-10*(t-0.5)));
    brake(t<0.5) = 0;
    yaw = 0.6 * (1 - exp(-3*(t-0.5)));   % yaw growth due to μ asymmetry
    yaw(t<0.5) = 0;
    plot(t, brake, 'r-', 'LineWidth', 1.5);
    plot(t, yaw*20, 'm-', 'LineWidth', 1.5);  % scale yaw for visibility
    xlabel('Time [s]'); ylabel('');
    legend({'Brake [bar]','Yaw growth [deg×20]'},'Location','best');
    title('Asymmetric brake force → yaw moment');
    sgtitle('B3: ISO 14512 Split-μ Braking','FontWeight','bold');
    saveFig(fig, outDir, 'scn_B3_split_mu');

    %% C1 Single bump
    fig = newFig('scn_C1'); set(fig,'Position',[80 80 1000 420]);
    subplot(1,2,1); hold on; grid on;
    x = linspace(-5, 10, 1000);
    bump = 0.08 * (1 - cos(2*pi*(x-1)/1.5)) / 2 .* (x>=1 & x<=2.5);
    plot(x, bump, 'k-', 'LineWidth', 1.5); fill([x fliplr(x)], [bump zeros(size(bump))],[0.7 0.5 0.3]);
    xlabel('x [m]'); ylabel('Road elevation z_{road} [m]');
    title('Cosine bump (height 80 mm, length 1.5 m)');
    ylim([-0.02 0.12]);
    subplot(1,2,2); hold on; grid on;
    t = linspace(0, 2, 1000);
    vx = 30 / 3.6;
    z_road_t = 0.08 * (1 - cos(2*pi*(vx*t - 1)/1.5)) / 2 .* (vx*t>=1 & vx*t<=2.5);
    plot(t, z_road_t*1000, 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('z_{road} [mm]');
    title('Wheel input (vx = 30 km/h)');
    sgtitle('C1: Single Bump (OEM ride sweep)','FontWeight','bold');
    saveFig(fig, outDir, 'scn_C1_bump');

    %% C3 ISO 8608 random road
    fig = newFig('scn_C3'); set(fig,'Position',[80 80 1000 420]);
    subplot(1,2,1); hold on; grid on;
    % Synth: each ISO class PSD with random phase
    L = 100; N = 2000;
    x = linspace(0, L, N);
    Gd_B = 32e-6;   % class B at n0=0.1 cycles/m
    Gd_C = 128e-6;  % class C
    n = (1:N/2) / L;
    rng(42);
    for cls = {'A','B','C'}
        Gd = struct('A',16e-6,'B',Gd_B,'C',Gd_C).(cls{1});
        amp = sqrt(2 * Gd * (0.1./max(n,0.01)).^2 * (n(2)-n(1)));
        phi = 2*pi*rand(size(n));
        zr = zeros(size(x));
        for k=1:numel(n)
            zr = zr + amp(k)*cos(2*pi*n(k)*x + phi(k));
        end
        plot(x, zr*1000, 'LineWidth', 1.0);
    end
    legend({'Class A (smooth)','Class B (good)','Class C (rough)'},'Location','best');
    xlabel('x [m]'); ylabel('z_{road} [mm]');
    title('ISO 8608 PSD-based road profiles');
    subplot(1,2,2); hold on; grid on;
    classes = {'A','B','C','D','E'};
    Gd_vals = [16, 64, 256, 1024, 4096] * 1e-6;
    bar(1:numel(classes), Gd_vals*1e6, 'FaceColor',[0.3 0.5 0.85]);
    set(gca,'XTickLabel',classes,'YScale','log');
    xlabel('Road class'); ylabel('G_d(n_0) [10^{-6} m^3]');
    title('ISO 8608 class roughness');
    sgtitle('C3: ISO 8608 Random Road','FontWeight','bold');
    saveFig(fig, outDir, 'scn_C3_random_road');

    %% D1 DLC + braking
    fig = newFig('scn_D1'); set(fig,'Position',[80 80 1000 440]);
    subplot(1,2,1); hold on; axis equal; grid on;
    drawDLCTrack_ISO3888_1();
    title('Track: ISO 3888-1 DLC');
    xlabel('x [m]'); ylabel('y [m]');
    subplot(1,2,2); hold on; grid on;
    t = linspace(0, 8, 1000);
    steer = zeros(size(t));
    steer(t>=2.0 & t<3.0) = 3*sin(pi*(t(t>=2.0 & t<3.0)-2));
    steer(t>=3.0 & t<4.5) = -3*sin(pi*(t(t>=3.0 & t<4.5)-3)/1.5);
    steer(t>=4.5 & t<6.0) = 3*sin(pi*(t(t>=4.5 & t<6.0)-4.5)/1.5);
    brake = zeros(size(t));
    brake(t>2.0 & t<5.5) = 3.0;  % 0.3g during DLC
    plot(t, steer, 'b-', 'LineWidth', 1.5);
    plot(t, brake, 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Inputs');
    legend({'Steer [deg]','Brake decel [m/s²]'},'Location','best');
    title('Simultaneous steer + brake');
    sgtitle('D1: DLC under 0.3g Braking (통합 ICC)','FontWeight','bold');
    saveFig(fig, outDir, 'scn_D1_dlc_brake');

    %% D4 Fishhook (NHTSA)
    fig = newFig('scn_D4'); set(fig,'Position',[80 80 1000 440]);
    subplot(1,2,1); hold on; axis equal; grid on;
    % Sketch: J-turn ish path with double pulse
    th = linspace(0, pi, 100);
    Rs = 40;
    plot(Rs*sin(th), -Rs*(1-cos(th)), 'k-', 'LineWidth', 1.5);
    plot(0,0,'ko','MarkerSize',10,'MarkerFaceColor','b');
    text(2, 2, 'Start','FontSize',9);
    xlabel('x [m]'); ylabel('y [m]'); title('Fishhook path (sharp double pulse)');
    subplot(1,2,2); hold on; grid on;
    t = linspace(0, 5, 1000);
    A = 6.5*5;  % 6.5 × A0 (FMVSS 126 reference amplitude)
    steer = zeros(size(t));
    % Phase 1: ramp to +A
    i1 = t > 0.5 & t < 1.0;  steer(i1) = A*(t(i1)-0.5)/0.5;
    % Hold at +A
    i2 = t >= 1.0 & t < 1.25; steer(i2) = A;
    % Counter to -A
    i3 = t >= 1.25 & t < 1.75; steer(i3) = A - 2*A*(t(i3)-1.25)/0.5;
    % Hold at -A (THE HOOK)
    i4 = t >= 1.75 & t < 4.75; steer(i4) = -A;
    % Recovery
    i5 = t >= 4.75; steer(i5) = -A + A*(t(i5)-4.75)/0.25;
    steer(steer<-A) = -A; steer(steer>A) = A;
    plot(t, steer, 'b-', 'LineWidth', 1.8);
    xlabel('Time [s]'); ylabel('Steering wheel angle [deg]');
    title('Fishhook steer profile (NHTSA)');
    text(2.5, -A+5, '3s dwell (hook)','HorizontalAlignment','center','FontSize',9);
    sgtitle('D4: NHTSA Fishhook — Rollover','FontWeight','bold');
    saveFig(fig, outDir, 'scn_D4_fishhook');

    fprintf('Scenario diagrams saved to %s\n', outDir);
end

%% ============================================================
function fig = newFig(name)
    fig = figure('Name', name, 'Color', 'w', 'Visible', 'off');
end
function saveFig(fig, outDir, name)
    exportgraphics(fig, fullfile(outDir, [name '.png']), 'Resolution', 150);
    close(fig);
end

function drawDLCTrack_ISO3888_1()
    % ISO 3888-1 cone arrangement (passenger car, vehicle width ~1.8 m)
    % Section 1: 15m straight, lane width 1.1*b+0.25 (b=vehicle width)
    b = 1.8;
    w1 = 1.1*b + 0.25;   % 2.23 m
    w2 = 1.0*b + 1.00;   % 2.80 m (offset lane)
    w3 = 1.3*b + 0.25;   % 2.59 m
    offset = 3.5;        % lane offset

    % Section 1: 12 m straight, width w1, y centered at 0
    drawCones([0 12], 0, w1);
    % Section 2: 13.5 m → 11 m offset lane, y centered at +offset, width w2
    drawCones([25 36], offset, w2);
    % Section 3: 12 m straight, back to y=0, width w3
    drawCones([61 73], 0, w3);
    % Reference path (idealized)
    xpath = [0 12 25 36 61 73 80];
    ypath = [0 0 offset offset 0 0 0];
    plot(xpath, ypath, 'b--', 'LineWidth', 1.2);
    xlim([-3 83]); ylim([-3 8]);
end

function drawDLCTrack_ISO3888_2()
    % Narrower than 3888-1
    b = 1.8;
    w1 = 1.1*b + 0.25;
    w2 = 1.0*b + 0.25;   % severe — same as 3888-1 narrow
    w3 = 1.3*b + 0.25;
    offset = 3.5;

    drawCones([0 12], 0, w1);
    drawCones([13.5 25], offset, w2);  % even more compact
    drawCones([28 40], 0, w3);
    xpath = [0 12 13.5 25 28 40 50];
    ypath = [0 0 offset offset 0 0 0];
    plot(xpath, ypath, 'b--', 'LineWidth', 1.2);
    xlim([-3 53]); ylim([-3 8]);
end

function drawCones(xrange, yc, w)
    yL = yc + w/2;
    yR = yc - w/2;
    plot([xrange(1) xrange(2)], [yL yL], 'k-','LineWidth',1);
    plot([xrange(1) xrange(2)], [yR yR], 'k-','LineWidth',1);
    plot([xrange(1) xrange(1) xrange(2) xrange(2)], [yR yL yL yR], 'k.','MarkerSize',8);
    text((xrange(1)+xrange(2))/2, yc + w/2 + 0.3, sprintf('w=%.2fm', w),...
        'HorizontalAlignment','center','FontSize',8);
end
