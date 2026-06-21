function util_plot_fbd(outDir)
%UTIL_PLOT_FBD 각 plant 모델의 free body diagram (FBD)을 PNG로 생성
%
%   outDir에 다음 파일 저장:
%     fbd_bicycle.png     (2-DOF, top view)
%     fbd_3dof.png        (3-DOF, top view + brake)
%     fbd_7dof.png        (7-DOF, 4-wheel top view + omega)
%     fbd_14dof.png       (14-DOF, top + side view + corner suspension)

    if ~exist(outDir,'dir'); mkdir(outDir); end

    %% Bicycle FBD
    fig = figure('Position',[80 80 900 600],'Visible','off','Color','w');
    ax = axes('Position',[0.05 0.08 0.9 0.85]); hold(ax,'on'); axis(ax,'equal'); axis(ax,'off');
    % Body line
    bodyLen = 3.0; lf = 1.5; lr = 1.5;
    plot([-lr lf],[0 0],'k-','LineWidth',2);
    plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',8);
    text(0.05, 0.18,'CoG','FontSize',10);
    % Front wheel (steered)
    delta = deg2rad(10);
    drawTire(lf, 0, delta, 0.55, 0.18, [0.2 0.4 0.85]);
    text(lf-0.3, 0.55,'\delta','FontSize',13,'Color',[0.2 0.4 0.85]);
    % Rear wheel
    drawTire(-lr, 0, 0, 0.55, 0.18, [0.2 0.4 0.85]);
    % Velocity vector at CoG
    quiver(0, 0, 1.6, 0.4, 0, 'Color',[0.1 0.6 0.1],'LineWidth',2,'MaxHeadSize',0.5);
    text(1.6, 0.6, 'V', 'FontSize',13,'Color',[0.1 0.6 0.1]);
    text(0.6, 0.15, 'v_x', 'FontSize',11);
    text(0.05, 0.45, 'v_y', 'FontSize',11);
    % Yaw rate r
    arrAng = linspace(0, deg2rad(110), 30);
    plot(0.4*cos(arrAng), 0.4*sin(arrAng), 'k-','LineWidth',1.5);
    plot(0.4*cos(deg2rad(110))-0.05, 0.4*sin(deg2rad(110))+0.07,'k^','MarkerFaceColor','k','MarkerSize',5);
    text(-0.15, 0.55,'r (yaw)','FontSize',11);
    % Tire forces
    quiver(lf, 0, -0.3, 1.2, 0, 'Color',[0.85 0.2 0.2],'LineWidth',2,'MaxHeadSize',0.5);
    text(lf-0.7, 1.3,'F_{yf}','FontSize',12,'Color',[0.85 0.2 0.2]);
    quiver(-lr, 0, 0, 0.9, 0, 'Color',[0.85 0.2 0.2],'LineWidth',2,'MaxHeadSize',0.5);
    text(-lr-0.4, 1.0,'F_{yr}','FontSize',12,'Color',[0.85 0.2 0.2]);
    % Slip angles
    quiver(lf, 0, 1.0, -0.3, 0, 'Color',[0.5 0.3 0.7],'LineStyle','--','LineWidth',1.5);
    text(lf+0.5, -0.5,'\alpha_f','FontSize',12,'Color',[0.5 0.3 0.7]);
    quiver(-lr, 0, 1.0, -0.15, 0, 'Color',[0.5 0.3 0.7],'LineStyle','--','LineWidth',1.5);
    text(-lr+0.5, -0.45,'\alpha_r','FontSize',12,'Color',[0.5 0.3 0.7]);
    % lf / lr labels
    plot([0 lf],[-1.1 -1.1],'k-'); plot([0 0],[-1.05 -1.15],'k-'); plot([lf lf],[-1.05 -1.15],'k-');
    text(lf/2, -1.3,'l_f','FontSize',12);
    plot([-lr 0],[-1.1 -1.1],'k-'); plot([-lr -lr],[-1.05 -1.15],'k-');
    text(-lr/2, -1.3,'l_r','FontSize',12);
    % Title
    title('Bicycle Model (2-DOF) — Top View','FontSize',14,'FontWeight','bold');
    text(-1.7, -1.7, 'States: [v_y, r].  v_x is constant.  Inputs: \delta.','FontSize',11);
    text(-1.7, -1.95, 'F_{yf} = C_f \alpha_f,   F_{yr} = C_r \alpha_r  (linear)','FontSize',11);
    xlim([-2.3 3.0]); ylim([-2.3 1.8]);
    exportgraphics(fig, fullfile(outDir,'fbd_bicycle.png'),'Resolution',150);
    close(fig);

    %% 3-DOF FBD (top view + brake forces)
    fig = figure('Position',[80 80 900 650],'Visible','off','Color','w');
    ax = axes('Position',[0.05 0.08 0.9 0.85]); hold(ax,'on'); axis(ax,'equal'); axis(ax,'off');
    bodyLen = 3.0;
    plot([-lr lf],[0 0],'k-','LineWidth',2);
    plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',8);
    text(0.05, 0.18,'CoG','FontSize',10);
    drawTire(lf, 0, delta, 0.55, 0.18, [0.2 0.4 0.85]);
    text(lf-0.3, 0.55,'\delta','FontSize',13,'Color',[0.2 0.4 0.85]);
    drawTire(-lr, 0, 0, 0.55, 0.18, [0.2 0.4 0.85]);
    % v vector
    quiver(0, 0, 1.6, 0.4, 0, 'Color',[0.1 0.6 0.1],'LineWidth',2,'MaxHeadSize',0.5);
    text(1.6, 0.6, 'V', 'FontSize',13,'Color',[0.1 0.6 0.1]);
    text(0.6, 0.15, 'v_x', 'FontSize',11);
    text(0.05, 0.45, 'v_y', 'FontSize',11);
    arrAng = linspace(0, deg2rad(110), 30);
    plot(0.4*cos(arrAng), 0.4*sin(arrAng), 'k-','LineWidth',1.5);
    plot(0.4*cos(deg2rad(110))-0.05, 0.4*sin(deg2rad(110))+0.07,'k^','MarkerFaceColor','k','MarkerSize',5);
    text(-0.15, 0.55,'r','FontSize',11);
    % Fy (lateral)
    quiver(lf, 0, -0.3, 1.2, 0, 'Color',[0.85 0.2 0.2],'LineWidth',2,'MaxHeadSize',0.5);
    text(lf-0.7, 1.3,'F_{yf}','FontSize',12,'Color',[0.85 0.2 0.2]);
    quiver(-lr, 0, 0, 0.9, 0, 'Color',[0.85 0.2 0.2],'LineWidth',2,'MaxHeadSize',0.5);
    text(-lr-0.4, 1.0,'F_{yr}','FontSize',12,'Color',[0.85 0.2 0.2]);
    % Fx (longitudinal brake, both axles)
    quiver(lf, 0, -1.0, 0, 0, 'Color',[0.85 0.5 0.1],'LineWidth',2,'MaxHeadSize',0.4);
    text(lf-1.4, -0.3,'F_{xf} (brake)','FontSize',11,'Color',[0.85 0.5 0.1]);
    quiver(-lr, 0, -0.6, 0, 0, 'Color',[0.85 0.5 0.1],'LineWidth',2,'MaxHeadSize',0.4);
    text(-lr-1.4, -0.3,'F_{xr}','FontSize',11,'Color',[0.85 0.5 0.1]);
    % Aero drag
    quiver(2.0, 0, -0.9, 0, 0, 'Color',[0.5 0.5 0.5],'LineWidth',1.8,'MaxHeadSize',0.4);
    text(2.0, -0.25,'F_{aero}','FontSize',11,'Color',[0.5 0.5 0.5]);
    % Title + notes
    title('3-DOF Nonlinear Model — Top View','FontSize',14,'FontWeight','bold');
    text(-1.7,-1.5, 'States: [v_x, v_y, r].','FontSize',11);
    text(-1.7,-1.75,'Tire forces: Pacejka Magic Formula  F_y(\alpha, F_z).','FontSize',11);
    text(-1.7,-2.0, 'Axle-level F_z with longitudinal load transfer.','FontSize',11);
    text(-1.7,-2.25,'Friction-ellipse saturation: \surd(F_x^2+F_y^2) \leq \mu F_z.','FontSize',11);
    xlim([-2.6 3.2]); ylim([-2.6 1.8]);
    exportgraphics(fig, fullfile(outDir,'fbd_3dof.png'),'Resolution',150);
    close(fig);

    %% 7-DOF FBD (4-wheel top view + omega)
    fig = figure('Position',[80 80 900 700],'Visible','off','Color','w');
    ax = axes('Position',[0.05 0.08 0.9 0.85]); hold(ax,'on'); axis(ax,'equal'); axis(ax,'off');
    tf = 0.8; tr = 0.8;
    % Body rectangle
    rectangle('Position',[-lr -tf, lr+lf, 2*tf],'EdgeColor','k','LineWidth',1.5);
    plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',8);
    text(0.05, 0.18,'CoG','FontSize',10);
    % 4 wheels (FL, FR, RL, RR)
    drawTire(lf,  tf, delta, 0.45, 0.16, [0.2 0.4 0.85]);
    drawTire(lf, -tf, delta, 0.45, 0.16, [0.2 0.4 0.85]);
    drawTire(-lr,  tr, 0, 0.45, 0.16, [0.2 0.4 0.85]);
    drawTire(-lr, -tr, 0, 0.45, 0.16, [0.2 0.4 0.85]);
    text(lf+0.5,  tf, 'FL', 'FontSize',10);
    text(lf+0.5, -tf, 'FR', 'FontSize',10);
    text(-lr-0.55,  tr, 'RL', 'FontSize',10);
    text(-lr-0.55, -tr, 'RR', 'FontSize',10);
    % Per-wheel omega arrows (rotation indicator)
    for pos = [lf tf; lf -tf; -lr tr; -lr -tr]'
        % ω indicator: small curved arrow
        th = linspace(deg2rad(-60), deg2rad(170), 20);
        plot(pos(1)+0.27*cos(th), pos(2)+0.27*sin(th), 'k-','LineWidth',1);
        plot(pos(1)+0.27*cos(th(end)), pos(2)+0.27*sin(th(end)), 'k>','MarkerFaceColor','k','MarkerSize',4);
    end
    text(lf+0.15, tf+0.55, '\omega_{FL}','FontSize',10,'FontWeight','bold');
    % Forces per wheel (one shown for clarity)
    quiver(lf,  tf, 0.7, 0.4, 0, 'Color',[0.85 0.2 0.2],'LineWidth',1.5);
    text(lf+0.8, tf+0.45,'F_{x,FL},F_{y,FL}','FontSize',10,'Color',[0.85 0.2 0.2]);
    % v vector
    quiver(0, 0, 1.4, 0.2, 0, 'Color',[0.1 0.6 0.1],'LineWidth',2,'MaxHeadSize',0.5);
    text(1.4, 0.4, 'V', 'FontSize',13,'Color',[0.1 0.6 0.1]);
    % Brake torque indicator
    text(-lr-1.3, -tr-0.55, 'T_{brk,RL}','FontSize',10,'Color',[0.85 0.5 0.1]);
    quiver(-lr-0.8, -tr, 0.4, 0, 0, 'Color',[0.85 0.5 0.1],'LineWidth',1.5,'MaxHeadSize',0.5);
    % Track lines
    plot([lf lf],[tf -tf],'k:'); plot([-lr -lr],[tr -tr],'k:');
    text(lf+0.15,(tf-tf)/2,'t_f','FontSize',11);
    text(-lr+0.15,(tr-tr)/2,'t_r','FontSize',11);
    title('7-DOF Model — 4-wheel Top View','FontSize',14,'FontWeight','bold');
    text(-2.7,-2.0,'States: [v_x, v_y, r, \omega_{FL}, \omega_{FR}, \omega_{RL}, \omega_{RR}].','FontSize',11);
    text(-2.7,-2.3,'Per-wheel \kappa = (\omega·r_w - v_{xw})/|v_{xw}| and combined-slip MF tire force.','FontSize',11);
    text(-2.7,-2.6,'Per-wheel F_z with longitudinal + lateral load transfer.','FontSize',11);
    xlim([-3.0 3.5]); ylim([-3.0 2.0]);
    exportgraphics(fig, fullfile(outDir,'fbd_7dof.png'),'Resolution',150);
    close(fig);

    %% 14-DOF FBD (top view + side view + corner suspension)
    fig = figure('Position',[60 40 1200 720],'Visible','off','Color','w');

    % Top view (left)
    subplot(1,2,1); hold on; axis equal; axis off;
    rectangle('Position',[-lr -tf, lr+lf, 2*tf],'EdgeColor','k','LineWidth',1.5);
    plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',8);
    text(0.05, 0.18,'CoG','FontSize',10);
    drawTire(lf,  tf, delta, 0.45, 0.16, [0.2 0.4 0.85]);
    drawTire(lf, -tf, delta, 0.45, 0.16, [0.2 0.4 0.85]);
    drawTire(-lr,  tr, 0, 0.45, 0.16, [0.2 0.4 0.85]);
    drawTire(-lr, -tr, 0, 0.45, 0.16, [0.2 0.4 0.85]);
    % Roll axis indicator
    plot([-lr lf],[0 0],'b--','LineWidth',1.2);
    text(lf+0.05, -0.15,'roll axis (\phi)','FontSize',9,'Color','b');
    text(0.05, 0.4, '+pitch (\theta) from side view','FontSize',9,'Color','b');
    quiver(0, 0, 1.4, 0.2, 0, 'Color',[0.1 0.6 0.1],'LineWidth',2);
    text(1.4, 0.4, 'V', 'FontSize',13,'Color',[0.1 0.6 0.1]);
    title('14-DOF — Top View','FontSize',13,'FontWeight','bold');
    text(-2.6, -1.7,'19 states: 7 body + 4 z_s + 4 \dot z_s + 4 \omega.','FontSize',10);
    text(-2.6, -2.0,'Adds: roll \phi, pitch \theta, 4-corner suspension.','FontSize',10);
    xlim([-3.0 3.3]); ylim([-2.5 1.5]);

    % Side view (right): suspension + pitch + anti-dive
    subplot(1,2,2); hold on; axis equal; axis off;
    % Body box
    sb = [-1.5 -0.6; 1.5 -0.6; 1.5 0.3; -1.5 0.3; -1.5 -0.6];
    plot(sb(:,1), sb(:,2), 'k-','LineWidth',1.5);
    text(-0.3, -0.15, 'm_s', 'FontSize',11);
    plot(0, -0.15,'ko','MarkerFaceColor','k','MarkerSize',6);
    text(0.05, -0.3,'CoG','FontSize',9);
    % Wheels at front and rear
    drawWheelSide(1.3, -0.95, 0.3);
    drawWheelSide(-1.3, -0.95, 0.3);
    % Suspension springs (springs between body and wheel)
    drawSpring(1.3, -0.6, 1.3, -0.78, 6, 0.12);
    drawSpring(-1.3, -0.6, -1.3, -0.78, 6, 0.12);
    text(1.45, -0.7, 'k_s,c_s', 'FontSize',9,'Color',[0.4 0.3 0.6]);
    % Tire spring kt
    plot([1.3 1.3],[-1.25 -1.42],'-','LineWidth',2,'Color',[0.5 0.5 0.5]);
    plot([-1.3 -1.3],[-1.25 -1.42],'-','LineWidth',2,'Color',[0.5 0.5 0.5]);
    text(1.42, -1.35,'k_t','FontSize',9,'Color',[0.4 0.4 0.4]);
    % Ground
    plot([-2.2 2.2],[-1.42 -1.42],'k-','LineWidth',1.5);
    for x = -2.0:0.25:2.0
        plot([x x-0.07],[-1.42 -1.55],'k-','LineWidth',0.8);
    end
    % Pitch indicator
    arrAng = linspace(deg2rad(195), deg2rad(285), 20);
    plot(0.5*cos(arrAng)+0, 0.5*sin(arrAng)+0.5, 'b-','LineWidth',1.2);
    text(0.55, 0.05,'\theta (pitch)','FontSize',10,'Color','b');
    % Anti-dive force decomposition
    quiver(1.3, -1.0, -0.5, 0.4, 0, 'Color',[0.85 0.2 0.2],'LineWidth',1.5);
    text(0.7, -0.65,'F_x (brake)','FontSize',10,'Color',[0.85 0.2 0.2]);
    text(-0.5, -1.6, '\eta_{AD}: 0.70 (브레이크), \eta_{AS}: 0.30 (가속)', 'FontSize',9, 'Color',[0.85 0.2 0.2]);
    title('14-DOF — Side View (Pitch + Suspension)','FontSize',13,'FontWeight','bold');
    text(-2.2, 1.0, 'Suspension EOM (loaded-eq):','FontSize',10);
    text(-2.2, 0.8, ' m_u \ddot z_s = -(k_s + k_t) z_s - c_s \dot z_s','FontSize',10);
    text(-2.2, 0.55,'Pitch:','FontSize',10);
    text(-2.2, 0.35,' I_y \ddot \theta = -k_\theta\theta - c_\theta\dot\theta','FontSize',10);
    text(-2.2, 0.15,'   -(1-\eta_{AD}) m_s a_x h_{CoG}','FontSize',10);
    xlim([-2.4 2.4]); ylim([-1.8 1.3]);

    sgtitle('14-DOF Free Body Diagram','FontWeight','bold','FontSize',14);
    exportgraphics(fig, fullfile(outDir,'fbd_14dof.png'),'Resolution',150);
    close(fig);

    fprintf('FBDs saved to %s\n', outDir);
end

%% Helpers --------------------------------------------------------------
function drawTire(x, y, ang, len, wid, col)
    R = [cos(ang) -sin(ang); sin(ang) cos(ang)];
    pts = [-len/2 -wid/2; len/2 -wid/2; len/2 wid/2; -len/2 wid/2; -len/2 -wid/2];
    pts = (R * pts')';
    pts(:,1) = pts(:,1) + x;
    pts(:,2) = pts(:,2) + y;
    fill(pts(:,1), pts(:,2), col, 'EdgeColor','k','LineWidth',1.2,'FaceAlpha',0.3);
end

function drawWheelSide(x, y, r)
    th = linspace(0, 2*pi, 30);
    plot(x + r*cos(th), y + r*sin(th), 'k-','LineWidth',1.5);
    plot(x, y, 'k+','MarkerSize',6);
end

function drawSpring(x1, y1, x2, y2, ncoils, width)
    % vertical spring schematic
    L = y1 - y2;
    yy = linspace(y1, y2, 2*ncoils+1);
    xx = repmat([0 width -width], 1, ncoils);
    xx = xx(1:numel(yy));
    plot(x1 + xx, yy, '-','Color',[0.4 0.3 0.6],'LineWidth',1.5);
end
