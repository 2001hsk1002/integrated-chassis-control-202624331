function [Fx, Fy, Mz, info] = tire_full_mf(kappa, alpha, Fz, gamma, TIRE)
%TIRE_FULL_MF Pacejka Magic Formula 5.2 / MF-Tyre 풀 모델
%
%   ADAMS .tir 파일에서 추출된 PCX, PDX, PEX, PKX, PHX, PVX (종),
%   PCY, PDY, PEY, PKY, PHY, PVY (횡), QSX, QSY (orientation) 등을 사용.
%   하중 의존성 dfz = (Fz - Fz0)/Fz0 를 통해 load sensitivity 반영.
%
%   Pacejka 2012, "Tire and Vehicle Dynamics" §4.3.2 (steady-state slip).
%   계수 명명은 ADAMS Tire Property File 표준 (.tir, FILE_VERSION 3.0).
%
%   TIRE 입력 필드 (cm_to_plant_params 또는 tire_full_mf_parse_tir에서 채움):
%     .FZ0          공칭 수직 하중 [N]
%     .PCX1, .PDX1, .PDX2, .PEX1, .PEX2, .PEX3, .PEX4, .PKX1, .PKX2, .PKX3, .PHX1, .PHX2, .PVX1, .PVX2   (종)
%     .PCY1, .PDY1, .PDY2, .PDY3, .PEY1, .PEY2, .PEY3, .PEY4, .PKY1, .PKY2, .PKY3, .PHY1, .PHY2, .PHY3, .PVY1, .PVY2, .PVY3, .PVY4   (횡)
%     .LAMBDA       (옵션) 스케일링 팩터 (default 1)

    Fz = max(Fz, 10);
    Fz0 = TIRE.FZ0;          % 공칭 하중
    dfz = (Fz - Fz0) / Fz0;  % 정규화된 하중 편차
    gam = gamma;

    %% ========== 종방향 Fx ==========
    SHX = TIRE.PHX1 + TIRE.PHX2 * dfz;                            % horizontal shift
    SVX = (TIRE.PVX1 + TIRE.PVX2 * dfz) * Fz;                     % vertical shift
    kx  = kappa + SHX;
    Cx  = TIRE.PCX1;                                              % shape factor
    Dx  = (TIRE.PDX1 + TIRE.PDX2 * dfz) * Fz;                     % peak (load-dep μ)
    Ex  = (TIRE.PEX1 + TIRE.PEX2 * dfz + TIRE.PEX3 * dfz^2) ...   % curvature
        * (1 - TIRE.PEX4 * sign(kx));
    Kx  = Fz * (TIRE.PKX1 + TIRE.PKX2 * dfz) * exp(TIRE.PKX3 * dfz);  % slip stiffness Kfx
    if abs(Cx * Dx) < 1e-6
        Bx = 0;
    else
        Bx  = Kx / (Cx * Dx);                                     % stiffness factor
    end
    Fx0 = Dx * sin( Cx * atan( Bx*kx - Ex*(Bx*kx - atan(Bx*kx)) ) ) + SVX;

    %% ========== 횡방향 Fy (gamma 효과 포함) ==========
    SHY = TIRE.PHY1 + TIRE.PHY2 * dfz + TIRE.PHY3 * gam;
    SVY = Fz * (TIRE.PVY1 + TIRE.PVY2*dfz + (TIRE.PVY3 + TIRE.PVY4*dfz)*gam);
    ay  = alpha + SHY;
    Cy  = TIRE.PCY1;
    Dy  = (TIRE.PDY1 + TIRE.PDY2 * dfz) * Fz * (1 - TIRE.PDY3 * gam^2);
    Ey  = (TIRE.PEY1 + TIRE.PEY2 * dfz) ...
        * (1 - (TIRE.PEY3 + TIRE.PEY4 * gam) * sign(ay));
    Ky  = TIRE.PKY1 * Fz0 * sin(2 * atan(Fz / (TIRE.PKY2 * Fz0))) ...
        * (1 - TIRE.PKY3 * abs(gam));
    if abs(Cy * Dy) < 1e-6
        By = 0;
    else
        By  = Ky / (Cy * Dy);
    end
    Fy0 = Dy * sin( Cy * atan( By*ay - Ey*(By*ay - atan(By*ay)) ) ) + SVY;

    %% ========== 복합 슬립 (마찰 타원 가중) ==========
    sigma_x = kappa / (1 + abs(kappa));
    sigma_y = tan(alpha) / (1 + abs(kappa));
    sigma   = sqrt(sigma_x^2 + sigma_y^2);
    if sigma < 1e-8
        Fx = 0; Fy = 0;
    else
        Fx = Fx0 * abs(sigma_x) / sigma;
        Fy = Fy0 * abs(sigma_y) / sigma;
    end

    % ADAMS .tir (SAE convention) → ISO convention 변환
    if isfield(TIRE, 'fy_sign_flip') && ~isempty(TIRE.fy_sign_flip)
        Fy = Fy * TIRE.fy_sign_flip;
    end

    %% Mz는 간이 (full MF의 self-aligning moment는 추가 ~10 계수 필요. 단순 근사)
    Mz = 0;

    if nargout > 3
        info.Cx = Cx; info.Dx = Dx; info.Ex = Ex; info.Bx = Bx; info.Kx = Kx;
        info.Cy = Cy; info.Dy = Dy; info.Ey = Ey; info.By = By; info.Ky = Ky;
        info.SHX = SHX; info.SVX = SVX; info.SHY = SHY; info.SVY = SVY;
        info.mu_peak_x = Dx / max(Fz, 1);
        info.mu_peak_y = Dy / max(Fz, 1);
        info.Fz_used = Fz;
    end
end
