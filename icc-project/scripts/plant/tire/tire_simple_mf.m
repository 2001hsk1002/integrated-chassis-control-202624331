function [Fx, Fy, Mz, info] = tire_simple_mf(kappa, alpha, Fz, gamma, TIRE)
%TIRE_SIMPLE_MF 단순 4-param Pacejka Magic Formula + 마찰원 가중
%
%   기존 tire_combined_slip.m과 동일한 거동 (backward compatible).
%   F = D·Fz·sin( C·atan( B·s − E·(B·s − atan(B·s)) ) )
%   복합 슬립은 마찰원 정규화로 분배.
%
%   TIRE fields:
%       .B,  .C,  .D,  .E   — 횡 (D는 mu_peak에 해당)
%       .Bx, .Cx, .Dx, .Ex  — 종

    Fz = max(Fz, 10);

    % 순수 종방향
    Bx = TIRE.Bx;  Cx = TIRE.Cx;  Dx = TIRE.Dx * Fz;  Ex = TIRE.Ex;
    xk  = Bx * kappa;
    Fx0 = Dx * sin( Cx * atan( xk - Ex * (xk - atan(xk)) ) );

    % 순수 횡방향
    By = TIRE.B;   Cy = TIRE.C;   Dy = TIRE.D  * Fz;  Ey = TIRE.E;
    xa  = By * alpha;
    Fy0 = Dy * sin( Cy * atan( xa - Ey * (xa - atan(xa)) ) );

    % 복합 슬립 가중 (마찰 타원 근사)
    sigma_x = kappa / (1 + abs(kappa));
    sigma_y = tan(alpha) / (1 + abs(kappa));
    sigma   = sqrt(sigma_x^2 + sigma_y^2);
    if sigma < 1e-8
        Fx = 0; Fy = 0;
    else
        Fx = Fx0 * abs(sigma_x) / sigma;
        Fy = Fy0 * abs(sigma_y) / sigma;
    end

    % Self-aligning moment: simple model omits (set 0)
    Mz = 0;

    if nargout > 3
        info.slipStiffness_long = Bx * Cx * TIRE.Dx;
        info.slipStiffness_lat  = By * Cy * TIRE.D;
        info.mu_peak            = TIRE.D;
        info.Fz_used            = Fz;
    end
end
