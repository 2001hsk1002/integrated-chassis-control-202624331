function [Fx, Fy] = tire_combined_slip(kappa, alpha, Fz, TIRE)
%TIRE_COMBINED_SLIP 복합 슬립 Magic Formula 타이어 모델
%
%   [Fx, Fy] = TIRE_COMBINED_SLIP(kappa, alpha, Fz, TIRE)
%
%   순수 종방향/횡방향 Magic Formula에 마찰 타원 가중을 적용하여
%   복합 슬립 상태의 타이어 힘을 계산한다.
%
%   Inputs:
%       kappa - (double) 타이어 슬립 비 [-] (제동: 음수, 구동: 양수)
%       alpha - (double) 타이어 슬립 앵글 [rad]
%       Fz    - (double) 수직 하중 [N]
%       TIRE  - (struct) 타이어 파라미터
%           횡: .B, .C, .D, .E (기존)
%           종: .Bx, .Cx, .Dx, .Ex
%
%   Outputs:
%       Fx - (double) 종방향 타이어 힘 [N]
%       Fy - (double) 횡방향 타이어 힘 [N]

    %% 하중 보호
    Fz = max(Fz, 10);

    %% 순수 종방향 (Magic Formula)
    Bx = TIRE.Bx;
    Cx = TIRE.Cx;
    Dx = TIRE.Dx * Fz;
    Ex = TIRE.Ex;
    xk = Bx * kappa;
    Fx0 = Dx * sin(Cx * atan(xk - Ex * (xk - atan(xk))));

    %% 순수 횡방향 (기존 calc_tire_force 동일 수식)
    By = TIRE.B;
    Cy = TIRE.C;
    Dy = TIRE.D * Fz;
    Ey = TIRE.E;
    xa = By * alpha;
    Fy0 = Dy * sin(Cy * atan(xa - Ey * (xa - atan(xa))));

    %% 복합 슬립 가중 (마찰 타원 근사)
    % 정규화된 슬립 합성
    sigma_x = kappa / (1 + abs(kappa));
    sigma_y = tan(alpha) / (1 + abs(kappa));
    sigma   = sqrt(sigma_x^2 + sigma_y^2);

    if sigma < 1e-8
        Fx = 0;
        Fy = 0;
    else
        % Fx0/Fy0는 Magic Formula의 sin()이 이미 부호를 보존하므로
        % 양의 가중치(|sigma_*|/sigma)만 곱해 마찰 타원 분배 적용.
        Fx = Fx0 * abs(sigma_x) / sigma;
        Fy = Fy0 * abs(sigma_y) / sigma;
    end

end
