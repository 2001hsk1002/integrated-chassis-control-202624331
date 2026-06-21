function Fy = calc_tire_force(alpha, Fz, TIRE)
%CALC_TIRE_FORCE Magic Formula 타이어 횡력 계산
%
%   Fy = CALC_TIRE_FORCE(alpha, Fz, TIRE)
%
%   Pacejka Magic Formula를 사용하여 타이어 횡력을 계산한다.
%   F = D * sin(C * atan(B*x - E*(B*x - atan(B*x))))
%
%   Inputs:
%       alpha - (double) 타이어 슬립 앵글 [rad]
%       Fz    - (double) 수직 하중 [N]
%       TIRE  - (struct) 타이어 파라미터 (.B, .C, .D, .E)
%
%   Outputs:
%       Fy - (double) 타이어 횡력 [N] (타이어 좌표계)

    B = TIRE.B;
    C = TIRE.C;
    D = TIRE.D * Fz;   % 피크 힘 = mu * Fz
    E = TIRE.E;

    x = B .* alpha;
    Fy = D .* sin(C .* atan(x - E .* (x - atan(x))));

end
