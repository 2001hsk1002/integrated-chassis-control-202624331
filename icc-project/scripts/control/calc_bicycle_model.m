function [A, B, C, D] = calc_bicycle_model(vx, VEH)
%CALC_BICYCLE_MODEL 2-DOF Bicycle Model 상태공간 행렬 계산
%
%   [A, B, C, D] = CALC_BICYCLE_MODEL(vx, VEH)
%
%   상태: x = [vy; yawRate]  (횡방향 속도, 요 레이트)
%   입력: u = [steerAngle]   (로드휠 조향각)
%   출력: y = [vy; yawRate]
%
%   Inputs:
%       vx  - (double) 종방향 속도 [m/s], 스칼라
%       VEH - (struct) 차량 파라미터
%
%   Outputs:
%       A, B, C, D - (double) 상태공간 행렬

    m  = VEH.mass;
    Iz = VEH.Iz;
    lf = VEH.lf;
    lr = VEH.lr;
    Cf = VEH.Cf;
    Cr = VEH.Cr;

    % 극저속 보호
    if abs(vx) < 1.0
        vx = sign(vx + eps) * 1.0;
    end

    % A 행렬
    a11 = -(Cf + Cr) / (m * vx);
    a12 = -vx - (Cf * lf - Cr * lr) / (m * vx);
    a21 = -(Cf * lf - Cr * lr) / (Iz * vx);
    a22 = -(Cf * lf^2 + Cr * lr^2) / (Iz * vx);

    A = [a11, a12;
         a21, a22];

    % B 행렬
    B = [Cf / m;
         Cf * lf / Iz];

    % C, D 행렬 (전 상태 출력)
    C = eye(2);
    D = zeros(2, 1);

end
