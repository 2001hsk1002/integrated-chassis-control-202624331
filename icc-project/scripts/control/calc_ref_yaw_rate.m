function yawRateRef = calc_ref_yaw_rate(vx, steerAngle, VEH)
%CALC_REF_YAW_RATE Bicycle Model 기반 목표 요 레이트 계산
%
%   yawRateRef = CALC_REF_YAW_RATE(vx, steerAngle, VEH)
%
%   선형 Bicycle Model의 정상상태 응답으로부터 목표 요 레이트를 계산한다.
%   언더스티어 그래디언트를 고려하여 속도에 따른 감쇠 효과를 반영한다.
%
%   Inputs:
%       vx         - (double) 종방향 속도 [m/s], 스칼라 또는 벡터
%       steerAngle - (double) 로드휠 조향각 [rad], 스칼라 또는 벡터
%       VEH        - (struct) 차량 파라미터 (.mass, .Iz, .lf, .lr, .Cf, .Cr)
%
%   Outputs:
%       yawRateRef - (double) 목표 요 레이트 [rad/s]
%
%   Example:
%       run('config/sim_params.m');
%       yr = calc_ref_yaw_rate(20, deg2rad(2), VEH);

    % 축간 거리
    L = VEH.lf + VEH.lr;

    % 언더스티어 그래디언트
    Kus = (VEH.mass * VEH.lr) / (2 * VEH.Cf * L) - ...
          (VEH.mass * VEH.lf) / (2 * VEH.Cr * L);

    % 정상상태 요 레이트 (Bicycle Model)
    %   r_ss = (vx / L) * delta / (1 + Kus * vx^2)
    yawRateRef = (vx .* steerAngle) ./ (L + Kus .* vx.^2);

    % 극저속 보호 (vx < 1 m/s → 요 레이트 0)
    yawRateRef(abs(vx) < 1.0) = 0;

end
