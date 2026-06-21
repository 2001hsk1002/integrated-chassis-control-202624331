function [Fx, Fy, Mz, info] = tire_rt_proxy(kappa, alpha, Fz, gamma, TIRE)
%TIRE_RT_PROXY CarMaker RT 타이어 (binary lookup) 의 MF 근사
%
%   CarMaker 의 RT_225_55R17 (BMW_5 사용)는 .bin lookup 테이블이라 직접 사용이 불가.
%   대신 CarMaker .erg 결과에서 회귀된 simple MF 계수를 그대로 사용 (현재 calibrated).
%   필드:
%     .B, .C, .D, .E       — 횡 (회귀된 axle Cf = D·C·B·Fz / Cf_lin ≈ 77 kN/rad)
%     .Bx, .Cx, .Dx, .Ex   — 종 (회귀된 brake gain 기반)
%
%   본질적으로 tire_simple_mf와 동일하지만 의도/문서 분리. 추후 RT bin 디코딩
%   라이브러리가 생기면 이 함수만 교체하면 됨.

    [Fx, Fy, Mz, info] = tire_simple_mf(kappa, alpha, Fz, gamma, TIRE);
    if nargout > 3
        info.source = 'rt_proxy (regressed MF from CarMaker .erg)';
    end
end
