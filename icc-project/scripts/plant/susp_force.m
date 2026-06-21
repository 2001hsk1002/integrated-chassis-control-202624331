function [Fspring, Fdamper] = susp_force(zs, zsdot, ks, cActive)
%SUSP_FORCE 서스펜션 스프링-댐퍼 힘 계산
%
%   [Fspring, Fdamper] = SUSP_FORCE(zs, zsdot, ks, cActive)
%
%   Inputs:
%       zs      - (double) 서스펜션 변위 [m] (압축: 양수)
%       zsdot   - (double) 서스펜션 속도 [m/s]
%       ks      - (double) 스프링 강성 [N/m]
%       cActive - (double) 감쇠 계수 [Ns/m] (CDC 명령값)
%
%   Outputs:
%       Fspring - (double) 스프링 힘 [N] (복원 방향)
%       Fdamper - (double) 댐퍼 힘 [N] (속도 반대 방향)

    Fspring = -ks * zs;
    Fdamper = -cActive * zsdot;

end
