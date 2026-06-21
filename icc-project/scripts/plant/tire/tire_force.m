function [Fx, Fy, Mz, info] = tire_force(kappa, alpha, Fz, gamma, TIRE)
%TIRE_FORCE 다중 타이어 모델 디스패처
%
%   [Fx, Fy, Mz, info] = TIRE_FORCE(kappa, alpha, Fz, gamma, TIRE)
%
%   타이어 모델 식별자 TIRE.model 에 따라 분기:
%     'simple_mf'  — 4-param Pacejka (B,C,D,E), backward-compatible
%     'full_mf'    — Pacejka MF 5.2 (load sensitivity, camber 포함, .tir에서 로드)
%     'rt_proxy'   — CarMaker RT tire 근사 (회귀된 MF 계수)
%
%   Inputs:
%       kappa - (double) 종방향 슬립 비 [-]
%       alpha - (double) 횡방향 슬립 앵글 [rad]
%       Fz    - (double) 수직 하중 [N]
%       gamma - (double) 캠버각 [rad] (옵션, default 0)
%       TIRE  - (struct) 타이어 파라미터. 필수: TIRE.model
%
%   Outputs:
%       Fx   - (double) 종방향 타이어 힘 [N] (제동 시 음수)
%       Fy   - (double) 횡방향 타이어 힘 [N]
%       Mz   - (double) self-aligning moment [N·m] (선택)
%       info - (struct) diagnostics (slip stiffness, peak μ, 등)

    if nargin < 4 || isempty(gamma)
        gamma = 0;
    end
    if ~isfield(TIRE, 'model') || isempty(TIRE.model)
        TIRE.model = 'simple_mf';  % default backward-compat
    end

    switch lower(TIRE.model)
        case 'simple_mf'
            [Fx, Fy, Mz, info] = tire_simple_mf(kappa, alpha, Fz, gamma, TIRE);
        case 'full_mf'
            [Fx, Fy, Mz, info] = tire_full_mf(kappa, alpha, Fz, gamma, TIRE);
        case 'rt_proxy'
            [Fx, Fy, Mz, info] = tire_rt_proxy(kappa, alpha, Fz, gamma, TIRE);
        otherwise
            error('[tire_force] Unknown tire model: %s', TIRE.model);
    end
end
