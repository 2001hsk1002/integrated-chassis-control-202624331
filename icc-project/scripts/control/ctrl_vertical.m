function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL CDC — Hybrid Skyhook + Groundhook 가변 감쇠 (per-wheel)
%
%   설계 개요:
%     - Skyhook   : sprung mass 절대속도 zs_dot 를 줄이는 방향으로 감쇠 변조
%                   (body bounce 1-2 Hz 억제 -> 승차감)
%     - Groundhook: unsprung mass 절대속도 zu_dot 를 줄이는 방향
%                   (wheel hop 10-15 Hz 억제 -> 접지력)
%     - 빈도 분리 : zs_dot 에 ~3 Hz 1차 LPF 를 적용해 저주파(body) 성분만
%                   skyhook 에 사용, 고주파(wheel hop) 성분은 groundhook 항이
%                   raw zu_dot 로 처리 — 두 모드에 다른 전략 적용.
%     - semi-active 제약: 댐퍼는 힘을 '생성' 못하므로, 요구 힘이 댐퍼가 낼 수
%       있는 방향일 때만 c 를 키우고(c=cDes), 아니면 cMin (on-off skyhook 의
%       연속 버전 = clipped continuous skyhook).
%
%   Inputs:
%       suspState - .zs_dot(4), .zu_dot(4), .zs(4), .zu(4)
%                   (bicycle/3dof plant 에서는 필드가 없을 수 있음 -> passive)
%       ctrlState - 내부 상태 (.zsF: LPF 상태)
%       CTRL      - .VER.cMin, .cMax, .skyGain
%       dt        - sample time [s]
%
%   Output:
%       dampingCmd - 4x1 damping coefficient [Ns/m], cMin <= c <= cMax
%
%   요구사항 대응: skyhook(+hybrid groundhook), cMin/cMax 제한,
%   body-bounce/wheel-hop 빈도 분리 모두 구현.

    cMin = CTRL.VER.cMin;
    cMax = CTRL.VER.cMax;
    cSky = CTRL.VER.skyGain * 1.6;      % [Ns/m] skyhook 게인
    cGnd = CTRL.VER.skyGain * 0.6;      % [Ns/m] groundhook 게인
    cNom = 1500;                        % [Ns/m] passive fallback
    fLP  = 3.0;                         % [Hz] body 성분 LPF 차단 주파수

    %% ---- 0. plant 가 수직 자유도를 제공하지 않으면 passive fallback -----
    if ~isstruct(suspState) || ~isfield(suspState, 'zs_dot') ...
            || ~isfield(suspState, 'zu_dot')
        dampingCmd = cNom * ones(4, 1);
        return;
    end

    zsd = suspState.zs_dot(:);
    zud = suspState.zu_dot(:);
    if numel(zsd) ~= 4 || numel(zud) ~= 4
        dampingCmd = cNom * ones(4, 1);
        return;
    end

    %% ---- 1. 빈도 분리: body(저주파) 속도 추출 ---------------------------
    if ~isfield(ctrlState, 'zsF'); ctrlState.zsF = zeros(4,1); end
    aLP = (2*pi*fLP*dt) / (1 + 2*pi*fLP*dt);
    ctrlState.zsF = (1 - aLP) * ctrlState.zsF + aLP * zsd;
    zsBody = ctrlState.zsF;             % body bounce 성분 (skyhook 용)

    %% ---- 2. Hybrid skyhook + groundhook (clipped continuous) -----------
    dampingCmd = zeros(4,1);
    for w = 1:4
        vRel = zsd(w) - zud(w);                 % 댐퍼 상대 속도
        % 요구 힘: F_des = -cSky*zs_body - (-cGnd*zu) (방향 반대 정의)
        Fdes = -cSky * zsBody(w) + cGnd * zud(w);
        if abs(vRel) > 1e-4
            cDes = Fdes / (-vRel);              % F_damper = -c * vRel
        else
            cDes = cNom;
        end
        % semi-active 제약: 댐퍼가 그 힘을 낼 수 있는 경우만 (c > 0)
        if cDes < cMin || ~isfinite(cDes)
            cDes = cMin;                        % 힘 방향 불가 -> 최소 감쇠
        end
        dampingCmd(w) = min(max(cDes, cMin), cMax);
    end

end
