function VEH = cm_to_plant_params(cmInfoFile, baseVEH)
%CM_TO_PLANT_PARAMS CarMaker BMW_5 INFOFILE → 14-DOF VEH 파라미터 매핑
%
%   VEH = CM_TO_PLANT_PARAMS(cmInfoFile, baseVEH)
%
%   CarMaker INFOFILE에서 추출 가능한 차량 파라미터(질량, 관성, 휠 위치,
%   타이어 사양 등)를 14-DOF plant가 요구하는 VEH 구조체 필드로 변환한다.
%   INFOFILE에 없는 항목(서스 강성/감쇠, 타이어 수직강성, 롤센터 등)은
%   baseVEH에서 그대로 가져온다.
%
%   Inputs:
%       cmInfoFile - (char) CarMaker Vehicle INFOFILE 경로
%       baseVEH    - (struct) 기본 generic VEH (누락 항목 fallback)
%
%   Outputs:
%       VEH - (struct) 14-DOF 호환 차량 파라미터

    if ~isfile(cmInfoFile)
        error('[cm_to_plant_params] File not found: %s', cmInfoFile);
    end
    p = cm_parse_infofile(cmInfoFile);

    VEH = baseVEH;  % 누락 항목 fallback

    %% 질량
    ms = local_get(p, 'Body_mass', baseVEH.ms);
    mwc_f = local_get(p, 'WheelCarrier_fl_mass', NaN);
    mwc_r = local_get(p, 'WheelCarrier_rl_mass', NaN);
    mw_f  = local_get(p, 'Wheel_fl_mass', NaN);
    mw_r  = local_get(p, 'Wheel_rl_mass', NaN);

    if all(~isnan([mwc_f, mwc_r, mw_f, mw_r]))
        mUnsprung_front_per = mwc_f + mw_f;
        mUnsprung_rear_per  = mwc_r + mw_r;
        % 14-DOF는 mu_w를 per-wheel 단일값으로 받음 → 평균 사용
        VEH.mu_w = (mUnsprung_front_per + mUnsprung_rear_per) / 2;
        mTotal = ms + 2*mUnsprung_front_per + 2*mUnsprung_rear_per;
    else
        VEH.mu_w = baseVEH.mu_w;
        mTotal = ms + 4 * baseVEH.mu_w;
    end
    VEH.ms   = ms;
    VEH.mass = mTotal;

    %% 관성 모멘트 (Body 기준 — sprung mass)
    BodyI = local_get(p, 'Body_I', [baseVEH.Ix, baseVEH.Iy, baseVEH.Iz]);
    if numel(BodyI) >= 3
        VEH.Ix = BodyI(1);
        VEH.Iy = BodyI(2);
        VEH.Iz = BodyI(3);
    end

    %% 휠 위치 → 휠베이스, 트랙, lf/lr
    jackFL = local_get(p, 'Jack_fl_pos', []);
    jackFR = local_get(p, 'Jack_fr_pos', []);
    jackRL = local_get(p, 'Jack_rl_pos', []);
    jackRR = local_get(p, 'Jack_rr_pos', []);
    bodyPos = local_get(p, 'Body_pos', []);

    if numel(jackFL) == 3 && numel(jackRL) == 3
        VEH.L = jackFL(1) - jackRL(1);
    end
    if numel(jackFL) == 3 && numel(jackFR) == 3
        VEH.track_f = abs(jackFL(2)) + abs(jackFR(2));
    end
    if numel(jackRL) == 3 && numel(jackRR) == 3
        VEH.track_r = abs(jackRL(2)) + abs(jackRR(2));
    end
    if numel(bodyPos) == 3 && numel(jackFL) == 3 && numel(jackRL) == 3
        % CoG x is body.pos(1). 전축 jack x는 jackFL(1), 후축은 jackRL(1).
        VEH.lf = jackFL(1) - bodyPos(1);
        VEH.lr = bodyPos(1) - jackRL(1);
        VEH.h_cog = bodyPos(3);
    end

    %% 휠 회전 관성 (rotation axis = y in CM)
    wheelI_fl = local_get(p, 'Wheel_fl_I', []);
    wheelI_rl = local_get(p, 'Wheel_rl_I', []);
    if numel(wheelI_fl) >= 2 && numel(wheelI_rl) >= 2
        VEH.Iw = (wheelI_fl(2) + wheelI_rl(2)) / 2;
    end

    %% 타이어 반경 (Tire.0 = "Examples/RT_225_55R17_p2.50" → 0.3397 m)
    tire0 = local_get(p, 'Tire_0', '');
    rEff = local_parse_tire_radius(tire0);
    if ~isnan(rEff)
        VEH.rw = rEff;
    end

    %% 공기역학
    Ax = local_get(p, 'Aero_Ax', NaN);
    if ~isnan(Ax)
        VEH.Af = Ax;
    end
    % Cd는 BMW_5 INFOFILE에 6x1 계수표(Aero.Coeff)로 존재 — 첫 항이 종방향 항력 계수에 해당
    aeroCoeff = local_get(p, 'Aero_Coeff', []);
    cdFromCM = local_first_numeric(aeroCoeff);
    if ~isnan(cdFromCM)
        VEH.Cd = cdFromCM;
    end

    %% 코너링 강성 — BMW_5 .erg에서 회귀 추출 (4개 곡선 시나리오, R^2≈0.96/0.63)
    %   alpha=0.3~3 deg 선형 영역의 (axle Fy, axle alpha) 회귀
    VEH.Cf = 77422;  % [N/rad] 전축 코너링 강성
    VEH.Cr = 74094;  % [N/rad] 후축 코너링 강성

    %% 출처/메타데이터
    VEH.source = struct();
    VEH.source.name      = 'cm_to_plant_params';
    VEH.source.infoFile  = cmInfoFile;
    VEH.source.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
end

%% --------------------------------------------------------------
function v = local_get(s, fld, default)
    if isfield(s, fld)
        v = s.(fld);
        if ischar(v) && strcmp(default, v)  % 빈문자열 케이스도 통과
            return;
        end
        if isnumeric(v) && isempty(v)
            v = default;
        end
    else
        v = default;
    end
end

%% --------------------------------------------------------------
function r = local_parse_tire_radius(tireName)
% 'Examples/RT_225_55R17_p2.50' → effective rolling radius [m]
% (sidewall = width * aspect, radius = rim/2 + sidewall, 유효반경 ≈ 0.97 × free)
    r = NaN;
    if isempty(tireName) || ~ischar(tireName)
        return;
    end
    % 패턴: <width>_<aspect>R<rim>
    tok = regexp(tireName, '(\d+)_(\d+)R(\d+)', 'tokens', 'once');
    if isempty(tok)
        return;
    end
    width_mm  = str2double(tok{1});
    aspect    = str2double(tok{2});
    rim_in    = str2double(tok{3});
    if any(isnan([width_mm, aspect, rim_in]))
        return;
    end
    sidewall_m = width_mm * 1e-3 * aspect / 100;
    rim_m      = rim_in * 0.0254 / 2;
    r_free     = rim_m + sidewall_m;
    r = 0.97 * r_free;  % 유효반경 (적재 시)
end

%% --------------------------------------------------------------
function v = local_first_numeric(blockVal)
% cell of strings or single numeric row → 첫 숫자 토큰 추출
    v = NaN;
    if isempty(blockVal)
        return;
    end
    if isnumeric(blockVal) && ~isempty(blockVal)
        v = blockVal(1);
        return;
    end
    if iscell(blockVal)
        for k = 1:numel(blockVal)
            line = blockVal{k};
            if ischar(line)
                tok = regexp(line, '[-+]?\d*\.?\d+([eE][-+]?\d+)?', 'match', 'once');
                if ~isempty(tok)
                    v = str2double(tok);
                    return;
                end
            end
        end
    end
end
