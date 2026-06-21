function scenario = scn_C1_single_bump(SIM)
%SCN_C1_SINGLE_BUMP Single cosine bump (height 80 mm, length 1.5 m) @ 30 km/h
%
%   OEM ride 사양 — 단일 충격에 대한 CDC/서스 응답 평가

    scenario.id          = 'C1';
    scenario.name        = 'Single Bump (80mm × 1.5m) @ 30 km/h';
    scenario.refStandard = 'OEM ride sweep (ISO 8608 baseline)';
    scenario.tEnd        = 3.0;
    scenario.vx0         = 30 / 3.6;        % 8.33 m/s

    scenario.steerDriver = @(t) 0;
    scenario.brakeCmd    = @(t) zeros(4, 1);

    % 노면 입력: 각 휠이 다른 시간에 bump 통과
    %   FL/FR는 동일 (좌우 동일 위상), RL/RR는 wheelbase/vx 만큼 지연
    bump_x_start = 5.0;   % m
    bump_len     = 1.5;
    bump_height  = 0.08;
    vx           = scenario.vx0;
    L_wb         = 2.047;
    scenario.z_road = @(t, w) local_bump(t, w, vx, bump_x_start, bump_len, bump_height, L_wb);
    scenario.mu_wheel = @(t, w) 1.0;

    scenario.kpis = {'rideRMS','rideVDV','suspPeakTravel','bodyAccPeak'};
end

function z = local_bump(t, wheel, vx, x_start, len, h, L_wb)
    % wheel ∈ {1=FL,2=FR,3=RL,4=RR}
    %  front wheels 위치 = vx*t
    %  rear wheels 위치 = vx*t - L_wb (지연됨)
    if wheel <= 2
        x = vx * t;
    else
        x = vx * t - L_wb;
    end
    if x >= x_start && x <= x_start + len
        z = h * (1 - cos(2*pi*(x - x_start)/len)) / 2;
    else
        z = 0;
    end
end
