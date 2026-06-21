%SIM_PARAMS ICC 시뮬레이션 공통 파라미터 정의
%   프로젝트 전체에서 사용되는 차량/제어기/시뮬레이션 파라미터를 설정한다.
%   이 스크립트는 init_project.m에서 자동 호출된다.

%% 시뮬레이션 설정
SIM.dt       = 0.001;   % [s] 고정 스텝 크기 (controller sample time — 변경 금지)
SIM.t_end    = 30;      % [s] 기본 시뮬레이션 종료 시간
SIM.out_rate = 100;     % [Hz] 출력 저장 레이트

%% Plant 적분기 선택 (substep 내부 적분 방식 — outer dt 는 유지)
%   'ode45'  : MATLAB adaptive RK4-5 (Dormand-Prince) — default
%   'ode23'  : MATLAB adaptive RK2-3 (Bogacki-Shampine) — 빠르나 정확도 낮음
%   'ode15s' : MATLAB stiff solver (BDF) — 저속/포화 영역에서 안정
%   'rk4'    : 고정 스텝 4차 Runge-Kutta (3DOF/7DOF/14DOF 본래 방식)
%   'euler'  : 1차 전진 Euler (bicycle 본래 방식, 학습/비교용)
SIM.solver        = 'ode45';
SIM.solver_RelTol = 1e-3;   % ode* 상대 오차 허용치
SIM.solver_AbsTol = 1e-5;   % ode* 절대 오차 허용치

%% 물리 상수
CONST.g       = 9.81;    % [m/s^2] 중력 가속도
CONST.rho_air = 1.225;   % [kg/m^3] 공기 밀도
CONST.kmh2ms  = 1/3.6;   % km/h → m/s 변환 계수

%% 차량 파라미터 (C-segment Sedan)
VEH.mass    = 1500;    % [kg] 차량 총 질량
VEH.Iz      = 2500;    % [kg*m^2] 요 관성 모멘트
VEH.lf      = 1.2;     % [m] CG-전축 거리
VEH.lr      = 1.4;     % [m] CG-후축 거리
VEH.L       = VEH.lf + VEH.lr;  % [m] 축간 거리
VEH.Cf      = 80000;   % [N/rad] 전륜 코너링 강성
VEH.Cr      = 85000;   % [N/rad] 후륜 코너링 강성
VEH.h_cog   = 0.55;    % [m] CG 높이
VEH.track_f = 1.55;    % [m] 전륜 트레드
VEH.track_r = 1.55;    % [m] 후륜 트레드
VEH.Cd      = 0.30;    % [-] 공기저항 계수
VEH.Af      = 2.2;     % [m^2] 전면 투영 면적
VEH.rw      = 0.31;    % [m] 타이어 유효 반경

%% 타이어 파라미터 (Magic Formula 간이 모델)
TIRE.B  = 12;      % Stiffness factor
TIRE.C  = 1.6;     % Shape factor
TIRE.D  = 1.0;     % Peak friction coefficient (mu_peak)
TIRE.E  = -0.5;    % Curvature factor
TIRE.mu_peak = 1.0;  % 최대 마찰 계수

%% 제어기 파라미터 — 횡방향 (Lateral)
CTRL.LAT.Kp     = 1.0;     % 비례 게인
CTRL.LAT.Ki     = 0.1;     % 적분 게인
CTRL.LAT.Kd     = 0.05;    % 미분 게인
CTRL.LAT.intMax = 5.0;     % 적분 안티와인드업 한계 [rad]

%% 제어기 파라미터 — 종방향 (Longitudinal)
CTRL.LON.Kp     = 0.5;     % 비례 게인
CTRL.LON.Ki     = 0.05;    % 적분 게인
CTRL.LON.intMax = 2000;    % 적분 안티와인드업 한계 [Nm]

%% 제어기 파라미터 — 수직 (Vertical / CDC)
CTRL.VER.cMin    = 500;    % [Ns/m] 최소 감쇠 계수
CTRL.VER.cMax    = 5000;   % [Ns/m] 최대 감쇠 계수
CTRL.VER.skyGain = 2500;   % [Ns/m] Skyhook 게인

%% 제어기 파라미터 — 통합 조율기 (Coordinator)
CTRL.COORD.wLat  = 1.0;    % 횡방향 가중치
CTRL.COORD.wLon  = 1.0;    % 종방향 가중치
CTRL.COORD.wVer  = 0.5;    % 수직 가중치
CTRL.COORD.wEff  = 0.1;    % 에너지 효율 가중치

%% 액추에이터 한계
LIM.MAX_STEER_ANGLE = deg2rad(540 / 15);  % [rad] 최대 로드휠 조향각 (SW 540deg / ratio 15)
LIM.MAX_STEER_RATE  = deg2rad(500 / 15);  % [rad/s] 최대 조향 속도
LIM.MAX_BRAKE_TRQ   = 3000;   % [Nm] 최대 브레이크 토크 (per wheel)
LIM.MAX_AX          = 10.0;   % [m/s^2] 최대 종가속도
LIM.MAX_AY          = 10.0;   % [m/s^2] 최대 횡가속도
LIM.MAX_JERK        = 50.0;   % [m/s^3] 최대 저크
LIM.MAX_YAW_RATE    = deg2rad(60);   % [rad/s] 최대 요 레이트
LIM.MAX_SLIP_ANGLE  = deg2rad(12);   % [rad] 최대 슬립 앵글

%% Plant Model 선택
SIM.plantModel = 'bicycle';  % 'bicycle' | '3dof' | '7dof' | '14dof'

%% 확장 차량 파라미터 (7DOF/14DOF용)
VEH.Ix    = 600;      % [kg*m^2] 롤 관성 모멘트
VEH.Iy    = 2000;     % [kg*m^2] 피치 관성 모멘트
VEH.Iw    = 1.5;      % [kg*m^2] 바퀴 회전 관성
VEH.ms    = 1350;     % [kg] 스프렁 매스
VEH.mu_w  = 37.5;     % [kg] 언스프렁 매스 (per wheel)
VEH.ks_f  = 25000;    % [N/m] 전륜 스프링 강성
VEH.ks_r  = 22000;    % [N/m] 후륜 스프링 강성
VEH.cs_f  = 1500;     % [Ns/m] 전륜 기본 감쇠 계수
VEH.cs_r  = 1400;     % [Ns/m] 후륜 기본 감쇠 계수
VEH.kt    = 200000;   % [N/m] 타이어 수직 강성
VEH.hrc   = 0.45;     % [m] 롤 센터 높이
VEH.antiDive = 0.70;  % [-] 안티다이브 비율 (0~1) — BMW_5 CarMaker 측정 기반
VEH.antiSquat = 0.30; % [-] 안티스쿼트 비율 (가속 시 nose-up 억제)

%% 종방향 타이어 파라미터 (7DOF 이상, Magic Formula)
TIRE.Bx = 14;       % Longitudinal stiffness factor
TIRE.Cx = 1.65;     % Longitudinal shape factor
TIRE.Dx = 1.1;      % Longitudinal peak friction coefficient
TIRE.Ex = -0.3;     % Longitudinal curvature factor

%% 타이어 모델 선택 — multi-model dispatcher용
%   'simple_mf' : 현재 4-param Pacejka (backward compat, 빠른 반복)
%   'full_mf'   : Pacejka MF 5.2 풀 (~30 계수, load sensitivity + camber)
%                 cm_to_plant_params에서 .tir 자동 로딩 시 활성
%   'rt_proxy'  : CarMaker RT tire (BMW_5) 근사 — 회귀된 MF (현재 simple_mf와 동등)
TIRE.model = 'simple_mf';

% Full MF 활성 시 사용할 .tir 파일 (옵션)
SIM.tire_tir_file = 'C:/IPG/carmaker/win64-15.0/Data/Tire/Examples/TirePropertyFile/MF_205_60R15_V91.tir';

%% 차량 셋 선택 (실차/일반)
% 'generic'      : 위에 정의된 C-segment 일반값 그대로 사용
% 'bmw5_cm15'    : CarMaker BMW_5_15_030326 INFOFILE에서 추출한 실차 파라미터
SIM.vehicleSet = 'bmw5_cm15';

SIM.cm_vehicleFile = 'C:/Users/VIC/Projects/carmaker_data/Data/Vehicle/BMW_5_15_030326';

switch SIM.vehicleSet
    case 'generic'
        % VEH 그대로 사용
    case 'bmw5_cm15'
        if isfile(SIM.cm_vehicleFile)
            VEH = cm_to_plant_params(SIM.cm_vehicleFile, VEH);
            fprintf('[sim_params] Vehicle params loaded from CarMaker BMW_5 (%s).\n', ...
                    SIM.cm_vehicleFile);
        else
            warning('[sim_params] CarMaker vehicle file not found, falling back to generic. (%s)', ...
                    SIM.cm_vehicleFile);
            SIM.vehicleSet = 'generic';
        end
    otherwise
        warning('[sim_params] Unknown vehicleSet "%s", using generic.', SIM.vehicleSet);
        SIM.vehicleSet = 'generic';
end

fprintf('[sim_params] Parameters loaded. Plant=%s, VehicleSet=%s\n', ...
        SIM.plantModel, SIM.vehicleSet);
