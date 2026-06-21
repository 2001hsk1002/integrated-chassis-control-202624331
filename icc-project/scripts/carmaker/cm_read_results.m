function result = cm_read_results(ergFile)
%CM_READ_RESULTS CarMaker .erg 결과 파일 읽기 및 구조체 변환
%
%   result = CM_READ_RESULTS(ergFile)
%
%   CarMaker의 .erg 결과 파일을 읽어 표준 결과 구조체로 변환한다.
%   cmread() 함수가 필요하다 (CarMaker MATLAB 툴박스).
%
%   Inputs:
%       ergFile - (char) .erg 파일 경로
%
%   Outputs:
%       result - (struct) 표준화된 결과 구조체

    if ~isfile(ergFile)
        error('[cm_read_results] File not found: %s', ergFile);
    end

    %% 원시 데이터 읽기
    raw = cmread(ergFile);

    %% 차체 운동 상태
    result.time      = raw.Time;
    result.vx        = raw.Car_vx;
    result.vy        = raw.Car_vy;
    result.ax        = raw.Car_ax;
    result.ay        = raw.Car_ay;
    result.yawRate   = raw.Car_YawRate;
    result.slipAngle = atan2(raw.Car_vy, max(abs(raw.Car_vx), 1));
    result.roll      = raw.Car_Roll;
    result.pitch     = raw.Car_Pitch;

    %% 운전자 입력
    result.steerAngle = raw.Driver_Steer_Ang;
    result.brakePedal = raw.Driver_Brake;
    result.gasPedal   = raw.Driver_Gas;

    %% 4륜 타이어 정보
    wheels = {'FL', 'FR', 'RL', 'RR'};
    for w = 1:4
        wn = wheels{w};
        result.tire.(wn).Fx        = raw.(['Tire_' wn '_Fx']);
        result.tire.(wn).Fy        = raw.(['Tire_' wn '_Fy']);
        result.tire.(wn).Fz        = raw.(['Tire_' wn '_Fz']);
        result.tire.(wn).slipAngle = raw.(['Tire_' wn '_SlipAngle']);
        result.tire.(wn).slipRatio = raw.(['Tire_' wn '_SlipRatio']);
    end

    %% 서스펜션 정보
    for w = 1:4
        wn = wheels{w};
        result.susp.(wn).springFl  = raw.(['Susp_' wn '_Spring_Fl']);
        result.susp.(wn).damperFrc = raw.(['Susp_' wn '_Damper_Frc']);
    end

    %% LTR 계산
    Fz_left  = result.tire.FL.Fz + result.tire.RL.Fz;
    Fz_right = result.tire.FR.Fz + result.tire.RR.Fz;
    Fz_total = Fz_left + Fz_right;
    Fz_total(Fz_total == 0) = eps;
    result.LTR = (Fz_right - Fz_left) ./ Fz_total;

    %% 저크 계산
    dt = mean(diff(result.time));
    result.jerk = [0; diff(result.ax)] / dt;

    fprintf('[cm_read_results] Loaded: %s (%.1f s, %d samples)\n', ...
            ergFile, result.time(end), numel(result.time));

end
