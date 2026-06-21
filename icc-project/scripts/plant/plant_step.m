function [plantOut, plantState] = plant_step(plantState, actuatorCmd, steerDriver, params, dt)
%PLANT_STEP 통합 플랜트 모델 스텝 함수
%
%   [plantOut, plantState] = PLANT_STEP(plantState, actuatorCmd, steerDriver, params, dt)
%
%   선택된 플랜트 모델(bicycle/3dof/7dof/14dof)로 한 스텝을 진행한다.
%   제어기 코드 변경 없이 플랜트만 교체 가능하도록 통합 인터페이스를 제공한다.
%
%   Inputs:
%       plantState  - (struct) 현재 플랜트 상태 (.modelType, .x, .vx 등)
%       actuatorCmd - (struct) ctrl_coordinator 출력
%           .steerAngle   - 보조 조향각 [rad]
%           .brakeTorque  - 개별 브레이크 토크 (4x1) [Nm] [FL;FR;RL;RR]
%           .dampingCoeff - 개별 감쇠 계수 (4x1) [Ns/m]
%       steerDriver - (double) 운전자 조향 입력 [rad]
%       params      - (struct) 파라미터 (.VEH, .TIRE, .CONST, .SIM)
%       dt          - (double) 시간 스텝 [s]
%
%   Outputs:
%       plantOut   - (struct) 표준화된 플랜트 출력
%           .vx, .vy, .ax, .ay       - 차체 속도/가속도
%           .yawRate, .slipAngle      - 요 레이트, 차체 슬립 앵글
%           .roll, .pitch             - 롤/피치 각도
%           .suspVel (4x1)            - 서스펜션 상대 속도 (ctrl_vertical용)
%           .bodyVel (4x1)            - 차체 수직 속도 (ctrl_vertical용)
%           .tire.{FL,FR,RL,RR}       - 타이어 힘/슬립 데이터
%           .susp.{FL,FR,RL,RR}       - 서스펜션 힘 데이터
%       plantState - (struct) 업데이트된 플랜트 상태

    switch plantState.modelType
        case 'bicycle'
            [plantOut, plantState] = plant_bicycle(plantState, actuatorCmd, steerDriver, params, dt);
        case '3dof'
            [plantOut, plantState] = plant_3dof(plantState, actuatorCmd, steerDriver, params, dt);
        case '7dof'
            [plantOut, plantState] = plant_7dof(plantState, actuatorCmd, steerDriver, params, dt);
        case '14dof'
            [plantOut, plantState] = plant_14dof(plantState, actuatorCmd, steerDriver, params, dt);
        otherwise
            error('[plant_step] Unknown plant model: %s', plantState.modelType);
    end

end
