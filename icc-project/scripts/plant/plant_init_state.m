function plantState = plant_init_state(modelType, vx0, params)
%PLANT_INIT_STATE 플랜트 모델 상태 초기화
%
%   plantState = PLANT_INIT_STATE(modelType, vx0, params)
%
%   Inputs:
%       modelType - (char) 'bicycle' | '3dof' | '7dof' | '14dof'
%       vx0       - (double) 초기 종방향 속도 [m/s]
%       params    - (struct) 파라미터 (.VEH, .TIRE, .CONST)
%
%   Outputs:
%       plantState - (struct) 초기화된 플랜트 상태

    plantState.modelType = modelType;
    plantState.vx = vx0;

    switch modelType
        case 'bicycle'
            % [vy; yawRate]
            plantState.x = [0; 0];

        case '3dof'
            % [vx; vy; yawRate]
            plantState.x = [vx0; 0; 0];

        case '7dof'
            % [vx; vy; yawRate; omega_FL; omega_FR; omega_RL; omega_RR]
            omega0 = vx0 / params.VEH.rw;
            plantState.x = [vx0; 0; 0; omega0; omega0; omega0; omega0];

        case '14dof'
            % [vx; vy; yawRate; roll; rollRate; pitch; pitchRate;
            %  zs_FL; zs_FR; zs_RL; zs_RR;
            %  zsdot_FL; zsdot_FR; zsdot_RL; zsdot_RR;
            %  omega_FL; omega_FR; omega_RL; omega_RR]
            omega0 = vx0 / params.VEH.rw;
            plantState.x = zeros(19, 1);
            plantState.x(1) = vx0;
            plantState.x(16:19) = omega0;

        otherwise
            error('[plant_init_state] Unknown model type: %s', modelType);
    end

end
