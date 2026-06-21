function [delta, drvState] = driver_dispatch(scenario, pose, vx, t, drvState)
%DRIVER_DISPATCH 시나리오의 driverType 에 따라 driver_path_follow 또는 driver_steer_robot 호출
%
%   [delta, drvState] = DRIVER_DISPATCH(scenario, pose, vx, t, drvState)
%
%   scenario.driverType (없으면 'robot' 추론):
%       'robot'                        → steer_robot (forced function)
%       'path_follow_stanley' (또는 'path_follow') → path_follow Stanley
%       'path_follow_purepursuit'      → path_follow Pure Pursuit
%       'open_loop'                    → δ=0 (조향 입력 없음, brake/road만)

    if ~isfield(scenario, 'driverType') || isempty(scenario.driverType)
        scenario.driverType = 'robot';
    end

    switch lower(scenario.driverType)
        case 'robot'
            [delta, drvState] = driver_steer_robot(scenario, pose, vx, t, drvState);
        case {'path_follow', 'path_follow_stanley', 'path_follow_purepursuit'}
            [delta, drvState] = driver_path_follow(scenario, pose, vx, t, drvState);
        case 'open_loop'
            delta = 0;
        otherwise
            warning('[driver_dispatch] Unknown driverType "%s", defaulting to robot', ...
                    scenario.driverType);
            [delta, drvState] = driver_steer_robot(scenario, pose, vx, t, drvState);
    end
end
