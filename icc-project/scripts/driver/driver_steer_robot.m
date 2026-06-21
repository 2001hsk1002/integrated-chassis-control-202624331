function [delta, drvState] = driver_steer_robot(scenario, ~, ~, t, drvState)
%DRIVER_STEER_ROBOT 강제(open-loop) 조향 — scenario.steerDriver(t) identity wrapper
%
%   [delta, drvState] = DRIVER_STEER_ROBOT(scenario, pose, vx, t, drvState)
%
%   A3 step / A5 sine-with-dwell / A6 sweep 처럼 운전자 모델 없이 시변 함수로
%   조향각을 직접 인가하는 시나리오용. 차량 응답과 무관하게 t만 보고 출력.

    delta = scenario.steerDriver(t);
end
