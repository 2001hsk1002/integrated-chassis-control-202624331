%TEST_PLANT_3DOF 3DOF 플랜트 모델 검증
%
%   Usage:
%       results = runtests('tests/test_plant_3dof.m')

%% Setup
run('config/sim_params.m');
params = struct('VEH', VEH, 'TIRE', TIRE, 'CONST', CONST, 'SIM', SIM);
dt = 0.001;
vx0 = 80 * CONST.kmh2ms;

zeroCmd.steerAngle   = 0;
zeroCmd.brakeTorque  = zeros(4, 1);
zeroCmd.dampingCoeff = 1500 * ones(4, 1);

%% Test 1: 정상상태 요 레이트 — Bicycle 레퍼런스와 ±5% 일치
delta = deg2rad(2);
ps = plant_init_state('3dof', vx0, params);

cmd = zeroCmd;
for k = 1:10000  % 10초
    [out, ps] = plant_step(ps, cmd, delta, params, dt);
end

% bicycle ref는 현재 vx 기준 — 시뮬레이션 동안 vx 감속(타이어 스크러브+공력)을 반영
yrRef = calc_ref_yaw_rate(out.vx, delta, VEH);
yrActual = out.yawRate;
relError = abs(yrActual - yrRef) / abs(yrRef) * 100;

assert(relError < 5, ...
    sprintf('Steady-state yaw rate error too large: %.1f%% (actual=%.3f, ref=%.3f deg/s)', ...
    relError, rad2deg(yrActual), rad2deg(yrRef)));
fprintf('[PASS] Test 1: Steady-state yaw rate (error=%.1f%%, vx=%.1f m/s)\n', relError, out.vx);

%% Test 2: 비선형 타이어 포화 — 큰 조향에서 횡력 포화
delta_large = deg2rad(10);
ps = plant_init_state('3dof', vx0, params);

maxAy = 0;
for k = 1:10000
    [out, ps] = plant_step(ps, zeroCmd, delta_large, params, dt);
    maxAy = max(maxAy, abs(out.ay));
end

% ay 한계: mu * g ≈ 9.81 m/s^2 근처에서 포화
assert(maxAy < TIRE.D * CONST.g * 1.2, ...
    sprintf('Lateral accel exceeds tire limit: %.1f m/s^2', maxAy));
assert(maxAy > 3.0, ...
    sprintf('Lateral accel too low for large steer: %.1f m/s^2', maxAy));
fprintf('[PASS] Test 2: Tire saturation (max ay=%.1f m/s^2)\n', maxAy);

%% Test 3: 제동 감속 — 브레이크 입력 시 vx 감소
ps = plant_init_state('3dof', vx0, params);
brakeCmd = zeroCmd;
brakeCmd.brakeTorque = [500; 500; 500; 500];  % 전 바퀴 500 Nm

for k = 1:5000  % 5초
    [out, ps] = plant_step(ps, brakeCmd, 0, params, dt);
end

assert(out.vx < vx0, 'Speed should decrease with braking');
assert(out.vx > 0,   'Speed should remain positive');
fprintf('[PASS] Test 3: Braking deceleration (vx: %.1f → %.1f m/s)\n', vx0, out.vx);

%% Test 4: 에너지 보존 — 입력 없으면 공기저항만으로 감속
ps = plant_init_state('3dof', vx0, params);

prevVx = vx0;
for k = 1:5000
    [out, ps] = plant_step(ps, zeroCmd, 0, params, dt);
    assert(out.vx <= prevVx + 1e-6, 'Speed should decrease monotonically (aero drag)');
    prevVx = out.vx;
end
fprintf('[PASS] Test 4: Monotonic speed decay (aero drag only, final vx=%.2f)\n', out.vx);

%% Test 5: DLC 시뮬레이션 안정성
ps = plant_init_state('3dof', vx0, params);
tEnd = 10;
nSteps = tEnd / dt;

maxBeta = 0;
for k = 1:nSteps
    tNow = k * dt;
    if tNow >= 1 && tNow < 3
        steer = deg2rad(3) * sin(pi * (tNow - 1) / 2);
    elseif tNow >= 3 && tNow < 5
        steer = -deg2rad(3) * sin(pi * (tNow - 3) / 2);
    else
        steer = 0;
    end
    [out, ps] = plant_step(ps, zeroCmd, steer, params, dt);
    maxBeta = max(maxBeta, abs(rad2deg(out.slipAngle)));
end

assert(~any(isnan(ps.x)), 'Simulation should not diverge during DLC');
assert(maxBeta < 5, sprintf('Slip angle too large during DLC: %.1f deg', maxBeta));
fprintf('[PASS] Test 5: DLC stability (max beta=%.2f deg)\n', maxBeta);

%% Summary
fprintf('\n=== All 3DOF plant tests PASSED ===\n');
