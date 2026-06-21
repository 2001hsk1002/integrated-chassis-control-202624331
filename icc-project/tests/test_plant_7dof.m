%TEST_PLANT_7DOF 7DOF 플랜트 모델 검증
%
%   Usage:
%       results = runtests('tests/test_plant_7dof.m')

%% Setup
run('config/sim_params.m');
params = struct('VEH', VEH, 'TIRE', TIRE, 'CONST', CONST, 'SIM', SIM);
dt = 0.001;
vx0 = 80 * CONST.kmh2ms;

zeroCmd.steerAngle   = 0;
zeroCmd.brakeTorque  = zeros(4, 1);
zeroCmd.dampingCoeff = 1500 * ones(4, 1);

%% Test 1: 슬립 비 생성 — 제동 시 slipRatio ≠ 0
ps = plant_init_state('7dof', vx0, params);
brakeCmd = zeroCmd;
brakeCmd.brakeTorque = [1000; 1000; 1000; 1000];

minKappa = 0;
for k = 1:3000  % 3초
    [out, ps] = plant_step(ps, brakeCmd, 0, params, dt);
    kFL = out.tire.FL.slipRatio;
    minKappa = min(minKappa, kFL);
end

assert(minKappa < -0.01, ...
    sprintf('Slip ratio should be negative during braking (got %.4f)', minKappa));
fprintf('[PASS] Test 1: Slip ratio generated during braking (min kappa=%.4f)\n', minKappa);

%% Test 2: 과제동 — 바퀴 잠김 감지
ps = plant_init_state('7dof', vx0, params);
heavyBrake = zeroCmd;
heavyBrake.brakeTorque = [3000; 3000; 3000; 3000];  % 최대 브레이크

for k = 1:5000
    [out, ps] = plant_step(ps, heavyBrake, 0, params, dt);
end

% 바퀴 속도가 매우 낮아야 함 (잠김 근접)
omega_FL = ps.x(4);
assert(omega_FL < vx0 / VEH.rw * 0.5, ...
    sprintf('Wheel should be near lockup (omega=%.1f rad/s)', omega_FL));
fprintf('[PASS] Test 2: Heavy braking wheel slowdown (omega_FL=%.1f rad/s)\n', omega_FL);

%% Test 3: 3DOF 일관성 — 저 슬립에서 요 레이트 ±10% 일치
delta = deg2rad(2);

% 7DOF
ps7 = plant_init_state('7dof', vx0, params);
for k = 1:10000
    [out7, ps7] = plant_step(ps7, zeroCmd, delta, params, dt);
end

% 3DOF
ps3 = plant_init_state('3dof', vx0, params);
for k = 1:10000
    [out3, ps3] = plant_step(ps3, zeroCmd, delta, params, dt);
end

relError = abs(out7.yawRate - out3.yawRate) / abs(out3.yawRate) * 100;
assert(relError < 10, ...
    sprintf('7DOF vs 3DOF yaw rate mismatch: %.1f%%', relError));
fprintf('[PASS] Test 3: 7DOF/3DOF consistency (yaw rate error=%.1f%%)\n', relError);

%% Test 4: 제로 입력 시 슬립 비 ≈ 0
ps = plant_init_state('7dof', vx0, params);
for k = 1:1000
    [out, ps] = plant_step(ps, zeroCmd, 0, params, dt);
end

assert(abs(out.tire.FL.slipRatio) < 0.01, ...
    sprintf('Slip ratio should be ~0 without braking (got %.4f)', out.tire.FL.slipRatio));
fprintf('[PASS] Test 4: Zero input → near-zero slip ratio\n');

%% Test 5: DLC 안정성
ps = plant_init_state('7dof', vx0, params);
tEnd = 10;
nSteps = tEnd / dt;

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
end

assert(~any(isnan(ps.x)), 'DLC should not cause divergence in 7DOF');
assert(abs(rad2deg(out.slipAngle)) < 5, 'Slip angle should stay safe during DLC');
fprintf('[PASS] Test 5: 7DOF DLC stability\n');

%% Summary
fprintf('\n=== All 7DOF plant tests PASSED ===\n');
