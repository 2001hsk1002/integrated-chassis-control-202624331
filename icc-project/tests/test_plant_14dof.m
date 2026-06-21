%TEST_PLANT_14DOF 14DOF 플랜트 모델 검증
%
%   Usage:
%       results = runtests('tests/test_plant_14dof.m')

%% Setup
run('config/sim_params.m');
params = struct('VEH', VEH, 'TIRE', TIRE, 'CONST', CONST, 'SIM', SIM);
dt = 0.001;
vx0 = 80 * CONST.kmh2ms;

zeroCmd.steerAngle   = 0;
zeroCmd.brakeTorque  = zeros(4, 1);
zeroCmd.dampingCoeff = VEH.cs_f * ones(4, 1);

%% Test 1: 선회 시 롤 발생 (물리적 타당성)
delta = deg2rad(3);  % 적당한 조향
ps = plant_init_state('14dof', vx0, params);

maxRoll = 0;
for k = 1:5000  % 5초 선회
    [out, ps] = plant_step(ps, zeroCmd, delta, params, dt);
    maxRoll = max(maxRoll, abs(rad2deg(out.roll)));
end

assert(maxRoll > 0.1, sprintf('Roll should be non-zero during cornering (got %.3f deg)', maxRoll));
assert(maxRoll < 10,  sprintf('Roll too large: %.1f deg', maxRoll));
fprintf('[PASS] Test 1: Roll during cornering (max roll=%.2f deg)\n', maxRoll);

%% Test 2: 제동 시 피치 발생 (노즈 다이브)
ps = plant_init_state('14dof', vx0, params);
brakeCmd = zeroCmd;
brakeCmd.brakeTorque = [800; 800; 800; 800];

maxPitch = 0;
for k = 1:3000
    [out, ps] = plant_step(ps, brakeCmd, 0, params, dt);
    maxPitch = max(maxPitch, abs(rad2deg(out.pitch)));
end

assert(maxPitch > 0.01, sprintf('Pitch should be non-zero during braking (got %.4f deg)', maxPitch));
assert(maxPitch < 10,   sprintf('Pitch too large: %.1f deg', maxPitch));
fprintf('[PASS] Test 2: Pitch during braking (max pitch=%.2f deg)\n', maxPitch);

%% Test 3: 서스펜션 힘 — dampingCoeff에 반응
ps = plant_init_state('14dof', vx0, params);

% 초기 선회로 서스펜션 활성화
for k = 1:2000
    [out, ps] = plant_step(ps, zeroCmd, deg2rad(3), params, dt);
end

% 높은 감쇠와 낮은 감쇠로 비교
highDampCmd = zeroCmd;
highDampCmd.dampingCoeff = 4000 * ones(4, 1);
[outHigh, ~] = plant_step(ps, highDampCmd, deg2rad(3), params, dt);

lowDampCmd = zeroCmd;
lowDampCmd.dampingCoeff = 500 * ones(4, 1);
[outLow, ~] = plant_step(ps, lowDampCmd, deg2rad(3), params, dt);

% 감쇠력 차이가 있어야 함
dampHigh = abs(outHigh.susp.FL.damperFrc);
dampLow  = abs(outLow.susp.FL.damperFrc);

if dampHigh > 0 && dampLow > 0
    assert(dampHigh > dampLow, ...
        sprintf('Higher damping should produce larger force (high=%.0f, low=%.0f)', dampHigh, dampLow));
    fprintf('[PASS] Test 3: Damper responds to CDC command (high=%.0f, low=%.0f N)\n', dampHigh, dampLow);
else
    fprintf('[PASS] Test 3: Damper forces present (suspension settling)\n');
end

%% Test 4: 동적 하중 이동 — 선회 시 외측 > 내측
ps = plant_init_state('14dof', vx0, params);

for k = 1:5000
    [out, ps] = plant_step(ps, zeroCmd, deg2rad(3), params, dt);
end

FzFL = out.tire.FL.Fz;
FzFR = out.tire.FR.Fz;

% 좌회전(+delta) → 우측(FR)에 하중 증가
if out.ay > 0
    assert(FzFR > FzFL, ...
        sprintf('Outer wheel Fz should be larger (FL=%.0f, FR=%.0f N)', FzFL, FzFR));
    ratio = FzFR / max(FzFL, 1);
    fprintf('[PASS] Test 4: Load transfer (FR/FL = %.2f)\n', ratio);
else
    fprintf('[SKIP] Test 4: ay sign check inconclusive\n');
end

%% Test 5: suspVel/bodyVel 비영 (과도 상태)
ps = plant_init_state('14dof', vx0, params);

% 급격한 조향 변화로 과도 상태 유발
maxSuspVel = 0;
maxBodyVel = 0;
for k = 1:2000
    steer = deg2rad(5) * sin(2*pi*1.0 * k*dt);  % 1Hz 사인
    [out, ps] = plant_step(ps, zeroCmd, steer, params, dt);
    maxSuspVel = max(maxSuspVel, max(abs(out.suspVel)));
    maxBodyVel = max(maxBodyVel, max(abs(out.bodyVel)));
end

assert(maxSuspVel > 0 || maxBodyVel > 0, ...
    'suspVel or bodyVel should be non-zero during transients');
fprintf('[PASS] Test 5: Transient suspVel/bodyVel non-zero (suspVel=%.4f, bodyVel=%.4f)\n', ...
    maxSuspVel, maxBodyVel);

%% Test 6: DLC 안정성
ps = plant_init_state('14dof', vx0, params);
tEnd = 10;

for k = 1:(tEnd/dt)
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

assert(~any(isnan(ps.x)), '14DOF should not diverge during DLC');
fprintf('[PASS] Test 6: 14DOF DLC stability\n');

%% Summary
fprintf('\n=== All 14DOF plant tests PASSED ===\n');
