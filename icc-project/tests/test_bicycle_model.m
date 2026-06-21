%TEST_BICYCLE_MODEL Bicycle Model 유닛 테스트
%
%   이 테스트 스크립트는 calc_bicycle_model, calc_ref_yaw_rate의
%   기본 동작과 물리적 정합성을 검증한다.
%
%   Usage:
%       results = runtests('tests/test_bicycle_model.m')

%% Setup: 차량 파라미터
VEH.mass = 1500;
VEH.Iz   = 2500;
VEH.lf   = 1.2;
VEH.lr   = 1.4;
VEH.L    = VEH.lf + VEH.lr;
VEH.Cf   = 80000;
VEH.Cr   = 85000;

%% Test 1: 상태공간 행렬 크기
vx = 20;
[A, B, C, D] = calc_bicycle_model(vx, VEH);
assert(all(size(A) == [2 2]), 'A matrix should be 2x2');
assert(all(size(B) == [2 1]), 'B matrix should be 2x1');
assert(all(size(C) == [2 2]), 'C matrix should be 2x2');
assert(all(size(D) == [2 1]), 'D matrix should be 2x1');
fprintf('[PASS] Test 1: Matrix dimensions\n');

%% Test 2: A 행렬 안정성 (고유값 음수 실수부)
eigenValues = eig(A);
assert(all(real(eigenValues) < 0), 'System should be stable at vx=20 m/s');
fprintf('[PASS] Test 2: Eigenvalue stability (vx=20 m/s)\n');

%% Test 3: 목표 요 레이트 — 직진 시 0
yrRef = calc_ref_yaw_rate(20, 0, VEH);
assert(abs(yrRef) < 1e-10, 'Yaw rate ref should be 0 for zero steer');
fprintf('[PASS] Test 3: Zero steer → zero yaw rate\n');

%% Test 4: 목표 요 레이트 — 부호 일관성
yrLeft  = calc_ref_yaw_rate(20,  deg2rad(2), VEH);
yrRight = calc_ref_yaw_rate(20, -deg2rad(2), VEH);
assert(yrLeft > 0,  'Left steer should produce positive yaw rate');
assert(yrRight < 0, 'Right steer should produce negative yaw rate');
assert(abs(yrLeft + yrRight) < 1e-10, 'Yaw rate should be antisymmetric');
fprintf('[PASS] Test 4: Yaw rate sign consistency\n');

%% Test 5: 속도 증가 → 요 레이트 감쇠 (언더스티어)
delta = deg2rad(2);
yr_low  = calc_ref_yaw_rate(10, delta, VEH);
yr_high = calc_ref_yaw_rate(40, delta, VEH);
% 언더스티어 차량: 고속에서 r/delta 감소 → r_high/vx_high < r_low/vx_low
gain_low  = yr_low  / 10;  % yaw rate gain = r / vx
gain_high = yr_high / 40;

% Kus 계산으로 언더스티어 확인
Kus = (VEH.mass*VEH.lr)/(2*VEH.Cf*VEH.L) - (VEH.mass*VEH.lf)/(2*VEH.Cr*VEH.L);
if Kus > 0
    assert(gain_high < gain_low, 'Understeer: yaw rate gain should decrease with speed');
    fprintf('[PASS] Test 5: Understeer gradient (Kus = %.4f)\n', Kus);
else
    fprintf('[SKIP] Test 5: Vehicle is oversteer (Kus = %.4f)\n', Kus);
end

%% Test 6: 극저속 보호
yrSlow = calc_ref_yaw_rate(0.5, deg2rad(10), VEH);
assert(yrSlow == 0, 'Yaw rate should be 0 below vx threshold');
fprintf('[PASS] Test 6: Low speed protection\n');

%% Test 7: 벡터 입력 처리
vxVec    = [10; 20; 30];
deltaVec = deg2rad([1; 2; 3]);
yrVec = calc_ref_yaw_rate(vxVec, deltaVec, VEH);
assert(numel(yrVec) == 3, 'Should handle vector inputs');
fprintf('[PASS] Test 7: Vector input handling\n');

%% Summary
fprintf('\n=== All bicycle model tests PASSED ===\n');
