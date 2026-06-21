%TEST_TIRE_FORCE Magic Formula 타이어 모델 유닛 테스트
%
%   Usage:
%       results = runtests('tests/test_tire_force.m')

%% Setup
TIRE.B = 12;
TIRE.C = 1.6;
TIRE.D = 1.0;
TIRE.E = -0.5;
Fz = 4000;  % [N] 정적 하중

%% Test 1: 슬립 앵글 0 → 횡력 0
Fy = calc_tire_force(0, Fz, TIRE);
assert(abs(Fy) < 1e-6, 'Zero slip should produce zero force');
fprintf('[PASS] Test 1: Zero slip → zero force\n');

%% Test 2: 양수 슬립 → 양수 횡력
Fy = calc_tire_force(deg2rad(2), Fz, TIRE);
assert(Fy > 0, 'Positive slip should produce positive force');
fprintf('[PASS] Test 2: Positive slip → positive force\n');

%% Test 3: 반대칭성
Fy_pos = calc_tire_force(deg2rad(3), Fz, TIRE);
Fy_neg = calc_tire_force(-deg2rad(3), Fz, TIRE);
assert(abs(Fy_pos + Fy_neg) < 1e-6, 'Force should be antisymmetric');
fprintf('[PASS] Test 3: Antisymmetry\n');

%% Test 4: 피크 힘 = mu * Fz 이하
alphaRange = deg2rad(linspace(-20, 20, 200));
FyRange = calc_tire_force(alphaRange, Fz, TIRE);
assert(max(abs(FyRange)) <= TIRE.D * Fz * 1.01, 'Peak force should not exceed mu*Fz');
fprintf('[PASS] Test 4: Peak force <= mu*Fz (peak: %.0f N, limit: %.0f N)\n', ...
    max(abs(FyRange)), TIRE.D * Fz);

%% Test 5: 벡터 입력
FyVec = calc_tire_force([deg2rad(1); deg2rad(3); deg2rad(5)], Fz, TIRE);
assert(numel(FyVec) == 3, 'Should handle vector input');
assert(all(FyVec > 0), 'All positive slips should give positive forces');
fprintf('[PASS] Test 5: Vector input handling\n');

%% Test 6: 하중 비례
Fy_low  = calc_tire_force(deg2rad(2), 2000, TIRE);
Fy_high = calc_tire_force(deg2rad(2), 6000, TIRE);
assert(Fy_high > Fy_low, 'Higher load should produce higher force');
fprintf('[PASS] Test 6: Load proportionality\n');

%% Summary
fprintf('\n=== All tire force tests PASSED ===\n');
