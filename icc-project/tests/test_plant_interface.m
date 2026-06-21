%TEST_PLANT_INTERFACE 전 플랜트 모델 출력 인터페이스 검증
%
%   모든 플랜트 모델(bicycle/3dof/7dof/14dof)이 동일한 출력 구조체를
%   올바르게 생성하는지 검증한다.
%
%   Usage:
%       results = runtests('tests/test_plant_interface.m')

%% Setup
run('config/sim_params.m');
params = struct('VEH', VEH, 'TIRE', TIRE, 'CONST', CONST, 'SIM', SIM);
vx0 = 20;  % [m/s]
dt  = 0.001;

zeroCmd.steerAngle   = 0;
zeroCmd.brakeTorque  = zeros(4, 1);
zeroCmd.dampingCoeff = 1500 * ones(4, 1);

models = {'bicycle', '3dof', '7dof', '14dof'};

requiredFields = {'vx','vy','ax','ay','yawRate','slipAngle','roll','pitch','suspVel','bodyVel','tire','susp'};
tireFields = {'Fx','Fy','Fz','slipAngle','slipRatio'};
suspFields = {'springFrc','damperFrc'};
wheels = {'FL','FR','RL','RR'};

%% Test 1: 모든 모델이 필수 필드를 출력
for m = 1:numel(models)
    modelName = models{m};
    ps = plant_init_state(modelName, vx0, params);
    [out, ~] = plant_step(ps, zeroCmd, 0, params, dt);

    for f = 1:numel(requiredFields)
        assert(isfield(out, requiredFields{f}), ...
            sprintf('[%s] Missing field: %s', modelName, requiredFields{f}));
    end

    % 타이어 하위 필드
    for w = 1:4
        for f = 1:numel(tireFields)
            assert(isfield(out.tire.(wheels{w}), tireFields{f}), ...
                sprintf('[%s] Missing tire.%s.%s', modelName, wheels{w}, tireFields{f}));
        end
    end

    % 서스펜션 하위 필드
    for w = 1:4
        for f = 1:numel(suspFields)
            assert(isfield(out.susp.(wheels{w}), suspFields{f}), ...
                sprintf('[%s] Missing susp.%s.%s', modelName, wheels{w}, suspFields{f}));
        end
    end

    fprintf('[PASS] Test 1 (%s): All required fields present\n', modelName);
end

%% Test 2: 100 스텝 실행 후 NaN/Inf 없음
for m = 1:numel(models)
    modelName = models{m};
    ps = plant_init_state(modelName, vx0, params);

    for k = 1:100
        [out, ps] = plant_step(ps, zeroCmd, 0, params, dt);
    end

    assert(~isnan(out.vx),   sprintf('[%s] vx is NaN', modelName));
    assert(~isinf(out.vx),   sprintf('[%s] vx is Inf', modelName));
    assert(~isnan(out.vy),   sprintf('[%s] vy is NaN', modelName));
    assert(~isnan(out.yawRate), sprintf('[%s] yawRate is NaN', modelName));
    assert(~isnan(out.ay),   sprintf('[%s] ay is NaN', modelName));

    fprintf('[PASS] Test 2 (%s): No NaN/Inf after 100 steps\n', modelName);
end

%% Test 3: 제로 입력 시 직진 유지 (vx > 0, yawRate ≈ 0)
for m = 1:numel(models)
    modelName = models{m};
    ps = plant_init_state(modelName, vx0, params);

    for k = 1:500
        [out, ps] = plant_step(ps, zeroCmd, 0, params, dt);
    end

    assert(out.vx > 0, sprintf('[%s] vx should be positive', modelName));
    assert(abs(out.yawRate) < deg2rad(0.1), ...
        sprintf('[%s] yawRate should be ~0 with zero steer (got %.4f deg/s)', ...
        modelName, rad2deg(out.yawRate)));

    fprintf('[PASS] Test 3 (%s): Straight-line stability (vx=%.1f, yr=%.4f deg/s)\n', ...
        modelName, out.vx, rad2deg(out.yawRate));
end

%% Test 4: suspVel/bodyVel 크기 검증
for m = 1:numel(models)
    modelName = models{m};
    ps = plant_init_state(modelName, vx0, params);
    [out, ~] = plant_step(ps, zeroCmd, 0, params, dt);

    assert(numel(out.suspVel) == 4, sprintf('[%s] suspVel should be 4x1', modelName));
    assert(numel(out.bodyVel) == 4, sprintf('[%s] bodyVel should be 4x1', modelName));

    fprintf('[PASS] Test 4 (%s): suspVel/bodyVel are 4x1\n', modelName);
end

%% Summary
fprintf('\n=== All plant interface tests PASSED ===\n');
