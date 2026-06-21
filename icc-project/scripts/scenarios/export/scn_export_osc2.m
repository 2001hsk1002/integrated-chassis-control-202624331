function oscPath = scn_export_osc2(scenario, outDir, opts)
%SCN_EXPORT_OSC2 우리 scenario → ASAM OpenSCENARIO 2.0 (.osc DSL) — CarMaker 13/15 호환
%
%   OSC 2.0은 1.x XML과 달리 **declarative DSL** (Python-like 텍스트). 본 함수는
%   minimal OSC 2.0 syntax로 우리 시나리오 핵심 (ego 차량, 초기 속도, 운전자 입력)을
%   변환한다.
%
%   호환:
%     - CarMaker 13/15 (native OSC 2.0 importer)
%     - 향후 RoadRunner Scenario도 OSC 2.0 export/import 예정
%
%   주의:
%     - OSC 2.0은 ASAM Reference Manual 2.0.0 (2023) 기반
%     - imports: osc.standard, osc.standard.vehicle, osc.standard.action
%     - 본 writer는 minimal subset — 복잡한 trigger/condition은 미지원

    if nargin < 2 || isempty(outDir)
        outDir = fullfile(pwd, 'data', 'scenarios_export');
    end
    if ~exist(outDir,'dir'); mkdir(outDir); end
    if nargin < 3; opts = struct(); end

    oscPath = fullfile(outDir, [scenario.id '.osc']);
    fid = fopen(oscPath, 'w');
    cleanup = onCleanup(@() fclose(fid));

    %% Imports
    fprintf(fid, '# ASAM OpenSCENARIO 2.0 — generated from ICC project\n');
    fprintf(fid, '# Scenario: %s (%s)\n', scenario.id, scenario.name);
    fprintf(fid, '# Reference: %s\n', scenario.refStandard);
    fprintf(fid, '# Generated: %s\n\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, 'import osc.standard\n');
    fprintf(fid, 'import osc.standard.vehicle\n');
    fprintf(fid, 'import osc.standard.action\n');
    fprintf(fid, 'import osc.standard.event\n\n');

    %% Type / scenario
    fprintf(fid, 'scenario %s:\n', local_safe_id(scenario.id));
    fprintf(fid, '    # Actors\n');
    fprintf(fid, '    ego_vehicle: vehicle\n');
    fprintf(fid, '    road_network: road\n\n');

    %% Storyboard
    fprintf(fid, '    do serial:\n');
    fprintf(fid, '        # 1) Initial placement on lane 1, longitudinal s=10m\n');
    fprintf(fid, '        ego_vehicle.set_position(road_network.lane(1), 10.0m, 0.0m)\n\n');
    fprintf(fid, '        # 2) Initial speed\n');
    fprintf(fid, '        ego_vehicle.assign_speed(speed: %.3fmps)\n\n', scenario.vx0);

    % Steering events: discretize scenario.steerDriver into key timestamps
    fprintf(fid, '        # 3) Driver maneuvers — discretized from steerDriver(t)\n');
    fprintf(fid, '        do parallel:\n');
    local_emit_steer_events(fid, scenario);
    local_emit_brake_events(fid, scenario);
    fprintf(fid, '\n');

    fprintf(fid, '        # 4) End condition\n');
    fprintf(fid, '        wait elapsed(%.2fs)\n', scenario.tEnd);
    fprintf(fid, '        emit end\n\n');

    %% Optional: friction action (uniform mu)
    fprintf(fid, '    # Note: For per-wheel friction or road surface, use OpenCRG companion file.\n');
end

function local_emit_steer_events(fid, scenario)
% steerDriver(t)에서 주요 peak/zero-crossing 추출하여 OSC 2.0 lane_change / steer event 생성
    dt = 0.01;
    t = (0:dt:scenario.tEnd)';
    delta = arrayfun(scenario.steerDriver, t);
    if all(abs(delta) < deg2rad(0.1))
        fprintf(fid, '            # No significant steer input\n');
        return;
    end
    % Find peaks (positive and negative)
    [~, ipos] = findpeaks_simple(delta);
    [~, ineg] = findpeaks_simple(-delta);
    keyTimes = sort([t(ipos); t(ineg)]);
    keyDeltas = arrayfun(scenario.steerDriver, keyTimes);
    fprintf(fid, '            # Steer profile (peaks)\n');
    for k = 1:numel(keyTimes)
        side = 'left'; if keyDeltas(k) < 0; side = 'right'; end
        fprintf(fid, '            ego_vehicle.change_lane(side: %s, duration: 1.0s) with:\n', side);
        fprintf(fid, '                start: at_time(%.3fs)\n', keyTimes(k));
    end
end

function local_emit_brake_events(fid, scenario)
% brakeCmd(t)의 onset/offset 검출 → assign_acceleration 또는 deceleration_action
    dt = 0.01;
    t = (0:dt:scenario.tEnd)';
    sumBrk = arrayfun(@(tt) sum(scenario.brakeCmd(tt)), t);
    active = sumBrk > 10;  % threshold
    if ~any(active); return; end
    idx = find(diff([0; active; 0]) ~= 0);
    onsetT = t(idx(1:2:end));
    offsetT = t(min(idx(2:2:end), numel(t)));
    fprintf(fid, '            # Brake events (onset/offset detected)\n');
    for k = 1:numel(onsetT)
        % Approximate decel level: avg torque / r_w / mass / 4 wheels
        avgT = mean(sumBrk(t >= onsetT(k) & t <= offsetT(k)));
        decel = avgT / 0.33 / 1600;  % rough: total brake force / mass
        fprintf(fid, '            ego_vehicle.deceleration(rate: %.2fmps2, duration: %.2fs) with:\n', ...
            abs(decel), offsetT(k) - onsetT(k));
        fprintf(fid, '                start: at_time(%.3fs)\n', onsetT(k));
    end
end

function [pks, locs] = findpeaks_simple(v)
% MATLAB Signal Processing Toolbox findpeaks 없을 때 사용하는 단순 implementation
    pks = []; locs = [];
    for k = 2:numel(v)-1
        if v(k) > v(k-1) && v(k) > v(k+1) && abs(v(k)) > deg2rad(0.5)
            pks(end+1) = v(k); locs(end+1) = k;
        end
    end
end

function s = local_safe_id(s)
% OSC 2.0 identifier 규약 (a-z, 0-9, _, 알파벳 시작)
    s = lower(regexprep(s, '[^A-Za-z0-9_]', '_'));
    if isempty(s) || ~isstrprop(s(1),'alpha'); s = ['scn_' s]; end
end
