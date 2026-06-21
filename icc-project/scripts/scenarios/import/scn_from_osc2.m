function scenario = scn_from_osc2(oscFile, opts)
%SCN_FROM_OSC2 ASAM OpenSCENARIO 2.0 DSL (.osc) → 우리 scenario struct
%
%   scenario = SCN_FROM_OSC2(oscFile)
%
%   Minimal recursive-descent parser. 우리 scn_export_osc2.m가 만들어내는
%   subset만 처리:
%     scenario <name>:
%         <actor>: <type>
%         do serial:
%             <actor>.set_position(...)
%             <actor>.assign_speed(speed: Xmps)
%             do parallel:
%                 <actor>.change_lane(side: left|right, duration: Xs) with:
%                     start: at_time(Xs)
%                 <actor>.deceleration(rate: Xmps2, duration: Xs) with:
%                     start: at_time(Xs)
%             wait elapsed(Xs)
%             emit end
%
%   주의: 본 parser는 ASAM OSC 2.0 표준 전체를 지원하지 않음.
%   복잡한 DSL (composition, struct, parametric override 등)은 미지원.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts,'verbose'); opts.verbose = true; end

    if ~isfile(oscFile)
        error('[scn_from_osc2] File not found: %s', oscFile);
    end

    %% Read all lines
    fid = fopen(oscFile, 'r');
    lines = {};
    while ~feof(fid)
        l = fgetl(fid);
        if ~ischar(l); break; end
        lines{end+1, 1} = l;
    end
    fclose(fid);

    %% Strip comments + empty lines
    proc = {};
    for k = 1:numel(lines)
        l = lines{k};
        idx = strfind(l, '#');
        if ~isempty(idx); l = l(1:idx(1)-1); end
        if ~isempty(strtrim(l))
            proc{end+1, 1} = l;
        end
    end

    %% Parse scenario header
    scenario.id = 'imported_osc2';
    scenario.name = 'Imported from OpenSCENARIO 2.0';
    scenario.refStandard = 'ASAM OpenSCENARIO 2.0 (imported)';
    for k = 1:numel(proc)
        tok = regexp(strtrim(proc{k}), '^scenario\s+(\w+)\s*:', 'tokens', 'once');
        if ~isempty(tok)
            scenario.id = tok{1};
            scenario.name = sprintf('Imported scenario: %s', tok{1});
            break;
        end
    end

    %% Default fields
    scenario.vx0 = 22.22;
    scenario.tEnd = 8.0;
    eventSteer = {};   % {time, side, duration}
    eventBrake = {};   % {time, decel, duration}

    %% Parse action lines
    for k = 1:numel(proc)
        l = strtrim(proc{k});

        % Initial speed: ego_vehicle.assign_speed(speed: 22.222mps)
        m = regexp(l, 'assign_speed\(speed:\s*([\d.]+)mps\)', 'tokens', 'once');
        if ~isempty(m)
            scenario.vx0 = str2double(m{1});
            continue;
        end

        % Lane change: ego_vehicle.change_lane(side: left, duration: 1.0s)
        m = regexp(l, 'change_lane\(side:\s*(left|right),\s*duration:\s*([\d.]+)s\)', 'tokens', 'once');
        if ~isempty(m)
            eventSteer{end+1} = {m{1}, str2double(m{2}), NaN}; %#ok<AGROW>  % time set by next line
            continue;
        end

        % Deceleration: ego_vehicle.deceleration(rate: 5.5mps2, duration: 2.5s)
        m = regexp(l, 'deceleration\(rate:\s*([\d.]+)mps2,\s*duration:\s*([\d.]+)s\)', 'tokens', 'once');
        if ~isempty(m)
            eventBrake{end+1} = {str2double(m{1}), str2double(m{2}), NaN}; %#ok<AGROW>
            continue;
        end

        % at_time trigger: start: at_time(2.0s)
        m = regexp(l, 'at_time\(([\d.]+)s\)', 'tokens', 'once');
        if ~isempty(m)
            tTrig = str2double(m{1});
            % attach to the LAST queued event without time
            for j = numel(eventSteer):-1:1
                if isnan(eventSteer{j}{3})
                    eventSteer{j}{3} = tTrig; break;
                end
            end
            for j = numel(eventBrake):-1:1
                if isnan(eventBrake{j}{3})
                    eventBrake{j}{3} = tTrig; break;
                end
            end
            continue;
        end

        % End time: wait elapsed(8.0s)
        m = regexp(l, 'wait\s+elapsed\(([\d.]+)s\)', 'tokens', 'once');
        if ~isempty(m)
            scenario.tEnd = str2double(m{1});
            continue;
        end
    end

    %% Build steerDriver from change_lane events
    %   각 event: at_time(t), side ∈ {left,right}, duration → sine pulse
    scenario.steerDriver = @(t) local_compose_steer(t, eventSteer);
    scenario.brakeCmd    = @(t) local_compose_brake(t, eventBrake);
    scenario.z_road      = @(t, w) 0;
    scenario.mu_wheel    = @(t, w) 1.0;
    scenario.kpis        = {'yawRateOvershoot','sideSlipMax','LTR_max'};

    if opts.verbose
        fprintf('[scn_from_osc2] imported %s\n', oscFile);
        fprintf('  id=%s, tEnd=%.2f, vx0=%.2f m/s\n', scenario.id, scenario.tEnd, scenario.vx0);
        fprintf('  Steer events (lane changes): %d\n', numel(eventSteer));
        fprintf('  Brake events (decelerations): %d\n', numel(eventBrake));
    end
end

%% ------------------------------------------------------------
function delta = local_compose_steer(t, events)
% lane change pulse를 sine half-wave로 표현 (peak 3 deg roadwheel)
    delta = 0;
    AMP = deg2rad(3.0);
    for k = 1:numel(events)
        ev = events{k};
        side = ev{1}; dur = ev{2}; t0 = ev{3};
        if isnan(t0); continue; end
        if t >= t0 && t <= t0 + dur
            sign_ = 1; if strcmp(side,'right'); sign_ = -1; end
            delta = delta + sign_ * AMP * sin(pi * (t - t0) / dur);
        end
    end
end

function bk = local_compose_brake(t, events)
    bk = zeros(4,1);
    for k = 1:numel(events)
        ev = events{k};
        decel = ev{1}; dur = ev{2}; t0 = ev{3};
        if isnan(t0); continue; end
        if t >= t0 && t <= t0 + dur
            % decel m/s² → total torque ≈ m*decel*r_w
            T_total = 1600 * decel * 0.33;
            % distribute 55:45 F:R
            bk = bk + [0.275; 0.275; 0.225; 0.225] * T_total;
        end
    end
end
