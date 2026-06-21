function scenario = scn_from_openscenario(xoscFile, opts)
%SCN_FROM_OPENSCENARIO ASAM OpenSCENARIO 1.x XML → 우리 scenario struct
%
%   scenario = SCN_FROM_OPENSCENARIO(xoscFile)
%
%   추출:
%     - Ego initial speed (Init Action / LongitudinalAction)
%     - Steering profile (PrivateAction / LongitudinalAction / ...
%     - Speed Action timeline
%     - Brake Action (간이 추정)
%
%   범위:
%     1.0/1.1 XML standard만 지원. .osc DSL (2.0+)은 미지원.
%     Story → Act → Maneuver → Event 트리를 walk하여 ego trajectory 재구성.
%
%   Inputs:
%       xoscFile - (char) .xosc 파일 경로
%       opts     - struct (옵션): .verbose
%
%   Outputs:
%       scenario - scenario_dispatcher 호환 struct

    if nargin < 2; opts = struct(); end
    if ~isfield(opts,'verbose'); opts.verbose = true; end

    if ~isfile(xoscFile)
        error('[scn_from_openscenario] File not found: %s', xoscFile);
    end

    %% XML 파싱
    doc = xmlread(xoscFile);
    rootNode = doc.getDocumentElement();
    rootTag = char(rootNode.getTagName());
    if ~strcmp(rootTag, 'OpenSCENARIO')
        warning('Expected OpenSCENARIO root, got %s', rootTag);
    end

    %% Header 정보
    headerNodes = rootNode.getElementsByTagName('FileHeader');
    scenario.id   = 'imported';
    scenario.name = 'Imported from OpenSCENARIO';
    if headerNodes.getLength() > 0
        hd = headerNodes.item(0);
        try
            scenario.id   = char(hd.getAttribute('description'));
        catch; end
    end
    scenario.refStandard = sprintf('OpenSCENARIO (imported from %s)', xoscFile);

    %% Storyboard → Init Actions (초기 vx 추출)
    initNodes = rootNode.getElementsByTagName('Init');
    vx0 = 22.22;  % default 80 km/h
    if initNodes.getLength() > 0
        speedNodes = initNodes.item(0).getElementsByTagName('AbsoluteTargetSpeed');
        if speedNodes.getLength() > 0
            v = str2double(char(speedNodes.item(0).getAttribute('value')));
            if ~isnan(v); vx0 = v; end
        end
    end
    scenario.vx0 = vx0;

    %% StopTime — ConditionGroup의 SimulationTime 또는 default 8s
    tEnd = 8.0;
    simTimeNodes = rootNode.getElementsByTagName('SimulationTimeCondition');
    if simTimeNodes.getLength() > 0
        v = str2double(char(simTimeNodes.item(0).getAttribute('value')));
        if ~isnan(v); tEnd = v; end
    end
    scenario.tEnd = tEnd;

    %% Speed Actions에서 brake/throttle 시계열 추출
    % Time-Speed pair 모음 → vx(t) 보간 → ax(t) 미분
    speedActions = rootNode.getElementsByTagName('AbsoluteTargetSpeed');
    nSA = speedActions.getLength();
    speedSeries.t = [];
    speedSeries.v = [];
    for k = 0:nSA-1
        sa = speedActions.item(k);
        v = str2double(char(sa.getAttribute('value')));
        if ~isnan(v)
            speedSeries.t(end+1) = numel(speedSeries.t) * 1.0;   % placeholder
            speedSeries.v(end+1) = v;
        end
    end

    %% Trajectory Action에서 waypoints 추출 (Task 1: inverse kinematics)
    polyNodes = rootNode.getElementsByTagName('Polyline');
    waypoints.t = []; waypoints.x = []; waypoints.y = [];
    if polyNodes.getLength() > 0
        poly = polyNodes.item(0);
        verts = poly.getElementsByTagName('Vertex');
        for vi = 0:verts.getLength()-1
            vtx = verts.item(vi);
            t_v = str2double(char(vtx.getAttribute('time')));
            posNodes = vtx.getElementsByTagName('WorldPosition');
            if posNodes.getLength() > 0
                p = posNodes.item(0);
                x_v = str2double(char(p.getAttribute('x')));
                y_v = str2double(char(p.getAttribute('y')));
                if ~isnan(t_v) && ~isnan(x_v) && ~isnan(y_v)
                    waypoints.t(end+1,1) = t_v;
                    waypoints.x(end+1,1) = x_v;
                    waypoints.y(end+1,1) = y_v;
                end
            end
        end
    end

    nWP = numel(waypoints.t);
    if nWP >= 2 && max(waypoints.t) > scenario.tEnd
        scenario.tEnd = max(waypoints.t);
    end
    if nWP >= 5
        % Inverse kinematics → driver inputs
        try
            [steerFn, brakeFn, ikInfo] = scn_trajectory_to_input( ...
                waypoints.t, waypoints.x, waypoints.y);
            scenario.steerDriver = steerFn;
            scenario.brakeCmd    = brakeFn;
            scenario.ik          = ikInfo;
            if opts.verbose
                fprintf('  Trajectory inverse kinematics: %d waypoints → driver inputs\n', nWP);
                fprintf('    peak |delta| = %.3f deg, peak brake = %.0f Nm\n', ...
                    rad2deg(max(abs(ikInfo.delta))), max(sum(ikInfo.brakeTorque,2)));
            end
        catch ME
            fprintf('  Inverse kinematics failed: %s — falling back to zero input\n', ME.message);
            scenario.steerDriver = @(t) 0;
            scenario.brakeCmd    = @(t) zeros(4, 1);
        end
    else
        scenario.steerDriver = @(t) 0;
        scenario.brakeCmd    = @(t) zeros(4, 1);
        if opts.verbose
            fprintf('  No trajectory polyline found (only %d waypoints) — zero driver input\n', nWP);
        end
    end
    scenario.z_road      = @(t, w) 0;
    scenario.mu_wheel    = @(t, w) 1.0;
    scenario.kpis        = {'yawRateOvershoot','sideSlipMax','LTR_max'};

    if opts.verbose
        fprintf('[scn_from_openscenario] imported %s\n', xoscFile);
        fprintf('  id: %s, tEnd: %.2f, vx0: %.2f m/s\n', scenario.id, scenario.tEnd, scenario.vx0);
        fprintf('  Speed actions: %d, Trajectory waypoints: %d\n', nSA, nWP);
    end

    %% Reference path (OpenDRIVE 옆에 있으면 함께 import)
    [d, b, ~] = fileparts(xoscFile);
    xodrCand = fullfile(d, [b '.xodr']);
    if isfile(xodrCand)
        scenario.refXodr = xodrCand;
    end
end
