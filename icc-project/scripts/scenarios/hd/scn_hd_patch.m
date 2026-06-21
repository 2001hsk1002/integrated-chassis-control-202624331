function scn_hd_patch(xoscFile, xodrFile, scenario)
%SCN_HD_PATCH OpenSCENARIO + OpenDRIVE 산출물에 HD scenario 자산 in-place inject
%
%   scn_hd_patch(xoscFile, xodrFile, scenario)
%
%   scenario.hd / scenario.weather 필드를 읽어 XML 파일을 직접 수정:
%     .xodr ← lane material (split-μ), superelevation (banking), objects (cones, barriers)
%     .xosc ← EnvironmentAction (weather), VehicleCatalog reference
%
%   MATLAB built-in xmlread / xmlwrite (java DOM API) 사용.

    if isempty(xoscFile) || ~isfile(xoscFile)
        error('[scn_hd_patch] xosc file not found: %s', xoscFile);
    end
    if isempty(xodrFile) || ~isfile(xodrFile)
        error('[scn_hd_patch] xodr file not found: %s', xodrFile);
    end

    % ---------- XODR ----------
    xodrDom = xmlread(xodrFile);
    local_patch_lane_material(xodrDom, scenario.hd.laneMaterial);
    local_patch_banking      (xodrDom, scenario.hd.banking);
    local_patch_objects      (xodrDom, scenario.hd.objects);
    xmlwrite(xodrFile, xodrDom);

    % ---------- XOSC ----------
    %   VehicleCatalog는 osc2cm의 -i (egoinf) CLI 옵션으로 바인딩하므로 inject 하지 않음
    xoscDom = xmlread(xoscFile);
    local_bump_xosc_version    (xoscDom);                              % 1.1 → 1.3
    local_fix_time_reference   (xoscDom);                              % OSC 1.3 strict: <None/> 채움
    local_patch_weather        (xoscDom, scenario.weather);
    xmlwrite(xoscFile, xoscDom);

    fprintf('  ✓ HD patch applied: weather=%s, banking=%s, laneMat=%d, objects=%d\n', ...
        scenario.weather.name, ...
        bool2str(~isempty(scenario.hd.banking)), ...
        numel(scenario.hd.laneMaterial), ...
        numel(scenario.hd.objects));
end

%% ============================================================
function local_patch_lane_material(dom, laneMaterial)
    if isempty(laneMaterial); return; end
    laneNodes = dom.getElementsByTagName('lane');
    for k = 0:laneNodes.getLength()-1
        ln = laneNodes.item(k);
        lid = str2double(char(ln.getAttribute('id')));
        for m = 1:numel(laneMaterial)
            spec = laneMaterial{m};
            if lid == spec.laneId
                matEl = dom.createElement('material');
                matEl.setAttribute('sOffset',  '0.0');
                matEl.setAttribute('surface',  'asphalt');
                matEl.setAttribute('friction', sprintf('%.3f', spec.friction));
                matEl.setAttribute('roughness','0.0');
                % insert as first child for clean XSD ordering
                if ln.hasChildNodes()
                    ln.insertBefore(matEl, ln.getFirstChild());
                else
                    ln.appendChild(matEl);
                end
            end
        end
    end
end

%% ============================================================
function local_patch_banking(dom, banking)
    if isempty(banking); return; end
    latProfile = dom.getElementsByTagName('lateralProfile');
    if latProfile.getLength() == 0; return; end
    lp = latProfile.item(0);

    % 기존 superelevation (a=b=c=d=0) 노드 제거
    existing = lp.getElementsByTagName('superelevation');
    while existing.getLength() > 0
        lp.removeChild(existing.item(0));
    end

    % 새 superelevation: angle_deg를 rad로 변환해 a 계수에 입력
    se = dom.createElement('superelevation');
    se.setAttribute('s', sprintf('%.4e', banking.s0));
    se.setAttribute('a', sprintf('%.6e', deg2rad(banking.angle_deg)));
    se.setAttribute('b', '0.0');
    se.setAttribute('c', '0.0');
    se.setAttribute('d', '0.0');
    lp.appendChild(se);
end

%% ============================================================
function local_patch_objects(dom, objects)
    if isempty(objects); return; end
    roadNodes = dom.getElementsByTagName('road');
    if roadNodes.getLength() == 0; return; end
    road = roadNodes.item(0);

    % <objects> 컨테이너 (없으면 생성)
    objContainer = road.getElementsByTagName('objects');
    if objContainer.getLength() > 0
        objs = objContainer.item(0);
    else
        objs = dom.createElement('objects');
        road.appendChild(objs);
    end

    for k = 1:numel(objects)
        o = objects{k};
        el = dom.createElement('object');
        el.setAttribute('id',     sprintf('%d', k));
        el.setAttribute('name',   o.name);
        el.setAttribute('type',   o.type);
        el.setAttribute('s',      sprintf('%.4e', o.s));
        el.setAttribute('t',      sprintf('%.4e', o.t));
        el.setAttribute('zOffset','0.0');
        el.setAttribute('hdg',    '0.0');
        el.setAttribute('roll',   '0.0');
        el.setAttribute('pitch',  '0.0');
        el.setAttribute('height', sprintf('%.3f', o.height));
        el.setAttribute('radius', '0.15');
        el.setAttribute('validLength','0.0');
        el.setAttribute('orientation','none');
        objs.appendChild(el);
    end
end

%% ============================================================
function local_bump_xosc_version(dom)
% FileHeader revMinor를 "1" (= MATLAB ADT export 한계) → "3"으로 bump.
% CarMaker 15는 1.0/1.1/1.2/1.3을 모두 import 하지만 osc2cm은 default로 file의 revMinor를 따름.
    headers = dom.getElementsByTagName('FileHeader');
    if headers.getLength() == 0; return; end
    hd = headers.item(0);
    hd.setAttribute('revMajor', '1');
    hd.setAttribute('revMinor', '3');
end

%% ============================================================
function local_fix_time_reference(dom)
% MATLAB ADT export는 <TimeReference/>를 비워둠. OSC 1.3 schema는 child <None> 또는 <Timing> 요구.
    refs = dom.getElementsByTagName('TimeReference');
    for k = 0:refs.getLength()-1
        ref = refs.item(k);
        if ~ref.hasChildNodes()
            none = dom.createElement('None');
            ref.appendChild(none);
        end
    end
end

%% ============================================================
function local_patch_weather(dom, weather)
    if strcmp(weather.name,'dry'); return; end   % 'dry'는 default — emit 안 해도 됨
    initNodes = dom.getElementsByTagName('Init');
    if initNodes.getLength() == 0; return; end
    init = initNodes.item(0);
    actionsNodes = init.getElementsByTagName('Actions');
    if actionsNodes.getLength() == 0; return; end
    actions = actionsNodes.item(0);

    globalAct = dom.createElement('GlobalAction');
    envAct    = dom.createElement('EnvironmentAction');
    env       = dom.createElement('Environment');
    env.setAttribute('name', weather.name);
    timeOfDay = dom.createElement('TimeOfDay');
    timeOfDay.setAttribute('animation','false');
    timeOfDay.setAttribute('dateTime','2025-06-01T12:00:00');
    env.appendChild(timeOfDay);
    wx = dom.createElement('Weather');
    wx.setAttribute('cloudState','overcast');
    sun = dom.createElement('Sun');
    sun.setAttribute('intensity','0.6');
    sun.setAttribute('azimuth','0');
    sun.setAttribute('elevation','1.0');
    wx.appendChild(sun);
    fog = dom.createElement('Fog');
    fog.setAttribute('visualRange','1000');
    wx.appendChild(fog);
    precip = dom.createElement('Precipitation');
    precip.setAttribute('precipitationType', weather.precipitation);
    precip.setAttribute('intensity', sprintf('%.2f', weather.intensity));
    wx.appendChild(precip);
    env.appendChild(wx);
    roadCond = dom.createElement('RoadCondition');
    roadCond.setAttribute('frictionScaleFactor', sprintf('%.3f', weather.mu_scale));
    env.appendChild(roadCond);
    envAct.appendChild(env);
    globalAct.appendChild(envAct);

    % Init/Actions의 첫 자식 앞에 삽입 (Storyboard 시작 시 적용)
    if actions.hasChildNodes()
        actions.insertBefore(globalAct, actions.getFirstChild());
    else
        actions.appendChild(globalAct);
    end
end

%% ============================================================
function local_patch_vehicle_catalog(dom, catalog)
    if isempty(catalog) || ~isfield(catalog,'name'); return; end

    catLocs = dom.getElementsByTagName('CatalogLocations');
    if catLocs.getLength() == 0; return; end
    cl = catLocs.item(0);

    vehCats = cl.getElementsByTagName('VehicleCatalog');
    if vehCats.getLength() > 0
        vc = vehCats.item(0);
    else
        vc = dom.createElement('VehicleCatalog');
        cl.appendChild(vc);
    end

    % BMW_5 catalog 디렉터리 — CarMaker Data/Vehicle을 가리킴 (osc2cm이 해석)
    dirs = vc.getElementsByTagName('Directory');
    while dirs.getLength() > 0
        vc.removeChild(dirs.item(0));
    end
    dirEl = dom.createElement('Directory');
    dirEl.setAttribute('path', './Data/Vehicle');
    vc.appendChild(dirEl);

    % Entities 안의 Car1 CatalogReference를 BMW_5 entry로 갱신
    refs = dom.getElementsByTagName('CatalogReference');
    for k = 0:refs.getLength()-1
        ref = refs.item(k);
        if strcmp(char(ref.getParentNode().getNodeName()), 'ScenarioObject')
            ref.setAttribute('catalogName', sprintf('%sCatalog', catalog.name));
            ref.setAttribute('entryName',   catalog.infoFile);
        end
    end
end

%% ============================================================
function s = bool2str(b)
    if b; s = 'yes'; else; s = 'no'; end
end
