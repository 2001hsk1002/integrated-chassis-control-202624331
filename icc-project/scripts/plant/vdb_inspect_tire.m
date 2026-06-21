%VDB_INSPECT_TIRE PassVeh14DOF의 Wheels and Tires subsystem 구조 + tire block 식별
%
%   1. Wheels and Tires 서브시스템 안의 블록 트리 walk
%   2. 'Tire' / 'CombinedSlip' / 'Magic' / 'Fiala' 패턴 매칭
%   3. tire 블록의 BlockType, MaskType, parameter 출력
%   4. 사용 가능한 variant 확인

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));
modelDir = fullfile(projectRoot, 'models', 'simulink', 'vdb_ref');
addpath(modelDir);
cd(modelDir);

mdl = 'PassVeh14DOF';
load_system(mdl);

%% Step 1. Wheels and Tires 서브시스템 찾기
wt = find_system(mdl, 'SearchDepth', 1, 'Name', 'Wheels and Tires');
fprintf('Wheels and Tires subsystem: %s\n', wt{1});

%% Step 2. 모든 하위 블록 (depth 무한)
allBlocks = find_system(wt{1}, 'LookUnderMasks', 'all', 'FollowLinks', 'on');
fprintf('\nTotal blocks under Wheels and Tires: %d\n', numel(allBlocks));

%% Step 3. Tire 관련 블록 필터링
fprintf('\n=== Tire / Wheel 관련 블록 ===\n');
keywords = {'Tire','MagicFormula','Magic Formula','CombinedSlip','Combined Slip','Fiala','LongWheel','SideSlip'};
tireBlocks = {};
for k = 1:numel(allBlocks)
    bp = allBlocks{k};
    [~, nm] = fileparts(bp);
    found = false;
    for kk = 1:numel(keywords)
        if contains(nm, keywords{kk}, 'IgnoreCase', true)
            found = true; break;
        end
    end
    if found
        bt = '';
        mt = '';
        try; bt = get_param(bp, 'BlockType'); catch; end
        try; mt = get_param(bp, 'MaskType'); catch; end
        fprintf('  %-70s  [%s]  Mask=%s\n', strrep(bp, [mdl '/'],''), bt, mt);
        tireBlocks{end+1} = bp;
    end
end

%% Step 4. 핵심 tire 블록 1개 골라서 mask param 출력
if ~isempty(tireBlocks)
    % MaskType이 정의된 첫 블록
    chosen = '';
    for k = 1:numel(tireBlocks)
        try
            mt = get_param(tireBlocks{k}, 'MaskType');
            if ~isempty(mt)
                chosen = tireBlocks{k};
                break;
            end
        catch
        end
    end
    if ~isempty(chosen)
        fprintf('\n=== Chosen tire block: ===\n  %s\n', chosen);
        fprintf('  MaskType: %s\n', get_param(chosen, 'MaskType'));
        try
            params = get_param(chosen, 'MaskNames');
            fprintf('  Mask parameters (%d):\n', numel(params));
            for kk = 1:numel(params)
                pv = '';
                try; pv = get_param(chosen, params{kk}); catch; end
                if length(pv) > 60; pv = [pv(1:60) '...']; end
                fprintf('    %-30s = %s\n', params{kk}, pv);
            end
        catch ME
            fprintf('  param read error: %s\n', ME.message);
        end
    end
end

%% Step 5. Variant Subsystem 탐색 (tire model 선택 가능 여부)
fprintf('\n=== Variant Subsystems (tire model switch 가능 여부) ===\n');
variants = find_system(wt{1}, 'LookUnderMasks', 'all', 'BlockType', 'SubSystem', 'Variant', 'on');
for k = 1:numel(variants)
    [~, nm] = fileparts(variants{k});
    try
        vc = get_param(variants{k}, 'VariantChoices');
        fprintf('  %-50s Variants: ', nm);
        for vv = 1:numel(vc)
            fprintf('%s, ', vc(vv).Name);
        end
        fprintf('\n');
    catch
    end
end

%% Step 6. tire-related workspace variables count
ws = get_param(mdl, 'ModelWorkspace');
varNames = ws.whos;
varList = {varNames.name};
tireRelated = varList(contains(varList, 'P', 'IgnoreCase', false) & ...
    (contains(varList,'CX') | contains(varList,'CY') | ...
     contains(varList,'DX') | contains(varList,'DY') | ...
     contains(varList,'KX') | contains(varList,'KY')));
fprintf('\n=== MF coefficient vars in workspace (~%d) ===\n', numel(tireRelated));
for k=1:numel(tireRelated)
    fprintf('  %s\n', tireRelated{k});
end

close_system(mdl, 0);
