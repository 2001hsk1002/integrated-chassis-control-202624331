%VDB_INSPECT_TIRE2 Tires Variant Subsystem 상세 — active variant + 각 variant의 이름/타입

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));
modelDir = fullfile(projectRoot, 'models', 'simulink', 'vdb_ref');
addpath(modelDir);
cd(modelDir);

mdl = 'PassVeh14DOF';
load_system(mdl);

%% Find Tires variant subsystem
tiresVar = find_system(mdl, 'LookUnderMasks','all','FollowLinks','on', ...
    'MatchFilter', @Simulink.match.allVariants, ...
    'BlockType','SubSystem','Variant','on','Name','Tires');
fprintf('Tires variant subsystem: %s\n', tiresVar{1});

%% Variant choices
tireVar = tiresVar{1};
choices = get_param(tireVar, 'VariantChoices');
fprintf('\n=== Tire model variants (%d) ===\n', numel(choices));
for k = 1:numel(choices)
    fprintf('  Variant %d:\n', k);
    fprintf('    Name      : %s\n', choices(k).Name);
    fprintf('    BlockName : %s\n', choices(k).BlockName);
    if isprop(choices(k), 'Condition') || isfield(choices(k),'Condition')
        try; fprintf('    Condition : %s\n', char(choices(k).Condition)); catch; end
    end
end

%% Active variant
try
    activeName = get_param(tireVar, 'CompiledActiveChoiceBlock');
    fprintf('\nCompiled active choice block: %s\n', activeName);
catch
end
try
    cv = get_param(tireVar, 'LabelModeActiveChoice');
    fprintf('LabelModeActiveChoice: %s\n', cv);
catch
end
try
    cc = get_param(tireVar, 'OverrideUsingVariant');
    fprintf('OverrideUsingVariant: %s\n', cc);
catch
end

%% Find all children under Tires (including inactive variants)
allChildren = find_system(tireVar, 'LookUnderMasks','all','FollowLinks','on', ...
    'MatchFilter', @Simulink.match.allVariants);
fprintf('\nTotal blocks under Tires (all variants): %d\n', numel(allChildren));
fprintf('\nDirect children of Tires:\n');
direct = find_system(tireVar, 'SearchDepth', 1, 'MatchFilter', @Simulink.match.allVariants);
for k = 1:numel(direct)
    bp = direct{k};
    if strcmp(bp, tireVar); continue; end
    [~, nm] = fileparts(bp);
    try; bt = get_param(bp,'BlockType'); catch; bt = ''; end
    try; mt = get_param(bp,'MaskType'); catch; mt = ''; end
    fprintf('  %-50s [%s]  Mask=%s\n', nm, bt, mt);
end

close_system(mdl, 0);
