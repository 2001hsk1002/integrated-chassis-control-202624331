%TEST_CM_PARSE_INFOFILE BMW_5 INFOFILE 알려진 값과의 일치 검증
%
%   카운트, 스칼라/벡터 파싱, 알려진 값 5종 점검.

clear; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(projectRoot, 'scripts')));

bmwFile = 'C:/Users/VIC/Projects/carmaker_data/Data/Vehicle/BMW_5_15_030326';

assert(isfile(bmwFile), 'BMW_5 INFOFILE not found: %s', bmwFile);

params = cm_parse_infofile(bmwFile);

nFields = numel(fieldnames(params));
fprintf('[test_cm_parse_infofile] Parsed %d fields from BMW_5.\n', nFields);
assert(nFields > 100, '필드 수가 너무 적음 (%d)', nFields);

%% 알려진 값 검증
checks = {
    'WheelCarrier_fl_mass',  22.134,                     'scalar';
    'Wheel_fl_mass',         29.478,                     'scalar';
    'Wheel_fl_I',            [0.82 1.64 0.82],           'vector';
    'Jack_fl_pos',           [3.62 0.8 0.309],           'vector';
    'WheelCarrier_rl_mass',  35.308,                     'scalar';
};

for i = 1:size(checks, 1)
    fld = checks{i, 1};
    expected = checks{i, 2};
    kind = checks{i, 3};

    assert(isfield(params, fld), '필드 누락: %s', fld);
    actual = params.(fld);

    switch kind
        case 'scalar'
            assert(isnumeric(actual) && isscalar(actual), '스칼라 아님: %s', fld);
            assert(abs(actual - expected) < 1e-6, ...
                '값 불일치 %s: expected=%g, got=%g', fld, expected, actual);
        case 'vector'
            assert(isnumeric(actual) && numel(actual) == numel(expected), ...
                '벡터 길이 불일치: %s (%d vs %d)', fld, numel(actual), numel(expected));
            assert(norm(actual(:) - expected(:)) < 1e-6, ...
                '벡터 값 불일치 %s', fld);
    end
    fprintf('  PASS: %s\n', fld);
end

%% .erg.info 같은 다른 INFOFILE도 동일 파서로 동작하는지
ergInfoFile = 'C:/Users/VIC/Projects/carmaker_data/SimOutput/bsong/20260428/LK_CCIR_ST_CM15_124910.erg.info';
if isfile(ergInfoFile)
    infoParams = cm_parse_infofile(ergInfoFile);
    assert(isfield(infoParams, 'File_Format'), 'erg.info: File.Format 누락');
    assert(strcmp(strtrim(infoParams.File_Format), 'erg'), 'erg.info Format 불일치');
    assert(isfield(infoParams, 'File_ByteOrder'), 'erg.info: File.ByteOrder 누락');
    fprintf('  PASS: erg.info 파싱\n');
end

fprintf('[test_cm_parse_infofile] ALL PASS\n');
