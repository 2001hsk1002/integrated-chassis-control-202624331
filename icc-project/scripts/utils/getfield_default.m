function val = getfield_default(s, name, dflt)
%GETFIELD_DEFAULT struct 필드를 안전하게 읽고, 없으면 기본값 반환.
%
%   val = GETFIELD_DEFAULT(s, name, dflt)
%
%   s      - struct
%   name   - 필드 이름 (char)
%   dflt   - 필드가 없을 때 반환할 기본값

    if isstruct(s) && isfield(s, name)
        val = s.(name);
    else
        val = dflt;
    end
end
