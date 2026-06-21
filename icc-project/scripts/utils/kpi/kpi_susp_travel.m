function out = kpi_susp_travel(zs)
%KPI_SUSP_TRAVEL 4-corner suspension peak displacement (C.3)
%
%   Inputs:
%       zs - N×4 suspension displacements [m] (FL, FR, RL, RR)
%
%   Outputs:
%       out.peakPerCorner  4×1 max |zs|
%       out.peakOverall    max over all corners
%       out.peakOverall_mm scaled to mm

    if isempty(zs) || size(zs, 2) ~= 4
        out.peakPerCorner = NaN(4,1);
        out.peakOverall   = NaN;
        out.peakOverall_mm = NaN;
        return;
    end
    out.peakPerCorner  = max(abs(zs), [], 1)';
    out.peakOverall    = max(out.peakPerCorner);
    out.peakOverall_mm = out.peakOverall * 1000;
end
