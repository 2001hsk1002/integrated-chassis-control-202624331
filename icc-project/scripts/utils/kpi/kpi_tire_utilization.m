function out = kpi_tire_utilization(tire, mu)
%KPI_TIRE_UTILIZATION 마찰원 사용률 KPI (D.2)
%
%   u_i(t) = sqrt(Fx_i² + Fy_i²) / (μ_i · Fz_i)
%   1.0에 가까울수록 saturation, 안정성/제어성 저하.
%
%   Inputs:
%       tire  struct .FL/.FR/.RL/.RR each with .Fx, .Fy, .Fz (N×1 vectors)
%       mu    scalar 또는 4-vector (per-wheel mu_peak)
%
%   Outputs:
%       out.uMaxPerCorner  4×1 each corner max utilization
%       out.uMaxOverall    max over all corners and time
%       out.uRMSOverall    RMS over all corners and time

    if nargin < 2 || isempty(mu); mu = 1.0; end
    if isscalar(mu); mu = mu * ones(4,1); end

    wheels = {'FL','FR','RL','RR'};
    u_all = [];
    out.uMaxPerCorner = zeros(4,1);
    for w = 1:4
        wn = wheels{w};
        if ~isfield(tire, wn); continue; end
        Fx = tire.(wn).Fx; Fy = tire.(wn).Fy; Fz = max(tire.(wn).Fz, 10);
        u = sqrt(Fx.^2 + Fy.^2) ./ (mu(w) .* Fz);
        out.uMaxPerCorner(w) = max(u);
        u_all = [u_all; u(:)]; %#ok<AGROW>
    end
    if isempty(u_all)
        out.uMaxOverall = NaN; out.uRMSOverall = NaN;
    else
        out.uMaxOverall = max(u_all);
        out.uRMSOverall = sqrt(mean(u_all.^2));
    end
end
