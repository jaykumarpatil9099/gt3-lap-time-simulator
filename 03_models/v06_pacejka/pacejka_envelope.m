function Fx_max = pacejka_envelope(Fy_req, Fz, p)
%PACEJKA_ENVELOPE  Maximum |Fx| achievable while maintaining Fy ≥ Fy_req.
%
%   Fx_max = pacejka_envelope(Fy_req, Fz, p)
%
% Replaces the friction-circle approximation `sqrt(F_grip² − F_y²)` with
% the actual Pacejka MF 5.2 combined-slip envelope. At a given vertical
% load Fz, the envelope is the set of reachable (Fy, Fx) as slip-angle α
% and slip-ratio κ vary. For a required lateral force Fy_req, the maximum
% |Fx| sits at the (α, κ) pair on the envelope boundary that satisfies
% the lateral-force constraint.
%
% IMPLEMENTATION
% --------------
% A naive per-call 2D sweep over (α, κ) would push lap-sim runtime from
% ~10 s to ~5 min. Instead we build a 2D lookup table of
%
%   Fx_max_norm(Fy_norm, Fz)   where Fy_norm = Fy_req / Fy_peak(Fz)
%
% on first call (lazy initialisation, persistent). Subsequent calls are
% an interp2 + a multiply by Fy_peak(Fz). Total table-build cost
% ≈ 1 second; per-call cost ≈ µs.
%
% The table is keyed off the coefficient struct's `created` field. If the
% coefficients change (e.g. v06.1 refits them), pass a fresh struct and
% the table rebuilds automatically.
%
% INPUTS
%   Fy_req  [N]   required lateral force at this tyre. Scalar.
%   Fz      [N]   vertical load on this tyre. Scalar (≥ 50 N).
%   p       struct  Pacejka MF 5.2 coefficient set (one axle's worth)
%
% OUTPUT
%   Fx_max  [N]   max |Fx| achievable subject to Fy_combined ≥ Fy_req.
%                  If Fy_req exceeds the pure-cornering peak Fy_pk(Fz),
%                  returns 0 (lateral demand cannot be satisfied).
%
% Author:  Jaykumar Patil
% Created: 2026-04-22  (v06 Pacejka — Phase 6.5, v06.0b)

persistent TABLE

% --------------------------------------------------------------------
% Lazy table build. Re-built whenever the coefficient version changes.
% --------------------------------------------------------------------
key = sprintf('%s|%s', p.version, p.created);
if isempty(TABLE) || ~strcmp(TABLE.key, key)
    TABLE = local_build_table(p, key);
end

% --------------------------------------------------------------------
% Lookup
% --------------------------------------------------------------------
% Clamp Fz into table range
Fz_q = min(max(Fz, TABLE.Fz_grid(1)), TABLE.Fz_grid(end));

% Find peak Fy at this Fz (interp on the precomputed peak vector)
Fy_peak = interp1(TABLE.Fz_grid, TABLE.Fy_peak_vec, Fz_q, 'linear');

% If demand exceeds peak, no solution.
if Fy_req >= Fy_peak * 0.999
    Fx_max = 0;
    return;
end
if Fy_req <= 0
    % Pure straight-line: full longitudinal grip
    Fx_max = interp1(TABLE.Fz_grid, TABLE.Fx_peak_vec, Fz_q, 'linear');
    return;
end

% Normalise lateral demand and look up envelope
Fy_norm = Fy_req / Fy_peak;
Fy_norm = min(max(Fy_norm, 0), 1);

Fx_norm = interp2(TABLE.Fz_grid, TABLE.Fy_norm_grid, ...
                  TABLE.Fx_max_norm_table, Fz_q, Fy_norm, 'linear');

% Denormalise back to Newtons (peak-Fx scale is approx peak-Fy scale,
% but more accurate is the per-Fz peak-Fx vector built once below)
Fx_peak = interp1(TABLE.Fz_grid, TABLE.Fx_peak_vec, Fz_q, 'linear');
Fx_max  = max(Fx_norm * Fx_peak, 0);
end


% =================================================================
% Build the lookup table
% =================================================================
function T = local_build_table(p, key)
    fprintf('pacejka_envelope: building lookup table for "%s"...\n', key);
    t0 = tic;

    % Grids
    T.Fz_grid       = (500:500:15000)';        % 30 Fz values [N]
    T.Fy_norm_grid  = (0:0.04:1)';              % 26 Fy/Fy_peak ratios
    T.key           = key;
    nFz = numel(T.Fz_grid);
    nFy = numel(T.Fy_norm_grid);

    T.Fy_peak_vec       = zeros(nFz,1);
    T.Fx_peak_vec       = zeros(nFz,1);
    T.Fx_max_norm_table = zeros(nFy, nFz);

    % Sweep grid for the envelope
    alpha_grid = linspace(0.5, 25, 30) * pi/180;       % 0.5..25°
    kappa_grid = linspace(-0.30, 0.30, 41);             % drive + brake
    [A, K] = meshgrid(alpha_grid, kappa_grid);
    A = A(:); K = K(:);

    for ii = 1:nFz
        Fz = T.Fz_grid(ii);

        % Pure peaks at this Fz
        Fy_pure_pk = max(abs(pacejka_fy(alpha_grid', Fz, p)));
        Fx_pure_pk = max(abs(pacejka_fx(kappa_grid', Fz, p)));
        T.Fy_peak_vec(ii) = Fy_pure_pk;
        T.Fx_peak_vec(ii) = Fx_pure_pk;

        % Build the envelope at this Fz
        [Fy, Fx] = pacejka_combined(A, K, Fz*ones(size(A)), p);
        Fy = abs(Fy);  Fx = abs(Fx);

        % For each target Fy_norm, find max Fx over the (α,κ) grid
        % satisfying Fy_combined >= Fy_norm·Fy_pure_pk
        for jj = 1:nFy
            Fy_thr = T.Fy_norm_grid(jj) * Fy_pure_pk;
            mask = Fy >= Fy_thr;
            if any(mask)
                T.Fx_max_norm_table(jj, ii) = max(Fx(mask)) / Fx_pure_pk;
            else
                T.Fx_max_norm_table(jj, ii) = 0;
            end
        end
    end

    % Force monotonic Fy_norm column (envelope must be non-increasing
    % in Fy_norm — small grid resolution can give tiny non-monotonic
    % artefacts that interp2 propagates). cummin along Fy_norm = dim 1.
    T.Fx_max_norm_table = cummin(T.Fx_max_norm_table, 1);

    fprintf('  built in %.2f s (%d × %d grid).\n', toc(t0), nFy, nFz);
end
