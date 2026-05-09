function [Fy, info] = pacejka_fy(alpha, Fz, p, gamma)
%PACEJKA_FY  Lateral tyre force from Pacejka Magic Formula 5.2 (pure slip).
%
%   Fy = pacejka_fy(alpha, Fz, p)
%   Fy = pacejka_fy(alpha, Fz, p, gamma)
%   [Fy, info] = pacejka_fy(...)
%
% Pure-slip lateral force. Combined slip (Fy reduction in the presence of
% Fx) is handled separately by the G_alpha factor in pacejka_combined.m;
% this function returns the Fy0 baseline that G_alpha multiplies.
%
% INPUTS
%   alpha [rad]   slip angle. Positive for slip toward outside of the
%                  corner (ISO convention). Scalar or column vector.
%   Fz    [N]     vertical load. Scalar or column vector. Same size as
%                  alpha, or broadcast-compatible.
%   p     struct  Pacejka coefficient set (see pacejka_coeffs_pirelli_dhf_gt3.m
%                  for the field list). One axle's worth.
%   gamma [rad]   camber angle (optional, default 0). Positive for wheel
%                  leaning into the corner (ISO).
%
% OUTPUTS
%   Fy    [N]     lateral force at the contact patch. Positive Fy points
%                  toward the inside of the corner; under steady cornering
%                  with positive alpha, Fy is positive (the tyre pushes
%                  the car around the corner).
%   info  struct  optional diagnostics: .B, .C, .D, .E, .Sh, .Sv, .alpha_t,
%                  .mu_y. Useful for plotting and for v06.1 fitting.
%
% MATH (MF 5.2, lateral, pure slip)
%   dfz     = (Fz − Fz0) / Fz0
%   Sh      = (pHy1 + pHy2·dfz) + pHy3·γ
%   Sv      = Fz·((pVy1 + pVy2·dfz) + (pVy3 + pVy4·dfz)·γ)
%   α_t     = α + Sh
%   μ_y     = (pDy1 + pDy2·dfz)·(1 − pDy3·γ²)
%   D       = μ_y · Fz
%   C       = pCy1
%   E       = (pEy1 + pEy2·dfz)·(1 + pEy5·γ² − (pEy3 + pEy4·γ)·sign(α_t))
%   Ky      = pKy1·Fz0·sin(2·atan(Fz / (pKy2·Fz0)))·(1 − pKy3·|γ|)
%   B       = Ky / (C·D)                  (with division-by-zero guard)
%   Fy      = D·sin(C·atan(B·α_t − E·(B·α_t − atan(B·α_t)))) + Sv
%
% See Pacejka, "Tyre and Vehicle Dynamics", 2nd ed (2002) §4.3.2 for the
% derivation, and the project's technical reference §12 for the project
% notation.
%
% Author:  Jaykumar Patil
% Created: 2026-04-22  (v06 Pacejka — Phase 6.5, Step 1)

if nargin < 4 || isempty(gamma), gamma = 0; end

% Vectorise: ensure column vectors and broadcast compatibility
alpha = alpha(:);
Fz    = Fz(:);
gamma = gamma(:);

% Broadcast scalars
n = max([numel(alpha), numel(Fz), numel(gamma)]);
if numel(alpha) == 1, alpha = alpha * ones(n,1); end
if numel(Fz)    == 1, Fz    = Fz    * ones(n,1); end
if numel(gamma) == 1, gamma = gamma * ones(n,1); end

%% ---- Normalised load and shifts -----------------------------------
dfz = (Fz - p.Fz0) / p.Fz0;
Sh  = (p.pHy1 + p.pHy2 .* dfz) + p.pHy3 .* gamma;
Sv  = Fz .* ( (p.pVy1 + p.pVy2 .* dfz) ...
            + (p.pVy3 + p.pVy4 .* dfz) .* gamma );

alpha_t = alpha + Sh;

%% ---- Macro factors -------------------------------------------------
mu_y = (p.pDy1 + p.pDy2 .* dfz) .* (1 - p.pDy3 .* gamma.^2);
D    = mu_y .* Fz;
C    = p.pCy1 .* ones(size(Fz));

E    = (p.pEy1 + p.pEy2 .* dfz) ...
       .* (1 + p.pEy5 .* gamma.^2 ...
              - (p.pEy3 + p.pEy4 .* gamma) .* sign(alpha_t));

% Cornering stiffness Ky and resulting B
Ky = p.pKy1 .* p.Fz0 .* sin( 2 .* atan(Fz ./ (p.pKy2 .* p.Fz0)) ) ...
            .* (1 - p.pKy3 .* abs(gamma));

% Guard against C·D = 0 (only at Fz = 0). Replace divisor with eps when so.
denom = C .* D;
denom(abs(denom) < 1e-9) = 1e-9 * sign(denom(abs(denom) < 1e-9) + eps);
B = Ky ./ denom;

%% ---- Magic-formula evaluation -------------------------------------
arg = B .* alpha_t - E .* (B .* alpha_t - atan(B .* alpha_t));
Fy0 = D .* sin( C .* atan(arg) );
Fy  = Fy0 + Sv;

%% ---- Diagnostics --------------------------------------------------
if nargout > 1
    info.B       = B;
    info.C       = C;
    info.D       = D;
    info.E       = E;
    info.Sh      = Sh;
    info.Sv      = Sv;
    info.alpha_t = alpha_t;
    info.mu_y    = mu_y;
    info.Ky      = Ky;
end
end
