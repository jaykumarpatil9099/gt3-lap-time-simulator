function [Fy, Fx, info] = pacejka_combined(alpha, kappa, Fz, p, gamma)
%PACEJKA_COMBINED  Combined-slip Fy and Fx via Pacejka MF 5.2 G-factors.
%
%   [Fy, Fx] = pacejka_combined(alpha, kappa, Fz, p)
%   [Fy, Fx] = pacejka_combined(alpha, kappa, Fz, p, gamma)
%   [Fy, Fx, info] = pacejka_combined(...)
%
% In MF 5.2, lateral and longitudinal forces couple through dimensionless
% reduction factors:
%
%     Fy_combined = G_yκ(α, κ, Fz, γ) · Fy0(α, Fz, γ) + Sv_yκ
%     Fx_combined = G_xα(α, κ, Fz)    · Fx0(κ, Fz, γ)
%
% G_yκ → 1 when κ = 0 (pure cornering: Fy unchanged) and shrinks toward
% zero as |κ| grows (large drive/brake slip steals the friction budget
% from cornering grip). G_xα is the symmetric reducer.
%
% The G factors use the cosine version of the magic formula:
%
%     G = cos( C · atan( B·H − E·(B·H − atan(B·H)) ) ) /
%         cos( C · atan( B · S_H ) )
%
% This gives a smooth elliptical envelope in the (Fy, Fx) plane and is
% the form race-team QSS sub-tools typically use.
%
% INPUTS
%   alpha [rad]   slip angle  (column or scalar)
%   kappa [-]     longitudinal slip ratio
%   Fz    [N]     vertical load
%   p     struct  Pacejka coefficient set
%   gamma [rad]   camber (optional, default 0)
%
% OUTPUTS
%   Fy    [N]     combined-slip lateral force at the contact patch
%   Fx    [N]     combined-slip longitudinal force
%   info  struct  diagnostics: .Fy0, .Fx0, .G_yk, .G_xa, .Sv_yk
%
% Author:  Jaykumar Patil
% Created: 2026-04-22  (v06 Pacejka — Phase 6.5, Step 2)

if nargin < 5 || isempty(gamma), gamma = 0; end

% Vector broadcasting
alpha = alpha(:);  kappa = kappa(:);  Fz = Fz(:);  gamma = gamma(:);
n = max([numel(alpha), numel(kappa), numel(Fz), numel(gamma)]);
if numel(alpha) == 1, alpha = alpha * ones(n,1); end
if numel(kappa) == 1, kappa = kappa * ones(n,1); end
if numel(Fz)    == 1, Fz    = Fz    * ones(n,1); end
if numel(gamma) == 1, gamma = gamma * ones(n,1); end

dfz = (Fz - p.Fz0) / p.Fz0;

%% ---- Pure-slip baselines ------------------------------------------
Fy0 = pacejka_fy(alpha, Fz, p, gamma);
Fx0 = pacejka_fx(kappa, Fz, p, gamma);

%% ---- G_yκ : Fy reduction by longitudinal slip ---------------------
S_Hyk = p.rHy1 + p.rHy2 .* dfz;
B_yk  = (p.rBy1 + p.rBy3 .* gamma.^2) .* cos(atan(p.rBy2 .* (alpha - p.rBy3 .* gamma)));
C_yk  = p.rCy1;
E_yk  = p.rEy1 + p.rEy2 .* dfz;

H_yk  = B_yk .* (kappa + S_Hyk);
H0_yk = B_yk .* S_Hyk;

G_yk_num = cos( C_yk .* atan( H_yk - E_yk .* (H_yk - atan(H_yk)) ) );
G_yk_den = cos( C_yk .* atan( H0_yk - E_yk .* (H0_yk - atan(H0_yk)) ) );
G_yk_den(abs(G_yk_den) < 1e-9) = 1e-9;
G_yk = G_yk_num ./ G_yk_den;
G_yk = max(G_yk, 0);                  % numerical safety: G ≥ 0

% Vertical-shift correction Sv_yκ (small; zero at v06.0 defaults)
mu_y      = (p.pDy1 + p.pDy2 .* dfz) .* (1 - p.pDy3 .* gamma.^2);
DV_yk     = mu_y .* Fz .* (p.rVy1 + p.rVy2 .* dfz + p.rVy3 .* gamma) ...
            .* cos(atan(p.rVy4 .* alpha));
Sv_yk     = DV_yk .* sin( p.rVy5 .* atan(p.rVy6 .* kappa) );

%% ---- G_xα : Fx reduction by slip angle ----------------------------
S_Hxa = p.rHx1 .* ones(size(Fz));
B_xa  = p.rBx1 .* cos(atan(p.rBx2 .* kappa));
C_xa  = p.rCx1;
E_xa  = p.rEx1 + p.rEx2 .* dfz;

H_xa  = B_xa .* (alpha + S_Hxa);
H0_xa = B_xa .* S_Hxa;

G_xa_num = cos( C_xa .* atan( H_xa - E_xa .* (H_xa - atan(H_xa)) ) );
G_xa_den = cos( C_xa .* atan( H0_xa - E_xa .* (H0_xa - atan(H0_xa)) ) );
G_xa_den(abs(G_xa_den) < 1e-9) = 1e-9;
G_xa = G_xa_num ./ G_xa_den;
G_xa = max(G_xa, 0);

%% ---- Combined-slip outputs ----------------------------------------
Fy = G_yk .* Fy0 + Sv_yk;
Fx = G_xa .* Fx0;

if nargout > 2
    info.Fy0   = Fy0;
    info.Fx0   = Fx0;
    info.G_yk  = G_yk;
    info.G_xa  = G_xa;
    info.Sv_yk = Sv_yk;
end
end
