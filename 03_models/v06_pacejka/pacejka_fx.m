function [Fx, info] = pacejka_fx(kappa, Fz, p, gamma)
%PACEJKA_FX  Longitudinal tyre force from Pacejka MF 5.2 (pure slip).
%
%   Fx = pacejka_fx(kappa, Fz, p)
%   Fx = pacejka_fx(kappa, Fz, p, gamma)
%   [Fx, info] = pacejka_fx(...)
%
% Pure-slip longitudinal force. Combined-slip reduction (Fx drop in the
% presence of Fy) is the G_xα factor in pacejka_combined.m.
%
% INPUTS
%   kappa [-]    longitudinal slip ratio. κ > 0 = drive (wheel spinning
%                  faster than road); κ < 0 = brake. Range typically
%                  ±0.20 in racing.
%   Fz    [N]    vertical load. Same broadcast rules as pacejka_fy.
%   p     struct Pacejka coefficient set (see pacejka_coeffs_*.m).
%   gamma [rad]  camber angle (optional, default 0). Pure long. is
%                  weakly camber-dependent; we keep it for completeness.
%
% OUTPUTS
%   Fx    [N]    longitudinal force at the contact patch. Positive Fx
%                  is in the direction of travel (drive); negative is
%                  brake.
%   info  struct optional diagnostics (.B, .C, .D, .E, .Sh, .Sv, .Kx).
%
% MATH (MF 5.2, longitudinal, pure slip)
%   dfz = (Fz − Fz0) / Fz0
%   Sh  = pHx1 + pHx2·dfz
%   Sv  = Fz·(pVx1 + pVx2·dfz)
%   κ_t = κ + Sh
%   μ_x = (pDx1 + pDx2·dfz)·(1 − pDx3·γ²)
%   D   = μ_x · Fz
%   C   = pCx1
%   E   = (pEx1 + pEx2·dfz + pEx3·dfz²) · (1 − pEx4·sign(κ_t))
%   Kx  = Fz·(pKx1 + pKx2·dfz)·exp(pKx3·dfz)
%   B   = Kx / (C·D)
%   Fx  = D·sin(C·atan(B·κ_t − E·(B·κ_t − atan(B·κ_t)))) + Sv
%
% Author:  Jaykumar Patil
% Created: 2026-04-22  (v06 Pacejka — Phase 6.5, Step 2)

if nargin < 4 || isempty(gamma), gamma = 0; end

kappa = kappa(:);
Fz    = Fz(:);
gamma = gamma(:);

n = max([numel(kappa), numel(Fz), numel(gamma)]);
if numel(kappa) == 1, kappa = kappa * ones(n,1); end
if numel(Fz)    == 1, Fz    = Fz    * ones(n,1); end
if numel(gamma) == 1, gamma = gamma * ones(n,1); end

dfz = (Fz - p.Fz0) / p.Fz0;
Sh  = p.pHx1 + p.pHx2 .* dfz;
Sv  = Fz .* (p.pVx1 + p.pVx2 .* dfz);

kappa_t = kappa + Sh;

mu_x = (p.pDx1 + p.pDx2 .* dfz) .* (1 - p.pDx3 .* gamma.^2);
D    = mu_x .* Fz;
C    = p.pCx1 .* ones(size(Fz));

E    = (p.pEx1 + p.pEx2 .* dfz + p.pEx3 .* dfz.^2) ...
       .* (1 - p.pEx4 .* sign(kappa_t));

Kx = Fz .* (p.pKx1 + p.pKx2 .* dfz) .* exp(p.pKx3 .* dfz);

denom = C .* D;
denom(abs(denom) < 1e-9) = 1e-9;
B = Kx ./ denom;

arg = B .* kappa_t - E .* (B .* kappa_t - atan(B .* kappa_t));
Fx0 = D .* sin( C .* atan(arg) );
Fx  = Fx0 + Sv;

if nargout > 1
    info.B    = B;
    info.C    = C;
    info.D    = D;
    info.E    = E;
    info.Sh   = Sh;
    info.Sv   = Sv;
    info.Kx   = Kx;
    info.mu_x = mu_x;
end
end
