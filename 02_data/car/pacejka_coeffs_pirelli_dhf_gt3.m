%% pacejka_coeffs_pirelli_dhf_gt3.m
%  Pacejka Magic Formula 5.2 coefficient set for the front and rear tyres
%  of the Mercedes-AMG GT3 (Pirelli DHF-class racing slick, ~280/680-18).
%
%  STARTING POINT
%  --------------
%  These coefficients are NOT measured on the actual Pirelli DHF — they
%  are starting values derived from publicly available racing-slick data
%  in the FSAE Tire Test Consortium (Calspan rolling-rig) archive,
%  scaled so that the peak μ at the nominal load matches what we already
%  calibrated for v05 (μ_eff ≈ 1.50 at Fz ≈ 4500 N). They are the v06.0
%  baseline; v06.1 will replace them with values fitted from logged
%  iRacing slip-angle and lateral-force data.
%
%  CONVENTION
%  ----------
%  Sign convention: ISO. Slip angle α positive for slip towards the
%  outside of the corner. Camber γ positive when the wheel leans into
%  the corner. Load Fz positive (N).
%
%  COEFFICIENT GROUPS
%  ------------------
%  pCy1 ............ shape factor C (curve sharpness)
%  pDy1, pDy2 ...... peak factor D — base value and load drop dfz term
%  pDy3 ............ camber sensitivity of peak grip
%  pEy1, pEy2 ...... curvature factor E — base + dfz term
%  pEy3, pEy4, pEy5  asymmetric / camber corrections to E
%  pKy1 ............ cornering stiffness gradient at Fz0
%  pKy2 ............ Fz weight on stiffness
%  pKy3 ............ camber sensitivity of stiffness
%  pHy1, pHy2, pHy3  horizontal shift Sh (typically near 0 for racing slicks)
%  pVy1, pVy2, pVy3, pVy4   vertical shift Sv (ply-steer / conicity terms;
%                            assumed 0 for symmetric racing slicks)
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-22  (v06 Pacejka — Phase 6.5)

p = struct();

%% Reference / nominal load
p.Fz0 = 5000;          % [N]   nominal vertical load (≈ axle base load)

%% Shape and peak
%  pDy1 and pDy2 are scaled so that μ_y(Fz) matches the v05 [CAL] tyre
%  curve μ = mu_0 − k·Fz at the operating point. With v05 calibrated to
%  mu_0=1.75 and load_sens_k=5.5e-5, at Fz=Fz0=5000:
%      μ_v05 = 1.75 − 5.5e-5·5000 = 1.475       → pDy1 = 1.475
%  Slope: dμ_v05/dFz = −5.5e-5 (per N) ;  dμ_y/dFz = pDy2/Fz0 (per N)
%      pDy2 = −5.5e-5 · 5000 = −0.275
%  This makes v06's Pacejka peak grip equal v05's [CAL] grip at the
%  operating point while preserving the curve SHAPE (sin/atan magic
%  formula) that v05's surrogate didn't have. v06.1 will re-fit these
%  from logged slip-angle / Fy data; the values here are anchor points,
%  not free parameters.
p.pCy1 = 1.35;         % [-]   typical for racing slicks (1.30..1.40)
p.pDy1 = 1.475;        % [-]   peak μ at Fz0; matches v05 [CAL] at Fz=5000
p.pDy2 = -0.275;       % [-]   load drop; matches v05 load_sens_k=5.5e-5
p.pDy3 = 2.0;          % [-]   camber adds peak grip; we keep γ=0 for QSS

%% Curvature
p.pEy1 = -0.6;         % [-]   typical "rounded peak" shape
p.pEy2 = 0.05;
p.pEy3 = 0.0;          % asymmetry: 0 for symmetric racing slick
p.pEy4 = 0.0;
p.pEy5 = -0.5;

%% Cornering stiffness
p.pKy1 = -35.0;        % [-]   gradient at Fz0; negative by convention
                        %       maps to ~K_y0 = pKy1·Fz0 = -175 kN/rad
p.pKy2 = 1.7;          % [-]   Fz weighting in Ky(Fz)
p.pKy3 = 0.05;         % [-]   small camber sensitivity

%% Horizontal/vertical shifts (negligible for symmetric slick + γ=0)
p.pHy1 = 0.0;
p.pHy2 = 0.0;
p.pHy3 = 0.0;
p.pVy1 = 0.0;
p.pVy2 = 0.0;
p.pVy3 = 0.0;
p.pVy4 = 0.0;

%% ====================================================================
%  LONGITUDINAL (Fx) coefficients — pure-slip Magic Formula
%  ====================================================================
p.pCx1 = 1.60;         % shape (longitudinal slightly sharper than lateral)
p.pDx1 = 1.475;        % peak μ_x at Fz0; matches scaled lateral
p.pDx2 = -0.275;       % load drop; matches scaled lateral
p.pDx3 = 0.0;          % no camber sensitivity in pure long.
p.pEx1 = -0.1;         % rounded peak
p.pEx2 = 0.05;
p.pEx3 = 0.0;
p.pEx4 = 0.0;          % asymmetry in drive vs brake (set 0 for symmetry)
p.pKx1 = 30.0;         % longitudinal slip stiffness gradient at Fz0
p.pKx2 = 0.0;
p.pKx3 = -0.5;         % stiffness drops slightly with load
p.pHx1 = 0.0;
p.pHx2 = 0.0;
p.pVx1 = 0.0;
p.pVx2 = 0.0;

%% ====================================================================
%  COMBINED SLIP coefficients (G-factor reduction, MF 5.2)
%  ====================================================================
%  G_yκ multiplies Fy0 to reduce lateral force when longitudinal slip κ
%  is non-zero. Symmetric: G_xα multiplies Fx0 to reduce longitudinal
%  force when slip angle α is non-zero. Together they produce a smooth
%  friction-ellipse envelope.
%
%  Defaults below produce a near-elliptical envelope; v06.1 can re-fit
%  these from logged combined-slip events (heavy braking into a corner,
%  power-on at corner exit).

% Fy reduction by longitudinal slip
p.rBy1 = 8.0;
p.rBy2 = 8.0;
p.rBy3 = 0.0;
p.rCy1 = 1.0;
p.rEy1 = 0.0;
p.rEy2 = 0.0;
p.rHy1 = 0.0;
p.rHy2 = 0.0;
p.rVy1 = 0.0;          % vertical-shift terms — keep zero at v06.0
p.rVy2 = 0.0;
p.rVy3 = 0.0;
p.rVy4 = 0.0;
p.rVy5 = 1.9;
p.rVy6 = 10.0;

% Fx reduction by slip angle
p.rBx1 = 15.0;
p.rBx2 = 8.0;
p.rCx1 = 1.0;
p.rEx1 = 0.0;
p.rEx2 = 0.0;
p.rHx1 = 0.0;

%% Provenance metadata
p.source     = 'FSAE TTC racing-slick generic; D scaled to match v05 calibration';
p.version    = 'MF 5.2 (lateral + longitudinal + combined slip)';
p.created    = '2026-04-22';
p.notes      = ['Same coefficients used for front and rear initially. ' ...
                'v06.1 will fit per-axle separately from logged data.'];

% Same coefficients front and rear at v06.0; per-axle split comes in v06.1
car.tyre.pacejka.front = p;
car.tyre.pacejka.rear  = p;

clear p;
fprintf('Pacejka MF 5.2 coefficients loaded for AMG GT3 tyres (front + rear).\n');
