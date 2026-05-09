function out = get_axle_grip_v06(v, a_long_signed, a_lat, car)
%GET_AXLE_GRIP_V06  Per-axle grip via Pacejka MF 5.2 (pure lateral peak).
%
%   out = get_axle_grip_v06(v, a_long_signed, a_lat, car)
%
% Mirrors get_axle_grip_v05's interface but computes peak lateral grip
% per tyre by sweeping the Pacejka MF 5.2 curve instead of using the
% surrogate μ(Fz)·Fz. Outputs are interface-compatible so v06's three-
% pass solver can be a near-drop-in replacement for v05.
%
% INPUTS
%   v               [m/s]   vehicle speed at this track point
%   a_long_signed   [m/s²]  signed longitudinal acceleration (+ drive, − brake)
%   a_lat           [m/s²]  lateral acceleration (positive)
%   car             struct  vehicle parameter struct (incl. car.tyre.pacejka)
%
% OUTPUT (struct, same field names as v05's helper plus extras)
%   .F_grip_f, .F_grip_r            [N]    per-axle peak lateral grip
%   .Fz_f, .Fz_r                    [N]    per-axle vertical loads (sum of pair)
%   .Fz_f_out, .Fz_f_in,            [N]    per-tyre loads after lateral transfer
%   .Fz_r_out, .Fz_r_in
%   .alpha_peak_f_out,              [rad]  slip angle at peak |Fy| per tyre
%   .alpha_peak_f_in,                       (the operating point if the tyre
%   .alpha_peak_r_out,                       is at its lateral grip limit)
%   .alpha_peak_r_in
%   .mu_eff_f, .mu_eff_r            [-]    effective μ at peak for each axle
%
% Author:  Jaykumar Patil
% Created: 2026-04-22  (v06 Pacejka — Phase 6.5, Step 3a)

g = 9.81;

% ====================================================================
% 1. Vertical loads — per axle, per tyre. Same physics as v05:
%    static + aero + longitudinal transfer + lateral transfer (ARB).
% ====================================================================

% Static + aero per axle
Fz_aero_total = car.aero_df_coeff * v^2;
Fz_f_static_aero = car.Fz_f_static + Fz_aero_total *      car.aero_balance_f;
Fz_r_static_aero = car.Fz_r_static + Fz_aero_total * (1 - car.aero_balance_f);

% Longitudinal transfer (positive a_long → load shifts to rear)
dFz_long = car.mass * a_long_signed * car.h_cog / car.wheelbase;
Fz_f_axle = max(Fz_f_static_aero - dFz_long, 100);
Fz_r_axle = max(Fz_r_static_aero + dFz_long, 100);

% Lateral transfer — rigid-body total split by ARB roll-stiffness ratio
t_avg          = 0.5 * (car.track_f + car.track_r);
dFz_lat_total  = car.mass * a_lat * car.h_cog / t_avg;
dFz_lat_f      = dFz_lat_total * car.suspension.roll_dist_f;
dFz_lat_r      = dFz_lat_total * car.suspension.roll_dist_r;

Fz_f_base_per_tyre = 0.5 * Fz_f_axle;
Fz_r_base_per_tyre = 0.5 * Fz_r_axle;

Fz_f_out = max(Fz_f_base_per_tyre + dFz_lat_f, 50);
Fz_f_in  = max(Fz_f_base_per_tyre - dFz_lat_f, 50);
Fz_r_out = max(Fz_r_base_per_tyre + dFz_lat_r, 50);
Fz_r_in  = max(Fz_r_base_per_tyre - dFz_lat_r, 50);

% ====================================================================
% 2. Pacejka peak lateral grip per tyre — sweep α to find max |Fy|.
% ====================================================================
p_f = car.tyre.pacejka.front;
p_r = car.tyre.pacejka.rear;

[Fy_pk_f_out, a_pk_f_out] = local_peak_fy(Fz_f_out, p_f);
[Fy_pk_f_in,  a_pk_f_in ] = local_peak_fy(Fz_f_in,  p_f);
[Fy_pk_r_out, a_pk_r_out] = local_peak_fy(Fz_r_out, p_r);
[Fy_pk_r_in,  a_pk_r_in ] = local_peak_fy(Fz_r_in,  p_r);

% ====================================================================
% 3. Axle grip = sum of two tyres on each axle
% ====================================================================
F_grip_f = Fy_pk_f_out + Fy_pk_f_in;
F_grip_r = Fy_pk_r_out + Fy_pk_r_in;

% ====================================================================
% 4. Pack output struct
% ====================================================================
out.F_grip_f          = F_grip_f;
out.F_grip_r          = F_grip_r;
out.Fz_f              = Fz_f_axle;
out.Fz_r              = Fz_r_axle;
out.Fz_f_out          = Fz_f_out;
out.Fz_f_in           = Fz_f_in;
out.Fz_r_out          = Fz_r_out;
out.Fz_r_in           = Fz_r_in;
out.alpha_peak_f_out  = a_pk_f_out;
out.alpha_peak_f_in   = a_pk_f_in;
out.alpha_peak_r_out  = a_pk_r_out;
out.alpha_peak_r_in   = a_pk_r_in;
out.mu_eff_f          = F_grip_f / (Fz_f_out + Fz_f_in);
out.mu_eff_r          = F_grip_r / (Fz_r_out + Fz_r_in);
end

%% ---- helper -------------------------------------------------------
function [F_pk, a_pk] = local_peak_fy(Fz, p)
% Sweep slip angle 0.5..25° at this Fz, return peak |Fy| and its α.
% Vectorised: scalar Fz only (called per tyre).
    if Fz <= 50
        F_pk = 0;
        a_pk = 0;
        return;
    end
    a_grid = (0.5:0.25:25)' * pi/180;          % 99 points
    Fy = pacejka_fy(a_grid, Fz, p);
    [F_pk, idx] = max(abs(Fy));
    a_pk = a_grid(idx);
end
