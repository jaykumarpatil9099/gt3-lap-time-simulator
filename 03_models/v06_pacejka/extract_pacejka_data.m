function data = extract_pacejka_data(ibt_path, car, opts)
%EXTRACT_PACEJKA_DATA  Build a (α, Fz, Fy) data table from the IBT log.
%
%   data = extract_pacejka_data(ibt_path, car)
%   data = extract_pacejka_data(ibt_path, car, opts)
%
% Reads the iRacing IBT, computes the kinematic slip angle at each axle,
% the per-tyre vertical loads (same physics as v05/v06), and the per-axle
% lateral force from m·a_lat split via moment balance. Filters out non-
% cornering samples and returns a struct of (|α|, Fz, |F_y|) data points
% suitable for non-linear least-squares fitting of Pacejka coefficients.
%
% INPUTS
%   ibt_path   path to the .ibt file (use ref.meta.source_file)
%   car        struct from amg_gt3_params (needs .mass, .h_cog, .wheelbase,
%              .weight_dist_f, .track_f, .track_r, .aero_*, .steering_ratio,
%              .suspension.roll_dist_f/r)
%   opts       (optional) struct with:
%                .a_lat_min        keep |a_lat| > this [m/s²]   default 5
%                .a_long_max       keep |a_long| < this [m/s²]  default 5
%                .clean_only       keep only clean laps          default true
%
% OUTPUT
%   data       struct with column vectors (one row per kept sample):
%                .alpha_f, .alpha_r          [rad]  abs slip angles
%                .Fz_f_out, .Fz_f_in         [N]    per-tyre Fz, front
%                .Fz_r_out, .Fz_r_in         [N]    per-tyre Fz, rear
%                .Fy_f_axle, .Fy_r_axle      [N]    abs axle lateral force
%                .v, .a_lat, .a_long         (kinematics)
%                .source                     ibt_path
%                .n_samples_total            samples in the IBT
%                .n_samples_kept             after filtering
%
% SLIP-ANGLE KINEMATICS (bicycle model, ISO sign convention)
%   a       = weight_dist_r · wheelbase    (front axle to CoG)
%   b       = weight_dist_f · wheelbase    (CoG to rear axle)
%   δ_wheel = SteeringWheelAngle / steering_ratio
%   α_f     = δ_wheel − atan2(VelocityY + a·YawRate, VelocityX)
%   α_r     =          − atan2(VelocityY − b·YawRate, VelocityX)
%
% Author:  Jaykumar Patil
% Created: 2026-04-22  (v06.1 Pacejka data extraction)

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'a_lat_min'),   opts.a_lat_min  = 5;    end   % m/s²
if ~isfield(opts, 'a_long_max'),  opts.a_long_max = 5;    end   % m/s²
if ~isfield(opts, 'clean_only'),  opts.clean_only = true; end

%% ---- Read the IBT channels we need -------------------------------
fprintf('\nextract_pacejka_data: reading IBT...\n');
channels = {'SessionTime','Lap','LapDistPct','OnPitRoad', ...
            'Speed','VelocityX','VelocityY','YawRate', ...
            'SteeringWheelAngle','LatAccel','LongAccel','Throttle','Brake'};
[d, meta] = read_ibt(ibt_path, channels);
n_total = numel(d.SessionTime);

%% ---- Bicycle geometry --------------------------------------------
g = 9.81;
m = car.mass;
L = car.wheelbase;
a = car.weight_dist_r * L;        % front axle → CoG
b = car.weight_dist_f * L;        % CoG → rear axle
sr = car.steering_ratio;

%% ---- Per-sample slip angles + per-tyre Fz + axle Fy --------------
v_clamp = max(d.Speed, 5);                              % avoid divide by 0
delta_wheel = d.SteeringWheelAngle / sr;

% Slip angles (rad), absolute value (Pacejka is symmetric)
alpha_f = abs( delta_wheel - atan2(d.VelocityY + a*d.YawRate, d.VelocityX) );
alpha_r = abs( - atan2(d.VelocityY - b*d.YawRate, d.VelocityX) );

% Per-tyre Fz (same physics as get_axle_grip_v06, vectorised)
F_df_total = car.aero_df_coeff * d.Speed.^2;
Fz_f_static_aero = car.Fz_f_static + F_df_total *      car.aero_balance_f;
Fz_r_static_aero = car.Fz_r_static + F_df_total * (1 - car.aero_balance_f);

dFz_long = m * d.LongAccel * car.h_cog / L;
Fz_f_axle = max(Fz_f_static_aero - dFz_long, 200);
Fz_r_axle = max(Fz_r_static_aero + dFz_long, 200);

t_avg          = 0.5 * (car.track_f + car.track_r);
dFz_lat_total  = m * abs(d.LatAccel) * car.h_cog / t_avg;
dFz_lat_f      = dFz_lat_total * car.suspension.roll_dist_f;
dFz_lat_r      = dFz_lat_total * car.suspension.roll_dist_r;

Fz_f_out = max(0.5*Fz_f_axle + dFz_lat_f, 50);
Fz_f_in  = max(0.5*Fz_f_axle - dFz_lat_f, 50);
Fz_r_out = max(0.5*Fz_r_axle + dFz_lat_r, 50);
Fz_r_in  = max(0.5*Fz_r_axle - dFz_lat_r, 50);

% Axle lateral force from rigid-body moment balance:
%   F_y_f / F_y_r = b / a   ⇒   F_y_f = m·a_lat·b/L,  F_y_r = m·a_lat·a/L
Fy_f_axle = abs( m * d.LatAccel * b / L );
Fy_r_axle = abs( m * d.LatAccel * a / L );

%% ---- Filter ------------------------------------------------------
mask_corner   = abs(d.LatAccel)  >= opts.a_lat_min;
mask_no_long  = abs(d.LongAccel) <= opts.a_long_max;
mask_no_pit   = ~d.OnPitRoad;
mask_v        = d.Speed > 15;        % > 54 km/h
mask_alpha_ok = alpha_f < 25*pi/180 & alpha_r < 25*pi/180; % no insane outliers

mask = mask_corner & mask_no_long & mask_no_pit & mask_v & mask_alpha_ok;
n_kept = sum(mask);

%% ---- Pack --------------------------------------------------------
data.alpha_f       = alpha_f(mask);
data.alpha_r       = alpha_r(mask);
data.Fz_f_out      = Fz_f_out(mask);
data.Fz_f_in       = Fz_f_in(mask);
data.Fz_r_out      = Fz_r_out(mask);
data.Fz_r_in       = Fz_r_in(mask);
data.Fy_f_axle     = Fy_f_axle(mask);
data.Fy_r_axle     = Fy_r_axle(mask);
data.v             = d.Speed(mask);
data.a_lat         = d.LatAccel(mask);
data.a_long        = d.LongAccel(mask);
data.source        = ibt_path;
data.n_samples_total = n_total;
data.n_samples_kept  = n_kept;
data.opts            = opts;

fprintf('  total samples:   %d\n', n_total);
fprintf('  kept samples:    %d (|a_lat|>=%g m/s², |a_long|<=%g m/s²)\n', ...
        n_kept, opts.a_lat_min, opts.a_long_max);
fprintf('  α_f range:       %.2f .. %.2f deg\n', ...
        min(data.alpha_f)*180/pi, max(data.alpha_f)*180/pi);
fprintf('  α_r range:       %.2f .. %.2f deg\n', ...
        min(data.alpha_r)*180/pi, max(data.alpha_r)*180/pi);
fprintf('  Fy_f_axle:       %.0f .. %.0f N\n', ...
        min(data.Fy_f_axle), max(data.Fy_f_axle));
fprintf('  Fy_r_axle:       %.0f .. %.0f N\n', ...
        min(data.Fy_r_axle), max(data.Fy_r_axle));

end
