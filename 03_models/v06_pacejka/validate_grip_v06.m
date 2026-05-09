%% validate_grip_v06.m
%  v06 Step 3a sanity check — compare v06's per-axle peak grip against
%  v05's at a sweep of operating points, and verify the implied slip
%  angles are physically reasonable.
%
%  WHY THIS IS THE STEP-3a GATE
%  ----------------------------
%  v06 is a structural rewrite of v05's grip computation. Before we wire
%  it into the lap solver, we need to confirm:
%    (a) For pure cornering (a_long = 0), v06's peak grip per tyre is
%        within a few percent of v05's μ(Fz)·Fz. This is the "v06
%        collapses to v05" sanity check — they should agree at peak.
%    (b) The slip angle at peak |Fy| sits in the realistic 6..12° band
%        across the load range we'll see at the N24.
%
%  USAGE
%    >> startup_project
%    >> import_reference_lap_ibt
%    >> build_track
%    >> lap_sim_v05                    % to populate sim05
%    >> run('03_models/v06_pacejka/validate_grip_v06.m')
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-22  (v06 Pacejka — Phase 6.5, Step 3a)

if ~exist('car','var') || ~exist('track','var') || ~exist('sim05','var')
    error('Load car + track + run lap_sim_v05 first.');
end
if ~isfield(car.tyre, 'pacejka')
    pacejka_coeffs_pirelli_dhf_gt3;
end

fprintf('\n=== v06 Step 3a — get_axle_grip_v06 vs v05 ===\n');

n = numel(track.dist);
F_grip_f_v06 = zeros(n,1);
F_grip_r_v06 = zeros(n,1);
mu_eff_f_v06 = zeros(n,1);
mu_eff_r_v06 = zeros(n,1);
alpha_pk_f   = zeros(n,1);
alpha_pk_r   = zeros(n,1);

% Reproduce v05's grip at each track point using the calibrated sim's v.
% At v05's converged operating point, a_lat = v² · κ. a_long ≈ 0 in pure
% cornering parts of the lap; in mixed phases v05 carries an a_long_signed
% but we ignore it here — pure-lateral comparison only.
v_op   = sim05.v;
a_lat  = v_op.^2 .* track.kappa;
a_long = zeros(size(v_op));    % pure lateral for the comparison

% Loop track-point-by-track-point. Slow (~5-10 s) but exact.
fprintf('Computing v06 grip at %d track points...', n);
t0 = tic;
for k = 1:n
    g = get_axle_grip_v06(v_op(k), a_long(k), a_lat(k), car);
    F_grip_f_v06(k) = g.F_grip_f;
    F_grip_r_v06(k) = g.F_grip_r;
    mu_eff_f_v06(k) = g.mu_eff_f;
    mu_eff_r_v06(k) = g.mu_eff_r;
    alpha_pk_f(k)   = mean([g.alpha_peak_f_out, g.alpha_peak_f_in]);
    alpha_pk_r(k)   = mean([g.alpha_peak_r_out, g.alpha_peak_r_in]);
end
fprintf(' done in %.1f s.\n', toc(t0));

%% ---- Figure: v06 grip + slip angles around the lap ------------------
figure('Name','v06 Step 3a — grip and slip angle around the lap', ...
       'NumberTitle','off', 'Position',[100 100 1300 800]);

subplot(3,1,1);
plot(track.dist/1000, v_op*3.6, 'k', 'LineWidth', 0.8);
ylabel('v_{op} [km/h]'); grid on;
title('Operating speed (sim05 calibrated lap)');
xlim([0 track.dist(end)/1000]);

subplot(3,1,2); hold on;
plot(track.dist/1000, F_grip_f_v06/1000, 'b', 'LineWidth', 0.7, ...
     'DisplayName','v06 front');
plot(track.dist/1000, F_grip_r_v06/1000, 'r', 'LineWidth', 0.7, ...
     'DisplayName','v06 rear');
ylabel('Per-axle grip [kN]'); grid on; legend;
title('v06 peak lateral grip per axle (Pacejka MF 5.2 sweep)');
xlim([0 track.dist(end)/1000]);

subplot(3,1,3); hold on;
plot(track.dist/1000, alpha_pk_f * 180/pi, 'b', 'LineWidth', 0.7, ...
     'DisplayName','front α at peak');
plot(track.dist/1000, alpha_pk_r * 180/pi, 'r', 'LineWidth', 0.7, ...
     'DisplayName','rear α at peak');
ylabel('Slip angle [°]'); xlabel('Distance [km]'); grid on; legend;
title('Pacejka peak slip angle around the lap (= cornering operating point)');
xlim([0 track.dist(end)/1000]);

%% ---- Figure: histogram of slip angles -------------------------------
figure('Name','v06 Step 3a — slip-angle distribution','NumberTitle','off', ...
       'Position',[100 100 900 400]);
subplot(1,2,1);
histogram(alpha_pk_f*180/pi, 30); grid on;
xlabel('Front α at peak |Fy| [°]'); ylabel('count');
title('Front-tyre peak slip angle distribution');

subplot(1,2,2);
histogram(alpha_pk_r*180/pi, 30); grid on;
xlabel('Rear α at peak |Fy| [°]'); ylabel('count');
title('Rear-tyre peak slip angle distribution');

%% ---- Numerical landmarks --------------------------------------------
% Spot-check at a corner near the centre of the lap (cornering, low speed)
[~, ki_corner] = min(v_op);
fprintf('\n=== Spot-check at min-speed corner (track distance %.0f m, v=%.1f km/h) ===\n', ...
        track.dist(ki_corner), v_op(ki_corner)*3.6);
g = get_axle_grip_v06(v_op(ki_corner), 0, ...
                      v_op(ki_corner)^2*track.kappa(ki_corner), car);
fprintf('  Fz per tyre: F_out=%5.0f F_in=%5.0f  R_out=%5.0f R_in=%5.0f N\n', ...
        g.Fz_f_out, g.Fz_f_in, g.Fz_r_out, g.Fz_r_in);
fprintf('  Peak |Fy|/Fz per tyre: F_out=%.2f F_in=%.2f  R_out=%.2f R_in=%.2f\n', ...
        g.F_grip_f/(g.Fz_f_out+g.Fz_f_in), g.F_grip_r/(g.Fz_r_out+g.Fz_r_in), ...
        g.F_grip_r/(g.Fz_r_out+g.Fz_r_in), g.F_grip_r/(g.Fz_r_out+g.Fz_r_in));
fprintf('  Slip angle at peak: F_out=%.1f F_in=%.1f  R_out=%.1f R_in=%.1f deg\n', ...
        g.alpha_peak_f_out*180/pi, g.alpha_peak_f_in*180/pi, ...
        g.alpha_peak_r_out*180/pi, g.alpha_peak_r_in*180/pi);

% Across the lap
fprintf('\n=== Per-axle peak slip angle distribution across the lap ===\n');
fprintf('  Front:  mean = %.1f°,  median = %.1f°,  range = [%.1f .. %.1f]°\n', ...
        mean(alpha_pk_f)*180/pi, median(alpha_pk_f)*180/pi, ...
        min(alpha_pk_f)*180/pi,  max(alpha_pk_f)*180/pi);
fprintf('  Rear :  mean = %.1f°,  median = %.1f°,  range = [%.1f .. %.1f]°\n', ...
        mean(alpha_pk_r)*180/pi, median(alpha_pk_r)*180/pi, ...
        min(alpha_pk_r)*180/pi,  max(alpha_pk_r)*180/pi);

% Compare effective μ to v05's μ across the lap
fprintf('\n=== Effective μ comparison (v06 vs v05) ===\n');
fprintf('  v06 mu_eff front:  mean=%.3f  range=[%.3f .. %.3f]\n', ...
        mean(mu_eff_f_v06), min(mu_eff_f_v06), max(mu_eff_f_v06));
fprintf('  v06 mu_eff rear :  mean=%.3f  range=[%.3f .. %.3f]\n', ...
        mean(mu_eff_r_v06), min(mu_eff_r_v06), max(mu_eff_r_v06));
fprintf('  v05 mu_0=%.3f, load_sens_k=%.2g (the calibration baseline).\n', ...
        car.tyre.mu_0, car.tyre.load_sens_k);

fprintf('\n=== Step 3a gates ===\n');
front_in_band = mean((alpha_pk_f*180/pi >= 5) & (alpha_pk_f*180/pi <= 14));
rear_in_band  = mean((alpha_pk_r*180/pi >= 5) & (alpha_pk_r*180/pi <= 14));
fprintf('  Slip angles in 5..14°: front %.0f%%, rear %.0f%%  (target ≥ 90%%)\n', ...
        100*front_in_band, 100*rear_in_band);
fprintf('=== Done. ===\n\n');
