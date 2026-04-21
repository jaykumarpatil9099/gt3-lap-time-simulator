%% lap_sim_v05_refined.m
%  Lap time simulator v05_refined: Bicycle model with ARB-corrected lateral load transfer.
%
%  v05_refined = v05 + anti-roll bar modeling
%
%  v05 was too conservative because it calculated FULL dynamic lateral load transfer.
%  Real cars have ARBs that decouple lateral transfer from body roll, reducing the
%  effective load transfer to ~30-40% of the theoretical full value.
%
%  This version applies ARB stiffness corrections:
%    dFz_lat_eff = dFz_lat_full * (K_tire / (K_ARB + K_tire))
%
%  The result should be realistic and close to the ±1% correlation target.
%
%  INPUT:  'ref' struct (run import_reference_lap first)
%          'track' struct (run build_track_from_gps first)
%          'car' struct (loaded by startup_project) — must have suspension.load_xfer_reduction_*
%
%  OUTPUT: Lap time + verification plots
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-18

%% ========================================================================
%  0. CHECK INPUTS AND INITIALIZE
%  ========================================================================

if ~exist('ref', 'var')
    error('Run import_reference_lap first.');
end
if ~exist('track', 'var')
    error('Run build_track_from_gps first.');
end
if ~exist('car', 'var')
    error('Run startup_project first.');
end

fprintf('\n=== Lap Sim v05_refined — Bicycle Model with ARB-Corrected Lateral Transfer ===\n');
fprintf('Car:   %s\n', car.meta.name);
fprintf('Track: %s (%.3f km)\n', track.meta.name, track.length/1000);

g = 9.81;

% Tyre parameters
mu_0 = car.tyre.mu_0;
k_load_sens = car.tyre.load_sens_k;

% Aero coefficients
rho = car.rho;
A_frontal = car.frontal_area;
Cd = car.aero_drag_coeff;
drag_coeff = 0.5 * rho * Cd * A_frontal;
aero_df_coeff = car.aero_df_coeff;

% Peak engine power
engine_rpm_array = car.engine.rpm;
engine_torque_array = car.engine.torque;
P_array = engine_torque_array .* (engine_rpm_array * 2 * pi / 60);
P_peak = max(P_array);

% Track geometry
L = car.wheelbase;
h_cog = car.h_cog;
track_f = car.track_f;
track_r = car.track_r;

% ARB load transfer reduction factors
load_xfer_reduction_f = car.suspension.load_xfer_reduction_f;
load_xfer_reduction_r = car.suspension.load_xfer_reduction_r;

fprintf('ARB lateral load transfer reduction: F=%.1f%%, R=%.1f%%\n', ...
        load_xfer_reduction_f*100, load_xfer_reduction_r*100);

%% ========================================================================
%  PASS 1: CORNERING SPEED LIMITS (ARB-corrected lateral transfer)
%  ========================================================================

fprintf('Pass 1: Cornering speed limits (ARB-corrected lateral transfer)...\n');

v_corner = zeros(track.n, 1);
points_at_limit = 0;

for i = 1:track.n
    kappa = track.kappa(i);

    if kappa < 1e-6
        v_corner(i) = 400;
        continue;
    end

    % Iterate to find v where outside tyre reaches grip limit
    v_try = 150;

    for iter = 1:20
        v_try_old = v_try;

        % Centripetal acceleration
        a_lat = v_try^2 * kappa;

        % Aero downforce (distributed by aero balance)
        Fz_aero_total = aero_df_coeff * v_try^2;
        Fz_aero_f = Fz_aero_total * car.aero_balance_f;
        Fz_aero_r = Fz_aero_total * (1 - car.aero_balance_f);

        % Static load per tyre
        Fz_static_per_tyre_f = (car.mass * g) * car.weight_dist_f / 2;
        Fz_static_per_tyre_r = (car.mass * g) * car.weight_dist_r / 2;
        Fz_aero_per_tyre_f = Fz_aero_f / 2;
        Fz_aero_per_tyre_r = Fz_aero_r / 2;

        % FULL lateral load transfer (before ARB correction)
        dFz_lat_full_f = car.mass * a_lat * h_cog / track_f;
        dFz_lat_full_r = car.mass * a_lat * h_cog / track_r;

        % EFFECTIVE lateral load transfer (with ARB reduction)
        dFz_lat_eff_f = dFz_lat_full_f * load_xfer_reduction_f;
        dFz_lat_eff_r = dFz_lat_full_r * load_xfer_reduction_r;

        % Outside tyres (loaded, with ARB correction)
        Fz_out_f = Fz_static_per_tyre_f + Fz_aero_per_tyre_f + dFz_lat_eff_f / 2;
        Fz_out_r = Fz_static_per_tyre_r + Fz_aero_per_tyre_r + dFz_lat_eff_r / 2;

        % Inside tyres (unloaded, clamped to avoid negative load)
        Fz_in_f = max(Fz_static_per_tyre_f + Fz_aero_per_tyre_f - dFz_lat_eff_f / 2, 1);
        Fz_in_r = max(Fz_static_per_tyre_r + Fz_aero_per_tyre_r - dFz_lat_eff_r / 2, 1);

        % Grip at outside tyres
        mu_out_f = mu_0 - k_load_sens * Fz_out_f;
        mu_out_r = mu_0 - k_load_sens * Fz_out_r;
        mu_out_f = max(mu_out_f, 0.1);
        mu_out_r = max(mu_out_r, 0.1);

        % Cornering speed limit (minimum of front/rear)
        v_corner_f = sqrt(mu_out_f * g / kappa);
        v_corner_r = sqrt(mu_out_r * g / kappa);
        v_corner_iter = min(v_corner_f, v_corner_r);

        % Damp iteration
        v_try = 0.5 * v_try_old + 0.5 * v_corner_iter;

        if abs(v_try - v_try_old) < 0.1
            break;
        end
    end

    v_corner(i) = v_try;

    if kappa > 0.01
        points_at_limit = points_at_limit + 1;
    end
end

fprintf('  Min cornering speed: %.1f km/h\n', min(v_corner)*3.6);
fprintf('  Points at cap: %d / %d\n', points_at_limit, track.n);

% Grip diagnostic (with ARB effect shown)
fprintf('  Grip with ARB-corrected lateral transfer (κ=0.02 corner, example speeds):\n');
for speed_test = [80, 150, 200, 260]
    v_ms = speed_test / 3.6;
    a_lat_test = v_ms^2 * 0.02;

    Fz_aero_test = aero_df_coeff * v_ms^2;
    Fz_static_f = car.mass * g * car.weight_dist_f / 2;
    Fz_aero_f = Fz_aero_test * car.aero_balance_f / 2;

    dFz_lat_full_f = car.mass * a_lat_test * h_cog / track_f;
    dFz_lat_eff_f = dFz_lat_full_f * load_xfer_reduction_f;  % ARB correction applied

    Fz_out_f = Fz_static_f + Fz_aero_f + dFz_lat_eff_f / 2;
    mu_out_f = mu_0 - k_load_sens * Fz_out_f;
    mu_out_f = max(mu_out_f, 0.1);

    fprintf('    @ %3d km/h: mu_out_f = %.3f, Fz_out_f = %.0f N\n', ...
            speed_test, mu_out_f, Fz_out_f);
end

%% ========================================================================
%  PASS 2: FORWARD PASS (longitudinal transfer, no lateral)
%  ========================================================================

fprintf('Pass 2: Forward pass...\n');

v_fwd = zeros(track.n, 1);
v_fwd(1) = 232.8 / 3.6;  % Start speed [m/s]

for i = 2:track.n
    v_prev = v_fwd(i-1);
    v_cand = v_corner(i);

    F_drag = drag_coeff * v_prev^2;

    a_long = 0;
    for iter_accel = 1:10
        a_old = a_long;

        Fz_aero_r = aero_df_coeff * v_prev^2 / 2;
        dFz_long = car.mass * a_long * h_cog / L;
        Fz_r_static = car.mass * g * car.weight_dist_r / 2;
        Fz_r = Fz_r_static + Fz_aero_r + dFz_long;
        Fz_r = max(Fz_r, 100);

        mu_r = mu_0 - k_load_sens * Fz_r;
        mu_r = max(mu_r, 0.1);

        a_traction_max = mu_r * g;

        if v_prev > 0.1
            a_power_max = (P_peak / v_prev - F_drag) / car.mass;
            a_power_max = max(a_power_max, 0);
        else
            a_power_max = 0;
        end

        a_long = min(a_traction_max, a_power_max);

        if abs(a_long - a_old) < 0.01, break; end
    end

    ds = track.ds;
    v_next_squared = v_prev^2 + 2 * a_long * ds;
    v_next = sqrt(max(v_next_squared, 0));

    v_fwd(i) = min(v_next, v_cand);
end

fprintf('  Max speed: %.1f km/h\n', max(v_fwd)*3.6);

%% ========================================================================
%  PASS 3: BACKWARD PASS (longitudinal transfer, no lateral)
%  ========================================================================

fprintf('Pass 3: Backward pass...\n');

v_bwd = zeros(track.n, 1);
v_bwd(track.n) = v_fwd(track.n);

for i = track.n-1:-1:1
    v_next = v_bwd(i+1);
    v_cand = v_corner(i);

    a_brake = 0;
    for iter_brake = 1:10
        a_old = a_brake;

        Fz_aero_f = aero_df_coeff * v_next^2 / 2;
        dFz_long_brake = car.mass * a_brake * h_cog / L;
        Fz_f_static = car.mass * g * car.weight_dist_f / 2;
        Fz_f = Fz_f_static + Fz_aero_f + dFz_long_brake;
        Fz_f = max(Fz_f, 100);

        Fz_aero_r = aero_df_coeff * v_next^2 / 2;
        Fz_r_static = car.mass * g * car.weight_dist_r / 2;
        Fz_r = Fz_r_static + Fz_aero_r - dFz_long_brake;
        Fz_r = max(Fz_r, 100);

        mu_f = mu_0 - k_load_sens * Fz_f;
        mu_r = mu_0 - k_load_sens * Fz_r;
        mu_f = max(mu_f, 0.1);
        mu_r = max(mu_r, 0.1);

        a_brake_f = mu_f * g;
        a_brake_r = mu_r * g;
        a_brake = min(a_brake_f, a_brake_r);

        if abs(a_brake - a_old) < 0.01, break; end
    end

    ds = track.ds;
    v_max_here_squared = v_next^2 + 2 * a_brake * ds;
    v_max_here = sqrt(v_max_here_squared);

    v_bwd(i) = min(v_max_here, v_cand);
end

fprintf('  Backward pass complete\n');

%% ========================================================================
%  LAP CONTINUITY ITERATION
%  ========================================================================

fprintf('Lap continuity check:\n');

v_final = min(v_fwd, v_bwd);
v_start_after_bwd = v_final(1);

fprintf('  Start: %.2f km/h (from iteration 0)\n', v_start_after_bwd*3.6);

for iter_lap = 1:5
    v_fwd = zeros(track.n, 1);
    v_fwd(1) = v_start_after_bwd;

    for i = 2:track.n
        v_prev = v_fwd(i-1);
        v_cand = v_corner(i);
        F_drag = drag_coeff * v_prev^2;

        a_long = 0;
        for iter_accel = 1:10
            a_old = a_long;
            Fz_aero_r = aero_df_coeff * v_prev^2 / 2;
            dFz_long = car.mass * a_long * h_cog / L;
            Fz_r_static = car.mass * g * car.weight_dist_r / 2;
            Fz_r = Fz_r_static + Fz_aero_r + dFz_long;
            Fz_r = max(Fz_r, 100);
            mu_r = mu_0 - k_load_sens * Fz_r;
            mu_r = max(mu_r, 0.1);
            a_traction_max = mu_r * g;
            if v_prev > 0.1
                a_power_max = (P_peak / v_prev - F_drag) / car.mass;
                a_power_max = max(a_power_max, 0);
            else
                a_power_max = 0;
            end
            a_long = min(a_traction_max, a_power_max);
            if abs(a_long - a_old) < 0.01, break; end
        end

        ds = track.ds;
        v_next_squared = v_prev^2 + 2 * a_long * ds;
        v_next = sqrt(max(v_next_squared, 0));
        v_fwd(i) = min(v_next, v_cand);
    end

    v_bwd = zeros(track.n, 1);
    v_bwd(track.n) = v_fwd(track.n);

    for i = track.n-1:-1:1
        v_next = v_bwd(i+1);
        v_cand = v_corner(i);

        a_brake = 0;
        for iter_brake = 1:10
            a_old = a_brake;
            Fz_aero_f = aero_df_coeff * v_next^2 / 2;
            dFz_long_brake = car.mass * a_brake * h_cog / L;
            Fz_f_static = car.mass * g * car.weight_dist_f / 2;
            Fz_f = Fz_f_static + Fz_aero_f + dFz_long_brake;
            Fz_f = max(Fz_f, 100);
            Fz_aero_r = aero_df_coeff * v_next^2 / 2;
            Fz_r_static = car.mass * g * car.weight_dist_r / 2;
            Fz_r = Fz_r_static + Fz_aero_r - dFz_long_brake;
            Fz_r = max(Fz_r, 100);
            mu_f = mu_0 - k_load_sens * Fz_f;
            mu_r = mu_0 - k_load_sens * Fz_r;
            mu_f = max(mu_f, 0.1);
            mu_r = max(mu_r, 0.1);
            a_brake_f = mu_f * g;
            a_brake_r = mu_r * g;
            a_brake = min(a_brake_f, a_brake_r);
            if abs(a_brake - a_old) < 0.01, break; end
        end

        ds = track.ds;
        v_max_here_squared = v_next^2 + 2 * a_brake * ds;
        v_max_here = sqrt(v_max_here_squared);
        v_bwd(i) = min(v_max_here, v_cand);
    end

    v_final = min(v_fwd, v_bwd);
    v_start_new = v_final(1);
    v_end = v_final(track.n);

    fprintf('  >> Iter %d: start=%.2f, end=%.2f km/h', iter_lap, v_start_new*3.6, v_end*3.6);

    if abs(v_start_new - v_start_after_bwd) < 0.1
        fprintf(' [Converged]\n');
        break;
    else
        fprintf('\n');
        v_start_after_bwd = v_start_new;
    end
end

v_sim = v_final;

%% ========================================================================
%  COMPUTE LAP TIME
%  ========================================================================

dt = track.ds ./ max(v_sim, 0.1);
laptime = trapz(dt);

fprintf('\n================ v05_refined CONVERGED RESULTS =================\n');
fprintf('  v05_refined lap time:  %.3f s  (%.0f:%.2f)\n', laptime, floor(laptime/60), mod(laptime, 60));
fprintf('  Reference:             8:11.341 s\n');
fprintf('  Delta vs ref:          %.3f s (%.1f%%)\n', laptime - 491.341, (laptime - 491.341) / 491.341 * 100);

fprintf('\n  --- Version comparison (GPS-based curvature, ARBs included in v05_refined) ---\n');
fprintf('  v01 (point-mass):            8:44.586  (delta vs ref: +33.2 s)\n');
fprintf('  v02 (+ aero):                8:10.290  (delta vs ref: -1.1 s)\n');
fprintf('  v03 (+ load sens):           8:08.288  (delta vs ref: -3.1 s)\n');
fprintf('  v04 (+ long. transfer):      8:13.482  (delta vs ref: +2.1 s)\n');
fprintf('  v05 (+ lateral, no ARB):     8:45.658  (delta vs ref: +34.3 s)  [too conservative]\n');
fprintf('  v05_refined (+ lateral+ARB): %.0f:%.2f  (delta vs ref: %.1f s)\n', ...
        floor(laptime/60), mod(laptime, 60), laptime - 491.341);

fprintf('  Lateral transfer cost (with ARBs): %.1f s (v05_refined vs v04)\n', laptime - 493.482);
fprintf('  (positive = slower; ARBs reduce but don''t eliminate lateral effect)\n');

fprintf('  Min speed: %.1f km/h\n', min(v_sim)*3.6);
fprintf('  Max speed: %.1f km/h\n', max(v_sim)*3.6);
fprintf('  Mean speed: %.1f km/h\n', mean(v_sim)*3.6);
fprintf('=========================================================\n\n');

%% ========================================================================
%  VERIFICATION PLOTS
%  ========================================================================

figure('Name', 'v05_refined: Bicycle + ARBs', 'NumberTitle', 'off', 'Position', [100, 100, 1400, 900]);

subplot(3,2,1);
plot(track.dist/1000, v_sim*3.6, 'b', 'LineWidth', 1);
xlabel('Distance [km]'); ylabel('Speed [km/h]');
title('v05_refined Speed Profile');
grid on;

subplot(3,2,2);
plot(track.dist/1000, v_sim*3.6, 'b', 'LineWidth', 1.5); hold on;
plot(track.dist/1000, track.ref.v*3.6, 'r--', 'LineWidth', 1);
xlabel('Distance [km]'); ylabel('Speed [km/h]');
title('v05_refined vs Reference');
legend('v05_refined', 'Reference');
grid on;

subplot(3,2,3);
plot(track.dist/1000, track.kappa, 'k', 'LineWidth', 0.8);
ylabel('Curvature [1/m]'); xlabel('Distance [km]');
title('Track Curvature (GPS, 94% preservation)');
grid on;

subplot(3,2,4);
scatter(track.kappa*1000, v_sim*3.6, 3, 'b', 'filled'); hold on;
scatter(track.kappa*1000, track.ref.v*3.6, 3, 'r', 'filled');
xlabel('Curvature [1/m]'); ylabel('Speed [km/h]');
title('Speed vs Curvature');
legend('v05_refined', 'Reference');
grid on;

subplot(3,2,5);
distance_window = track.dist < 10000;
v_window = v_sim(distance_window);
dist_window = track.dist(distance_window);
kappa_window = track.kappa(distance_window);
dFz_lat_window = zeros(size(v_window));
for j = 1:length(v_window)
    a_lat_j = v_window(j)^2 * kappa_window(j);
    dFz_lat_full = car.mass * a_lat_j * h_cog / track_f;
    dFz_lat_eff = dFz_lat_full * load_xfer_reduction_f;
    dFz_lat_window(j) = dFz_lat_eff;
end
plot(dist_window/1000, dFz_lat_window/1000, 'g', 'LineWidth', 1);
xlabel('Distance [km]'); ylabel('Effective lat. load transfer [kN]');
title('Lateral Load Transfer (ARB-corrected, first 10 km)');
grid on;

subplot(3,2,6);
t_cumsum = cumsum([0; dt(1:end-1)]);
v_ref_safe = max(track.ref.v(1:end-1), 0.1);
t_ref = cumsum([0; diff(track.dist) ./ v_ref_safe]);
plot(t_cumsum, v_sim*3.6, 'b', 'LineWidth', 1.5); hold on;
plot(t_ref, track.ref.v*3.6, 'r--', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Speed [km/h]');
title('Speed vs Time');
legend('v05_refined', 'Reference');
grid on;

fprintf('=== v05_refined Simulation Complete ===\n\n');
