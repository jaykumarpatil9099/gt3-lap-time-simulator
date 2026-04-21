%% lap_sim_v05.m
%  Lap time simulator v05: Bicycle model with lateral load transfer.
%
%  v05 adds lateral load transfer (lateral g effects on front/rear grip).
%  During cornering, inside tyres unload, outside tyres load up.
%  The OUTSIDE tyre reaches its grip limit first.
%
%  Combined with v04's longitudinal transfer, we now have:
%    - Longitudinal transfer (accel/braking): affects front/rear symmetrically
%    - Lateral transfer (cornering): affects inside/outside, per corner
%
%  This is the last step before suspension dynamics.
%
%  PHYSICS:
%    Lateral load transfer: dFz_lat = m * a_lat * h_cog / track_width
%    Outside tyre load: Fz_out = Fz_base + dFz_lat
%    Inside tyre load:  Fz_in = Fz_base - dFz_lat
%    Grip at outside (limiting): mu_eff = mu_0 - k*Fz_out
%
%  INPUT:  'ref' struct (run import_reference_lap first)
%          'track' struct (run build_track_from_gps first)
%          'car' struct (loaded by startup_project)
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

fprintf('\n=== Lap Sim v05 — Bicycle Model with Lateral Load Transfer ===\n');
fprintf('Car:   %s\n', car.meta.name);
fprintf('Track: %s (%.3f km)\n', track.meta.name, track.length/1000);

g = 9.81;  % [m/s^2]

% Tyre parameters
mu_0 = car.tyre.mu_0;
k_load_sens = car.tyre.load_sens_k;

% Aero coefficients (precomputed)
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
L = car.wheelbase;           % [m]
h_cog = car.h_cog;          % [m]
track_f = car.track_f;      % [m] front track width
track_r = car.track_r;      % [m] rear track width

%% ========================================================================
%  PASS 1: CORNERING SPEED LIMITS (bicycle model, lateral transfer)
%  ========================================================================
%
%  At each point with curvature κ, find v_corner such that at least one
%  of the four tyres (front-left, front-right, rear-left, rear-right)
%  reaches its grip limit.
%
%  For a left turn (a_lat > 0), the right-side tyres load up, left-side
%  unload. The OUTSIDE (right-side) tyres reach their limit first.
%
%  Load transfer per axle:
%    Front: dFz_lat_f = m * a_lat * h_cog / track_f  (to outside)
%    Rear:  dFz_lat_r = m * a_lat * h_cog / track_r  (to outside)
%
%  For simplicity, assume both front and rear have equal load
%  distribution in steady cornering (no aero asymmetry). The OUTSIDE
%  tyre load is what matters for the grip limit.

fprintf('Pass 1: Cornering speed limits (lateral load transfer)...\n');

v_corner = zeros(track.n, 1);
points_at_limit = 0;

for i = 1:track.n
    kappa = track.kappa(i);

    if kappa < 1e-6
        v_corner(i) = 400;  % straight
        continue;
    end

    % Iterate to find v where outside tyre reaches grip limit
    v_try = 150;  % initial guess [m/s]

    for iter = 1:20
        v_try_old = v_try;

        % Centripetal acceleration at this speed
        a_lat = v_try^2 * kappa;

        % Aero downforce (distributed to front/rear based on aero balance)
        Fz_aero_total = aero_df_coeff * v_try^2;
        Fz_aero_f = Fz_aero_total * car.aero_balance_f;
        Fz_aero_r = Fz_aero_total * (1 - car.aero_balance_f);

        % Static load per tyre (one tyre, so divide by 2 per axle, then by 2 for left/right)
        Fz_static_per_tyre_f = (car.mass * g) * car.weight_dist_f / 2;
        Fz_static_per_tyre_r = (car.mass * g) * car.weight_dist_r / 2;

        % Aero load per tyre (assume equally distributed left/right)
        Fz_aero_per_tyre_f = Fz_aero_f / 2;
        Fz_aero_per_tyre_r = Fz_aero_r / 2;

        % Lateral load transfer per axle
        dFz_lat_f = car.mass * a_lat * h_cog / track_f;
        dFz_lat_r = car.mass * a_lat * h_cog / track_r;

        % Outside tyres (loaded)
        Fz_out_f = Fz_static_per_tyre_f + Fz_aero_per_tyre_f + dFz_lat_f / 2;
        Fz_out_r = Fz_static_per_tyre_r + Fz_aero_per_tyre_r + dFz_lat_r / 2;

        % Inside tyres (unloaded) — clamp to zero if they lift off
        Fz_in_f = max(Fz_static_per_tyre_f + Fz_aero_per_tyre_f - dFz_lat_f / 2, 1);
        Fz_in_r = max(Fz_static_per_tyre_r + Fz_aero_per_tyre_r - dFz_lat_r / 2, 1);

        % Grip coefficient at outside tyres (these are the limiting ones)
        mu_out_f = mu_0 - k_load_sens * Fz_out_f;
        mu_out_r = mu_0 - k_load_sens * Fz_out_r;
        mu_out_f = max(mu_out_f, 0.1);
        mu_out_r = max(mu_out_r, 0.1);

        % Cornering acceleration limit: a_lat <= mu * g
        % => v <= sqrt(mu * g / kappa)
        % Both axles contribute; take minimum (tighter constraint)
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

% Grip diagnostic
fprintf('  Grip with lateral transfer (example speeds):\n');
for speed_test = [80, 150, 200, 260]
    v_ms = speed_test / 3.6;
    a_lat_test = v_ms^2 * 0.02;  % assume κ ≈ 0.02 for a typical corner

    Fz_aero_test = aero_df_coeff * v_ms^2;
    Fz_static_f = car.mass * g * car.weight_dist_f / 2;
    Fz_aero_f = Fz_aero_test * car.aero_balance_f / 2;
    dFz_lat_f = car.mass * a_lat_test * h_cog / track_f / 2;
    Fz_out_f = Fz_static_f + Fz_aero_f + dFz_lat_f;
    mu_out_f = mu_0 - k_load_sens * Fz_out_f;
    mu_out_f = max(mu_out_f, 0.1);

    a_lat_limit = mu_out_f * g / (v_ms^2 / (mu_out_f * g / 0.02));
    fprintf('    @ %3d km/h: mu_out_f = %.3f, Fz_out_f = %.0f N\n', ...
            speed_test, mu_out_f, Fz_out_f);
end

%% ========================================================================
%  PASS 2: FORWARD PASS (with longitudinal transfer from v04)
%  ========================================================================
%  Acceleration limited by rear traction and power.
%  Rear load transfer from v04 applied.

fprintf('Pass 2: Forward pass...\n');

v_fwd = zeros(track.n, 1);
v_fwd(1) = 232.8 / 3.6;  % 232.8 km/h start speed from v04 convergence, converted to m/s

for i = 2:track.n
    v_prev = v_fwd(i-1);
    v_cand = v_corner(i);

    F_drag = drag_coeff * v_prev^2;

    a_long = 0;
    for iter_accel = 1:10
        a_old = a_long;

        % Rear normal load during acceleration (longitudinal transfer from v04)
        Fz_aero_r = aero_df_coeff * v_prev^2 / 2;
        dFz_long = car.mass * a_long * h_cog / L;
        Fz_r_static = car.mass * g * car.weight_dist_r / 2;
        Fz_r = Fz_r_static + Fz_aero_r + dFz_long;  % longitudinal transfer increases rear load
        Fz_r = max(Fz_r, 100);

        % Grip at rear (load sensitive)
        mu_r = mu_0 - k_load_sens * Fz_r;
        mu_r = max(mu_r, 0.1);

        % Traction limit
        a_traction_max = mu_r * g;

        % Power limit
        if v_prev > 0.1
            a_power_max = (P_peak / v_prev - F_drag) / car.mass;
            a_power_max = max(a_power_max, 0);
        else
            a_power_max = 0;
        end

        % Actual acceleration
        a_long = min(a_traction_max, a_power_max);

        if abs(a_long - a_old) < 0.01, break; end
    end

    % Integrate
    ds = track.ds;
    v_next_squared = v_prev^2 + 2 * a_long * ds;
    v_next = sqrt(max(v_next_squared, 0));

    % Cap at cornering limit
    v_fwd(i) = min(v_next, v_cand);
end

fprintf('  Max speed: %.1f km/h\n', max(v_fwd)*3.6);

%% ========================================================================
%  PASS 3: BACKWARD PASS (with longitudinal transfer from v04)
%  ========================================================================
%  Braking limited by front traction.
%  Front load transfer from v04 applied.

fprintf('Pass 3: Backward pass...\n');

v_bwd = zeros(track.n, 1);
v_bwd(track.n) = v_fwd(track.n);

for i = track.n-1:-1:1
    v_next = v_bwd(i+1);
    v_cand = v_corner(i);

    a_brake = 0;
    for iter_brake = 1:10
        a_old = a_brake;

        % Front normal load during braking (longitudinal transfer from v04)
        Fz_aero_f = aero_df_coeff * v_next^2 / 2;
        dFz_long_brake = car.mass * a_brake * h_cog / L;
        Fz_f_static = car.mass * g * car.weight_dist_f / 2;
        Fz_f = Fz_f_static + Fz_aero_f + dFz_long_brake;  % long transfer increases front load
        Fz_f = max(Fz_f, 100);

        % Also compute rear (might be limiting in some cases)
        Fz_aero_r = aero_df_coeff * v_next^2 / 2;
        Fz_r_static = car.mass * g * car.weight_dist_r / 2;
        Fz_r = Fz_r_static + Fz_aero_r - dFz_long_brake;  % long transfer decreases rear load
        Fz_r = max(Fz_r, 100);

        % Grip at both axles
        mu_f = mu_0 - k_load_sens * Fz_f;
        mu_r = mu_0 - k_load_sens * Fz_r;
        mu_f = max(mu_f, 0.1);
        mu_r = max(mu_r, 0.1);

        % Braking is limited by whichever axle reaches limit first
        a_brake_f = mu_f * g;
        a_brake_r = mu_r * g;
        a_brake = min(a_brake_f, a_brake_r);

        if abs(a_brake - a_old) < 0.01, break; end
    end

    % Integrate
    ds = track.ds;
    v_max_here_squared = v_next^2 + 2 * a_brake * ds;
    v_max_here = sqrt(v_max_here_squared);

    % Cap at cornering limit
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
    % Re-run forward pass with updated start speed
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

    % Re-run backward pass
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

fprintf('\n================ v05 CONVERGED RESULTS =================\n');
fprintf('  v05 lap time:  %.3f s  (%.0f:%.2f)\n', laptime, floor(laptime/60), mod(laptime, 60));
fprintf('  Reference:     8:11.341 s\n');
fprintf('  Delta vs ref:  %.3f s (%.1f%%)\n', laptime - 491.341, (laptime - 491.341) / 491.341 * 100);

fprintf('\n  --- Version comparison (GPS-based curvature) ---\n');
fprintf('  v01 (point-mass):            8:44.586  (delta vs ref: +33.2 s)\n');
fprintf('  v02 (+ aero):                8:10.290  (delta vs ref: -1.1 s)\n');
fprintf('  v03 (+ load sens):           8:08.288  (delta vs ref: -3.1 s)\n');
fprintf('  v04 (+ long. transfer):      8:13.482  (delta vs ref: +2.1 s)\n');
fprintf('  v05 (+ lateral transfer):    %.0f:%.2f  (delta vs ref: %.1f s)\n', ...
        floor(laptime/60), mod(laptime, 60), laptime - 491.341);

fprintf('  Lateral transfer cost: %.1f s (v05 vs v04)\n', laptime - 493.482);
fprintf('  (positive = v05 slower = lateral transfer reduces grip)\n');

fprintf('  Min speed: %.1f km/h\n', min(v_sim)*3.6);
fprintf('  Max speed: %.1f km/h\n', max(v_sim)*3.6);
fprintf('  Mean speed: %.1f km/h\n', mean(v_sim)*3.6);
fprintf('=========================================================\n\n');

%% ========================================================================
%  VERIFICATION PLOTS
%  ========================================================================

figure('Name', 'v05 Results: Bicycle Model with Lateral Load Transfer', ...
       'NumberTitle', 'off', 'Position', [100, 100, 1400, 900]);

% Speed trace
subplot(3,2,1);
plot(track.dist/1000, v_sim*3.6, 'b', 'LineWidth', 1);
xlabel('Distance [km]');
ylabel('Speed [km/h]');
title('v05 Speed Profile');
grid on;

% Speed vs Reference
subplot(3,2,2);
plot(track.dist/1000, v_sim*3.6, 'b', 'LineWidth', 1.5); hold on;
plot(track.dist/1000, track.ref.v*3.6, 'r--', 'LineWidth', 1);
xlabel('Distance [km]');
ylabel('Speed [km/h]');
title('v05 Speed vs Reference');
legend('v05', 'Reference');
grid on;

% Curvature
subplot(3,2,3);
plot(track.dist/1000, track.kappa, 'k', 'LineWidth', 0.8);
ylabel('Curvature [1/m]');
xlabel('Distance [km]');
title('Track Curvature (GPS-based, 94% preservation)');
grid on;

% Speed vs Curvature
subplot(3,2,4);
scatter(track.kappa*1000, v_sim*3.6, 3, 'b', 'filled');
hold on;
scatter(track.kappa*1000, track.ref.v*3.6, 3, 'r', 'filled');
xlabel('Curvature [1/m]');
ylabel('Speed [km/h]');
title('Speed vs Curvature');
legend('v05', 'Reference');
grid on;

% Lateral load transfer effect (first 10 km)
subplot(3,2,5);
distance_window = track.dist < 10000;
v_window = v_sim(distance_window);
dist_window = track.dist(distance_window);
kappa_window = track.kappa(distance_window);

dFz_lat_window = zeros(size(v_window));
for j = 1:length(v_window)
    a_lat_j = v_window(j)^2 * kappa_window(j);
    dFz_lat_window(j) = car.mass * a_lat_j * h_cog / track_f;
end

plot(dist_window/1000, dFz_lat_window/1000, 'g', 'LineWidth', 1);
xlabel('Distance [km]');
ylabel('Lateral load transfer [kN]');
title('Lateral Load Transfer (first 10 km)');
grid on;

% Time series
subplot(3,2,6);
t_cumsum = cumsum([0; dt(1:end-1)]);
v_ref_safe = max(track.ref.v(1:end-1), 0.1);
t_ref = cumsum([0; diff(track.dist) ./ v_ref_safe]);
plot(t_cumsum, v_sim*3.6, 'b', 'LineWidth', 1.5); hold on;
plot(t_ref, track.ref.v*3.6, 'r--', 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Speed [km/h]');
title('Speed vs Time');
legend('v05', 'Reference');
grid on;

fprintf('=== v05 Simulation Complete ===\n\n');
