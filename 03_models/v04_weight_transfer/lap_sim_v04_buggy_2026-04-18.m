%% lap_sim_v04.m
%  Lap time simulator v04: Point-mass QSS with load-sensitive tyres +
%  longitudinal weight transfer.
%
%  v04 adds longitudinal load transfer (braking and acceleration) to v03.
%  During braking: weight shifts to front axle, increasing front grip,
%  decreasing rear grip (can cause rear-axle-limited braking). During
%  acceleration: weight shifts to rear axle, increasing rear grip,
%  decreasing front grip (rear-limited traction).
%
%  PHYSICS:
%    Longitudinal load transfer: dFz = m * a_long * h_cog / wheelbase
%    Front load: Fz_f = (m*g)/2 + aero_df/2 + dFz_long
%    Rear load:  Fz_r = (m*g)/2 + aero_df/2 - dFz_long
%    Grip at each axle: mu_eff = mu_0 - k*Fz (individual per axle)
%
%  INPUT:  'ref' struct (run import_reference_lap first)
%          'track' struct (run build_track first)
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
    error('Run build_track first.');
end
if ~exist('car', 'var')
    error('Run startup_project first.');
end

fprintf('\n=== Lap Sim v04 — Longitudinal Weight Transfer ===\n');
fprintf('Car:   %s\n', car.meta.name);
fprintf('Track: %s (%.3f km)\n', track.meta.name, track.length/1000);

g = 9.81;  % [m/s^2]

% Tyre parameters (from v03)
mu_0 = car.tyre.mu_0;           % [−] grip coefficient at zero load
k_load_sens = car.tyre.load_sens_k;  % [1/N] load sensitivity

% Aero coefficients (precomputed for use in loops)
rho = car.rho;                  % [kg/m^3] air density
A_frontal = car.frontal_area;   % [m^2] frontal area
Cd = car.aero_drag_coeff;       % [−] drag coefficient
% Drag force: F_drag = 0.5 * rho * v^2 * Cd * A
drag_coeff = 0.5 * rho * Cd * A_frontal;  % [kg/m] = [N·s^2/m^3]

% Downforce: F_df = 0.5 * rho * v^2 * Cl * A
% We use car.aero_df_coeff which is pre-computed as 0.5*rho*Cl*A
aero_df_coeff = car.aero_df_coeff;  % [N·s^2/m^2]

% Peak engine power
engine_rpm_array = car.engine.rpm;
engine_torque_array = car.engine.torque;
P_array = engine_torque_array .* (engine_rpm_array * 2 * pi / 60);
P_peak = max(P_array);  % [W]

%% ========================================================================
%  PASS 1: CORNERING SPEED LIMITS (load-transfer aware)
%  ========================================================================
%
%  At each point, we want v_corner such that the tyres at EITHER axle
%  reach their grip limit first.
%
%  Key difference from v03: now we compute grip separately for front and
%  rear, then take the minimum (whichever is tighter).
%
%  For pure cornering (no accel/brake), load transfer is zero, so front
%  and rear have equal load and equal grip. But as we add accel/brake,
%  one axle will become loaded while the other unloads, creating asymmetry.
%  We'll handle this in passes 2 and 3.
%
%  For now, cornering pass assumes steady-state cornering (no long accel).
%  This is a simplification but reasonable since corners are brief.

fprintf('Pass 1: Cornering speed limits (load-sensitive)...\n');

v_corner = zeros(track.n, 1);
points_at_limit = 0;

for i = 1:track.n
    kappa = track.kappa(i);

    if kappa < 1e-6
        % Straight: no cornering limit
        v_corner(i) = 400;  % arbitrary high speed
        continue;
    end

    % For a given speed v, the car needs centripetal acceleration a_lat = v^2 * kappa.
    % The lateral force comes from tyre friction: F_lat = mu * Fz.
    % In steady cornering (no long accel), Fz_f ≈ Fz_r ≈ (m*g + aero_df)/2 per axle.
    %
    % The cornering speed is limited where mu_eff drops due to load.
    % We solve: a_lat = mu_eff * g, where mu_eff = mu_0 - k*Fz.

    % Iterate to find v where the grip constraint is active
    % (similar to v03, but now we consider front/rear separation for
    %  cases with accel/brake. For pure cornering, they're equal.)

    % Start with estimate assuming equal front/rear loads
    Fz_static = (car.mass * g) / 2;  % per axle, static weight

    % Solve iteratively: v_corner where mu_eff at either axle hits limit
    v_try = 200;  % initial guess [m/s]

    for iter = 1:20
        v_try_old = v_try;

        % Aero downforce at this speed
        Fz_aero_per_axle = aero_df_coeff * v_try^2 / 2;

        % Normal load per axle (cornering, no long accel => no long load transfer)
        Fz_total_per_axle = Fz_static + Fz_aero_per_axle;

        % Load sensitivity: grip coefficient at this load
        mu_eff = mu_0 - k_load_sens * Fz_total_per_axle;
        mu_eff = max(mu_eff, 0.1);  % clamp to avoid negative grip

        % Centripetal acceleration limit: a_lat <= mu_eff * g
        % => v <= sqrt(mu_eff * g / kappa)
        v_corner_iter = sqrt(mu_eff * g / kappa);

        v_try = 0.5 * v_try_old + 0.5 * v_corner_iter;  % damp iteration

        if abs(v_try - v_try_old) < 0.1
            break;
        end
    end

    v_corner(i) = v_try;

    if kappa > 0.01  % meaningful corner
        points_at_limit = points_at_limit + 1;
    end
end

fprintf('  Min cornering speed: %.1f km/h\n', min(v_corner)*3.6);
fprintf('  Points at cap: %d / %d\n', points_at_limit, track.n);

% Report grip levels at different speeds for diagnostics
fprintf('  Load sensitivity effect:\n');
for speed_test = [80, 150, 200, 260]
    Fz_static = car.mass * g / 2;
    v_ms = speed_test / 3.6;
    Fz_aero = car.aero_df_coeff * v_ms^2 / 2;
    Fz_total = Fz_static + Fz_aero;
    mu = mu_0 - k_load_sens * Fz_total;
    mu = max(mu, 0.1);
    fprintf('    @ %3d km/h: mu = %.3f (Fz_per_axle = %.0f N)\n', ...
            speed_test, mu, Fz_total);
end

%% ========================================================================
%  PASS 2: FORWARD PASS (acceleration limited by traction + load transfer)
%  ========================================================================
%
%  Walk forward through the lap. At each point, the car accelerates from
%  the previous speed as much as possible, limited by engine power, drag,
%  and traction at the rear axle (where power is applied).
%
%  During acceleration, weight transfers to the rear. This INCREASES rear
%  grip but DECREASES front grip. We need to check rear axle limit:
%
%    a_traction_max = mu_rear * g
%
%  where mu_rear depends on rear axle load including acceleration transfer.

fprintf('Pass 2: Forward pass...\n');

v_fwd = zeros(track.n, 1);
v_fwd(1) = 233 / 3.6;  % Starting speed: 233 km/h converted to m/s (from v03 iteration convergence)

for i = 2:track.n
    v_prev = v_fwd(i-1);
    v_cand = v_corner(i);  % Don't exceed cornering limit

    % Try to accelerate from v_prev to v_cand
    % Check engine power and traction limit

    % Drag force at v_prev
    F_drag = drag_coeff * v_prev^2;

    % Traction limit at rear axle during acceleration
    % During forward accel: dFz_long = +m*a*h_cog/wheelbase (rear loads up)
    %
    % We need to find a_traction such that rear axle reaches grip limit.
    % But a_traction comes from both power and load. This couples forward
    % and we need to solve iteratively.

    % Simplification for now: solve for maximum rear-axle-limited traction
    % assuming the rear axle is fully engaged.

    % Rear normal load during acceleration with a = a_long:
    %   Fz_r = (m*g)/2 + aero_df_r/2 - m*a*h_cog/wheelbase
    %
    % Grip at rear:
    %   mu_r = mu_0 - k*Fz_r
    %   a_max = mu_r * g
    %
    % This is implicit in a (appears on LHS and RHS). Solve iteratively.

    a_long = 0;  % initial guess [m/s^2]
    for iter_accel = 1:10
        a_old = a_long;

        % Aero downforce
        Fz_aero_r = aero_df_coeff * v_prev^2 / 2;

        % Normal load at rear during acceleration
        % During accel (a_long > 0): weight transfers TO rear, so Fz_rear increases
        dFz_long = car.mass * a_long * car.h_cog / car.wheelbase;  % positive during accel
        Fz_r = (car.mass * g) / 2 + Fz_aero_r + dFz_long;  % PLUS (was minus)
        Fz_r = max(Fz_r, 100);  % keep positive

        % Grip at rear
        mu_r = mu_0 - k_load_sens * Fz_r;
        mu_r = max(mu_r, 0.1);

        % Maximum traction acceleration
        a_traction_max = mu_r * g;

        % Power-limited acceleration at current speed
        if v_prev > 0.1
            a_power_max = (P_peak / v_prev - F_drag) / car.mass;
            a_power_max = max(a_power_max, 0);
        else
            a_power_max = 0;
        end

        % Actual acceleration is minimum of traction and power
        a_long = min(a_traction_max, a_power_max);

        if abs(a_long - a_old) < 0.01
            break;
        end
    end

    % Integrate over distance ds = track.ds
    ds = track.ds;
    % v^2 = v_prev^2 + 2*a*ds
    v_next_squared = v_prev^2 + 2 * a_long * ds;
    v_next = sqrt(max(v_next_squared, 0));

    % Cap at cornering limit
    v_fwd(i) = min(v_next, v_cand);
end

fprintf('  Max speed: %.1f km/h\n', max(v_fwd)*3.6);

%% ========================================================================
%  PASS 3: BACKWARD PASS (braking limited by front axle grip)
%  ========================================================================
%
%  Walk backward. At each point, the car must be going slow enough to
%  stop or slow down before the next point using available braking.
%
%  During braking, weight transfers to the FRONT axle. This increases
%  front grip but decreases rear grip. The front becomes the limit.
%
%  Braking acceleration:
%    a_brake_max = mu_front_eff * g
%
%  where mu_front_eff includes load sensitivity.

fprintf('Pass 3: Backward pass...\n');

v_bwd = zeros(track.n, 1);
v_bwd(track.n) = v_fwd(track.n);  % Start from end of forward pass

for i = track.n-1:-1:1
    v_next = v_bwd(i+1);
    v_cand = v_corner(i);

    % Braking acceleration (check both front AND rear; minimum is the limit)
    % During braking: dFz_long = +m*a*h_cog/wheelbase (front loads up, rear unloads)
    % (note: a_brake is positive magnitude)

    a_brake = 0;  % initial guess [m/s^2]
    for iter_brake = 1:10
        a_old = a_brake;

        % Aero downforce
        Fz_aero_f = aero_df_coeff * v_next^2 / 2;
        Fz_aero_r = aero_df_coeff * v_next^2 / 2;

        % Normal load at front and rear during braking
        % (braking load transfer is positive at front, negative at rear)
        dFz_long_brake = car.mass * a_brake * car.h_cog / car.wheelbase;
        Fz_f = (car.mass * g) / 2 + Fz_aero_f + dFz_long_brake;
        Fz_r = (car.mass * g) / 2 + Fz_aero_r - dFz_long_brake;
        Fz_f = max(Fz_f, 100);
        Fz_r = max(Fz_r, 100);

        % Grip at both axles
        mu_f = mu_0 - k_load_sens * Fz_f;
        mu_r = mu_0 - k_load_sens * Fz_r;
        mu_f = max(mu_f, 0.1);
        mu_r = max(mu_r, 0.1);

        % Maximum braking acceleration is limited by whichever axle reaches its limit first
        a_brake_f = mu_f * g;
        a_brake_r = mu_r * g;
        a_brake = min(a_brake_f, a_brake_r);  % Take minimum (the tighter limit)

        if abs(a_brake - a_old) < 0.01
            break;
        end
    end

    % What speed could we sustain here and still brake in time for next corner?
    % v_here^2 - v_next^2 = 2*a_brake*ds
    % => v_here = sqrt(v_next^2 + 2*a_brake*ds)
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
%
%  After backward pass, we have a speed profile that respects braking and
%  cornering everywhere. But the start/end speeds may not match — the car
%  might have speed on entry that it can't use on exit. Iterate to find
%  the self-consistent lap speed profile.

fprintf('Lap continuity check:\n');

v_final = min(v_fwd, v_bwd);  % Pointwise minimum
v_start_after_bwd = v_final(1);

fprintf('  Start: %.2f km/h (from iteration 0)\n', v_start_after_bwd*3.6);

% Iterate lap continuity
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
            dFz_long = car.mass * a_long * car.h_cog / car.wheelbase;
            Fz_r = (car.mass * g) / 2 + Fz_aero_r + dFz_long;
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
            Fz_aero_r = aero_df_coeff * v_next^2 / 2;
            dFz_long_brake = car.mass * a_brake * car.h_cog / car.wheelbase;
            Fz_f = (car.mass * g) / 2 + Fz_aero_f + dFz_long_brake;
            Fz_r = (car.mass * g) / 2 + Fz_aero_r - dFz_long_brake;
            Fz_f = max(Fz_f, 100);
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

v_sim = v_final;  % Final speed profile

%% ========================================================================
%  COMPUTE LAP TIME
%  ========================================================================

% Lap time = integral of dt = integral of ds/v over lap
% Using trapezoidal rule on (ds / v)
dt = track.ds ./ max(v_sim, 0.1);  % clamp to avoid division by zero
laptime = trapz(dt);

fprintf('\n================ v04 CONVERGED RESULTS =================\n');
fprintf('  v04 lap time:  %.3f s  (%.0f:%.2f)\n', laptime, floor(laptime/60), mod(laptime, 60));
fprintf('  Reference:     8:11.341 s\n');
fprintf('  Delta vs ref:  %.3f s (%.1f%%)\n', laptime - 491.341, (laptime - 491.341) / 491.341 * 100);

fprintf('\n  --- Version comparison ---\n');
fprintf('  v01 (point-mass):        8:13.730  (delta vs ref: +2.4 s)\n');
fprintf('  v02 (+ aero):            7:35.919  (delta vs ref: -35.4 s)\n');
fprintf('  v03 (+ load sens):       7:46.382  (delta vs ref: -25.0 s)\n');
fprintf('  v04 (+ weight transfer): %d:%05.2f  (delta vs ref: %.1f s)\n', ...
        floor(laptime/60), mod(laptime, 60), laptime - 491.341);

fprintf('  Weight transfer cost: %.1f s (v04 vs v03)\n', laptime - 466.382);
fprintf('  (positive = v04 slower = weight transfer reduces grip; negative = improves)\n');

fprintf('  Min speed: %.1f km/h\n', min(v_sim)*3.6);
fprintf('  Max speed: %.1f km/h\n', max(v_sim)*3.6);
fprintf('  Mean speed: %.1f km/h\n', mean(v_sim)*3.6);
fprintf('=========================================================\n\n');

%% ========================================================================
%  VERIFICATION PLOTS
%  ========================================================================

figure('Name', 'v04 Results: Speed vs Curvature + Weight Transfer', ...
       'NumberTitle', 'off', 'Position', [100, 100, 1400, 900]);

% --- Speed trace ---
subplot(3,2,1);
plot(track.dist/1000, v_sim*3.6, 'b', 'LineWidth', 1);
xlabel('Distance [km]');
ylabel('Speed [km/h]');
title('v04 Speed Profile');
grid on;

% --- Speed vs Reference ---
subplot(3,2,2);
plot(track.dist/1000, v_sim*3.6, 'b', 'LineWidth', 1.5); hold on;
plot(track.dist/1000, track.ref.v*3.6, 'r--', 'LineWidth', 1);
xlabel('Distance [km]');
ylabel('Speed [km/h]');
title('v04 Speed vs Reference');
legend('v04', 'Reference');
grid on;

% --- Curvature (log scale for visibility) ---
subplot(3,2,3);
plot(track.dist/1000, track.kappa, 'k', 'LineWidth', 0.8);
ylabel('Curvature [1/m]');
xlabel('Distance [km]');
title('Track Curvature');
grid on;

% --- Speed vs Curvature (scatter) ---
subplot(3,2,4);
scatter(track.kappa*1000, v_sim*3.6, 3, 'b', 'filled');
hold on;
scatter(track.kappa*1000, track.ref.v*3.6, 3, 'r', 'filled');
xlabel('Curvature [1/m]');
ylabel('Speed [km/h]');
title('Speed vs Curvature');
legend('v04', 'Reference');
grid on;

% --- Load at front/rear axles (example: first 5 km) ---
subplot(3,2,5);
distance_window = track.dist < 5000;  % first 5 km
v_window = v_sim(distance_window);
dist_window = track.dist(distance_window);

Fz_front = zeros(size(v_window));
Fz_rear = zeros(size(v_window));
for j = 1:length(v_window)
    Fz_aero = aero_df_coeff * v_window(j)^2 / 2;
    Fz_front(j) = (car.mass * 9.81) / 2 + Fz_aero / 2;
    Fz_rear(j) = (car.mass * 9.81) / 2 + Fz_aero / 2;
end

plot(dist_window/1000, Fz_front/1000, 'b', 'LineWidth', 1); hold on;
plot(dist_window/1000, Fz_rear/1000, 'r', 'LineWidth', 1);
xlabel('Distance [km]');
ylabel('Normal load (static + aero) [kN]');
title('Axle Loads (first 5 km) — no braking/accel effect shown here');
legend('Front', 'Rear');
grid on;

% --- Time-series comparison ---
subplot(3,2,6);
t_cumsum = cumsum([0; dt(1:end-1)]);
plot(t_cumsum, v_sim*3.6, 'b', 'LineWidth', 1.5); hold on;

% reference time from import
v_ref_safe = max(track.ref.v(1:end-1), 0.1);
t_ref = cumsum([0; diff(track.dist) ./ v_ref_safe]);
plot(t_ref, track.ref.v*3.6, 'r--', 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Speed [km/h]');
title('Speed vs Time');
legend('v04', 'Reference');
grid on;

fprintf('=== v04 Simulation Complete ===\n\n');
