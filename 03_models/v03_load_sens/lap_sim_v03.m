%% lap_sim_v03.m
%  Point-mass QSS lap sim with TYRE LOAD SENSITIVITY (v03)
%
%  WHAT CHANGED FROM v02:
%    v02: mu = constant (1.60)
%    v03: mu(Fz) = mu_0 - k * Fz_per_tyre
%         As tyre load increases (from downforce), mu DECREASES.
%         This corrects v02's overestimate of high-speed grip.
%
%  The solver structure (3-pass + iteration) is unchanged.
%  Changes are ONLY in how grip is computed at each speed.
%
%  INPUTS (must be in workspace):
%    car   — vehicle parameters
%    track — track data
%
%  OUTPUT:
%    sim03 — struct with v03 results
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-16

%% ========================================================================
%  0. CHECK INPUTS & STORE PREVIOUS RESULTS
%  ========================================================================

if ~exist('car', 'var')
    error('Car parameters not loaded. Run startup_project first.');
end
if ~exist('track', 'var')
    error('Track data not loaded. Run build_track first.');
end

fprintf('\n=== Lap Sim v03 — Load-Sensitive Tyres ===\n');
fprintf('Car:   %s\n', car.meta.name);
fprintf('Track: %s (%.3f km)\n', track.meta.name, track.length/1000);

% Store previous results for comparison
if exist('sim', 'var')
    sim_v01 = sim;
    has_v01 = true;
else
    has_v01 = false;
end
if exist('sim02', 'var')
    has_v02 = true;
else
    has_v02 = false;
end

%% ========================================================================
%  1. HELPER: ENGINE FORCE (unchanged from v01/v02)
%  ========================================================================

    function [F_drive, gear_selected, rpm_selected] = get_drive_force(v, car)
        F_best = 0;
        gear_selected = 1;
        rpm_selected = 0;
        for g = 1:car.gearbox.n_gears
            rpm_g = v * car.gearbox.total_ratio(g) / car.tyre.rolling_radius ...
                    * (60 / (2*pi));
            if rpm_g < car.engine.rpm_idle || rpm_g > car.engine.rpm_max
                continue;
            end
            T_engine = interp1(car.engine.rpm, car.engine.torque, rpm_g, ...
                               'linear', 'extrap');
            F_wheel = T_engine * car.gearbox.total_ratio(g) * car.gearbox.efficiency ...
                      / car.tyre.rolling_radius;
            if F_wheel > F_best
                F_best = F_wheel;
                gear_selected = g;
                rpm_selected = rpm_g;
            end
        end
        F_drive = max(F_best, 0);
    end

%% ========================================================================
%  2. HELPER: LOAD-SENSITIVE GRIP (NEW IN v03)
%  ========================================================================
%  Computes the maximum total tyre acceleration at a given speed,
%  accounting for both downforce AND load sensitivity.
%
%  Physics:
%    Fz_total  = m*g + aero_df_coeff * v^2         (total normal force)
%    Fz_per_tyre = Fz_total / 4                     (average per tyre)
%    mu_eff    = mu_0 - k * Fz_per_tyre             (load-sensitive mu)
%    F_grip    = mu_eff * Fz_total                   (total grip force)
%    a_max     = F_grip / m                          (max acceleration)
%
%  Why divide by 4? We're treating the car as a point mass, so we
%  approximate "each tyre carries 1/4 of the total load". This isn't
%  exact (weight distribution is 46/54, and in corners the outside
%  tyres carry more), but it's the right approximation for a point-mass
%  model. v04/v05 will improve this with per-axle loads.

    function [a_max, mu_eff] = get_grip_accel_v03(v, car)
        F_downforce = car.aero_df_coeff * v^2;
        Fz_total = car.mass * car.g + F_downforce;
        Fz_per_tyre = Fz_total / 4;

        % Load-sensitive friction coefficient
        mu_eff = car.tyre.mu_0 - car.tyre.load_sens_k * Fz_per_tyre;

        % Safety: mu should never go below a minimum (physical limit)
        mu_eff = max(mu_eff, 0.5);

        % Total grip and acceleration
        F_grip = mu_eff * Fz_total;
        a_max = F_grip / car.mass;
    end

%% ========================================================================
%  3. PASS 1 — CORNERING SPEED LIMIT (numerical solve)
%  ========================================================================
%  In v03, the cornering equation is implicit:
%    m * v^2 * kappa = mu_eff(v) * Fz_total(v)
%
%  mu_eff depends on v (through downforce → load → load sensitivity).
%  We can't solve this algebraically. Instead, we define a residual:
%    f(v) = a_grip(v) - v^2 * kappa
%  and find v where f(v) = 0 using a simple iterative method.
%
%  Method: start with v02's estimate (constant mu), then iteratively
%  update mu based on the current speed estimate. Converges in 3-5 iters.

fprintf('\nPass 1: Cornering speed limits (load-sensitive)...\n');

g_acc = car.g;
n = track.n;
ds = track.ds;
kappa = track.kappa;
v_max_cap = 400 / 3.6;

v_corner = zeros(n, 1);

for i = 1:n
    if kappa(i) < 1e-6
        v_corner(i) = v_max_cap;
        continue;
    end

    % Iterative solve: start with a reasonable guess
    % Use v02-style formula with mu_peak as initial mu
    mu_guess = car.tyre.mu_peak;
    denom = car.mass * kappa(i) - mu_guess * car.aero_df_coeff;
    if denom <= 0
        v_guess = v_max_cap;
    else
        v_guess = sqrt(mu_guess * car.mass * g_acc / denom);
        v_guess = min(v_guess, v_max_cap);
    end

    % Iterate: compute mu at current speed, re-solve, repeat
    v_iter = v_guess;
    for j = 1:15
        [a_grip, mu_at_v] = get_grip_accel_v03(v_iter, car);

        % New cornering speed with this mu
        denom = car.mass * kappa(i) - mu_at_v * car.aero_df_coeff;
        if denom <= 0
            v_new = v_max_cap;
        else
            v_new = sqrt(mu_at_v * car.mass * g_acc / denom);
            v_new = min(v_new, v_max_cap);
        end

        % Check convergence
        if abs(v_new - v_iter) < 0.01   % 0.01 m/s tolerance
            v_iter = v_new;
            break;
        end
        v_iter = v_new;
    end

    v_corner(i) = v_iter;
end

fprintf('  Min cornering speed: %.1f km/h\n', min(v_corner)*3.6);
fprintf('  Points at cap: %d / %d\n', sum(v_corner >= v_max_cap - 0.1), n);

% Show mu range for context
[~, mu_low_speed] = get_grip_accel_v03(80/3.6, car);
[~, mu_high_speed] = get_grip_accel_v03(260/3.6, car);
fprintf('  mu at  80 km/h: %.3f\n', mu_low_speed);
fprintf('  mu at 260 km/h: %.3f\n', mu_high_speed);
fprintf('  mu drop: %.1f%% (this is load sensitivity at work)\n', ...
        (1 - mu_high_speed/mu_low_speed) * 100);

%% ========================================================================
%  4. PASS 2 — FORWARD PASS (with load-sensitive grip)
%  ========================================================================

fprintf('Pass 2: Forward pass...\n');

v_forward = zeros(n, 1);
gear_forward = zeros(n, 1);
v_forward(1) = v_corner(1);

for i = 1:n-1
    v_now = v_forward(i);

    % Engine and drag
    [F_drive, g_sel, ~] = get_drive_force(v_now, car);
    gear_forward(i) = g_sel;
    F_drag = car.aero_drag_coeff * v_now^2;
    a_engine = (F_drive - F_drag) / car.mass;

    % Load-sensitive grip (replaces constant mu)
    [a_grip_total, ~] = get_grip_accel_v03(v_now, car);

    % Friction circle: subtract lateral usage
    a_lat_used = v_now^2 * kappa(i);
    if a_lat_used >= a_grip_total
        a_grip_long = 0;
    else
        a_grip_long = sqrt(a_grip_total^2 - a_lat_used^2);
    end

    % Combine
    if a_engine > 0
        a_forward = min(a_engine, a_grip_long);
    else
        a_forward = a_engine;
    end

    v_next_sq = v_now^2 + 2 * a_forward * ds;
    v_next = sqrt(max(v_next_sq, 0));
    v_forward(i+1) = min(v_next, v_corner(i+1));
end
gear_forward(n) = gear_forward(n-1);

fprintf('  Max speed: %.1f km/h\n', max(v_forward)*3.6);

%% ========================================================================
%  5. PASS 3 — BACKWARD PASS (with load-sensitive grip)
%  ========================================================================

fprintf('Pass 3: Backward pass...\n');

v_backward = zeros(n, 1);
v_backward(n) = v_forward(n);

for i = n:-1:2
    v_now = v_backward(i);

    [a_grip_total, ~] = get_grip_accel_v03(v_now, car);
    a_lat_used = v_now^2 * kappa(i);

    if a_lat_used >= a_grip_total
        a_brake = 0;
    else
        a_brake = sqrt(a_grip_total^2 - a_lat_used^2);
    end

    v_prev = sqrt(v_now^2 + 2 * a_brake * ds);
    v_backward(i-1) = min(v_prev, v_forward(i-1));
end

%% ========================================================================
%  6. COMBINE + LAP TIME
%  ========================================================================

v_sim = min(v_forward, v_backward);
v_sim = min(v_sim, v_corner);

dt_sim = ds ./ v_sim;
lap_time = sum(dt_sim);
t_cum = cumsum(dt_sim);
lap_min = floor(lap_time / 60);
lap_sec = lap_time - lap_min * 60;

%% ========================================================================
%  7. LAP CONTINUITY ITERATION
%  ========================================================================

fprintf('\nLap continuity check:\n');
fprintf('  Start: %.2f km/h  End: %.2f km/h\n', v_sim(1)*3.6, v_sim(end)*3.6);

if abs(v_sim(end) - v_sim(1)) > 1.0
    fprintf('  >> Iterating...\n');

    for iter = 1:5
        v_forward(1) = v_sim(end);

        for i = 1:n-1
            v_now = v_forward(i);
            [F_drive, g_sel, ~] = get_drive_force(v_now, car);
            gear_forward(i) = g_sel;
            F_drag = car.aero_drag_coeff * v_now^2;
            a_engine = (F_drive - F_drag) / car.mass;
            [a_grip_total, ~] = get_grip_accel_v03(v_now, car);
            a_lat_used = v_now^2 * kappa(i);
            if a_lat_used >= a_grip_total
                a_grip_long = 0;
            else
                a_grip_long = sqrt(a_grip_total^2 - a_lat_used^2);
            end
            if a_engine > 0
                a_forward = min(a_engine, a_grip_long);
            else
                a_forward = a_engine;
            end
            v_next_sq = v_now^2 + 2 * a_forward * ds;
            v_next = sqrt(max(v_next_sq, 0));
            v_forward(i+1) = min(v_next, v_corner(i+1));
        end
        gear_forward(n) = gear_forward(n-1);

        v_backward(n) = v_forward(n);
        for i = n:-1:2
            v_now = v_backward(i);
            [a_grip_total, ~] = get_grip_accel_v03(v_now, car);
            a_lat_used = v_now^2 * kappa(i);
            if a_lat_used >= a_grip_total
                a_brake = 0;
            else
                a_brake = sqrt(a_grip_total^2 - a_lat_used^2);
            end
            v_prev = sqrt(v_now^2 + 2 * a_brake * ds);
            v_backward(i-1) = min(v_prev, v_forward(i-1));
        end

        v_sim = min(v_forward, v_backward);
        v_sim = min(v_sim, v_corner);
        dt_sim = ds ./ v_sim;
        lap_time = sum(dt_sim);
        t_cum = cumsum(dt_sim);
        lap_min = floor(lap_time / 60);
        lap_sec = lap_time - lap_min * 60;

        fprintf('  Iter %d: start=%.2f, end=%.2f km/h, lap=%.3f s\n', ...
                iter, v_sim(1)*3.6, v_sim(end)*3.6, lap_time);

        if abs(v_sim(end) - v_sim(1)) < 0.5
            fprintf('  Converged.\n');
            break;
        end
    end
end

%% ========================================================================
%  8. FINAL RESULTS — ALL VERSIONS COMPARED
%  ========================================================================

fprintf('\n================ v03 CONVERGED RESULTS =================\n');
fprintf('  v03 lap time:  %d:%06.3f\n', lap_min, lap_sec);
fprintf('  Reference:     %d:%06.3f\n', ...
        floor(track.meta.ref_laptime/60), ...
        track.meta.ref_laptime - floor(track.meta.ref_laptime/60)*60);
fprintf('  Delta vs ref:  %+.3f s (%+.1f%%)\n', ...
        lap_time - track.meta.ref_laptime, ...
        (lap_time - track.meta.ref_laptime) / track.meta.ref_laptime * 100);
fprintf('\n  --- Version comparison ---\n');
if has_v01
    fprintf('  v01 (point-mass):       %d:%06.3f  (delta vs ref: %+.1f s)\n', ...
            floor(sim_v01.lap_time/60), ...
            sim_v01.lap_time - floor(sim_v01.lap_time/60)*60, ...
            sim_v01.lap_time - track.meta.ref_laptime);
end
if has_v02
    fprintf('  v02 (+ aero):           %d:%06.3f  (delta vs ref: %+.1f s)\n', ...
            floor(sim02.lap_time/60), ...
            sim02.lap_time - floor(sim02.lap_time/60)*60, ...
            sim02.lap_time - track.meta.ref_laptime);
end
fprintf('  v03 (+ load sens):      %d:%06.3f  (delta vs ref: %+.1f s)\n', ...
        lap_min, lap_sec, lap_time - track.meta.ref_laptime);
fprintf('  Reference:              8:11.341\n');
if has_v02
    fprintf('\n  Load sensitivity cost: %+.1f s (v03 vs v02)\n', ...
            lap_time - sim02.lap_time);
    fprintf('  (positive = v03 is slower = load sensitivity reduced grip)\n');
end
fprintf('  Min speed:  %.1f km/h\n', min(v_sim)*3.6);
fprintf('  Max speed:  %.1f km/h\n', max(v_sim)*3.6);
fprintf('  Mean speed: %.1f km/h\n', mean(v_sim)*3.6);
fprintf('=========================================================\n');

%% ========================================================================
%  9. BUILD OUTPUT STRUCT
%  ========================================================================

sim03 = struct();
sim03.v         = v_sim;
sim03.v_kmh     = v_sim * 3.6;
sim03.v_corner  = v_corner;
sim03.v_forward = v_forward;
sim03.v_backward = v_backward;
sim03.gear      = gear_forward;
sim03.dt        = dt_sim;
sim03.t_cum     = t_cum;
sim03.lap_time  = lap_time;
sim03.dist      = track.dist;

sim03.meta.model   = 'v03 point-mass + aero + load sensitivity';
sim03.meta.car     = car.meta.name;
sim03.meta.track   = track.meta.name;
sim03.meta.mu_0    = car.tyre.mu_0;
sim03.meta.k       = car.tyre.load_sens_k;
sim03.meta.created = datestr(now, 'yyyy-mm-dd HH:MM');

%% ========================================================================
%  10. COMPARISON PLOTS
%  ========================================================================

figure('Name', 'v03 Results — All Versions', 'NumberTitle', 'off', ...
       'Position', [50, 50, 1400, 900]);

% --- Plot 1: Speed comparison (all versions + reference) ---
subplot(3,1,1);
hold on;
plot(track.dist/1000, track.ref.v * 3.6, 'b', 'LineWidth', 1.2, ...
     'DisplayName', 'Reference (iRacing)');
if has_v01
    plot(track.dist/1000, sim_v01.v * 3.6, 'Color', [0.75 0.75 0.75], ...
         'LineWidth', 0.5, 'DisplayName', sprintf('v01 %.1fs', sim_v01.lap_time));
end
if has_v02
    plot(track.dist/1000, sim02.v * 3.6, 'Color', [1.0 0.6 0.6], ...
         'LineWidth', 0.5, 'DisplayName', sprintf('v02 %.1fs', sim02.lap_time));
end
plot(track.dist/1000, v_sim * 3.6, 'r', 'LineWidth', 0.9, ...
     'DisplayName', sprintf('v03 %.1fs', lap_time));
hold off;
ylabel('Speed [km/h]');
title(sprintf('v03 Load Sensitivity — %d:%06.3f  |  Delta: %+.1f s (%+.1f%%)', ...
      lap_min, lap_sec, lap_time - track.meta.ref_laptime, ...
      (lap_time - track.meta.ref_laptime)/track.meta.ref_laptime*100));
legend('Location', 'best');
grid on;
xlim([0, track.length/1000]);
ylim([0, 320]);

% --- Plot 2: Speed delta v03 vs reference ---
subplot(3,1,2);
delta_v = v_sim * 3.6 - track.ref.v * 3.6;
hold on;
area(track.dist/1000, max(delta_v, 0), 'FaceColor', [0.8 0.2 0.2], ...
     'FaceAlpha', 0.4, 'EdgeColor', 'none', 'DisplayName', 'Sim faster');
area(track.dist/1000, min(delta_v, 0), 'FaceColor', [0.2 0.2 0.8], ...
     'FaceAlpha', 0.4, 'EdgeColor', 'none', 'DisplayName', 'Sim slower');
hold off;
ylabel('\Delta Speed [km/h]');
legend('Location', 'best');
grid on;
xlim([0, track.length/1000]);
title('Speed Delta: v03 minus Reference');

% --- Plot 3: mu vs speed (showing load sensitivity effect) ---
subplot(3,1,3);
v_range = linspace(0, 280/3.6, 200);
mu_v01 = car.tyre.mu_peak * ones(size(v_range));
mu_v03 = zeros(size(v_range));
for vi = 1:length(v_range)
    [~, mu_v03(vi)] = get_grip_accel_v03(v_range(vi), car);
end
hold on;
plot(v_range * 3.6, mu_v01, 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, ...
     'DisplayName', 'v01/v02: constant \mu = 1.60');
plot(v_range * 3.6, mu_v03, 'r', 'LineWidth', 1.5, ...
     'DisplayName', 'v03: \mu(Fz) load-sensitive');
hold off;
xlabel('Speed [km/h]');
ylabel('Effective \mu [-]');
legend('Location', 'northeast');
grid on;
title('Friction coefficient vs speed — load sensitivity reduces \mu at high speed');
ylim([1.2, 1.9]);

fprintf('\n=== v03 Simulation Complete ===\n\n');
