%% lap_sim_v02.m
%  Point-mass QSS lap time simulator with AERODYNAMIC DOWNFORCE (v02)
%
%  WHAT CHANGED FROM v01:
%    v01: grip = mu * m * g                    (constant)
%    v02: grip = mu * (m * g + Df(v))          (speed-dependent)
%         where Df(v) = aero_df_coeff * v^2    (downforce grows with v^2)
%
%  This means:
%    - Cornering speed limit now has a new closed-form equation
%    - The friction circle radius grows with speed
%    - Fast corners get faster (more grip from downforce)
%    - The solver structure (3-pass + iteration) is UNCHANGED from v01
%
%  INPUTS (must be in workspace):
%    car   — vehicle parameters (from amg_gt3_params.m)
%    track — track data (from build_track.m)
%    sim   — v01 result (optional, for comparison plots)
%
%  OUTPUT:
%    sim02 — struct with v02 simulated speed profile and lap time
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-16

%% ========================================================================
%  0. CHECK INPUTS
%  ========================================================================

if ~exist('car', 'var')
    error('Car parameters not loaded. Run startup_project first.');
end
if ~exist('track', 'var')
    error('Track data not loaded. Run build_track first.');
end

fprintf('\n=== Lap Sim v02 — Point Mass + Aero Downforce ===\n');
fprintf('Car:   %s\n', car.meta.name);
fprintf('Track: %s (%.3f km)\n', track.meta.name, track.length/1000);

% Save v01 result if it exists, for comparison
if exist('sim', 'var')
    sim_v01 = sim;
    has_v01 = true;
    fprintf('v01 result found — will overlay in plots.\n');
else
    has_v01 = false;
end

%% ========================================================================
%  1. HELPER FUNCTION: ENGINE FORCE
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
%  2. HELPER FUNCTION: SPEED-DEPENDENT GRIP (NEW IN v02)
%  ========================================================================
%  Returns the maximum total acceleration [m/s^2] the tyres can produce
%  at a given speed, accounting for downforce.
%
%  Physics:
%    F_normal = m*g + aero_df_coeff * v^2     (weight + downforce)
%    F_grip   = mu * F_normal
%    a_max    = F_grip / m = mu * (g + aero_df_coeff * v^2 / m)

    function a_max = get_grip_accel(v, car)
        F_downforce = car.aero_df_coeff * v^2;
        F_normal = car.mass * car.g + F_downforce;
        F_grip = car.tyre.mu_peak * F_normal;
        a_max = F_grip / car.mass;
    end

%% ========================================================================
%  3. PASS 1 — CORNERING SPEED LIMIT (with downforce)
%  ========================================================================
%  Force balance in a corner:
%    m * v^2 * kappa = mu * (m*g + aero_df_coeff * v^2)
%
%  Solving for v^2:
%    v^2 = mu * m * g / (m * kappa - mu * aero_df_coeff)
%
%  The denominator (m*kappa - mu*aero_df_coeff) can be:
%    > 0: normal corner, finite speed limit exists
%    <= 0: downforce grows faster than cornering demand; no grip limit,
%           only power/drag limits speed. We set v_corner = v_max_cap.

fprintf('\nPass 1: Cornering speed limits (with downforce)...\n');

mu = car.tyre.mu_peak;
g_acc = car.g;
n = track.n;
ds = track.ds;
kappa = track.kappa;

v_max_cap = 400 / 3.6;   % [m/s] safety cap

% Critical curvature below which there is no grip-limited speed
kappa_crit = mu * car.aero_df_coeff / car.mass;
fprintf('  Critical curvature: %.6f [1/m] (R = %.0f m)\n', kappa_crit, 1/kappa_crit);
fprintf('  Corners gentler than R=%.0f m have no grip limit (aero-dominated)\n', 1/kappa_crit);

v_corner = zeros(n, 1);
for i = 1:n
    denom = car.mass * kappa(i) - mu * car.aero_df_coeff;

    if denom <= 0 || kappa(i) < 1e-6
        % No grip-limited speed — downforce dominates or straight
        v_corner(i) = v_max_cap;
    else
        v_corner(i) = sqrt(mu * car.mass * g_acc / denom);
        v_corner(i) = min(v_corner(i), v_max_cap);
    end
end

fprintf('  Min cornering speed: %.1f km/h (tightest corner)\n', min(v_corner)*3.6);
fprintf('  Points at cap (aero-dominated or straight): %d / %d\n', ...
        sum(v_corner >= v_max_cap - 0.1), n);

%% ========================================================================
%  4. PASS 2 — FORWARD PASS (ACCELERATION LIMITED, with aero)
%  ========================================================================
%  Same structure as v01, but:
%    - Friction circle radius is now speed-dependent: get_grip_accel(v)
%    - Drag is unchanged (same as v01)

fprintf('Pass 2: Forward pass (acceleration + aero grip)...\n');

v_forward = zeros(n, 1);
gear_forward = zeros(n, 1);

v_forward(1) = v_corner(1);

for i = 1:n-1
    v_now = v_forward(i);

    % Engine force
    [F_drive, g_sel, ~] = get_drive_force(v_now, car);
    gear_forward(i) = g_sel;

    % Drag
    F_drag = car.aero_drag_coeff * v_now^2;

    % Net engine acceleration
    a_engine = (F_drive - F_drag) / car.mass;

    % Speed-dependent grip (NEW: friction circle grows with speed)
    a_grip_total = get_grip_accel(v_now, car);

    % Lateral acceleration used for cornering at this speed and curvature
    a_lat_used = v_now^2 * kappa(i);

    % Available longitudinal acceleration from friction circle
    if a_lat_used >= a_grip_total
        a_grip_long = 0;
    else
        a_grip_long = sqrt(a_grip_total^2 - a_lat_used^2);
    end

    % Combine: engine vs grip, allow drag deceleration
    if a_engine > 0
        a_forward = min(a_engine, a_grip_long);
    else
        a_forward = a_engine;   % drag deceleration (not grip-limited)
    end

    % Propagate speed
    v_next_sq = v_now^2 + 2 * a_forward * ds;
    v_next = sqrt(max(v_next_sq, 0));

    v_forward(i+1) = min(v_next, v_corner(i+1));
end
gear_forward(n) = gear_forward(n-1);

fprintf('  Max speed reached: %.1f km/h\n', max(v_forward)*3.6);

%% ========================================================================
%  5. PASS 3 — BACKWARD PASS (BRAKING LIMITED, with aero)
%  ========================================================================
%  Braking deceleration now benefits from downforce at high speed:
%  the car brakes HARDER at high speed because there's more grip.

fprintf('Pass 3: Backward pass (braking + aero grip)...\n');

v_backward = zeros(n, 1);
v_backward(n) = v_forward(n);

for i = n:-1:2
    v_now = v_backward(i);

    % Speed-dependent braking grip
    a_grip_total = get_grip_accel(v_now, car);

    % Lateral usage
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
            a_grip_total = get_grip_accel(v_now, car);
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
            a_grip_total = get_grip_accel(v_now, car);
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
%  8. FINAL RESULTS
%  ========================================================================

fprintf('\n=============== v02 CONVERGED RESULTS ================\n');
fprintf('  Lap time:  %d:%06.3f\n', lap_min, lap_sec);
fprintf('  Reference: %d:%06.3f\n', ...
        floor(track.meta.ref_laptime/60), ...
        track.meta.ref_laptime - floor(track.meta.ref_laptime/60)*60);
fprintf('  Delta vs ref:  %+.3f s (%+.1f%%)\n', ...
        lap_time - track.meta.ref_laptime, ...
        (lap_time - track.meta.ref_laptime) / track.meta.ref_laptime * 100);
if has_v01
    fprintf('  Delta vs v01:  %+.3f s\n', lap_time - sim_v01.lap_time);
    fprintf('  (negative = v02 is faster = downforce gained time)\n');
end
fprintf('  Min speed:  %.1f km/h\n', min(v_sim)*3.6);
fprintf('  Max speed:  %.1f km/h\n', max(v_sim)*3.6);
fprintf('  Mean speed: %.1f km/h\n', mean(v_sim)*3.6);
fprintf('======================================================\n');

%% ========================================================================
%  9. BUILD OUTPUT STRUCT
%  ========================================================================

sim02 = struct();
sim02.v         = v_sim;
sim02.v_kmh     = v_sim * 3.6;
sim02.v_corner  = v_corner;
sim02.v_forward = v_forward;
sim02.v_backward = v_backward;
sim02.gear      = gear_forward;
sim02.dt        = dt_sim;
sim02.t_cum     = t_cum;
sim02.lap_time  = lap_time;
sim02.dist      = track.dist;

sim02.meta.model   = 'v02 point-mass + aero downforce';
sim02.meta.car     = car.meta.name;
sim02.meta.track   = track.meta.name;
sim02.meta.mu      = mu;
sim02.meta.Cl      = car.Cl;
sim02.meta.Cd      = car.Cd;
sim02.meta.created = datestr(now, 'yyyy-mm-dd HH:MM');

%% ========================================================================
%  10. COMPARISON PLOTS
%  ========================================================================

figure('Name', 'v02 Results + Comparison', 'NumberTitle', 'off', ...
       'Position', [50, 50, 1400, 900]);

% --- Plot 1: Speed comparison (ref vs v01 vs v02) ---
subplot(3,1,1);
hold on;
plot(track.dist/1000, track.ref.v * 3.6, 'b', 'LineWidth', 1.0, ...
     'DisplayName', 'Reference (iRacing)');
if has_v01
    plot(track.dist/1000, sim_v01.v * 3.6, 'Color', [0.6 0.6 0.6], ...
         'LineWidth', 0.7, 'DisplayName', sprintf('v01 (%.1f s)', sim_v01.lap_time));
end
plot(track.dist/1000, v_sim * 3.6, 'r', 'LineWidth', 0.9, ...
     'DisplayName', sprintf('v02 (%.1f s)', lap_time));
hold off;
ylabel('Speed [km/h]');
title(sprintf('v02 Aero Downforce — %d:%06.3f  |  Delta vs ref: %+.1f s  |  vs v01: %+.1f s', ...
      lap_min, lap_sec, lap_time - track.meta.ref_laptime, ...
      lap_time - sim_v01.lap_time));
legend('Location', 'best');
grid on;
xlim([0, track.length/1000]);
ylim([0, 320]);

% --- Plot 2: Speed delta (v02 - reference) ---
subplot(3,1,2);
delta_v = v_sim * 3.6 - track.ref.v * 3.6;
hold on;
area(track.dist/1000, max(delta_v, 0), 'FaceColor', [0.8 0.2 0.2], ...
     'FaceAlpha', 0.4, 'EdgeColor', 'none', 'DisplayName', 'Sim faster');
area(track.dist/1000, min(delta_v, 0), 'FaceColor', [0.2 0.2 0.8], ...
     'FaceAlpha', 0.4, 'EdgeColor', 'none', 'DisplayName', 'Sim slower');
hold off;
ylabel('\Delta Speed [km/h]');
xlabel('Distance [km]');
legend('Location', 'best');
grid on;
xlim([0, track.length/1000]);
title('Speed Delta: v02 minus Reference (red = sim too fast, blue = sim too slow)');

% --- Plot 3: Grip acceleration comparison (v01 constant vs v02 aero) ---
subplot(3,1,3);
% Show how the friction circle grows with speed in v02
v_range = linspace(0, 280/3.6, 200);
a_grip_v01 = mu * g_acc * ones(size(v_range));
a_grip_v02 = arrayfun(@(v) get_grip_accel(v, car), v_range);
hold on;
plot(v_range * 3.6, a_grip_v01 / g_acc, 'Color', [0.6 0.6 0.6], ...
     'LineWidth', 1.5, 'DisplayName', 'v01: constant grip');
plot(v_range * 3.6, a_grip_v02 / g_acc, 'r', 'LineWidth', 1.5, ...
     'DisplayName', 'v02: grip + downforce');
hold off;
xlabel('Speed [km/h]');
ylabel('Max lateral accel [g]');
legend('Location', 'northwest');
grid on;
title('Friction circle radius vs speed — this is what downforce does');

fprintf('\n=== v02 Simulation Complete ===\n\n');
