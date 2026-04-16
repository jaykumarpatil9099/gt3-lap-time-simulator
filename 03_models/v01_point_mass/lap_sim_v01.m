%% lap_sim_v01.m
%  Point-mass quasi-steady-state lap time simulator (v01)
%
%  MODEL FIDELITY:
%    - Point mass (no weight transfer, no per-axle loads)
%    - Fixed friction circle: mu = constant, not speed-dependent
%    - Includes aero DRAG (limits top speed) but NOT downforce
%    - Engine torque curve with automatic gear selection
%    - Three-pass solver: cornering → forward (accel) → backward (brake)
%
%  INPUTS (must be in workspace):
%    car   — vehicle parameters (from amg_gt3_params.m)
%    track — track data (from build_track.m → n24_track.mat)
%
%  OUTPUT:
%    sim   — struct with simulated speed profile, lap time, and diagnostics
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-16
%
%  USAGE:
%    >> startup_project
%    >> import_reference_lap
%    >> build_track
%    >> lap_sim_v01

%% ========================================================================
%  0. CHECK INPUTS
%  ========================================================================

if ~exist('car', 'var')
    error('Car parameters not loaded. Run startup_project first.');
end
if ~exist('track', 'var')
    error('Track data not loaded. Run build_track first.');
end

fprintf('\n=== Lap Sim v01 — Point Mass QSS ===\n');
fprintf('Car:   %s\n', car.meta.name);
fprintf('Track: %s (%.3f km)\n', track.meta.name, track.length/1000);

%% ========================================================================
%  1. HELPER FUNCTION: ENGINE FORCE AT GIVEN SPEED
%  ========================================================================
%  For a given road speed [m/s], this function:
%    (a) Determines which gear gives the most wheel force (auto-shift)
%    (b) Looks up engine torque at the corresponding RPM
%    (c) Converts to wheel force
%
%  If RPM is below idle or above rev limit in all gears, force = 0.
%  This is a nested function — it can see the 'car' variable from the
%  parent workspace.

    function [F_drive, gear_selected, rpm_selected] = get_drive_force(v, car)
        % Compute RPM in each gear at this speed
        %   RPM = v * total_ratio / rolling_radius * 60/(2*pi)
        %       = v * total_ratio / rolling_radius / (2*pi/60)

        F_best = 0;
        gear_selected = 1;
        rpm_selected = 0;

        for g = 1:car.gearbox.n_gears
            % RPM in this gear at speed v
            rpm_g = v * car.gearbox.total_ratio(g) / car.tyre.rolling_radius ...
                    * (60 / (2*pi));

            % Check RPM is within operating range
            if rpm_g < car.engine.rpm_idle || rpm_g > car.engine.rpm_max
                continue;  % Skip this gear — RPM out of range
            end

            % Interpolate engine torque at this RPM
            T_engine = interp1(car.engine.rpm, car.engine.torque, rpm_g, ...
                               'linear', 'extrap');

            % Convert to wheel force:
            %   F = T_engine * total_ratio * efficiency / rolling_radius
            F_wheel = T_engine * car.gearbox.total_ratio(g) * car.gearbox.efficiency ...
                      / car.tyre.rolling_radius;

            % Keep the gear that gives the highest force (auto-shift logic)
            if F_wheel > F_best
                F_best = F_wheel;
                gear_selected = g;
                rpm_selected = rpm_g;
            end
        end

        F_drive = max(F_best, 0);  % Never negative (engine can't push backward)
    end

%% ========================================================================
%  2. PASS 1 — CORNERING SPEED LIMIT
%  ========================================================================
%  At each point, the maximum speed is limited by tyre grip and curvature:
%
%    a_lat_max = mu * g          (maximum lateral acceleration, constant for v01)
%    v_corner  = sqrt(a_lat_max / kappa)
%
%  On straights (kappa ≈ 0), v_corner → infinity. We cap it at a practical
%  maximum (400 km/h ≈ 111 m/s) to avoid numerical issues. No GT3 car
%  reaches anywhere near this, so the cap doesn't affect results.

fprintf('\nPass 1: Cornering speed limits...\n');

mu = car.tyre.mu_peak;          % constant friction coefficient for v01
g_acc = car.g;                  % gravitational acceleration
a_lat_max = mu * g_acc;         % maximum lateral acceleration [m/s^2]

v_max_cap = 400 / 3.6;          % speed cap [m/s] (400 km/h, never reached)

n = track.n;                     % number of track points
ds = track.ds;                   % distance step [m]
kappa = track.kappa;             % curvature at each point [1/m]

% Cornering speed limit at each point
v_corner = zeros(n, 1);
for i = 1:n
    if kappa(i) < 1e-6
        % Effectively straight — no cornering limit
        v_corner(i) = v_max_cap;
    else
        v_corner(i) = sqrt(a_lat_max / kappa(i));
    end
    % Apply the cap
    v_corner(i) = min(v_corner(i), v_max_cap);
end

fprintf('  Min cornering speed: %.1f km/h (tightest corner)\n', min(v_corner)*3.6);
fprintf('  Points at cap (straights): %d / %d\n', sum(v_corner >= v_max_cap - 0.1), n);

%% ========================================================================
%  3. PASS 2 — FORWARD PASS (ACCELERATION LIMITED)
%  ========================================================================
%  Starting from point 1, march forward through the track.
%  At each point, the car tries to accelerate as hard as possible.
%  Acceleration is limited by TWO things:
%
%  (a) Engine force minus drag:
%      a_engine = (F_drive - F_drag) / m
%
%  (b) Friction circle: total acceleration (lateral + longitudinal combined)
%      cannot exceed mu * g. The lateral acceleration is "used up" by
%      cornering, so the remaining longitudinal capability is:
%
%      a_lat_used = v^2 * kappa     (lateral g needed for the current corner)
%      a_long_max = sqrt((mu*g)^2 - a_lat_used^2)
%
%  The actual forward acceleration is the MINIMUM of (a) and (b).
%
%  Speed at the next point uses the kinematic equation:
%      v_next = sqrt(v_current^2 + 2 * a * ds)
%
%  Why this equation? From v² = u² + 2as (constant acceleration over
%  distance ds). This is the distance-domain version of v = u + at.

fprintf('Pass 2: Forward pass (acceleration)...\n');

v_forward = zeros(n, 1);
gear_forward = zeros(n, 1);

% Start with the cornering speed at point 1 (we'll iterate to fix this)
v_forward(1) = v_corner(1);

for i = 1:n-1
    v_now = v_forward(i);

    % --- Engine force at current speed ---
    [F_drive, g_sel, ~] = get_drive_force(v_now, car);
    gear_forward(i) = g_sel;

    % --- Drag force ---
    F_drag = car.aero_drag_coeff * v_now^2;

    % --- Net engine acceleration ---
    a_engine = (F_drive - F_drag) / car.mass;

    % --- Friction circle: available longitudinal acceleration ---
    a_lat_used = v_now^2 * kappa(i);  % lateral accel needed for this corner

    if a_lat_used >= a_lat_max
        % Already at or beyond cornering limit — no longitudinal capacity
        a_grip = 0;
    else
        a_grip = sqrt(a_lat_max^2 - a_lat_used^2);
    end

    % --- Actual acceleration ---
    %  Two cases:
    %  (a) a_engine > 0: car is trying to accelerate. Limited by grip.
    %      a_forward = min(a_engine, a_grip)
    %  (b) a_engine <= 0: drag exceeds engine force. Car decelerates.
    %      This is an aero effect, NOT a tyre effect, so the friction
    %      circle does NOT limit it. a_forward = a_engine (negative).
    if a_engine > 0
        a_forward = min(a_engine, a_grip);
    else
        a_forward = a_engine;  % drag deceleration, not grip-limited
    end

    % --- Speed at next point ---
    %  v² = u² + 2as. If a_forward is negative, v_next < v_now (slowing).
    v_next_sq = v_now^2 + 2 * a_forward * ds;
    v_next = sqrt(max(v_next_sq, 0));  % max with 0 prevents sqrt of negative

    % --- Cap at the next point's cornering limit ---
    v_forward(i+1) = min(v_next, v_corner(i+1));
end
gear_forward(n) = gear_forward(n-1);  % fill last point

fprintf('  Max speed reached: %.1f km/h\n', max(v_forward)*3.6);

%% ========================================================================
%  4. PASS 3 — BACKWARD PASS (BRAKING LIMITED)
%  ========================================================================
%  Same logic as the forward pass, but we march BACKWARD from the last
%  point to the first. At each point, the car is "braking" into the
%  corner ahead (which, going backward, means it's checking whether
%  it can decelerate from the current speed to the next corner's speed).
%
%  Braking deceleration is limited by the friction circle only (no engine
%  involvement in braking for v01 — we assume infinite brake hardware).
%
%  The backward pass catches situations like: "the car is going 250 km/h
%  on the straight, but there's an 80 km/h corner 200 m ahead — can it
%  stop in time?" If not, the backward pass forces the speed lower.

fprintf('Pass 3: Backward pass (braking)...\n');

v_backward = zeros(n, 1);

% Start the backward pass from the last point's forward-pass speed
v_backward(n) = v_forward(n);

for i = n:-1:2
    v_now = v_backward(i);

    % --- Friction circle: available braking deceleration ---
    a_lat_used = v_now^2 * kappa(i);

    if a_lat_used >= a_lat_max
        a_brake = 0;
    else
        a_brake = sqrt(a_lat_max^2 - a_lat_used^2);
    end

    % --- Speed at previous point (going backward, so we ADD energy) ---
    %  If the car is braking into point i, then at point i-1 it was going
    %  faster. So: v_(i-1) = sqrt(v_i^2 + 2 * a_brake * ds)
    v_prev = sqrt(v_now^2 + 2 * a_brake * ds);

    % --- Cap at what the forward pass already computed ---
    v_backward(i-1) = min(v_prev, v_forward(i-1));
end

%% ========================================================================
%  5. COMBINE: FINAL SPEED PROFILE
%  ========================================================================
%  The final speed at each point is the minimum of all three limits.
%  Actually, the backward pass already took the min with forward pass,
%  so v_backward IS the final answer. But let's be explicit:

v_sim = min(v_forward, v_backward);

% Also cap at cornering limit (should already be satisfied, but safety)
v_sim = min(v_sim, v_corner);

fprintf('\nSpeed profile computed.\n');
fprintf('  Min speed: %.1f km/h\n', min(v_sim)*3.6);
fprintf('  Max speed: %.1f km/h\n', max(v_sim)*3.6);
fprintf('  Mean speed: %.1f km/h\n', mean(v_sim)*3.6);

%% ========================================================================
%  6. COMPUTE LAP TIME
%  ========================================================================
%  At each 1 m segment, the time to traverse it is:
%      dt = ds / v
%  Total lap time = sum of all dt.
%
%  We also compute cumulative time for the time-vs-distance plot.

dt_sim = ds ./ v_sim;            % time per segment [s]
lap_time = sum(dt_sim);          % total lap time [s]
t_cum = cumsum(dt_sim);          % cumulative time [s]

% Convert to minutes:seconds.milliseconds
lap_min = floor(lap_time / 60);
lap_sec = lap_time - lap_min * 60;

fprintf('\n========================================\n');
fprintf('  SIMULATED LAP TIME: %d:%06.3f\n', lap_min, lap_sec);
fprintf('  Reference lap time: %d:%06.3f\n', ...
        floor(track.meta.ref_laptime/60), ...
        track.meta.ref_laptime - floor(track.meta.ref_laptime/60)*60);
fprintf('  Delta: %+.3f s (%+.1f%%)\n', ...
        lap_time - track.meta.ref_laptime, ...
        (lap_time - track.meta.ref_laptime) / track.meta.ref_laptime * 100);
fprintf('========================================\n');

%% ========================================================================
%  7. ITERATE FOR LAP CONTINUITY
%  ========================================================================
%  The lap is a loop: the car must cross start/finish at the same speed
%  it had when it left. If v_sim(end) ~= v_sim(1), we need to re-run
%  with the end speed as the new start speed and iterate until they match.
%
%  In practice, if the start/finish is on a straight (which it is at N24 —
%  it's on the GP pit straight), the speeds already match because the
%  car is at or near top speed at both ends. Let's check:

fprintf('\nLap continuity check:\n');
fprintf('  Start speed: %.2f km/h\n', v_sim(1)*3.6);
fprintf('  End speed:   %.2f km/h\n', v_sim(end)*3.6);
fprintf('  Delta:       %.2f km/h\n', (v_sim(end) - v_sim(1))*3.6);

if abs(v_sim(end) - v_sim(1)) > 1.0  % more than 1 m/s difference
    fprintf('  >> Iterating for convergence...\n');

    for iter = 1:5
        % Re-run forward pass with end speed as start speed
        v_forward(1) = v_sim(end);

        for i = 1:n-1
            v_now = v_forward(i);
            [F_drive, g_sel, ~] = get_drive_force(v_now, car);
            gear_forward(i) = g_sel;
            F_drag = car.aero_drag_coeff * v_now^2;
            a_engine = (F_drive - F_drag) / car.mass;
            a_lat_used = v_now^2 * kappa(i);
            if a_lat_used >= a_lat_max
                a_grip = 0;
            else
                a_grip = sqrt(a_lat_max^2 - a_lat_used^2);
            end
            if a_engine > 0
                a_forward = min(a_engine, a_grip);
            else
                a_forward = a_engine;
            end
            v_next_sq = v_now^2 + 2 * a_forward * ds;
            v_next = sqrt(max(v_next_sq, 0));
            v_forward(i+1) = min(v_next, v_corner(i+1));
        end
        gear_forward(n) = gear_forward(n-1);

        % Re-run backward pass
        v_backward(n) = v_forward(n);
        for i = n:-1:2
            v_now = v_backward(i);
            a_lat_used = v_now^2 * kappa(i);
            if a_lat_used >= a_lat_max
                a_brake = 0;
            else
                a_brake = sqrt(a_lat_max^2 - a_lat_used^2);
            end
            v_prev = sqrt(v_now^2 + 2 * a_brake * ds);
            v_backward(i-1) = min(v_prev, v_forward(i-1));
        end

        v_sim = min(v_forward, v_backward);
        v_sim = min(v_sim, v_corner);

        dt_sim = ds ./ v_sim;
        lap_time = sum(dt_sim);
        t_cum = cumsum(dt_sim);

        fprintf('  Iteration %d: start=%.2f, end=%.2f km/h, lap=%.3f s\n', ...
                iter, v_sim(1)*3.6, v_sim(end)*3.6, lap_time);

        if abs(v_sim(end) - v_sim(1)) < 0.5
            fprintf('  Converged.\n');
            break;
        end
    end

    % Recompute final lap time display
    lap_min = floor(lap_time / 60);
    lap_sec = lap_time - lap_min * 60;
else
    fprintf('  Start ≈ End — no iteration needed.\n');
end

% --- Final converged results summary ---
fprintf('\n============ CONVERGED RESULTS =============\n');
fprintf('  Lap time:  %d:%06.3f\n', lap_min, lap_sec);
fprintf('  Reference: %d:%06.3f\n', ...
        floor(track.meta.ref_laptime/60), ...
        track.meta.ref_laptime - floor(track.meta.ref_laptime/60)*60);
fprintf('  Delta:     %+.3f s (%+.1f%%)\n', ...
        lap_time - track.meta.ref_laptime, ...
        (lap_time - track.meta.ref_laptime) / track.meta.ref_laptime * 100);
fprintf('  Min speed: %.1f km/h\n', min(v_sim)*3.6);
fprintf('  Max speed: %.1f km/h\n', max(v_sim)*3.6);
fprintf('  Mean speed: %.1f km/h\n', mean(v_sim)*3.6);
fprintf('============================================\n');

%% ========================================================================
%  8. BUILD OUTPUT STRUCT
%  ========================================================================

sim = struct();
sim.v         = v_sim;               % [m/s]   simulated speed at each point
sim.v_kmh     = v_sim * 3.6;         % [km/h]  convenience
sim.v_corner  = v_corner;            % [m/s]   cornering speed limit
sim.v_forward = v_forward;           % [m/s]   forward pass result
sim.v_backward = v_backward;         % [m/s]   backward pass result
sim.gear      = gear_forward;        % [-]     gear selection
sim.dt        = dt_sim;              % [s]     time per segment
sim.t_cum     = t_cum;               % [s]     cumulative time
sim.lap_time  = lap_time;            % [s]     total lap time
sim.dist      = track.dist;          % [m]     distance (same as track grid)

sim.meta.model   = 'v01 point-mass QSS';
sim.meta.car     = car.meta.name;
sim.meta.track   = track.meta.name;
sim.meta.mu      = mu;
sim.meta.created = datestr(now, 'yyyy-mm-dd HH:MM');

%% ========================================================================
%  9. VERIFICATION PLOTS
%  ========================================================================

figure('Name', 'v01 Lap Simulation Results', 'NumberTitle', 'off', ...
       'Position', [50, 50, 1400, 900]);

% --- Plot 1: Speed comparison (sim vs reference) ---
subplot(3,1,1);
hold on;
plot(track.dist/1000, track.ref.v * 3.6, 'b', 'LineWidth', 0.8, ...
     'DisplayName', 'Reference (iRacing)');
plot(track.dist/1000, v_sim * 3.6, 'r', 'LineWidth', 0.8, ...
     'DisplayName', sprintf('Sim v01 (%.3f s)', lap_time));
hold off;
ylabel('Speed [km/h]');
title(sprintf('v01 Point-Mass QSS — Lap Time: %d:%06.3f  |  Ref: %d:%06.3f  |  Delta: %+.1f s', ...
      lap_min, lap_sec, ...
      floor(track.meta.ref_laptime/60), ...
      track.meta.ref_laptime - floor(track.meta.ref_laptime/60)*60, ...
      lap_time - track.meta.ref_laptime));
legend('Location', 'best');
grid on;
xlim([0, track.length/1000]);
ylim([0, 320]);

% --- Plot 2: Speed breakdown (which limit is active?) ---
subplot(3,1,2);
hold on;
plot(track.dist/1000, v_corner * 3.6, 'Color', [0.7 0.7 0.7], ...
     'LineWidth', 0.5, 'DisplayName', 'Cornering limit');
plot(track.dist/1000, v_forward * 3.6, 'Color', [0 0.7 0], ...
     'LineWidth', 0.5, 'DisplayName', 'Forward (accel) limit');
plot(track.dist/1000, v_backward * 3.6, 'Color', [0.8 0 0], ...
     'LineWidth', 0.5, 'DisplayName', 'Backward (brake) limit');
plot(track.dist/1000, v_sim * 3.6, 'k', 'LineWidth', 1.2, ...
     'DisplayName', 'Final speed');
hold off;
ylabel('Speed [km/h]');
legend('Location', 'best');
grid on;
xlim([0, track.length/1000]);
ylim([0, 320]);
title('Speed Breakdown — which limit controls each section');

% --- Plot 3: Gear selection ---
subplot(3,1,3);
plot(track.dist/1000, gear_forward, 'k', 'LineWidth', 0.8);
ylabel('Gear [-]');
xlabel('Distance [km]');
grid on;
xlim([0, track.length/1000]);
ylim([0, 7]);
title('Automatic Gear Selection');

fprintf('\n=== v01 Simulation Complete ===\n\n');
