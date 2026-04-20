%% lap_sim_v04.m — REWRITTEN 2026-04-19
%  Point-mass QSS lap sim with LOAD-SENSITIVE TYRES (v03) + PER-AXLE LOADS
%  + LONGITUDINAL WEIGHT TRANSFER (new in v04).
%
%  HISTORY: The 2026-04-18 version of v04 had six physics bugs that caused
%  an unphysical +49.7 s penalty vs v03. See logbook Entry 011 and
%  00_admin/v04_github_issue.md for the full diagnosis. The buggy file is
%  preserved as lap_sim_v04_buggy_2026-04-18.m.
%
%  WHAT CHANGED FROM v03:
%    v03: single grip envelope, Fz_total treated as uniform across 4 tyres.
%         mu_eff = mu_0 - k*Fz_total/4; F_grip = mu_eff * Fz_total.
%    v04: per-axle normal loads (honouring weight_dist_f=0.46 and
%         aero_balance_f=0.43), per-tyre load sensitivity at each axle,
%         longitudinal weight transfer in forward/backward passes, and
%         per-axle friction circle.
%
%  WHAT DID NOT CHANGE:
%    - 3-pass solver skeleton (cornering → forward → backward → iterate).
%    - v03's iterative cornering solve and continuity iteration.
%    - Friction-circle formulation: the same sqrt(F_grip^2 - F_y^2) logic,
%      just applied per-axle instead of to the total envelope.
%
%  SIGN CONVENTION for longitudinal transfer:
%    dFz_long > 0  =>  weight shifted TO rear (accel)
%    dFz_long < 0  =>  weight shifted TO front (braking)
%    Fz_f = Fz_f_static + Fz_f_aero - dFz_long
%    Fz_r = Fz_r_static + Fz_r_aero + dFz_long
%    where dFz_long = m * a_long_signed * h_cog / wheelbase
%    and a_long_signed > 0 for accel, < 0 for braking.
%
%  INPUTS (must be in workspace):
%    car   — vehicle parameters (from startup_project)
%    track — track data (from build_track)
%
%  OUTPUT:
%    sim04 — struct with results + per-axle diagnostics
%
%  Author:  Jaykumar Patil
%  Rewrite: 2026-04-19

%% ========================================================================
%  0. INPUT CHECKS & STORE PREVIOUS RESULTS
%  ========================================================================

if ~exist('car', 'var')
    error('Car parameters not loaded. Run startup_project first.');
end
if ~exist('track', 'var')
    error('Track data not loaded. Run build_track first.');
end

fprintf('\n=== Lap Sim v04 (rewritten) — Per-Axle Loads + Long. Transfer ===\n');
fprintf('Car:   %s\n', car.meta.name);
fprintf('Track: %s (%.3f km)\n', track.meta.name, track.length/1000);

% Store previous-version results for comparison
has_v01 = exist('sim',   'var') == 1;
has_v02 = exist('sim02', 'var') == 1;
has_v03 = exist('sim03', 'var') == 1;
if has_v01, sim_v01 = sim; end

%% ========================================================================
%  1. HELPER: ENGINE FORCE (unchanged from v01/v02/v03)
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
%  2. HELPER: PER-AXLE GRIP (NEW IN v04)
%  ========================================================================
%  Given a speed v and a signed longitudinal transfer dFz_long, return:
%    - Fz_f, Fz_r   per-axle normal loads  [N]
%    - mu_f, mu_r   per-axle friction coefficients (load-sensitive)
%    - F_grip_f, F_grip_r   per-axle grip force magnitudes [N]
%    - a_grip_tot   total grip as acceleration [m/s^2], = (F_f+F_r)/m
%
%  Per-tyre load sensitivity: the outside tyre in a corner carries ~ half
%  the axle load, so the per-tyre Fz is Fz_axle/2. The load_sens_k in the
%  params file is calibrated against per-tyre Fz (matching v03's Fz/4 for
%  the whole car under 50/50 split).

    function [a_grip_tot, mu_f, mu_r, Fz_f, Fz_r, F_grip_f, F_grip_r] = ...
            get_axle_grip_v04(v, dFz_long, car)

        % Aero downforce (split by aero_balance_f)
        F_df = car.aero_df_coeff * v^2;
        Fz_f_static = car.mass * car.g * car.weight_dist_f;
        Fz_r_static = car.mass * car.g * car.weight_dist_r;
        Fz_f_aero = F_df * car.aero_balance_f;
        Fz_r_aero = F_df * (1 - car.aero_balance_f);

        % Apply longitudinal transfer (sign convention in header)
        Fz_f = Fz_f_static + Fz_f_aero - dFz_long;
        Fz_r = Fz_r_static + Fz_r_aero + dFz_long;

        % Physical floor (a lifted wheel still has a tiny normal load in practice)
        Fz_f = max(Fz_f, 100);
        Fz_r = max(Fz_r, 100);

        % Per-tyre load sensitivity — each axle's k-penalty is based on its
        % per-tyre load, not per-axle.
        mu_f = car.tyre.mu_0 - car.tyre.load_sens_k * (Fz_f / 2);
        mu_r = car.tyre.mu_0 - car.tyre.load_sens_k * (Fz_r / 2);
        mu_f = max(mu_f, 0.5);
        mu_r = max(mu_r, 0.5);

        % Axle grip-force magnitudes and total-grip accel equivalent
        F_grip_f = mu_f * Fz_f;
        F_grip_r = mu_r * Fz_r;
        a_grip_tot = (F_grip_f + F_grip_r) / car.mass;
    end

%% ========================================================================
%  3. PASS 1 — CORNERING SPEED LIMIT (iterative, no long transfer)
%  ========================================================================
%  In pure steady-state cornering, a_long = 0 so dFz_long = 0.
%  Constraint:  m*v^2*kappa = F_grip_f(v,0) + F_grip_r(v,0)
%  Iterate because F_grip depends on v through downforce.

fprintf('\nPass 1: Cornering speed limits (per-axle, load-sensitive)...\n');

g_acc     = car.g;
n         = track.n;
ds        = track.ds;
kappa     = track.kappa;
v_max_cap = 400 / 3.6;

v_corner = zeros(n, 1);

for i = 1:n
    if kappa(i) < 1e-6
        v_corner(i) = v_max_cap;
        continue;
    end

    % Initial guess: use v03-style closed form with average mu
    mu_guess = car.tyre.mu_peak;
    denom = car.mass * kappa(i) - mu_guess * car.aero_df_coeff;
    if denom <= 0
        v_iter = v_max_cap;
    else
        v_iter = min(sqrt(mu_guess * car.mass * g_acc / denom), v_max_cap);
    end

    % Fixed-point iteration: v_new^2 = a_grip(v_iter, 0) / kappa
    for j = 1:20
        a_grip = get_axle_grip_v04(v_iter, 0, car);
        v_new  = sqrt(a_grip / kappa(i));
        v_new  = min(v_new, v_max_cap);
        if abs(v_new - v_iter) < 0.01
            v_iter = v_new;
            break;
        end
        v_iter = 0.5 * v_iter + 0.5 * v_new;   % damping
    end
    v_corner(i) = v_iter;
end

fprintf('  Min cornering speed: %.1f km/h\n', min(v_corner)*3.6);
fprintf('  Points at cap:       %d / %d\n', sum(v_corner >= v_max_cap - 0.1), n);

% Diagnostic: mu at representative speeds with zero transfer
fprintf('  Grip diagnostic (dFz_long = 0):\n');
for speed_test = [80, 150, 200, 260]
    [a_gt, mu_f_d, mu_r_d, Fz_f_d, Fz_r_d] = get_axle_grip_v04(speed_test/3.6, 0, car);
    fprintf('    @ %3d km/h: mu_f = %.3f, mu_r = %.3f, Fz_f = %5.0f N, Fz_r = %5.0f N, a_grip = %.2f m/s^2\n', ...
            speed_test, mu_f_d, mu_r_d, Fz_f_d, Fz_r_d, a_gt);
end

%% ========================================================================
%  4. PASS 2 — FORWARD PASS (RWD traction + friction circle + transfer)
%  ========================================================================
%  For RWD, only the rear axle can propel. In a corner, the rear axle's
%  grip is partly used for lateral force. The remainder is available for
%  longitudinal force (friction circle on the rear axle).
%
%  The rear's share of the lateral force is taken as proportional to Fz
%  (weight distribution). This is the standard QSS approximation — exact
%  only if mu_f == mu_r.
%
%  a_long is implicit (dFz_long depends on a_long), so iterate.

fprintf('Pass 2: Forward pass (RWD, friction circle on rear)...\n');

v_forward       = zeros(n, 1);
gear_forward    = zeros(n, 1);
a_long_forward  = zeros(n, 1);
dFz_forward     = zeros(n, 1);
Fz_f_forward    = zeros(n, 1);
Fz_r_forward    = zeros(n, 1);
v_forward(1)    = v_corner(1);

for i = 1:n-1
    v_now = v_forward(i);
    a_lat = v_now^2 * kappa(i);
    F_drag = car.aero_drag_coeff * v_now^2;

    [F_drive, g_sel, ~] = get_drive_force(v_now, car);
    gear_forward(i) = g_sel;
    a_engine = (F_drive - F_drag) / car.mass;

    % Iterate on a_long (a_long depends on dFz depends on a_long)
    a_long = max(a_engine, 0);   % initial guess: engine-limited
    for it = 1:10
        a_old = a_long;
        dFz  = car.mass * a_long * car.h_cog / car.wheelbase;

        [~, ~, mu_r, Fz_f, Fz_r, ~, F_grip_r] = get_axle_grip_v04(v_now, dFz, car);

        % Rear's share of lateral force (proportional to axle normal load)
        F_y_r = car.mass * a_lat * Fz_r / (Fz_f + Fz_r);

        % Rear friction circle: max longitudinal force available at rear
        if F_y_r >= F_grip_r
            F_x_r_max = 0;
        else
            F_x_r_max = sqrt(F_grip_r^2 - F_y_r^2);
        end
        a_traction_max = F_x_r_max / car.mass;

        % Combine traction and engine limits
        a_long_new = min(a_engine, a_traction_max);
        a_long_new = max(a_long_new, 0);

        if abs(a_long_new - a_old) < 0.01
            a_long = a_long_new;
            break;
        end
        a_long = 0.5 * a_old + 0.5 * a_long_new;   % damping
    end

    % Record diagnostics
    a_long_forward(i) = a_long;
    dFz_forward(i)    = car.mass * a_long * car.h_cog / car.wheelbase;
    Fz_f_forward(i)   = car.weight_dist_f*car.mass*car.g + ...
                        car.aero_balance_f*car.aero_df_coeff*v_now^2 - dFz_forward(i);
    Fz_r_forward(i)   = car.weight_dist_r*car.mass*car.g + ...
                        (1-car.aero_balance_f)*car.aero_df_coeff*v_now^2 + dFz_forward(i);

    v_next_sq = v_now^2 + 2 * a_long * ds;
    v_forward(i+1) = min(sqrt(max(v_next_sq, 0)), v_corner(i+1));
end
gear_forward(n) = gear_forward(n-1);

fprintf('  Max speed:        %.1f km/h\n', max(v_forward)*3.6);
fprintf('  Max a_long:       %.2f m/s^2 (%.2f g)\n', ...
        max(a_long_forward), max(a_long_forward)/g_acc);
fprintf('  Max dFz (accel):  %.0f N\n', max(dFz_forward));

%% ========================================================================
%  5. PASS 3 — BACKWARD PASS (combined braking under brake-bias constraint)
%  ========================================================================
%  Both axles brake. Friction circle applied per axle. Brake bias
%  (car.brakes.bias_f) constrains the front/rear split.
%
%  Required front force = bias * F_brake_total (must be <= F_x_f_max)
%  Required rear  force = (1-bias) * F_brake_total (must be <= F_x_r_max)
%  => F_brake_total <= min( F_x_f_max / bias , F_x_r_max / (1-bias) )
%
%  a_brake is positive magnitude; sign on dFz_long is -a_brake (front loads).

fprintf('Pass 3: Backward pass (combined braking, brake-bias constrained)...\n');

v_backward      = zeros(n, 1);
a_brake_record  = zeros(n, 1);
dFz_backward    = zeros(n, 1);
v_backward(n)   = v_forward(n);
bias_f          = car.brakes.bias_f;

for i = n:-1:2
    v_now = v_backward(i);
    a_lat = v_now^2 * kappa(i);

    a_brake = 0;   % initial guess
    for it = 1:10
        a_old = a_brake;
        % braking transfers weight to FRONT, so dFz_long is negative
        dFz = -car.mass * a_brake * car.h_cog / car.wheelbase;

        [~, ~, ~, Fz_f, Fz_r, F_grip_f, F_grip_r] = get_axle_grip_v04(v_now, dFz, car);

        % Distribute lateral by Fz
        F_y_f = car.mass * a_lat * Fz_f / (Fz_f + Fz_r);
        F_y_r = car.mass * a_lat * Fz_r / (Fz_f + Fz_r);

        % Per-axle friction circle
        F_x_f_max = sqrt(max(F_grip_f^2 - F_y_f^2, 0));
        F_x_r_max = sqrt(max(F_grip_r^2 - F_y_r^2, 0));

        % Brake bias constraint
        F_brake_tot = min(F_x_f_max / bias_f, F_x_r_max / (1 - bias_f));
        a_brake_new = F_brake_tot / car.mass;

        if abs(a_brake_new - a_old) < 0.01
            a_brake = a_brake_new;
            break;
        end
        a_brake = 0.5 * a_old + 0.5 * a_brake_new;
    end

    a_brake_record(i) = a_brake;
    dFz_backward(i)   = -car.mass * a_brake * car.h_cog / car.wheelbase;

    v_prev_sq = v_now^2 + 2 * a_brake * ds;
    v_backward(i-1) = min(sqrt(v_prev_sq), v_forward(i-1));
end

fprintf('  Max a_brake:       %.2f m/s^2 (%.2f g)\n', ...
        max(a_brake_record), max(a_brake_record)/g_acc);
fprintf('  Max |dFz| (brake): %.0f N\n', max(abs(dFz_backward)));

%% ========================================================================
%  6. COMBINE + LAP TIME (initial)
%  ========================================================================

v_sim   = min(v_forward, v_backward);
v_sim   = min(v_sim, v_corner);
dt_sim  = ds ./ max(v_sim, 0.1);
lap_time = sum(dt_sim);

fprintf('\nInitial lap (before continuity iter): %.3f s\n', lap_time);

%% ========================================================================
%  7. LAP CONTINUITY ITERATION
%  ========================================================================

fprintf('\nLap continuity check:\n');
fprintf('  Start: %.2f km/h  End: %.2f km/h\n', v_sim(1)*3.6, v_sim(end)*3.6);

if abs(v_sim(end) - v_sim(1)) > 1.0
    fprintf('  >> Iterating...\n');
    for iter = 1:5
        v_forward(1) = v_sim(end);

        % --- Re-run forward pass ---
        for i = 1:n-1
            v_now = v_forward(i);
            a_lat = v_now^2 * kappa(i);
            F_drag = car.aero_drag_coeff * v_now^2;
            [F_drive, g_sel, ~] = get_drive_force(v_now, car);
            gear_forward(i) = g_sel;
            a_engine = (F_drive - F_drag) / car.mass;

            a_long = max(a_engine, 0);
            for it = 1:10
                a_old = a_long;
                dFz  = car.mass * a_long * car.h_cog / car.wheelbase;
                [~, ~, ~, Fz_f, Fz_r, ~, F_grip_r] = get_axle_grip_v04(v_now, dFz, car);
                F_y_r = car.mass * a_lat * Fz_r / (Fz_f + Fz_r);
                if F_y_r >= F_grip_r
                    F_x_r_max = 0;
                else
                    F_x_r_max = sqrt(F_grip_r^2 - F_y_r^2);
                end
                a_traction_max = F_x_r_max / car.mass;
                a_long_new = max(min(a_engine, a_traction_max), 0);
                if abs(a_long_new - a_old) < 0.01, a_long = a_long_new; break; end
                a_long = 0.5*a_old + 0.5*a_long_new;
            end

            a_long_forward(i) = a_long;
            dFz_forward(i)    = car.mass * a_long * car.h_cog / car.wheelbase;
            Fz_f_forward(i)   = car.weight_dist_f*car.mass*car.g + ...
                                car.aero_balance_f*car.aero_df_coeff*v_now^2 - dFz_forward(i);
            Fz_r_forward(i)   = car.weight_dist_r*car.mass*car.g + ...
                                (1-car.aero_balance_f)*car.aero_df_coeff*v_now^2 + dFz_forward(i);

            v_next_sq = v_now^2 + 2 * a_long * ds;
            v_forward(i+1) = min(sqrt(max(v_next_sq, 0)), v_corner(i+1));
        end
        gear_forward(n) = gear_forward(n-1);

        % --- Re-run backward pass ---
        v_backward(n) = v_forward(n);
        for i = n:-1:2
            v_now = v_backward(i);
            a_lat = v_now^2 * kappa(i);
            a_brake = 0;
            for it = 1:10
                a_old = a_brake;
                dFz = -car.mass * a_brake * car.h_cog / car.wheelbase;
                [~, ~, ~, Fz_f, Fz_r, F_grip_f, F_grip_r] = get_axle_grip_v04(v_now, dFz, car);
                F_y_f = car.mass * a_lat * Fz_f / (Fz_f + Fz_r);
                F_y_r = car.mass * a_lat * Fz_r / (Fz_f + Fz_r);
                F_x_f_max = sqrt(max(F_grip_f^2 - F_y_f^2, 0));
                F_x_r_max = sqrt(max(F_grip_r^2 - F_y_r^2, 0));
                F_brake_tot = min(F_x_f_max / bias_f, F_x_r_max / (1 - bias_f));
                a_brake_new = F_brake_tot / car.mass;
                if abs(a_brake_new - a_old) < 0.01, a_brake = a_brake_new; break; end
                a_brake = 0.5*a_old + 0.5*a_brake_new;
            end
            a_brake_record(i) = a_brake;
            dFz_backward(i)   = -car.mass * a_brake * car.h_cog / car.wheelbase;

            v_prev_sq = v_now^2 + 2 * a_brake * ds;
            v_backward(i-1) = min(sqrt(v_prev_sq), v_forward(i-1));
        end

        v_sim   = min(v_forward, v_backward);
        v_sim   = min(v_sim, v_corner);
        dt_sim  = ds ./ max(v_sim, 0.1);
        lap_time = sum(dt_sim);

        fprintf('  Iter %d: start=%.2f, end=%.2f km/h, lap=%.3f s\n', ...
                iter, v_sim(1)*3.6, v_sim(end)*3.6, lap_time);
        if abs(v_sim(end) - v_sim(1)) < 0.5
            fprintf('  Converged.\n');
            break;
        end
    end
end

t_cum   = cumsum(dt_sim);
lap_min = floor(lap_time / 60);
lap_sec = lap_time - lap_min * 60;

%% ========================================================================
%  8. FINAL RESULTS
%  ========================================================================

fprintf('\n================ v04 CONVERGED RESULTS =================\n');
fprintf('  v04 lap time:   %d:%06.3f\n', lap_min, lap_sec);
fprintf('  Reference:      %d:%06.3f\n', ...
        floor(track.meta.ref_laptime/60), ...
        track.meta.ref_laptime - floor(track.meta.ref_laptime/60)*60);
fprintf('  Delta vs ref:   %+.3f s (%+.2f%%)\n', ...
        lap_time - track.meta.ref_laptime, ...
        (lap_time - track.meta.ref_laptime) / track.meta.ref_laptime * 100);

fprintf('\n  --- Version comparison ---\n');
if has_v01
    fprintf('  v01 (point-mass):        %d:%06.3f  (delta vs ref: %+.1f s)\n', ...
            floor(sim_v01.lap_time/60), ...
            sim_v01.lap_time - floor(sim_v01.lap_time/60)*60, ...
            sim_v01.lap_time - track.meta.ref_laptime);
end
if has_v02
    fprintf('  v02 (+ aero):            %d:%06.3f  (delta vs ref: %+.1f s)\n', ...
            floor(sim02.lap_time/60), ...
            sim02.lap_time - floor(sim02.lap_time/60)*60, ...
            sim02.lap_time - track.meta.ref_laptime);
end
if has_v03
    fprintf('  v03 (+ load sens):       %d:%06.3f  (delta vs ref: %+.1f s)\n', ...
            floor(sim03.lap_time/60), ...
            sim03.lap_time - floor(sim03.lap_time/60)*60, ...
            sim03.lap_time - track.meta.ref_laptime);
end
fprintf('  v04 (+ weight transfer): %d:%06.3f  (delta vs ref: %+.1f s)\n', ...
        lap_min, lap_sec, lap_time - track.meta.ref_laptime);
if has_v03
    fprintf('\n  Weight-transfer cost (v04 - v03): %+.2f s\n', ...
            lap_time - sim03.lap_time);
    fprintf('  (positive = v04 slower = transfer reduces combined grip)\n');
end

fprintf('  Min speed:  %.1f km/h\n', min(v_sim)*3.6);
fprintf('  Max speed:  %.1f km/h\n', max(v_sim)*3.6);
fprintf('  Mean speed: %.1f km/h\n', mean(v_sim)*3.6);
fprintf('=========================================================\n');

%% ========================================================================
%  9. OUTPUT STRUCT
%  ========================================================================

sim04               = struct();
sim04.v             = v_sim;
sim04.v_kmh         = v_sim * 3.6;
sim04.v_corner      = v_corner;
sim04.v_forward     = v_forward;
sim04.v_backward    = v_backward;
sim04.gear          = gear_forward;
sim04.dt            = dt_sim;
sim04.t_cum         = t_cum;
sim04.lap_time      = lap_time;
sim04.dist          = track.dist;

% Per-axle diagnostics (forward pass)
sim04.a_long        = a_long_forward;
sim04.a_brake       = a_brake_record;
sim04.dFz_forward   = dFz_forward;
sim04.dFz_backward  = dFz_backward;
sim04.Fz_f_forward  = Fz_f_forward;
sim04.Fz_r_forward  = Fz_r_forward;

sim04.meta.model    = 'v04 point-mass + aero + load sens + per-axle + long. transfer';
sim04.meta.car      = car.meta.name;
sim04.meta.track    = track.meta.name;
sim04.meta.mu_0     = car.tyre.mu_0;
sim04.meta.k        = car.tyre.load_sens_k;
sim04.meta.h_cog    = car.h_cog;
sim04.meta.bias_f   = car.brakes.bias_f;
sim04.meta.created  = datestr(now, 'yyyy-mm-dd HH:MM');

%% ========================================================================
%  10. PLOTS — STRIPPED 2026-04-19
%  ========================================================================
%  Comparison plots + weight-transfer bonus figure removed to keep run
%  output lean during calibration. Everything needed to replot lives in
%  sim04 (v, v_forward, v_backward, v_corner, a_long, a_brake, dFz_*,
%  Fz_f_forward, Fz_r_forward). For focused brake-spike inspection use
%  04_correlation/diagnose_brake_v04.m.

fprintf('\n=== v04 (rewritten) Simulation Complete ===\n\n');
