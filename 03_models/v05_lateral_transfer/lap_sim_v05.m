%% lap_sim_v05.m — REWRITTEN 2026-04-21
%  Per-axle QSS lap sim with LOAD-SENSITIVE TYRES (v03) + PER-AXLE LOADS (v04)
%  + LONGITUDINAL WEIGHT TRANSFER (v04) + LATERAL WEIGHT TRANSFER (new in v05)
%  + ANTI-ROLL BAR REDISTRIBUTION (new in v05).
%
%  HISTORY: the earlier files 03_models/v05_bicycle/lap_sim_v05.m and
%  03_models/v05_refined/lap_sim_v05_refined.m (both dated 2026-04-18) carried
%  eleven physics bugs between them, including a double-counted drag term, a
%  regression of passes 2 and 3 back to pre-v04 grip (no friction circle, no
%  brake-bias constraint), and an ARB model that *reduced* total lateral load
%  transfer rather than redistributing it. Those files are retired; this
%  rewrite is the canonical v05. See logbook Entry 016 for the full bug list.
%
%  WHAT CHANGED FROM v04:
%    v04: per-axle grip with longitudinal transfer only. Both tyres on an
%         axle are assumed equally loaded (Fz_axle/2 each).
%    v05: each axle is now split into OUTSIDE and INSIDE tyres under lateral
%         acceleration. Load migrates from inside to outside; the outside
%         tyre carries more Fz and therefore (via load-sensitive μ) less
%         grip per newton. Axle grip is the sum of the two tyres'
%         contributions, which drops quadratically with lateral transfer.
%
%  LATERAL TRANSFER PHYSICS:
%    Total lateral load transfer at lateral acceleration a_lat:
%        ΔFz_lat_total = m * a_lat * h_cog / t_avg,   t_avg = (t_f + t_r)/2
%    This total is a rigid-body consequence of the CG being above ground —
%    it cannot be reduced by any suspension choice. ARBs set how the total
%    *redistributes* between the front and rear axles:
%        ΔFz_lat_f = ΔFz_lat_total * roll_dist_f
%        ΔFz_lat_r = ΔFz_lat_total * roll_dist_r
%    with  roll_dist_f = K_roll_f / (K_roll_f + K_roll_r)
%    and   K_roll_axle = K_ARB_axle + K_tire_axle.
%    ΔFz_lat_axle is the amount that moves from the inside tyre to the
%    outside tyre on that axle:
%        Fz_axle_out = Fz_axle_base/2 + ΔFz_lat_axle
%        Fz_axle_in  = Fz_axle_base/2 - ΔFz_lat_axle
%
%  SIGN CONVENTION (unchanged from v04):
%    dFz_long > 0  =>  weight shifted TO rear (accel)
%    dFz_long < 0  =>  weight shifted TO front (braking)
%    a_lat is a MAGNITUDE — sign of cornering (left/right) is irrelevant
%    because outside/inside labels are symmetric.
%
%  INPUTS (must be in workspace):
%    car   — vehicle parameters (from startup_project; needs
%            car.suspension.roll_dist_f/_r from the 2026-04-21 ARB rewrite)
%    track — track data (from build_track)
%
%  OUTPUT:
%    sim05 — struct with results + per-tyre diagnostics
%
%  Author:  Jaykumar Patil
%  Rewrite: 2026-04-21

%% ========================================================================
%  0. INPUT CHECKS & STORE PREVIOUS RESULTS
%  ========================================================================

if ~exist('car', 'var')
    error('Car parameters not loaded. Run startup_project first.');
end
if ~exist('track', 'var')
    error('Track data not loaded. Run build_track first.');
end
if ~isfield(car, 'suspension') || ~isfield(car.suspension, 'roll_dist_f')
    error(['car.suspension.roll_dist_f is missing. The ARB model was ' ...
           'rewritten on 2026-04-21. Re-run startup_project to pick up ' ...
           'the new amg_gt3_params.m.']);
end

fprintf('\n=== Lap Sim v05 (rewritten) — Per-Axle + Long. + Lateral Transfer + ARBs ===\n');
fprintf('Car:   %s\n', car.meta.name);
fprintf('Track: %s (%.3f km)\n', track.meta.name, track.length/1000);
fprintf('Roll distribution (F/R): %.1f%% / %.1f%%\n', ...
        car.suspension.roll_dist_f*100, car.suspension.roll_dist_r*100);

% Store previous-version results for comparison
has_v01 = exist('sim',   'var') == 1;
has_v02 = exist('sim02', 'var') == 1;
has_v03 = exist('sim03', 'var') == 1;
has_v04 = exist('sim04', 'var') == 1;
if has_v01, sim_v01 = sim; end

%% ========================================================================
%  1. HELPER: ENGINE FORCE (unchanged from v01/v02/v03/v04)
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
%  2. HELPER: PER-AXLE GRIP WITH LATERAL TRANSFER (NEW IN v05)
%  ========================================================================
%  Given speed v [m/s], signed longitudinal acceleration a_long [m/s²]
%  (positive = accel, negative = brake), and lateral acceleration magnitude
%  a_lat [m/s²], return:
%    Fz_f, Fz_r            per-axle total normal loads            [N]
%    Fz_f_out, Fz_f_in     per-tyre front normal loads            [N]
%    Fz_r_out, Fz_r_in     per-tyre rear  normal loads            [N]
%    mu_f_out, mu_r_out    per-tyre outside μ (load-sensitive)    [-]
%    F_grip_f, F_grip_r    per-axle grip magnitudes = Σ μ_i·Fz_i  [N]
%    a_grip_tot            (F_grip_f + F_grip_r) / m              [m/s²]
%
%  Internals (in order):
%    1. Static + aero per-axle base load (same formulae as v04).
%    2. Longitudinal transfer shifts the front/rear base split.
%    3. Lateral transfer total ΔFz_lat = m·a_lat·h_cog/t_avg is split by
%       roll_dist_f/r (see amg_gt3_params.m §8). ΔFz_lat_axle is the shift
%       from the inside tyre to the outside tyre on that axle.
%    4. Per-tyre μ uses load-sensitive formula mu_0 - k·Fz_tyre. The
%       load_sens_k was calibrated against per-tyre Fz (see logbook Entry
%       007 / v03 calibration).
%    5. Axle grip = outside contribution + inside contribution. Under zero
%       lateral transfer this reduces to v04's 2·μ·(Fz_axle/2) = μ·Fz_axle.

    function [F_grip_f, F_grip_r, Fz_f, Fz_r, ...
              Fz_f_out, Fz_f_in, Fz_r_out, Fz_r_in, ...
              mu_f_out, mu_f_in, mu_r_out, mu_r_in] = ...
            get_axle_grip_v05(v, a_long, a_lat, car)

        % ---- 1. Static + aero per-axle base ----
        F_df        = car.aero_df_coeff * v^2;
        Fz_f_static = car.mass * car.g * car.weight_dist_f;
        Fz_r_static = car.mass * car.g * car.weight_dist_r;
        Fz_f_aero   = F_df * car.aero_balance_f;
        Fz_r_aero   = F_df * (1 - car.aero_balance_f);

        % ---- 2. Longitudinal transfer (same as v04) ----
        dFz_long  = car.mass * a_long * car.h_cog / car.wheelbase;
        Fz_f_base = Fz_f_static + Fz_f_aero - dFz_long;
        Fz_r_base = Fz_r_static + Fz_r_aero + dFz_long;

        % Physical floor on axle base (a lifted axle still has a tiny load)
        Fz_f_base = max(Fz_f_base, 200);
        Fz_r_base = max(Fz_r_base, 200);

        % ---- 3. Lateral transfer (total split by roll stiffness) ----
        t_avg         = 0.5 * (car.track_f + car.track_r);
        dFz_lat_total = car.mass * a_lat * car.h_cog / t_avg;
        dFz_lat_f     = dFz_lat_total * car.suspension.roll_dist_f;
        dFz_lat_r     = dFz_lat_total * car.suspension.roll_dist_r;

        % Per-tyre loads (outside/inside, applying the axle shift).
        Fz_f_out = Fz_f_base/2 + dFz_lat_f;
        Fz_f_in  = Fz_f_base/2 - dFz_lat_f;
        Fz_r_out = Fz_r_base/2 + dFz_lat_r;
        Fz_r_in  = Fz_r_base/2 - dFz_lat_r;

        % Physical floor per-tyre — an inside tyre may lift in extreme
        % cornering but never generates negative vertical force.
        Fz_f_out = max(Fz_f_out, 50);
        Fz_f_in  = max(Fz_f_in,  50);
        Fz_r_out = max(Fz_r_out, 50);
        Fz_r_in  = max(Fz_r_in,  50);

        % ---- 4. Per-tyre load-sensitive μ ----
        mu_f_out = max(car.tyre.mu_0 - car.tyre.load_sens_k * Fz_f_out, 0.5);
        mu_f_in  = max(car.tyre.mu_0 - car.tyre.load_sens_k * Fz_f_in,  0.5);
        mu_r_out = max(car.tyre.mu_0 - car.tyre.load_sens_k * Fz_r_out, 0.5);
        mu_r_in  = max(car.tyre.mu_0 - car.tyre.load_sens_k * Fz_r_in,  0.5);

        % ---- 5. Axle grip = sum of two tyres on that axle ----
        F_grip_f = mu_f_out * Fz_f_out + mu_f_in * Fz_f_in;
        F_grip_r = mu_r_out * Fz_r_out + mu_r_in * Fz_r_in;

        % Axle totals (used by passes 2/3 for lateral distribution)
        Fz_f = Fz_f_out + Fz_f_in;
        Fz_r = Fz_r_out + Fz_r_in;
    end

%% ========================================================================
%  3. PASS 1 — CORNERING SPEED LIMIT (iterative, lateral transfer active)
%  ========================================================================
%  Steady-state cornering: a_long = 0, so dFz_long = 0. But a_lat = v²·κ is
%  not zero, so lateral transfer IS active and shrinks axle grip. Iterate
%  because a_lat depends on v, and F_grip depends on both a_lat and v
%  (downforce, per-tyre Fz).
%
%  Constraint: m·v²·κ = F_grip_f(v, 0, a_lat) + F_grip_r(v, 0, a_lat)
%  → v²      = (F_grip_f + F_grip_r) / (m·κ)
%
%  The outside/inside split inside get_axle_grip_v05 means F_grip drops as
%  a_lat grows — this is the correct "tyre load sensitivity under roll"
%  behaviour that costs cornering speed in real cars.

fprintf('\nPass 1: Cornering speed limits (per-axle, load-sensitive, lateral transfer)...\n');

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

    % Initial guess: v04-style (long transfer = 0, lateral transfer = 0)
    [F_grip_f_g, F_grip_r_g] = get_axle_grip_v05(v_max_cap/2, 0, 0, car);
    a_guess = (F_grip_f_g + F_grip_r_g) / car.mass;
    v_iter  = min(sqrt(a_guess / kappa(i)), v_max_cap);

    % Fixed-point iteration
    for j = 1:30
        a_lat_iter = v_iter^2 * kappa(i);
        [F_grip_f, F_grip_r] = get_axle_grip_v05(v_iter, 0, a_lat_iter, car);
        a_grip_tot = (F_grip_f + F_grip_r) / car.mass;
        v_new      = sqrt(a_grip_tot / kappa(i));
        v_new      = min(v_new, v_max_cap);
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

% Diagnostic: grip at representative speeds with zero long transfer
fprintf('  Grip diagnostic (dFz_long = 0, κ = 0.02):\n');
for speed_test = [80, 150, 200, 260]
    v_ms   = speed_test / 3.6;
    a_lat  = v_ms^2 * 0.02;
    [F_f, F_r, ~, ~, Fz_f_o, Fz_f_i, Fz_r_o, Fz_r_i, mu_fo, ~, mu_ro] = ...
        get_axle_grip_v05(v_ms, 0, a_lat, car);
    a_g = (F_f + F_r) / car.mass;
    fprintf(['    @ %3d km/h: μ_f_out=%.3f μ_r_out=%.3f  Fz_f_out/in=%4.0f/%4.0f N ' ...
             'Fz_r_out/in=%4.0f/%4.0f N  a_grip=%.2f m/s²\n'], ...
            speed_test, mu_fo, mu_ro, Fz_f_o, Fz_f_i, Fz_r_o, Fz_r_i, a_g);
end

%% ========================================================================
%  4. PASS 2 — FORWARD PASS (RWD traction + friction circle + both transfers)
%  ========================================================================
%  Same skeleton as v04: RWD, rear axle lateral share ∝ Fz, rear friction
%  circle sets longitudinal capacity.
%
%  Difference from v04: at each point i, a_lat = v_now²·κ(i) is non-zero,
%  so get_axle_grip_v05 now includes lateral transfer. The rear-axle grip
%  envelope F_grip_r shrinks with lateral, which reduces available
%  traction in fast corners — exactly the real-car effect.
%
%  Inner iteration couples a_long ↔ dFz_long (same as v04). a_lat is known
%  from v_now at each step so it does NOT need iteration.

fprintf('Pass 2: Forward pass (RWD, friction circle on rear, lateral transfer)...\n');

v_forward       = zeros(n, 1);
gear_forward    = zeros(n, 1);
a_long_forward  = zeros(n, 1);
a_lat_forward   = zeros(n, 1);
dFz_forward     = zeros(n, 1);
dFz_lat_forward = zeros(n, 1);
Fz_f_forward    = zeros(n, 1);
Fz_r_forward    = zeros(n, 1);
v_forward(1)    = v_corner(1);

for i = 1:n-1
    v_now  = v_forward(i);
    a_lat  = v_now^2 * kappa(i);
    F_drag = car.aero_drag_coeff * v_now^2;

    [F_drive, g_sel, ~] = get_drive_force(v_now, car);
    gear_forward(i)     = g_sel;
    a_engine            = (F_drive - F_drag) / car.mass;

    % Iterate on a_long
    a_long = max(a_engine, 0);
    for it = 1:10
        a_old = a_long;

        [~, F_grip_r, Fz_f, Fz_r] = get_axle_grip_v05(v_now, a_long, a_lat, car);

        % Rear lateral force share (proportional to axle Fz, same QSS
        % approximation as v04 — exact only if μ_f = μ_r)
        F_y_r = car.mass * a_lat * Fz_r / (Fz_f + Fz_r);

        % Rear friction circle: longitudinal capacity after lateral
        if F_y_r >= F_grip_r
            F_x_r_max = 0;
        else
            F_x_r_max = sqrt(F_grip_r^2 - F_y_r^2);
        end
        a_traction_max = F_x_r_max / car.mass;

        a_long_new = max(min(a_engine, a_traction_max), 0);
        if abs(a_long_new - a_old) < 0.01
            a_long = a_long_new;
            break;
        end
        a_long = 0.5 * a_old + 0.5 * a_long_new;
    end

    % Diagnostics at this point
    [~, ~, Fz_f, Fz_r] = get_axle_grip_v05(v_now, a_long, a_lat, car);
    t_avg              = 0.5*(car.track_f + car.track_r);
    a_long_forward(i)  = a_long;
    a_lat_forward(i)   = a_lat;
    dFz_forward(i)     = car.mass * a_long * car.h_cog / car.wheelbase;
    dFz_lat_forward(i) = car.mass * a_lat  * car.h_cog / t_avg;
    Fz_f_forward(i)    = Fz_f;
    Fz_r_forward(i)    = Fz_r;

    v_next_sq       = v_now^2 + 2 * a_long * ds;
    v_forward(i+1)  = min(sqrt(max(v_next_sq, 0)), v_corner(i+1));
end
gear_forward(n) = gear_forward(n-1);

fprintf('  Max speed:         %.1f km/h\n', max(v_forward)*3.6);
fprintf('  Max a_long:        %.2f m/s² (%.2f g)\n', ...
        max(a_long_forward), max(a_long_forward)/g_acc);
fprintf('  Max dFz_long:      %.0f N\n', max(dFz_forward));
fprintf('  Max dFz_lat_total: %.0f N\n', max(dFz_lat_forward));

%% ========================================================================
%  5. PASS 3 — BACKWARD PASS (combined braking, brake-bias, lateral transfer)
%  ========================================================================
%  Brake bias constraint (from v04):
%    required front force = bias_f·F_brake_tot   (must be ≤ F_x_f_max)
%    required rear  force = (1−bias_f)·F_brake_tot   (must be ≤ F_x_r_max)
%    ⇒ F_brake_tot ≤ min( F_x_f_max/bias_f , F_x_r_max/(1−bias_f) )
%
%  Lateral transfer is active when braking into a corner (a_lat > 0), which
%  shrinks both axles' grip envelopes and therefore F_x_f_max/F_x_r_max.
%  This is why v05 braking under trail-braking should be more conservative
%  than v04 at the same bias.

fprintf('Pass 3: Backward pass (combined braking, brake-bias, lateral transfer)...\n');

v_backward       = zeros(n, 1);
a_brake_record   = zeros(n, 1);
a_lat_backward   = zeros(n, 1);
dFz_backward     = zeros(n, 1);
dFz_lat_backward = zeros(n, 1);
v_backward(n)    = v_forward(n);
bias_f           = car.brakes.bias_f;

for i = n:-1:2
    v_now = v_backward(i);
    a_lat = v_now^2 * kappa(i);

    a_brake = 0;
    for it = 1:10
        a_old = a_brake;
        % braking → a_long is NEGATIVE in the signed convention
        [F_grip_f, F_grip_r, Fz_f, Fz_r] = ...
            get_axle_grip_v05(v_now, -a_brake, a_lat, car);

        % Lateral share per axle (∝ Fz)
        F_y_f = car.mass * a_lat * Fz_f / (Fz_f + Fz_r);
        F_y_r = car.mass * a_lat * Fz_r / (Fz_f + Fz_r);

        % Per-axle longitudinal capacity after lateral
        F_x_f_max = sqrt(max(F_grip_f^2 - F_y_f^2, 0));
        F_x_r_max = sqrt(max(F_grip_r^2 - F_y_r^2, 0));

        % Brake-bias constraint (same as v04)
        F_brake_tot  = min(F_x_f_max / bias_f, F_x_r_max / (1 - bias_f));
        a_brake_new  = F_brake_tot / car.mass;

        if abs(a_brake_new - a_old) < 0.01
            a_brake = a_brake_new;
            break;
        end
        a_brake = 0.5 * a_old + 0.5 * a_brake_new;
    end

    t_avg               = 0.5*(car.track_f + car.track_r);
    a_brake_record(i)   = a_brake;
    a_lat_backward(i)   = a_lat;
    dFz_backward(i)     = -car.mass * a_brake * car.h_cog / car.wheelbase;
    dFz_lat_backward(i) = car.mass * a_lat   * car.h_cog / t_avg;

    v_prev_sq         = v_now^2 + 2 * a_brake * ds;
    v_backward(i-1)   = min(sqrt(v_prev_sq), v_forward(i-1));
end

fprintf('  Max a_brake:       %.2f m/s² (%.2f g)\n', ...
        max(a_brake_record), max(a_brake_record)/g_acc);
fprintf('  Max |dFz_long|:    %.0f N\n', max(abs(dFz_backward)));

%% ========================================================================
%  6. COMBINE + INITIAL LAP TIME
%  ========================================================================

v_sim    = min(v_forward, v_backward);
v_sim    = min(v_sim, v_corner);
dt_sim   = ds ./ max(v_sim, 0.1);
lap_time = sum(dt_sim);

fprintf('\nInitial lap (before continuity iter): %.3f s\n', lap_time);

%% ========================================================================
%  7. LAP CONTINUITY ITERATION
%  ========================================================================
%  Same outer loop as v04: re-run forward with the backward-end speed,
%  then re-run backward, until v_sim(1) ≈ v_sim(end).

fprintf('\nLap continuity check:\n');
fprintf('  Start: %.2f km/h   End: %.2f km/h\n', v_sim(1)*3.6, v_sim(end)*3.6);

if abs(v_sim(end) - v_sim(1)) > 1.0
    fprintf('  >> Iterating...\n');
    for iter = 1:5
        v_forward(1) = v_sim(end);

        % --- Re-run forward pass ---
        for i = 1:n-1
            v_now  = v_forward(i);
            a_lat  = v_now^2 * kappa(i);
            F_drag = car.aero_drag_coeff * v_now^2;
            [F_drive, g_sel, ~] = get_drive_force(v_now, car);
            gear_forward(i) = g_sel;
            a_engine        = (F_drive - F_drag) / car.mass;

            a_long = max(a_engine, 0);
            for it = 1:10
                a_old = a_long;
                [~, F_grip_r, Fz_f, Fz_r] = ...
                    get_axle_grip_v05(v_now, a_long, a_lat, car);
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

            [~, ~, Fz_f, Fz_r] = get_axle_grip_v05(v_now, a_long, a_lat, car);
            t_avg              = 0.5*(car.track_f + car.track_r);
            a_long_forward(i)  = a_long;
            a_lat_forward(i)   = a_lat;
            dFz_forward(i)     = car.mass * a_long * car.h_cog / car.wheelbase;
            dFz_lat_forward(i) = car.mass * a_lat  * car.h_cog / t_avg;
            Fz_f_forward(i)    = Fz_f;
            Fz_r_forward(i)    = Fz_r;

            v_next_sq       = v_now^2 + 2 * a_long * ds;
            v_forward(i+1)  = min(sqrt(max(v_next_sq, 0)), v_corner(i+1));
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
                [F_grip_f, F_grip_r, Fz_f, Fz_r] = ...
                    get_axle_grip_v05(v_now, -a_brake, a_lat, car);
                F_y_f = car.mass * a_lat * Fz_f / (Fz_f + Fz_r);
                F_y_r = car.mass * a_lat * Fz_r / (Fz_f + Fz_r);
                F_x_f_max = sqrt(max(F_grip_f^2 - F_y_f^2, 0));
                F_x_r_max = sqrt(max(F_grip_r^2 - F_y_r^2, 0));
                F_brake_tot = min(F_x_f_max / bias_f, F_x_r_max / (1 - bias_f));
                a_brake_new = F_brake_tot / car.mass;
                if abs(a_brake_new - a_old) < 0.01, a_brake = a_brake_new; break; end
                a_brake = 0.5*a_old + 0.5*a_brake_new;
            end
            t_avg               = 0.5*(car.track_f + car.track_r);
            a_brake_record(i)   = a_brake;
            a_lat_backward(i)   = a_lat;
            dFz_backward(i)     = -car.mass * a_brake * car.h_cog / car.wheelbase;
            dFz_lat_backward(i) = car.mass * a_lat   * car.h_cog / t_avg;

            v_prev_sq       = v_now^2 + 2 * a_brake * ds;
            v_backward(i-1) = min(sqrt(v_prev_sq), v_forward(i-1));
        end

        v_sim    = min(v_forward, v_backward);
        v_sim    = min(v_sim, v_corner);
        dt_sim   = ds ./ max(v_sim, 0.1);
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

fprintf('\n================ v05 CONVERGED RESULTS =================\n');
fprintf('  v05 lap time:   %d:%06.3f\n', lap_min, lap_sec);
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
if has_v04
    fprintf('  v04 (+ long. transfer):  %d:%06.3f  (delta vs ref: %+.1f s)\n', ...
            floor(sim04.lap_time/60), ...
            sim04.lap_time - floor(sim04.lap_time/60)*60, ...
            sim04.lap_time - track.meta.ref_laptime);
end
fprintf('  v05 (+ lateral + ARBs):  %d:%06.3f  (delta vs ref: %+.1f s)\n', ...
        lap_min, lap_sec, lap_time - track.meta.ref_laptime);
if has_v04
    fprintf('\n  Lateral transfer cost (v05 - v04): %+.2f s\n', ...
            lap_time - sim04.lap_time);
    fprintf('  (positive = v05 slower = lateral transfer shrinks grip envelope)\n');
end

fprintf('  Min speed:  %.1f km/h\n', min(v_sim)*3.6);
fprintf('  Max speed:  %.1f km/h\n', max(v_sim)*3.6);
fprintf('  Mean speed: %.1f km/h\n', mean(v_sim)*3.6);
fprintf('=========================================================\n');

%% ========================================================================
%  9. OUTPUT STRUCT
%  ========================================================================

sim05                  = struct();
sim05.v                = v_sim;
sim05.v_kmh            = v_sim * 3.6;
sim05.v_corner         = v_corner;
sim05.v_forward        = v_forward;
sim05.v_backward       = v_backward;
sim05.gear             = gear_forward;
sim05.dt               = dt_sim;
sim05.t_cum            = t_cum;
sim05.lap_time         = lap_time;
sim05.dist             = track.dist;

% Per-axle / per-tyre diagnostics (forward pass)
sim05.a_long           = a_long_forward;
sim05.a_brake          = a_brake_record;
sim05.a_lat_forward    = a_lat_forward;
sim05.a_lat_backward   = a_lat_backward;
sim05.dFz_forward      = dFz_forward;
sim05.dFz_backward     = dFz_backward;
sim05.dFz_lat_forward  = dFz_lat_forward;
sim05.dFz_lat_backward = dFz_lat_backward;
sim05.Fz_f_forward     = Fz_f_forward;
sim05.Fz_r_forward     = Fz_r_forward;

sim05.meta.model       = 'v05 per-axle + aero + load sens + long. transfer + lateral transfer + ARBs';
sim05.meta.car         = car.meta.name;
sim05.meta.track       = track.meta.name;
sim05.meta.mu_0        = car.tyre.mu_0;
sim05.meta.k           = car.tyre.load_sens_k;
sim05.meta.h_cog       = car.h_cog;
sim05.meta.bias_f      = car.brakes.bias_f;
sim05.meta.roll_dist_f = car.suspension.roll_dist_f;
sim05.meta.roll_dist_r = car.suspension.roll_dist_r;
sim05.meta.created     = datestr(now, 'yyyy-mm-dd HH:MM');

%% ========================================================================
%  10. PLOTS — STRIPPED (same policy as v04)
%  ========================================================================
%  Comparison plots removed to keep run output lean. Everything needed to
%  replot lives in sim05 (v, v_forward, v_backward, v_corner, a_long,
%  a_brake, a_lat_*, dFz_*, dFz_lat_*, Fz_f_forward, Fz_r_forward). For
%  focused per-corner inspection, use 04_correlation/diagnose_brake_v04.m
%  (works on sim05 because the struct shape is compatible on the fields it
%  reads) or a future diagnose_lateral_v05.m.

fprintf('\n=== v05 (rewritten) Simulation Complete ===\n\n');
