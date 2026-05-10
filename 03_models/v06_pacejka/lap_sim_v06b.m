%% lap_sim_v06b.m — PACEJKA MF 5.2 + FULL G-FACTOR COMBINED-SLIP COUPLING
%  Same as lap_sim_v06.m (v06.0) except Pass 2 and Pass 3 use the actual
%  Pacejka G-factor envelope instead of the friction-circle approximation
%  `F_x_max = sqrt(F_grip² − F_y²)`. The envelope is implemented as a
%  lookup table (pacejka_envelope.m) for runtime efficiency.
%
%  WHAT CHANGED FROM v06.0
%  -----------------------
%  v06.0 Pass 2 (drive):
%      F_x_r_max = sqrt(F_grip_r² − F_y_r²)
%      a_traction_max = F_x_r_max / m
%
%  v06.0b Pass 2 (drive):
%      F_x_r_max = pacejka_envelope(F_y_r, Fz_r_per_tyre, p_rear) * 2
%      a_traction_max = F_x_r_max / m
%
%  Same substitution in Pass 3 for the per-axle braking grip. The
%  `*2` is because `pacejka_envelope` operates on a single tyre and the
%  axle has two tyres in parallel.
%
%  EXPECTED LAP-TIME IMPACT
%  ------------------------
%  Per Entry 021's prediction: 0.2–0.5 s. Friction circle and Pacejka
%  G-factor envelopes have similar ceilings — they differ in shape, with
%  Pacejka being slightly less optimistic at the cardinal axes (low Fy
%  with high Fx, or vice versa) and slightly more generous at diagonal
%  operating points.
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-22  (v06 Pacejka — Phase 6.5, v06.0b)

%% ========================================================================
%  0. INPUT CHECKS
%  ========================================================================
if ~exist('car',   'var'),  error('car not loaded. Run startup_project.'); end
if ~exist('track', 'var'),  error('track not loaded. Run build_track.');   end
if ~isfield(car.tyre, 'pacejka')
    pacejka_coeffs_pirelli_dhf_gt3;
end
if ~isfield(car.suspension, 'roll_dist_f')
    error('car.suspension.roll_dist_f missing — re-run startup_project.');
end

fprintf('\n=== Lap Sim v06.0b — Pacejka MF 5.2 + G-factor envelope ===\n');
fprintf('Car:   %s\n', car.meta.name);
fprintf('Track: %s (%.3f km)\n', track.meta.name, track.length/1000);
fprintf('Roll distribution (F/R): %.1f%% / %.1f%%\n', ...
        car.suspension.roll_dist_f*100, car.suspension.roll_dist_r*100);
fprintf('Tyre model: %s + Pacejka G-factor envelope\n', car.tyre.pacejka.front.version);

has_v05 = exist('sim05', 'var') == 1;
has_v06 = exist('sim06', 'var') == 1;

%% ========================================================================
%  1. ENGINE FORCE HELPER (unchanged from v05/v06)
%  ========================================================================
    function [F_drive, gear_selected, rpm_selected] = get_drive_force(v, car)
        F_best = 0;  gear_selected = 1;  rpm_selected = 0;
        for g = 1:car.gearbox.n_gears
            rpm_g = v * car.gearbox.total_ratio(g) / car.tyre.rolling_radius ...
                    * (60 / (2*pi));
            if rpm_g < car.engine.rpm_idle || rpm_g > car.engine.rpm_max, continue; end
            T_engine = interp1(car.engine.rpm, car.engine.torque, rpm_g, ...
                               'linear', 'extrap');
            F_wheel = T_engine * car.gearbox.total_ratio(g) * car.gearbox.efficiency ...
                      / car.tyre.rolling_radius;
            if F_wheel > F_best
                F_best = F_wheel;  gear_selected = g;  rpm_selected = rpm_g;
            end
        end
        F_drive = max(F_best, 0);
    end

%% ========================================================================
%  2. PASS 1 — CORNERING SPEED LIMIT (Pacejka peak grip; unchanged from v06.0)
%  ========================================================================
fprintf('\nPass 1: Cornering speed limits (Pacejka peak grip per tyre)...\n');

g_acc     = car.g;
n         = track.n;
ds        = track.ds;
kappa     = track.kappa;
v_max_cap = 400 / 3.6;

v_corner          = zeros(n, 1);
alpha_pk_f_corner = zeros(n, 1);
alpha_pk_r_corner = zeros(n, 1);

for ii = 1:n
    if kappa(ii) < 1e-6
        v_corner(ii) = v_max_cap;
        continue;
    end
    g0 = get_axle_grip_v06(v_max_cap/2, 0, 0, car);
    a_guess = (g0.F_grip_f + g0.F_grip_r) / car.mass;
    v_iter  = min(sqrt(a_guess / kappa(ii)), v_max_cap);

    for jj = 1:30
        a_lat_iter = v_iter^2 * kappa(ii);
        gv = get_axle_grip_v06(v_iter, 0, a_lat_iter, car);
        a_grip_tot = (gv.F_grip_f + gv.F_grip_r) / car.mass;
        v_new = min(sqrt(a_grip_tot / kappa(ii)), v_max_cap);
        if abs(v_new - v_iter) < 0.01, v_iter = v_new; break; end
        v_iter = 0.5*v_iter + 0.5*v_new;
    end
    v_corner(ii) = v_iter;
    g_op = get_axle_grip_v06(v_iter, 0, v_iter^2*kappa(ii), car);
    alpha_pk_f_corner(ii) = mean([g_op.alpha_peak_f_out, g_op.alpha_peak_f_in]);
    alpha_pk_r_corner(ii) = mean([g_op.alpha_peak_r_out, g_op.alpha_peak_r_in]);
end

fprintf('  Min cornering speed: %.1f km/h\n', min(v_corner)*3.6);
fprintf('  Mean peak α (front / rear): %.1f° / %.1f°\n', ...
        mean(alpha_pk_f_corner(kappa>1e-3))*180/pi, ...
        mean(alpha_pk_r_corner(kappa>1e-3))*180/pi);

%% ========================================================================
%  3. PASS 2 — FORWARD PASS (RWD, Pacejka G-factor envelope on REAR axle)
%  ========================================================================
fprintf('Pass 2: Forward pass (Pacejka G-factor envelope, rear axle)...\n');

p_rear = car.tyre.pacejka.rear;

v_forward       = zeros(n, 1);
gear_forward    = zeros(n, 1);
a_long_forward  = zeros(n, 1);
a_lat_forward   = zeros(n, 1);
dFz_forward     = zeros(n, 1);
dFz_lat_forward = zeros(n, 1);
Fz_f_forward    = zeros(n, 1);
Fz_r_forward    = zeros(n, 1);
v_forward(1)    = v_corner(1);

for ii = 1:n-1
    v_now  = v_forward(ii);
    a_lat  = v_now^2 * kappa(ii);
    F_drag = car.aero_drag_coeff * v_now^2;

    [F_drive, g_sel, ~] = get_drive_force(v_now, car);
    gear_forward(ii) = g_sel;
    a_engine = (F_drive - F_drag) / car.mass;

    a_long = max(a_engine, 0);
    for it = 1:10
        a_old = a_long;
        gv = get_axle_grip_v06(v_now, a_long, a_lat, car);

        % Required rear lateral force (∝ Fz, same QSS approx as v05/v06)
        F_y_r_axle = car.mass * a_lat * gv.Fz_r / (gv.Fz_f + gv.Fz_r);

        % Per-tyre rear lateral demand and per-tyre Fz
        Fz_r_avg     = 0.5 * (gv.Fz_r_out + gv.Fz_r_in);   % per-tyre proxy
        F_y_r_tyre   = 0.5 * F_y_r_axle;

        % Pacejka envelope: max |Fx| per rear tyre at this (Fy, Fz)
        F_x_r_tyre_max = pacejka_envelope(F_y_r_tyre, Fz_r_avg, p_rear);
        F_x_r_max      = 2 * F_x_r_tyre_max;             % two tyres on axle

        a_traction_max = F_x_r_max / car.mass;
        a_long_new = max(min(a_engine, a_traction_max), 0);
        if abs(a_long_new - a_old) < 0.01, a_long = a_long_new; break; end
        a_long = 0.5*a_old + 0.5*a_long_new;
    end

    g_diag = get_axle_grip_v06(v_now, a_long, a_lat, car);
    t_avg               = 0.5*(car.track_f + car.track_r);
    a_long_forward(ii)  = a_long;
    a_lat_forward(ii)   = a_lat;
    dFz_forward(ii)     = car.mass * a_long * car.h_cog / car.wheelbase;
    dFz_lat_forward(ii) = car.mass * a_lat  * car.h_cog / t_avg;
    Fz_f_forward(ii)    = g_diag.Fz_f;
    Fz_r_forward(ii)    = g_diag.Fz_r;

    v_next_sq        = v_now^2 + 2 * a_long * ds;
    v_forward(ii+1)  = min(sqrt(max(v_next_sq, 0)), v_corner(ii+1));
end
gear_forward(n) = gear_forward(n-1);

fprintf('  Max speed:   %.1f km/h\n', max(v_forward)*3.6);
fprintf('  Max a_long:  %.2f m/s² (%.2f g)\n', ...
        max(a_long_forward), max(a_long_forward)/g_acc);

%% ========================================================================
%  4. PASS 3 — BACKWARD PASS (combined braking, Pacejka G-factor envelope)
%  ========================================================================
fprintf('Pass 3: Backward pass (Pacejka G-factor envelope, brake-bias min)...\n');

p_front = car.tyre.pacejka.front;

v_backward       = zeros(n, 1);
a_brake_record   = zeros(n, 1);
a_lat_backward   = zeros(n, 1);
dFz_backward     = zeros(n, 1);
dFz_lat_backward = zeros(n, 1);
v_backward(n)    = v_forward(n);
bias_f           = car.brakes.bias_f;

for ii = n:-1:2
    v_now = v_backward(ii);
    a_lat = v_now^2 * kappa(ii);

    a_brake = 0;
    for it = 1:10
        a_old = a_brake;
        gv = get_axle_grip_v06(v_now, -a_brake, a_lat, car);

        F_y_f_axle = car.mass * a_lat * gv.Fz_f / (gv.Fz_f + gv.Fz_r);
        F_y_r_axle = car.mass * a_lat * gv.Fz_r / (gv.Fz_f + gv.Fz_r);

        % Per-tyre demand and per-tyre Fz (axle has two tyres, equal share
        % approximated by axle/2 — same as Pass 2)
        Fz_f_avg = 0.5 * (gv.Fz_f_out + gv.Fz_f_in);
        Fz_r_avg = 0.5 * (gv.Fz_r_out + gv.Fz_r_in);
        F_y_f_tyre = 0.5 * F_y_f_axle;
        F_y_r_tyre = 0.5 * F_y_r_axle;

        % Pacejka envelope: max |Fx| per tyre
        F_x_f_tyre = pacejka_envelope(F_y_f_tyre, Fz_f_avg, p_front);
        F_x_r_tyre = pacejka_envelope(F_y_r_tyre, Fz_r_avg, p_rear);
        F_x_f_max = 2 * F_x_f_tyre;
        F_x_r_max = 2 * F_x_r_tyre;

        % Brake-bias constraint (same as v05/v06.0)
        F_brake_tot = min(F_x_f_max / bias_f, F_x_r_max / (1 - bias_f));
        a_brake_new = F_brake_tot / car.mass;

        if abs(a_brake_new - a_old) < 0.01, a_brake = a_brake_new; break; end
        a_brake = 0.5*a_old + 0.5*a_brake_new;
    end

    t_avg                = 0.5*(car.track_f + car.track_r);
    a_brake_record(ii)   = a_brake;
    a_lat_backward(ii)   = a_lat;
    dFz_backward(ii)     = -car.mass * a_brake * car.h_cog / car.wheelbase;
    dFz_lat_backward(ii) = car.mass * a_lat * car.h_cog / t_avg;

    v_prev_sq        = v_now^2 + 2 * a_brake * ds;
    v_backward(ii-1) = min(sqrt(v_prev_sq), v_forward(ii-1));
end

fprintf('  Max a_brake: %.2f m/s² (%.2f g)\n', ...
        max(a_brake_record), max(a_brake_record)/g_acc);

%% ========================================================================
%  5. COMBINE + LAP TIME
%  ========================================================================
v_sim    = min(v_forward, v_backward);
v_sim    = min(v_sim, v_corner);
dt_sim   = ds ./ max(v_sim, 0.1);
lap_time = sum(dt_sim);

fprintf('\nInitial lap (before continuity iter): %.3f s\n', lap_time);

%% ========================================================================
%  6. LAP CONTINUITY ITERATION
%  ========================================================================
fprintf('\nLap continuity check:\n');
fprintf('  Start: %.2f km/h   End: %.2f km/h\n', v_sim(1)*3.6, v_sim(end)*3.6);

if abs(v_sim(end) - v_sim(1)) > 1.0
    fprintf('  >> Iterating...\n');
    for iter = 1:5
        v_forward(1) = v_sim(end);

        % Re-run forward
        for ii = 1:n-1
            v_now  = v_forward(ii);
            a_lat  = v_now^2 * kappa(ii);
            F_drag = car.aero_drag_coeff * v_now^2;
            [F_drive, g_sel, ~] = get_drive_force(v_now, car);
            gear_forward(ii) = g_sel;
            a_engine = (F_drive - F_drag) / car.mass;

            a_long = max(a_engine, 0);
            for it = 1:10
                a_old = a_long;
                gv = get_axle_grip_v06(v_now, a_long, a_lat, car);
                F_y_r_axle = car.mass * a_lat * gv.Fz_r / (gv.Fz_f + gv.Fz_r);
                Fz_r_avg   = 0.5*(gv.Fz_r_out + gv.Fz_r_in);
                F_y_r_tyre = 0.5*F_y_r_axle;
                F_x_r_tyre = pacejka_envelope(F_y_r_tyre, Fz_r_avg, p_rear);
                F_x_r_max  = 2*F_x_r_tyre;
                a_traction_max = F_x_r_max / car.mass;
                a_long_new = max(min(a_engine, a_traction_max), 0);
                if abs(a_long_new - a_old) < 0.01, a_long = a_long_new; break; end
                a_long = 0.5*a_old + 0.5*a_long_new;
            end

            g_diag = get_axle_grip_v06(v_now, a_long, a_lat, car);
            t_avg               = 0.5*(car.track_f + car.track_r);
            a_long_forward(ii)  = a_long;
            a_lat_forward(ii)   = a_lat;
            dFz_forward(ii)     = car.mass * a_long * car.h_cog / car.wheelbase;
            dFz_lat_forward(ii) = car.mass * a_lat  * car.h_cog / t_avg;
            Fz_f_forward(ii)    = g_diag.Fz_f;
            Fz_r_forward(ii)    = g_diag.Fz_r;

            v_next_sq        = v_now^2 + 2 * a_long * ds;
            v_forward(ii+1)  = min(sqrt(max(v_next_sq, 0)), v_corner(ii+1));
        end
        gear_forward(n) = gear_forward(n-1);

        % Re-run backward
        v_backward(n) = v_forward(n);
        for ii = n:-1:2
            v_now = v_backward(ii);
            a_lat = v_now^2 * kappa(ii);
            a_brake = 0;
            for it = 1:10
                a_old = a_brake;
                gv = get_axle_grip_v06(v_now, -a_brake, a_lat, car);
                F_y_f_axle = car.mass * a_lat * gv.Fz_f / (gv.Fz_f + gv.Fz_r);
                F_y_r_axle = car.mass * a_lat * gv.Fz_r / (gv.Fz_f + gv.Fz_r);
                Fz_f_avg = 0.5*(gv.Fz_f_out + gv.Fz_f_in);
                Fz_r_avg = 0.5*(gv.Fz_r_out + gv.Fz_r_in);
                F_y_f_tyre = 0.5*F_y_f_axle;
                F_y_r_tyre = 0.5*F_y_r_axle;
                F_x_f_max = 2*pacejka_envelope(F_y_f_tyre, Fz_f_avg, p_front);
                F_x_r_max = 2*pacejka_envelope(F_y_r_tyre, Fz_r_avg, p_rear);
                F_brake_tot = min(F_x_f_max / bias_f, F_x_r_max / (1 - bias_f));
                a_brake_new = F_brake_tot / car.mass;
                if abs(a_brake_new - a_old) < 0.01, a_brake = a_brake_new; break; end
                a_brake = 0.5*a_old + 0.5*a_brake_new;
            end
            t_avg                = 0.5*(car.track_f + car.track_r);
            a_brake_record(ii)   = a_brake;
            a_lat_backward(ii)   = a_lat;
            dFz_backward(ii)     = -car.mass * a_brake * car.h_cog / car.wheelbase;
            dFz_lat_backward(ii) = car.mass * a_lat * car.h_cog / t_avg;

            v_prev_sq        = v_now^2 + 2 * a_brake * ds;
            v_backward(ii-1) = min(sqrt(v_prev_sq), v_forward(ii-1));
        end

        v_sim    = min(v_forward, v_backward);
        v_sim    = min(v_sim, v_corner);
        dt_sim   = ds ./ max(v_sim, 0.1);
        lap_time = sum(dt_sim);

        fprintf('  Iter %d: start=%.2f, end=%.2f km/h, lap=%.3f s\n', ...
                iter, v_sim(1)*3.6, v_sim(end)*3.6, lap_time);
        if abs(v_sim(end) - v_sim(1)) < 0.5
            fprintf('  Converged.\n'); break;
        end
    end
end

t_cum   = cumsum(dt_sim);
lap_min = floor(lap_time / 60);
lap_sec = lap_time - lap_min * 60;

%% ========================================================================
%  7. FINAL RESULTS
%  ========================================================================
fprintf('\n============== v06.0b CONVERGED RESULTS =================\n');
fprintf('  v06.0b lap time: %d:%06.3f\n', lap_min, lap_sec);
fprintf('  Reference:       %d:%06.3f\n', ...
        floor(track.meta.ref_laptime/60), ...
        track.meta.ref_laptime - floor(track.meta.ref_laptime/60)*60);
fprintf('  Δ vs ref:        %+.3f s (%+.2f%%)\n', ...
        lap_time - track.meta.ref_laptime, ...
        (lap_time - track.meta.ref_laptime) / track.meta.ref_laptime * 100);
if has_v06
    fprintf('  Δ vs v06.0:      %+.3f s\n', lap_time - sim06.lap_time);
end
if has_v05
    fprintf('  Δ vs v05 [CAL]:  %+.3f s\n', lap_time - sim05.lap_time);
end
fprintf('  Min/max/mean speed: %.1f / %.1f / %.1f km/h\n', ...
        min(v_sim)*3.6, max(v_sim)*3.6, mean(v_sim)*3.6);
fprintf('=========================================================\n');

%% ========================================================================
%  8. OUTPUT STRUCT
%  ========================================================================
sim06b                  = struct();
sim06b.v                = v_sim;
sim06b.v_kmh            = v_sim * 3.6;
sim06b.v_corner         = v_corner;
sim06b.v_forward        = v_forward;
sim06b.v_backward       = v_backward;
sim06b.gear             = gear_forward;
sim06b.dt               = dt_sim;
sim06b.t_cum            = t_cum;
sim06b.lap_time         = lap_time;
sim06b.dist             = track.dist;
sim06b.a_long           = a_long_forward;
sim06b.a_brake          = a_brake_record;
sim06b.a_lat_forward    = a_lat_forward;
sim06b.a_lat_backward   = a_lat_backward;
sim06b.dFz_forward      = dFz_forward;
sim06b.dFz_backward     = dFz_backward;
sim06b.dFz_lat_forward  = dFz_lat_forward;
sim06b.dFz_lat_backward = dFz_lat_backward;
sim06b.Fz_f_forward     = Fz_f_forward;
sim06b.Fz_r_forward     = Fz_r_forward;
sim06b.alpha_pk_f_corner = alpha_pk_f_corner;
sim06b.alpha_pk_r_corner = alpha_pk_r_corner;

sim06b.meta.model        = 'v06.0b: Pacejka MF 5.2 with full G-factor combined-slip envelope';
sim06b.meta.car          = car.meta.name;
sim06b.meta.track        = track.meta.name;
sim06b.meta.tyre_model   = car.tyre.pacejka.front.version;
sim06b.meta.tyre_source  = car.tyre.pacejka.front.source;
sim06b.meta.created      = datestr(now, 'yyyy-mm-dd HH:MM');

fprintf('\n=== v06.0b Simulation Complete ===\n\n');
