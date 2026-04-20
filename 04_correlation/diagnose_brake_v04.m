%% diagnose_brake_v04.m — Brake-spike classifier for v04
%  PURPOSE
%    Logbook Entry 012 flagged a max a_brake of 3.65 g in the v04
%    rewrite. GT3 reality sits around 2.5–2.8 g, so 3.65 g is physically
%    suspect. This script decides which of two failure modes is at play:
%
%      (a) SPIKE   — an isolated numerical artifact. Pass-3 in v04 uses
%                    a 10-iter fixed-point solve with 0.5/0.5 damping. If
%                    convergence hasn't been reached in 10 iters, the
%                    final damped average can sit anywhere — producing
%                    one or two outlier samples surrounded by realistic
%                    values. Fix: raise iter cap or cap a_brake.
%
%      (b) PLATEAU — a systemic bias-constraint pathology. At certain
%                    regimes the expression
%                        F_brake_tot = min(F_x_f_max / bias_f,
%                                          F_x_r_max / (1 - bias_f))
%                    delivers an unrealistic scaling — typically at very
%                    high speed (huge aero-boosted Fz) combined with very
%                    small lateral g, when the friction circle opens up
%                    and the only remaining limit is the bias ratio.
%                    A plateau shows up as a connected run of high-g
%                    samples at similar speed/curvature. Fix: revisit
%                    the constraint, possibly add a wheel-lift floor on
%                    dynamic Fz_f so that extreme forward transfer at
%                    high decel doesn't inflate the effective F_brake_tot.
%
%  METHOD
%    1. Distribution statistics: fraction of points above 2.5 / 2.8 /
%       3.0 / 3.2 / 3.5 g. A sharp drop-off after 2.8 g with very few
%       outliers argues for SPIKE.
%    2. Connected-run analysis on mask `a_brake > 3.0 g`. Singletons
%       and short runs = SPIKE; long runs = PLATEAU.
%    3. For the top-N outliers, reconstruct the axle-grip state
%       (re-running the same formula from lap_sim_v04.m), report
%       mu_f/mu_r, Fz_f/Fz_r, and which axle bound bit — front-bound
%       concentrated at high-speed low-curvature points is the classic
%       bias-constraint fingerprint.
%    4. Two plots: a_brake vs distance and a_brake vs speed, with
%       outlier points flagged in red. Deliberately minimal — we only
%       need enough to classify.
%
%  USAGE
%    Run lap_sim_v04 first so sim04, track, car are in the workspace.
%    Then: diagnose_brake_v04
%
%  OUTPUT
%    brake_diag struct (summary stats + verdict).
%
%  Author:  Jaykumar Patil  |  2026-04-19

%% ---------- 0. Input check ----------------------------------------------
if ~exist('sim04', 'var') || ~exist('track', 'var') || ~exist('car', 'var')
    error(['Need sim04, track, car in workspace. Run startup_project, ', ...
           'build_track, then lap_sim_v04 first.']);
end

g_acc  = car.g;
bias_f = car.brakes.bias_f;
a_brake_g = sim04.a_brake / g_acc;     % convert m/s^2 to g
n_pts = numel(a_brake_g);

%% ---------- 1. Distribution stats ---------------------------------------
fprintf('\n=================== v04 BRAKE-SPIKE DIAGNOSTIC ===================\n');
fprintf('Total points: %d\n', n_pts);

[a_max, idx_max] = max(a_brake_g);
fprintf('Max a_brake:  %.3f g at index %d (dist %.3f km, v %.1f km/h, kappa %.5f)\n', ...
        a_max, idx_max, track.dist(idx_max)/1000, ...
        sim04.v_backward(idx_max)*3.6, track.kappa(idx_max));
fprintf('Mean a_brake (where active, >0.5 g): %.3f g\n', ...
        mean(a_brake_g(a_brake_g > 0.5)));

thr_list = [2.5, 2.8, 3.0, 3.2, 3.5];
fprintf('\nDistribution tail:\n');
for thr = thr_list
    n_above = sum(a_brake_g > thr);
    pct     = 100 * n_above / n_pts;
    fprintf('  a_brake > %.1f g :  %5d pts  (%.3f %%)\n', thr, n_above, pct);
end

%% ---------- 2. Connected-run analysis -----------------------------------
THR_CLASSIFY = 3.0;                    % "high-g" threshold
mask = a_brake_g > THR_CLASSIFY;

% Convert mask to run-length structure using diff-padding trick
d = diff([0; mask(:); 0]);
run_starts = find(d ==  1);
run_ends   = find(d == -1) - 1;
run_lengths = run_ends - run_starts + 1;

fprintf('\nConnected runs of a_brake > %.1f g:\n', THR_CLASSIFY);
if isempty(run_lengths)
    fprintf('  (no points above threshold — the 3.65 g peak is a single sample.)\n');
    max_run = 0;
else
    fprintf('  # runs           : %d\n', numel(run_lengths));
    fprintf('  Max run length   : %d samples  (~%.1f m at ds = %.2f m)\n', ...
            max(run_lengths), max(run_lengths)*track.ds, track.ds);
    fprintf('  Mean run length  : %.2f samples\n', mean(run_lengths));
    fprintf('  Singletons (len=1): %d\n', sum(run_lengths == 1));
    max_run = max(run_lengths);
end

%% ---------- 3. Per-axle breakdown at top-N outliers ---------------------
N_TOP = 8;
[~, idx_sorted] = sort(a_brake_g, 'descend');
idx_top = idx_sorted(1:min(N_TOP, n_pts));

fprintf(['\nTop %d a_brake points — axle-state reconstruction:\n' ...
         ' rank  idx   dist[km]  v[km/h]  kappa      a_lat[g]  mu_f   mu_r   Fz_f[kN]  Fz_r[kN]  bound\n'], ...
         numel(idx_top));

front_bound_count = 0;
rear_bound_count  = 0;

for r = 1:numel(idx_top)
    i       = idx_top(r);
    v_now   = sim04.v_backward(i);
    kappa_i = track.kappa(i);
    a_lat   = v_now^2 * kappa_i;               % m/s^2
    a_brk   = sim04.a_brake(i);                % m/s^2

    % Replicate lap_sim_v04's Pass-3 axle-grip calc at this point
    dFz = -car.mass * a_brk * car.h_cog / car.wheelbase;   % braking => -
    F_df = car.aero_df_coeff * v_now^2;
    Fz_f = car.mass*car.g*car.weight_dist_f ...
           + F_df*car.aero_balance_f - dFz;
    Fz_r = car.mass*car.g*car.weight_dist_r ...
           + F_df*(1 - car.aero_balance_f) + dFz;
    Fz_f = max(Fz_f, 100);
    Fz_r = max(Fz_r, 100);
    mu_f = max(car.tyre.mu_0 - car.tyre.load_sens_k*(Fz_f/2), 0.5);
    mu_r = max(car.tyre.mu_0 - car.tyre.load_sens_k*(Fz_r/2), 0.5);
    F_grip_f = mu_f * Fz_f;
    F_grip_r = mu_r * Fz_r;

    % Lateral split proportional to axle normal load (same assumption as v04)
    F_y_f = car.mass * a_lat * Fz_f / (Fz_f + Fz_r);
    F_y_r = car.mass * a_lat * Fz_r / (Fz_f + Fz_r);
    F_x_f_max = sqrt(max(F_grip_f^2 - F_y_f^2, 0));
    F_x_r_max = sqrt(max(F_grip_r^2 - F_y_r^2, 0));

    % Which axle's bias-scaled limit binds the min()?
    front_candidate = F_x_f_max / bias_f;
    rear_candidate  = F_x_r_max / (1 - bias_f);
    if front_candidate <= rear_candidate
        bound_tag = 'FRONT';
        front_bound_count = front_bound_count + 1;
    else
        bound_tag = 'REAR ';
        rear_bound_count = rear_bound_count + 1;
    end

    fprintf(' %4d %5d   %7.3f   %6.1f  %8.5f    %5.2f   %.3f  %.3f    %5.2f    %5.2f    %s\n', ...
            r, i, track.dist(i)/1000, v_now*3.6, kappa_i, a_lat/g_acc, ...
            mu_f, mu_r, Fz_f/1000, Fz_r/1000, bound_tag);
end

fprintf('\n  Bound summary in top-%d: %d FRONT-bound, %d REAR-bound.\n', ...
        numel(idx_top), front_bound_count, rear_bound_count);

%% ---------- 4. Verdict --------------------------------------------------
n_outliers    = sum(a_brake_g > THR_CLASSIFY);
pct_outliers  = 100 * n_outliers / n_pts;

fprintf('\n--- VERDICT ---\n');
if max_run <= 2 && pct_outliers < 0.5
    verdict = 'SPIKE';
    fprintf('  SPIKE: isolated outlier(s). Most likely a Pass-3 fixed-point\n');
    fprintf('         iteration that hit the 10-iter cap without converging.\n');
    fprintf('  Fix  : either (a) raise Pass-3 iter cap from 10 to 30 and\n');
    fprintf('         retighten tolerance to 0.005 m/s^2, or (b) clip a_brake\n');
    fprintf('         to a physical ceiling (e.g. 2.9 g) after Pass 3.\n');
elseif max_run >= 10 || pct_outliers > 2
    verdict = 'PLATEAU';
    fprintf('  PLATEAU: sustained high-g segments indicate a systemic issue\n');
    fprintf('           with the bias constraint or the friction-circle split.\n');
    fprintf('  Fix   : look at the BOUND column — if FRONT-bound dominates at\n');
    fprintf('          high speed + low kappa, the front friction circle is\n');
    fprintf('          inflated by huge aero-boosted Fz_f. Consider adding a\n');
    fprintf('          wheel-lift floor / cap on dFz_long so dynamic Fz_r\n');
    fprintf('          cannot go negative via transfer, AND/OR cap mu at high Fz.\n');
else
    verdict = 'MIXED';
    fprintf('  MIXED: neither clearly spike nor plateau. Inspect plots and\n');
    fprintf('         the top-N table above to decide manually.\n');
end

%% ---------- 5. Two focused plots ---------------------------------------
figure('Name','v04 Brake Spike Diagnostic', ...
       'NumberTitle','off','Position',[120 120 1250 480]);

% Left: a_brake vs distance
subplot(1,2,1);
plot(track.dist/1000, a_brake_g, 'k', 'LineWidth', 0.6); hold on;
idx_hi = find(a_brake_g > THR_CLASSIFY);
if ~isempty(idx_hi)
    plot(track.dist(idx_hi)/1000, a_brake_g(idx_hi), 'r.', 'MarkerSize', 12);
end
yline(2.8, '--', 'GT3 realistic max (2.8 g)', 'LabelHorizontalAlignment','left');
yline(THR_CLASSIFY, ':', 'classifier threshold', 'LabelHorizontalAlignment','left');
xlabel('Distance [km]'); ylabel('a_{brake} [g]');
title(sprintf('Brake accel vs distance — %d outlier(s) > %.1f g', ...
              numel(idx_hi), THR_CLASSIFY));
grid on; xlim([0, track.length/1000]);

% Right: a_brake vs speed
subplot(1,2,2);
plot(sim04.v_backward*3.6, a_brake_g, 'k.', 'MarkerSize', 3); hold on;
if ~isempty(idx_hi)
    plot(sim04.v_backward(idx_hi)*3.6, a_brake_g(idx_hi), 'r.', 'MarkerSize', 12);
end
yline(2.8, '--'); yline(THR_CLASSIFY, ':');
xlabel('Speed [km/h]'); ylabel('a_{brake} [g]');
title('Brake accel vs speed (red = classified outlier)');
grid on;

%% ---------- 6. Output struct --------------------------------------------
brake_diag              = struct();
brake_diag.verdict      = verdict;
brake_diag.max_a_brake_g = a_max;
brake_diag.idx_max      = idx_max;
brake_diag.n_outliers   = n_outliers;
brake_diag.pct_outliers = pct_outliers;
brake_diag.max_run      = max_run;
brake_diag.run_lengths  = run_lengths;
brake_diag.thr_classify = THR_CLASSIFY;
brake_diag.top_idx      = idx_top;
brake_diag.bound_summary = struct('front', front_bound_count, 'rear', rear_bound_count);
brake_diag.created      = datestr(now, 'yyyy-mm-dd HH:MM');

fprintf('\n==================================================================\n');
fprintf('Diagnostic complete. Verdict: %s\n', verdict);
fprintf('Result in `brake_diag` struct. Plots in figure window.\n\n');
