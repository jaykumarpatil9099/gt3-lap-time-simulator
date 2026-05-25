%% lap_sim_v07.m — v07.0 FIRST-ORDER SLIP DYNAMICS CORRECTION
%
%  Adds the dominant transient slip effect on top of v06b's QSS lap:
%  the contact patch needs distance to develop the kinematic slip angle.
%  Tyres do not jump to peak grip the instant the kinematic conditions
%  demand it. Standard Pacejka relaxation model in the space domain:
%
%      d α_dyn / ds = (α_kin(s) − α_dyn(s)) / σ_α
%
%  At each track point we then evaluate the Pacejka curve at α_dyn
%  rather than α_kin and the lateral force is correspondingly less than
%  the peak v06b assumed. The cornering-speed cap drops, and the lap-time
%  integral grows. This captures the time the car loses entering tight
%  corners while the slip angle is still building up.
%
%  WHAT v07.0 DOES NOT DO
%  ----------------------
%  - It is a *post-process* on v06b's converged result, not a rewrite of
%    the three-pass solver. v07.1 (transient ODE end-to-end) is a far
%    bigger rewrite and out of scope for a single-session deliverable.
%  - No transient on longitudinal slip κ. Drive / brake transients exist
%    too but are smaller on QSS lap time.
%  - No coupling to roll / pitch dynamics, dampers, anti-dive, etc.
%
%  PRE-REQUISITES
%    sim06b in workspace      (from lap_sim_v06b — the project head)
%    track  in workspace
%    car.tyre.pacejka, car.tyre.sigma_alpha
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-22  (Phase 7 — v07.0)

if ~exist('sim06b','var')
    error(['sim06b missing. v07 builds on v06b — run lap_sim_v06b first.']);
end
if ~exist('track','var') || ~exist('car','var')
    error('track + car must be loaded.');
end
if ~isfield(car.tyre, 'pacejka') || ~isfield(car.tyre, 'sigma_alpha')
    error(['car.tyre.pacejka and car.tyre.sigma_alpha required. Re-run ' ...
           'startup_project to pick up the updated params + Pacejka set.']);
end

fprintf('\n=== Lap Sim v07.0 — First-order slip dynamics on v06b ===\n');
fprintf('Tyre relaxation length σ_α = %.2f m\n', car.tyre.sigma_alpha);

p_f = car.tyre.pacejka.front;
p_r = car.tyre.pacejka.rear;
sigma = car.tyre.sigma_alpha;
ds    = track.ds;
n     = track.n;

%% --- 1. Kinematic slip-angle profile from v06b ---------------------
%  sim06b.alpha_pk_*_corner is the slip angle at which Pacejka peak Fy
%  occurs at the converged v06b operating point. On straights it's NaN
%  / zero because we don't compute it there — replace zeros with 0.

alpha_kin_f = sim06b.alpha_pk_f_corner;
alpha_kin_r = sim06b.alpha_pk_r_corner;
alpha_kin_f(~isfinite(alpha_kin_f)) = 0;
alpha_kin_r(~isfinite(alpha_kin_r)) = 0;
% Straights: track.kappa < 1e-6 → no real slip demand
alpha_kin_f(track.kappa < 1e-6) = 0;
alpha_kin_r(track.kappa < 1e-6) = 0;

%% --- 2. Forward march α_dyn ---------------------------------------
%  Discrete first-order step: α_dyn(i+1) = α_dyn(i) + (α_kin(i) − α_dyn(i))·ds/σ
%  Initial: α_dyn(1) = α_kin(1) (assume lap starts already in steady state
%  if the start point is mid-corner; if start is a straight, α_kin(1) = 0
%  so the initial condition is also 0).

alpha_dyn_f = zeros(n, 1);
alpha_dyn_r = zeros(n, 1);
alpha_dyn_f(1) = alpha_kin_f(1);
alpha_dyn_r(1) = alpha_kin_r(1);

step_factor = ds / sigma;
% step_factor > 1 means the discrete update overshoots; clamp implicitly
% with the standard explicit-Euler stable factor.
step_factor = min(step_factor, 1.0);

for ii = 1:n-1
    alpha_dyn_f(ii+1) = alpha_dyn_f(ii) + ...
                       (alpha_kin_f(ii) - alpha_dyn_f(ii)) * step_factor;
    alpha_dyn_r(ii+1) = alpha_dyn_r(ii) + ...
                       (alpha_kin_r(ii) - alpha_dyn_r(ii)) * step_factor;
end

%% --- 3. Compute grip ratio η(s) per axle ---------------------------
%  η = Fy(α_dyn, Fz_avg) / Fy(α_kin, Fz_avg), evaluated per axle from
%  the per-tyre Fz that v06b stored. Per-tyre Fz isn't in sim06b directly
%  — re-derive on the v06b operating point with get_axle_grip_v06.

eta_f = ones(n, 1);
eta_r = ones(n, 1);

for ii = 1:n
    if track.kappa(ii) < 1e-6
        continue;   % straight: no demand, η stays at 1
    end
    v_now = sim06b.v(ii);
    a_lat = v_now^2 * track.kappa(ii);
    gv = get_axle_grip_v06(v_now, 0, a_lat, car);

    % Per-tyre average Fz for each axle
    Fz_f_avg = 0.5*(gv.Fz_f_out + gv.Fz_f_in);
    Fz_r_avg = 0.5*(gv.Fz_r_out + gv.Fz_r_in);

    % Pure-Pacejka Fy at the kinematic and dynamic α
    Fy_peak_f = abs(pacejka_fy(alpha_kin_f(ii), Fz_f_avg, p_f));
    Fy_dyn_f  = abs(pacejka_fy(alpha_dyn_f(ii), Fz_f_avg, p_f));
    Fy_peak_r = abs(pacejka_fy(alpha_kin_r(ii), Fz_r_avg, p_r));
    Fy_dyn_r  = abs(pacejka_fy(alpha_dyn_r(ii), Fz_r_avg, p_r));

    if Fy_peak_f > 1, eta_f(ii) = max(min(Fy_dyn_f / Fy_peak_f, 1), 0); end
    if Fy_peak_r > 1, eta_r(ii) = max(min(Fy_dyn_r / Fy_peak_r, 1), 0); end
end

%% --- 4. Derate v06b's converged v directly ------------------------
%  v06b's v_sim is the speed at each point assuming peak-grip both
%  axles. If a tyre is only at η·peak grip due to slip transient, the
%  cornering capacity is η × peak, so the sustainable v scales by
%  √η. Apply that as a direct multiplier on sim06b.v (the converged
%  trace), with NaN guards.

eta_combined = sqrt(0.5*(eta_f + eta_r));   % already √η ≤ 1

% Guard against any non-finite values inherited from upstream computations
eta_combined(~isfinite(eta_combined)) = 1;
eta_combined = min(max(eta_combined, 0.5), 1);   % floor: never less than half

v_sim_v07 = sim06b.v .* eta_combined;
v_sim_v07 = min(v_sim_v07, sim06b.v);            % never go above v06b
v_sim_v07 = max(v_sim_v07, 0.5);                 % avoid 1/0 in dt

dt_v07    = ds ./ v_sim_v07;
lap_v07   = sum(dt_v07);

%% --- 6. Report ----------------------------------------------------
ref_lap = track.meta.ref_laptime;
fprintf('\n=== v07.0 CONVERGED RESULTS ===\n');
fprintf('  v07.0 lap time:    %d:%06.3f\n', floor(lap_v07/60), ...
                                              lap_v07 - 60*floor(lap_v07/60));
fprintf('  Reference:         %d:%06.3f\n', floor(ref_lap/60), ...
                                              ref_lap - 60*floor(ref_lap/60));
fprintf('  Δ vs ref:          %+.3f s (%+.2f %%)\n', ...
        lap_v07 - ref_lap, 100*(lap_v07 - ref_lap)/ref_lap);
fprintf('  Δ vs v06b:         %+.3f s\n', lap_v07 - sim06b.lap_time);
fprintf('  Slip-build-up cost: %+.3f s\n', lap_v07 - sim06b.lap_time);

fprintf('\n  η_f range: %.3f .. %.3f   (min = worst slip-build penalty)\n', ...
        min(eta_f(track.kappa > 1e-3)), max(eta_f));
fprintf('  η_r range: %.3f .. %.3f\n', ...
        min(eta_r(track.kappa > 1e-3)), max(eta_r));

n_penalised = sum(eta_combined < 0.99 & track.kappa > 1e-3);
fprintf('  Points with η < 0.99: %d (of %d corner-points = %.1f %%)\n', ...
        n_penalised, sum(track.kappa > 1e-3), ...
        100*n_penalised/max(sum(track.kappa > 1e-3),1));

%% --- 7. Output struct ---------------------------------------------
sim07              = struct();
sim07.v            = v_sim_v07;
sim07.v_kmh        = v_sim_v07 * 3.6;
sim07.v_corner     = sim06b.v_corner;   % inherited from v06b (unchanged in v07.0)
sim07.dt           = dt_v07;
sim07.t_cum        = cumsum(dt_v07);
sim07.lap_time     = lap_v07;
sim07.dist         = track.dist;
sim07.alpha_kin_f  = alpha_kin_f;
sim07.alpha_kin_r  = alpha_kin_r;
sim07.alpha_dyn_f  = alpha_dyn_f;
sim07.alpha_dyn_r  = alpha_dyn_r;
sim07.eta_f        = eta_f;
sim07.eta_r        = eta_r;
sim07.eta_combined = eta_combined;
sim07.meta.model       = 'v07.0 first-order slip dynamics on top of v06b';
sim07.meta.sigma_alpha = sigma;
sim07.meta.parent      = 'sim06b';
sim07.meta.created     = datestr(now, 'yyyy-mm-dd HH:MM');

fprintf('\n=== v07.0 Simulation Complete ===\n\n');

%% --- 8. Diagnostic plot ------------------------------------------
figure('Name','v07 slip dynamics — α and η around the lap', ...
       'NumberTitle','off', 'Position',[100 100 1300 800]);

subplot(3,1,1); hold on;
plot(track.dist/1000, alpha_kin_f*180/pi, 'k-',  'LineWidth', 0.8, ...
     'DisplayName','α_{kin,f} (peak target)');
plot(track.dist/1000, alpha_dyn_f*180/pi, 'r-',  'LineWidth', 0.8, ...
     'DisplayName','α_{dyn,f} (developed)');
ylabel('Front α [°]'); legend('Location','northeast'); grid on;
xlim([0 track.dist(end)/1000]);
title(sprintf('v07 slip dynamics — σ_α = %.2f m', sigma));

subplot(3,1,2); hold on;
plot(track.dist/1000, alpha_kin_r*180/pi, 'k-', 'LineWidth', 0.8, ...
     'DisplayName','α_{kin,r}');
plot(track.dist/1000, alpha_dyn_r*180/pi, 'b-', 'LineWidth', 0.8, ...
     'DisplayName','α_{dyn,r}');
ylabel('Rear α [°]'); legend('Location','northeast'); grid on;
xlim([0 track.dist(end)/1000]);

subplot(3,1,3); hold on;
plot(track.dist/1000, eta_combined, 'Color',[0.6 0.2 0.6], 'LineWidth', 0.8, ...
     'DisplayName','η_{combined} = sqrt((η_f+η_r)/2)');
yline(1.0, ':', 'Color', [0.5 0.5 0.5]);
ylabel('η (grip available / peak)'); xlabel('Distance [km]');
ylim([0 1.05]); legend('Location','southeast'); grid on;
xlim([0 track.dist(end)/1000]);
