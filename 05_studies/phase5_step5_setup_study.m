%% phase5_step5_setup_study.m
%  Phase 5, Step 5 — Setup optimisation: aero_balance_f × roll_dist_f.
%
%  Runs on the calibrated baseline (tyre params already locked in
%  amg_gt3_params.m at mu_0=1.70, load_sens_k=4.4e-5 [CAL]).
%
%  Variables (the two highest-leverage *adjustable* setup knobs that
%  remain after calibration):
%    aero_balance_f     — fraction of total downforce on front axle
%                         (splitter / wing-angle equivalent)
%    roll_dist_f        — front share of total roll stiffness
%                         (ARB balance: bar-rate ratio at fixed total)
%
%  roll_dist_f is swept by adjusting K_ARB_f and K_ARB_r so the TOTAL
%  roll stiffness stays constant. That isolates the *balance* effect
%  from the *grip* effect — a race-engineering setup change, not a
%  hardware swap.
%
%  Cost metric: per-sector RMS Δt on telemetry source (driver-line
%  correlation). Lap delta tracked alongside but not used as primary
%  objective — see Entry 017 Think block.
%
%  USAGE
%    >> startup_project
%    >> import_reference_lap
%    >> track_source = 'telemetry'; build_track
%    >> run('05_studies/phase5_step5_setup_study.m')
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-21

if ~exist('ref','var') || ~exist('car','var') || ~exist('track','var')
    error('Load ref + car + telemetry track first.');
end

fprintf('\n==========================================================\n');
fprintf(' Phase 5 Step 5 — Setup study (aero_balance_f × roll_dist_f)\n');
fprintf('==========================================================\n');

car_baseline = car;
ref_lap      = track.meta.ref_laptime;

% Sectors: 6 equal-length, same as Step 2 / Step 4
edges   = linspace(0, track.dist(end), 7);
sectors = [edges(1:end-1)' edges(2:end)'];

% Total roll stiffness — keep constant across the sweep
K_total = car_baseline.suspension.K_roll_f + car_baseline.suspension.K_roll_r;

%% ---- Baseline run -----------------------------------------------------
evalc('lap_sim_v05;');
c0 = correlate_sim(sim05, track, sectors, 'baseline', false);
baseline_lap     = sim05.lap_time;
baseline_dlap    = baseline_lap - ref_lap;
baseline_secrms  = sqrt(mean([c0.per_sector.dt_sector].^2));
fprintf('\nBaseline (calibrated): lap = %.3f s, Δ = %+.3f s, sector_rms = %.3f s\n', ...
        baseline_lap, baseline_dlap, baseline_secrms);
fprintf('Total roll stiffness held at %.0f N·m/rad.\n\n', K_total);

%% ---- Sweep grid -------------------------------------------------------
aero_vals = [0.39 0.41 0.43 0.45 0.47];
roll_vals = [0.50 0.53 0.5625 0.59 0.62];   % baseline = 0.5625
[Na, Nr]  = deal(numel(aero_vals), numel(roll_vals));

lap_grid    = zeros(Na, Nr);
dlap_grid   = zeros(Na, Nr);
secrms_grid = zeros(Na, Nr);

fprintf('Running %d v05 sims...\n', Na*Nr);
t0 = tic;
for i = 1:Na
    for j = 1:Nr
        car = car_baseline;
        car.aero_balance_f = aero_vals(i);

        % Reconstruct ARB rates from desired roll_dist_f
        rd  = roll_vals(j);
        Krf = rd        * K_total;
        Krr = (1 - rd)  * K_total;
        car.suspension.K_roll_f    = Krf;
        car.suspension.K_roll_r    = Krr;
        car.suspension.K_ARB_f     = Krf - car.suspension.K_tire_f;
        car.suspension.K_ARB_r     = Krr - car.suspension.K_tire_r;
        car.suspension.roll_dist_f = rd;
        car.suspension.roll_dist_r = 1 - rd;

        evalc('lap_sim_v05;');
        c = correlate_sim(sim05, track, sectors, '', false);
        lap_grid(i,j)    = sim05.lap_time;
        dlap_grid(i,j)   = sim05.lap_time - ref_lap;
        secrms_grid(i,j) = sqrt(mean([c.per_sector.dt_sector].^2));
        fprintf('  aero=%.2f roll=%.4f  lap=%7.3f  Δ=%+6.3f  rms=%5.3f\n', ...
                aero_vals(i), roll_vals(j), ...
                lap_grid(i,j), dlap_grid(i,j), secrms_grid(i,j));
    end
end
car = car_baseline;
fprintf('\nElapsed: %.1f s\n', toc(t0));

%% ---- Find best ---------------------------------------------------------
[~, idx_rms] = min(secrms_grid(:));
[i_r, j_r]   = ind2sub([Na Nr], idx_rms);
[~, idx_lap] = min(abs(dlap_grid(:)));
[i_l, j_l]   = ind2sub([Na Nr], idx_lap);

fprintf('\n=== Best by sector_rms (recommended) ===\n');
fprintf('  aero_balance_f = %.2f, roll_dist_f = %.4f\n', ...
        aero_vals(i_r), roll_vals(j_r));
fprintf('  lap = %.3f s   Δ = %+.3f s (%+.2f%%)   sector_rms = %.3f s\n', ...
        lap_grid(i_r,j_r), dlap_grid(i_r,j_r), ...
        100*dlap_grid(i_r,j_r)/ref_lap, secrms_grid(i_r,j_r));
fprintf('  vs baseline: Δlap = %+.3f s, Δsector_rms = %+.3f s\n', ...
        lap_grid(i_r,j_r) - baseline_lap, ...
        secrms_grid(i_r,j_r) - baseline_secrms);

fprintf('\n=== Best by |Δlap| ===\n');
fprintf('  aero_balance_f = %.2f, roll_dist_f = %.4f\n', ...
        aero_vals(i_l), roll_vals(j_l));
fprintf('  lap = %.3f s   Δ = %+.3f s   sector_rms = %.3f s\n', ...
        lap_grid(i_l,j_l), dlap_grid(i_l,j_l), secrms_grid(i_l,j_l));

%% ---- Heatmaps ---------------------------------------------------------
figure('Name','Step 5 setup study','NumberTitle','off', ...
       'Position',[100 100 1300 500]);

subplot(1,2,1);
imagesc(roll_vals, aero_vals, dlap_grid); axis xy;
colorbar; xlabel('roll\_dist\_f [-]'); ylabel('aero\_balance\_f [-]');
title('Δlap vs ref [s]');
set(gca,'XTick',roll_vals,'YTick',aero_vals);
hold on;
plot(roll_vals(j_l), aero_vals(i_l), 'kp','MarkerSize',14,'MarkerFaceColor','y');
plot(0.5625, 0.43, 'wo','MarkerSize',10,'LineWidth',2);  % baseline marker

subplot(1,2,2);
imagesc(roll_vals, aero_vals, secrms_grid); axis xy;
colorbar; xlabel('roll\_dist\_f [-]'); ylabel('aero\_balance\_f [-]');
title('Sector RMS Δt [s]');
set(gca,'XTick',roll_vals,'YTick',aero_vals);
hold on;
plot(roll_vals(j_r), aero_vals(i_r), 'kp','MarkerSize',14,'MarkerFaceColor','r');
plot(0.5625, 0.43, 'wo','MarkerSize',10,'LineWidth',2);

%% ---- Sector report at best fit ----------------------------------------
fprintf('\n=== Sector report at best-fit setup ===\n');
car.aero_balance_f = aero_vals(i_r);
rd = roll_vals(j_r);
car.suspension.K_roll_f    = rd        * K_total;
car.suspension.K_roll_r    = (1 - rd)  * K_total;
car.suspension.K_ARB_f     = car.suspension.K_roll_f - car.suspension.K_tire_f;
car.suspension.K_ARB_r     = car.suspension.K_roll_r - car.suspension.K_tire_r;
car.suspension.roll_dist_f = rd;
car.suspension.roll_dist_r = 1 - rd;
evalc('lap_sim_v05;');
corr_best = correlate_sim(sim05, track, sectors, ...
    sprintf('Step 5 best: aero=%.2f roll=%.4f', aero_vals(i_r), rd), true);
car = car_baseline;

%% ---- Save -------------------------------------------------------------
result = struct();
result.aero_vals       = aero_vals;
result.roll_vals       = roll_vals;
result.K_total         = K_total;
result.lap_grid        = lap_grid;
result.dlap_grid       = dlap_grid;
result.secrms_grid     = secrms_grid;
result.baseline_lap    = baseline_lap;
result.baseline_secrms = baseline_secrms;
result.best_rms        = struct('aero_balance_f',aero_vals(i_r), ...
                                'roll_dist_f',roll_vals(j_r), ...
                                'lap',lap_grid(i_r,j_r), ...
                                'dlap',dlap_grid(i_r,j_r), ...
                                'secrms',secrms_grid(i_r,j_r));
result.corr_best       = corr_best;

outdir  = fileparts(mfilename('fullpath'));
outpath = fullfile(outdir, 'phase5_step5_result.mat');
save(outpath, 'result');
fprintf('\nSaved: %s\nDone.\n', outpath);
