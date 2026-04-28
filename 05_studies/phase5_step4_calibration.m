%% phase5_step4_calibration.m
%  Phase 5, Step 4 — Calibration of mu_0 × load_sens_k on telemetry.
%
%  Objective: find tyre parameters that match the reference driver's
%  actual lap (telemetry source). Two cost metrics tracked:
%    (1) |Δlap|        — total lap-time delta vs reference
%    (2) sector_rms    — RMS of per-sector Δt (penalises sector imbalance)
%
%  Best-fit = minimise sector_rms (more honest than |Δlap| alone, which
%  can hit zero with cancelling sector errors).
%
%  Sweep:
%    mu_0          ∈ [1.70, 1.75, 1.80, 1.85, 1.90]   (baseline 1.85)
%    load_sens_k   ∈ [4.4e-5, 5.0e-5, 5.5e-5, 6.0e-5, 6.6e-5]  (baseline 5.5e-5)
%  → 25 v05 runs.
%
%  Direction rationale (Step 2): sim is -8.9 s faster than ref on
%  telemetry → needs LESS grip → lower mu_0 and/or higher load_sens_k.
%
%  USAGE
%    >> startup_project
%    >> import_reference_lap
%    >> track_source = 'telemetry'; build_track
%    >> run('05_studies/phase5_step4_calibration.m')
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-21

if ~exist('ref','var') || ~exist('car','var') || ~exist('track','var')
    error('Load ref + car + telemetry track first.');
end

fprintf('\n==========================================================\n');
fprintf(' Phase 5 Step 4 — Calibration sweep (mu_0 × load_sens_k)   \n');
fprintf('==========================================================\n');

car_baseline = car;
ref_lap = track.meta.ref_laptime;

% Sector edges (6 equal-length sectors, same as Step 2)
edges   = linspace(0, track.dist(end), 7);
sectors = [edges(1:end-1)' edges(2:end)'];

% Sweep grid
mu0_vals = [1.70 1.75 1.80 1.85 1.90];
ksk_vals = [4.4e-5 5.0e-5 5.5e-5 6.0e-5 6.6e-5];
[Nmu, Nk] = deal(numel(mu0_vals), numel(ksk_vals));

lap_grid    = zeros(Nmu, Nk);
dlap_grid   = zeros(Nmu, Nk);
secrms_grid = zeros(Nmu, Nk);

fprintf('\nBaseline ref lap: %.3f s. Running %d sims...\n\n', ref_lap, Nmu*Nk);

t0 = tic;
% NOTE: lap_sim_v05 leaks 'i' and 'j' through evalc(); use ii/jj.
for ii = 1:Nmu
    for jj = 1:Nk
        car = car_baseline;
        car.tyre.mu_0        = mu0_vals(ii);
        car.tyre.load_sens_k = ksk_vals(jj);
        evalc('lap_sim_v05;');
        evalc('c = correlate_sim(sim05, track, sectors, sprintf(''mu0=%.2f k=%.2g'',mu0_vals(ii),ksk_vals(jj)), false);');
        lap_grid(ii,jj)    = sim05.lap_time;
        dlap_grid(ii,jj)   = sim05.lap_time - ref_lap;
        sec_dt             = [c.per_sector.dt_sector];
        secrms_grid(ii,jj) = sqrt(mean(sec_dt.^2));
        fprintf('  mu0=%.2f  k=%5.1e  lap=%7.3f  d=%+6.3f  rms=%5.3f\n', ...
                mu0_vals(ii), ksk_vals(jj), lap_grid(ii,jj), ...
                dlap_grid(ii,jj), secrms_grid(ii,jj));
    end
end
car = car_baseline;
fprintf('\nElapsed: %.1f s\n', toc(t0));

%% ---- Find best fit ---------------------------------------------------
[~, idx_lap] = min(abs(dlap_grid(:)));
[~, idx_rms] = min(secrms_grid(:));
[il, jl] = ind2sub([Nmu Nk], idx_lap);
[ir, jr] = ind2sub([Nmu Nk], idx_rms);

charter_pct = 100 * dlap_grid / ref_lap;

fprintf('\n=== Best by |Δlap| ===\n');
fprintf('  mu_0 = %.2f, load_sens_k = %.2g\n', mu0_vals(il), ksk_vals(jl));
fprintf('  Δlap = %+.3f s (%+.2f%%)   sector_rms = %.3f s\n', ...
        dlap_grid(il,jl), charter_pct(il,jl), secrms_grid(il,jl));

fprintf('\n=== Best by sector_rms (recommended) ===\n');
fprintf('  mu_0 = %.2f, load_sens_k = %.2g\n', mu0_vals(ir), ksk_vals(jr));
fprintf('  Δlap = %+.3f s (%+.2f%%)   sector_rms = %.3f s\n', ...
        dlap_grid(ir,jr), charter_pct(ir,jr), secrms_grid(ir,jr));

charter_pass = abs(charter_pct(ir,jr)) <= 1.0;
fprintf('\nCharter (±1%%): %s\n', ...
        ternary(charter_pass,'PASS','FAIL'));

%% ---- Heatmaps ---------------------------------------------------------
figure('Name','Step 4 calibration — heatmaps','NumberTitle','off', ...
       'Position',[100 100 1200 500]);

subplot(1,2,1);
imagesc(ksk_vals, mu0_vals, dlap_grid); axis xy;
colorbar; colormap(gca, 'parula');
xlabel('load\_sens\_k [1/N]'); ylabel('mu\_0 [-]');
title('Δlap vs ref [s]');
set(gca,'XTick',ksk_vals,'YTick',mu0_vals);
hold on; plot(ksk_vals(jl), mu0_vals(il), 'kp','MarkerSize',14,'MarkerFaceColor','y');

subplot(1,2,2);
imagesc(ksk_vals, mu0_vals, secrms_grid); axis xy;
colorbar; colormap(gca, 'parula');
xlabel('load\_sens\_k [1/N]'); ylabel('mu\_0 [-]');
title('Sector RMS Δt [s]');
set(gca,'XTick',ksk_vals,'YTick',mu0_vals);
hold on; plot(ksk_vals(jr), mu0_vals(ir), 'kp','MarkerSize',14,'MarkerFaceColor','r');

%% ---- Re-run best fit with full correlation report --------------------
fprintf('\n=== Sector report at best-fit (sector_rms) ===\n');
car.tyre.mu_0        = mu0_vals(ir);
car.tyre.load_sens_k = ksk_vals(jr);
evalc('lap_sim_v05;');
corr_best = correlate_sim(sim05, track, sectors, ...
    sprintf('Best-fit: mu0=%.2f, k=%.2g', mu0_vals(ir), ksk_vals(jr)), ...
    true);
car = car_baseline;

%% ---- Save -------------------------------------------------------------
result = struct();
result.mu0_vals    = mu0_vals;
result.ksk_vals    = ksk_vals;
result.lap_grid    = lap_grid;
result.dlap_grid   = dlap_grid;
result.secrms_grid = secrms_grid;
result.best_lap    = struct('mu_0',mu0_vals(il),'load_sens_k',ksk_vals(jl), ...
                            'dlap',dlap_grid(il,jl),'rms',secrms_grid(il,jl));
result.best_rms    = struct('mu_0',mu0_vals(ir),'load_sens_k',ksk_vals(jr), ...
                            'dlap',dlap_grid(ir,jr),'rms',secrms_grid(ir,jr));
result.corr_best   = corr_best;

outdir  = fileparts(mfilename('fullpath'));
outpath = fullfile(outdir, 'phase5_step4_result.mat');
save(outpath, 'result');
fprintf('\nSaved: %s\nDone.\n', outpath);

%% ---- helpers ----------------------------------------------------------
function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
