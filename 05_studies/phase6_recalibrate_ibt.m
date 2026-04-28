%% phase6_recalibrate_ibt.m
%  Phase 6 — Re-calibrate (mu_0, load_sens_k) against the multi-lap IBT
%  reference lap. Same 5×5 grid as Phase 5 Step 4; the only change is the
%  reference data source.
%
%  WHY THIS IS A DISTINCT STUDY (not a re-run of step 4)
%  -----------------------------------------------------
%  Phase 5 Step 4 fitted tyre parameters to a single lap exported from
%  PI Toolbox as Excel. The new IBT pipeline gives a slightly different
%  reference: track length 25175 m (matches GPS centreline) instead of
%  the trapezoidal-integration overshoot of 25206 m, plus the raw 60 Hz
%  noise floor instead of PI Toolbox's filtering. Carrying the Phase-5
%  [CAL] values forward without re-running would be calibration-against-
%  one-source quoted as calibration-against-another. This entry records
%  the recalibration honestly.
%
%  USAGE
%    >> startup_project
%    >> import_reference_lap_ibt
%    >> track_source = 'telemetry'; build_track
%    >> run('05_studies/phase6_recalibrate_ibt.m')
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-21  (Phase 6)

if ~exist('ref','var') || ~exist('car','var') || ~exist('track','var')
    error('Load ref + car + telemetry track first.');
end
if ~isfield(ref, 'meta') || ~isfield(ref.meta, 'source_mode')
    warning('phase6:OldRef', ...
            'Reference does not look IBT-sourced. Continuing anyway.');
end

fprintf('\n==========================================================\n');
fprintf(' Phase 6 — Recalibrate (mu_0 × load_sens_k) on IBT reference\n');
fprintf('==========================================================\n');

car_baseline = car;
ref_lap = track.meta.ref_laptime;

edges   = linspace(0, track.dist(end), 7);
sectors = [edges(1:end-1)' edges(2:end)'];

% Same grid as Phase 5 Step 4 — keeps results directly comparable
mu0_vals = [1.70 1.75 1.80 1.85 1.90];
ksk_vals = [4.4e-5 5.0e-5 5.5e-5 6.0e-5 6.6e-5];
[Nmu, Nk] = deal(numel(mu0_vals), numel(ksk_vals));

lap_grid    = zeros(Nmu, Nk);
dlap_grid   = zeros(Nmu, Nk);
secrms_grid = zeros(Nmu, Nk);

fprintf('\nReference lap: %.3f s. Running %d sims...\n\n', ref_lap, Nmu*Nk);
t0 = tic;
% NOTE: lap_sim_v05 internally uses 'i' and 'j' as loop counters and they
% leak into our workspace through evalc(). Use unique names here.
for ii = 1:Nmu
    for jj = 1:Nk
        car = car_baseline;
        car.tyre.mu_0        = mu0_vals(ii);
        car.tyre.load_sens_k = ksk_vals(jj);
        evalc('lap_sim_v05;');
        evalc('c = correlate_sim(sim05, track, sectors, ''quiet'', false);');
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
fprintf('\nCharter (±1%%): %s\n', ternary(charter_pass,'PASS','FAIL'));

%% ---- Compare against Phase 5 Step 4 ---------------------------------
prev_path = fullfile(fileparts(mfilename('fullpath')), 'phase5_step4_result.mat');
if exist(prev_path, 'file')
    P = load(prev_path);
    fprintf('\n=== Phase 5 vs Phase 6 (same grid, different reference) ===\n');
    fprintf('  Phase 5 (xls): mu_0=%.2f  k=%.2g  Δlap=%+.3f s  rms=%.3f s\n', ...
            P.result.best_rms.mu_0, P.result.best_rms.load_sens_k, ...
            P.result.best_rms.dlap, P.result.best_rms.rms);
    fprintf('  Phase 6 (ibt): mu_0=%.2f  k=%.2g  Δlap=%+.3f s  rms=%.3f s\n', ...
            mu0_vals(ir), ksk_vals(jr), ...
            dlap_grid(ir,jr), secrms_grid(ir,jr));
end

%% ---- Heatmaps -------------------------------------------------------
figure('Name','Phase 6 recalibration','NumberTitle','off', ...
       'Position',[100 100 1200 500]);
subplot(1,2,1);
imagesc(ksk_vals, mu0_vals, dlap_grid); axis xy; colorbar;
xlabel('load\_sens\_k [1/N]'); ylabel('mu\_0 [-]');
title('Δlap vs ref [s]');
set(gca,'XTick',ksk_vals,'YTick',mu0_vals);
hold on; plot(ksk_vals(jl), mu0_vals(il), 'kp','MarkerSize',14,'MarkerFaceColor','y');

subplot(1,2,2);
imagesc(ksk_vals, mu0_vals, secrms_grid); axis xy; colorbar;
xlabel('load\_sens\_k [1/N]'); ylabel('mu\_0 [-]');
title('Sector RMS Δt [s]');
set(gca,'XTick',ksk_vals,'YTick',mu0_vals);
hold on; plot(ksk_vals(jr), mu0_vals(ir), 'kp','MarkerSize',14,'MarkerFaceColor','r');

%% ---- Sector report at best fit --------------------------------------
fprintf('\n=== Sector report at best-fit (sector_rms) ===\n');
car.tyre.mu_0        = mu0_vals(ir);
car.tyre.load_sens_k = ksk_vals(jr);
evalc('lap_sim_v05;');
corr_best = correlate_sim(sim05, track, sectors, ...
    sprintf('Phase 6 best: mu0=%.2f, k=%.2g', mu0_vals(ir), ksk_vals(jr)), ...
    true);
car = car_baseline;

%% ---- Save -----------------------------------------------------------
result = struct();
result.source       = 'IBT (Phase 6)';
result.mu0_vals     = mu0_vals;
result.ksk_vals     = ksk_vals;
result.lap_grid     = lap_grid;
result.dlap_grid    = dlap_grid;
result.secrms_grid  = secrms_grid;
result.best_lap     = struct('mu_0',mu0_vals(il),'load_sens_k',ksk_vals(jl), ...
                             'dlap',dlap_grid(il,jl),'rms',secrms_grid(il,jl));
result.best_rms     = struct('mu_0',mu0_vals(ir),'load_sens_k',ksk_vals(jr), ...
                             'dlap',dlap_grid(ir,jr),'rms',secrms_grid(ir,jr));
result.corr_best    = corr_best;
result.ref_lap      = ref_lap;

outdir  = fileparts(mfilename('fullpath'));
outpath = fullfile(outdir, 'phase6_recalibrate_result.mat');
save(outpath, 'result');
fprintf('\nSaved: %s\nDone.\n', outpath);

%% ---- helper ---------------------------------------------------------
function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
