%% phase5_step2_sector_analysis.m
%  Phase 5, Step 2 — Sector correlation of v05 against the reference lap.
%
%  Uses the Step 1 result (both sources), and runs `correlate_sim` on:
%    (a) v05 on telemetry source  — driver-line correlation
%    (b) v05 on GPS source        — geometric-line correlation
%
%  Sector convention: 6 equal-length sectors (N24 is 25.2 km, so ~4.2 km
%  each). This is a pragmatic first cut; race-team sector maps can replace
%  the boundaries in a later revision.
%
%  USAGE  (from project root, after running Step 1)
%    >> load('05_studies/phase5_step1_result.mat')
%    >> run('05_studies/phase5_step2_sector_analysis.m')
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-21

if ~exist('sim05_tel','var') || ~exist('track_tel','var') ...
        || ~exist('sim05_gps','var') || ~exist('track_gps','var')
    error(['Step 1 workspace not loaded. Run:\n' ...
           '  load(''05_studies/phase5_step1_result.mat'')']);
end

fprintf('\n==========================================================\n');
fprintf(' Phase 5 Step 2 — Sector correlation (v05, both sources)   \n');
fprintf('==========================================================\n');

% 6 equal-length sectors (boundary vector length 7)
edges_tel = linspace(0, track_tel.dist(end), 7);
edges_gps = linspace(0, track_gps.dist(end), 7);
sectors_tel = [edges_tel(1:end-1)' edges_tel(2:end)'];
sectors_gps = [edges_gps(1:end-1)' edges_gps(2:end)'];

corr_tel = correlate_sim(sim05_tel, track_tel, sectors_tel, ...
                         'v05 — telemetry source', true);

corr_gps = correlate_sim(sim05_gps, track_gps, sectors_gps, ...
                         'v05 — GPS source', true);

%% ---- Side-by-side sector Δt table -------------------------------------
fprintf('\n=== Side-by-side: Δt per sector (sim - ref) ===\n');
fprintf('%-3s %10s %10s %10s\n', 'S', 'tel Δt', 'gps Δt', 'gps - tel');
fprintf('%s\n', repmat('-',1,42));
for k = 1:numel(corr_tel.per_sector)
    dt_t = corr_tel.per_sector(k).dt_sector;
    dt_g = corr_gps.per_sector(k).dt_sector;
    fprintf('%-3d %+10.3f %+10.3f %+10.3f\n', k, dt_t, dt_g, dt_g - dt_t);
end
fprintf('%s\n', repmat('-',1,42));

%% ---- Save -------------------------------------------------------------
outdir  = fileparts(mfilename('fullpath'));
outpath = fullfile(outdir, 'phase5_step2_result.mat');
save(outpath, 'corr_tel', 'corr_gps', 'sectors_tel', 'sectors_gps');
fprintf('\nSaved: %s\n', outpath);
fprintf('\nDone.\n');
