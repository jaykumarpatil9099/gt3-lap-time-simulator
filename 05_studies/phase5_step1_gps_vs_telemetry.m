%% phase5_step1_gps_vs_telemetry.m
%  Phase 5, Step 1 — GPS vs telemetry track-source experiment.
%
%  Builds the track from BOTH sources, runs v01..v05 on each, captures
%  lap times and peak curvature, and tabulates the deltas.
%
%  OUTPUT
%    result — struct with fields:
%      .telemetry.lap(1..5)       [s]
%      .telemetry.peak_kappa      [1/m]
%      .telemetry.min_radius      [m]
%      .gps.lap(1..5)             [s]
%      .gps.peak_kappa            [1/m]
%      .gps.min_radius            [m]
%      .delta_lap_sec(1..5)       gps - telemetry [s]
%      .ref_laptime               [s]
%
%  USAGE (from project root)
%    >> startup_project
%    >> import_reference_lap
%    >> run('05_studies/phase5_step1_gps_vs_telemetry.m')
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-21

%% ------------------------------------------------------------------------
%  0. Sanity checks
%  ------------------------------------------------------------------------
if ~exist('ref', 'var'),  error('Run import_reference_lap first.'); end
if ~exist('car', 'var'),  error('Run startup_project first.');      end

fprintf('\n==========================================================\n');
fprintf(' Phase 5 Step 1 — GPS vs Telemetry track source experiment \n');
fprintf('==========================================================\n\n');

result = struct();
result.ref_laptime = 8*60 + 11.341;

%% ------------------------------------------------------------------------
%  1. TELEMETRY SOURCE RUN
%  ------------------------------------------------------------------------
fprintf('--- [1/2] TELEMETRY source ---\n');
track_source = 'telemetry';
build_track;

result.telemetry.peak_kappa = max(track.kappa);
result.telemetry.min_radius = 1 / result.telemetry.peak_kappa;

lap_sim_v01;  result.telemetry.lap(1) = sim.lap_time;    % v01 names output 'sim'
lap_sim_v02;  result.telemetry.lap(2) = sim02.lap_time;
lap_sim_v03;  result.telemetry.lap(3) = sim03.lap_time;
lap_sim_v04;  result.telemetry.lap(4) = sim04.lap_time;
lap_sim_v05;  result.telemetry.lap(5) = sim05.lap_time;

% keep copies for later sector analysis
sim05_tel = sim05;
track_tel = track;

%% ------------------------------------------------------------------------
%  2. GPS SOURCE RUN
%  ------------------------------------------------------------------------
fprintf('\n--- [2/2] GPS source ---\n');
track_source = 'gps';
build_track;

result.gps.peak_kappa = max(track.kappa);
result.gps.min_radius = 1 / result.gps.peak_kappa;

lap_sim_v01;  result.gps.lap(1) = sim.lap_time;
lap_sim_v02;  result.gps.lap(2) = sim02.lap_time;
lap_sim_v03;  result.gps.lap(3) = sim03.lap_time;
lap_sim_v04;  result.gps.lap(4) = sim04.lap_time;
lap_sim_v05;  result.gps.lap(5) = sim05.lap_time;

sim05_gps = sim05;
track_gps = track;

%% ------------------------------------------------------------------------
%  3. DELTAS
%  ------------------------------------------------------------------------
result.delta_lap_sec = result.gps.lap - result.telemetry.lap;

%% ------------------------------------------------------------------------
%  4. REPORT
%  ------------------------------------------------------------------------
fprintf('\n==========================================================\n');
fprintf(' RESULTS                                                   \n');
fprintf('==========================================================\n');
fprintf('Reference lap:   %.3f s (8:%.3f)\n\n', ...
        result.ref_laptime, result.ref_laptime - 480);

fprintf('Peak curvature [1/m]:\n');
fprintf('  telemetry: %.5f   (R_min = %.2f m)\n', ...
        result.telemetry.peak_kappa, result.telemetry.min_radius);
fprintf('  gps:       %.5f   (R_min = %.2f m)\n\n', ...
        result.gps.peak_kappa, result.gps.min_radius);

fprintf('%-6s %12s %12s %12s %10s %10s\n', ...
        'Ver', 'tel [s]', 'gps [s]', 'Δ gps-tel', 'Δ tel-ref', 'Δ gps-ref');
fprintf('%s\n', repmat('-', 1, 70));
for v = 1:5
    dt_tel = result.telemetry.lap(v) - result.ref_laptime;
    dt_gps = result.gps.lap(v)       - result.ref_laptime;
    fprintf('v0%d  %12.3f %12.3f %12.3f %9.2f%% %9.2f%%\n', ...
            v, result.telemetry.lap(v), result.gps.lap(v), ...
            result.delta_lap_sec(v), ...
            100 * dt_tel / result.ref_laptime, ...
            100 * dt_gps / result.ref_laptime);
end
fprintf('%s\n', repmat('-', 1, 70));

%% ------------------------------------------------------------------------
%  5. SAVE
%  ------------------------------------------------------------------------
outdir  = fileparts(mfilename('fullpath'));
outpath = fullfile(outdir, 'phase5_step1_result.mat');
save(outpath, 'result', 'sim05_tel', 'sim05_gps', 'track_tel', 'track_gps');
fprintf('\nSaved: %s\n', outpath);
fprintf('\nDone.\n');
