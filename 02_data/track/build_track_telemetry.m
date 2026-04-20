%% build_track_telemetry.m
%  Builds the track data struct from REFERENCE LAP TELEMETRY.
%  Curvature is derived from kappa = a_lat / v^2, so the resulting
%  centerline is the driver's RACING LINE, not the geometric centerline.
%
%  HISTORICAL ROLE
%  ---------------
%  This was the original path for building the track, and it is kept as the
%  telemetry-based option of the two-source dispatcher (see build_track.m).
%  It is the right choice whenever we want the simulator to run on the same
%  line the reference driver actually took (useful for diagnosing DRIVER
%  vs. CAR effects in correlation).
%
%  PHYSICS
%    For circular motion:  a_lat = v^2 / R
%    Curvature kappa = 1/R = a_lat / v^2
%    where a_lat is in [m/s^2] and v is in [m/s].
%
%  WHY THE TWO-STAGE FILTER
%  ------------------------
%  Stage 1 — median filter on a_lat (15 samples, ~7.5 m):
%    The raw lateral-g from iRacing contains transient spikes from kerb
%    strikes, bumps, and suspension events. Those are NOT steady-state
%    cornering — a QSS solver has no business seeing them. A median filter
%    kills outliers while preserving step edges (real corner entries/exits).
%
%  Stage 2 — moving average on kappa (20 m):
%    After despiking, we apply a small moving average for final cleanup.
%    The previous single-stage 50 m MA destroyed 34% of peak curvature
%    (R_min went from 13.9 m to 21.2 m) which made v01 ~36 s too fast.
%    The 20 m MA with a clean signal in front of it preserves ~76% of peak
%    curvature — still imperfect, but the racing-line peak is an upper
%    bound on the true geometric peak, so 76% on racing-line data is not
%    the same as 76% of ground truth. See Entry 008 and Entry 009 in the
%    logbook for the full diagnosis.
%
%  INPUT:  'ref' struct must be in workspace (run import_reference_lap first)
%  OUTPUT: 'track' struct in workspace + saved .mat file (n24_track.mat)
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-16  (originally as build_track.m; split out 2026-04-20)

%% ========================================================================
%  1. CHECK THAT REFERENCE DATA EXISTS
%  ========================================================================

if ~exist('ref', 'var')
    error(['Reference lap data not found in workspace. ' ...
           'Run import_reference_lap.m first.']);
end

fprintf('\n=== Building Track Data (telemetry / racing-line source) ===\n');

%% ========================================================================
%  2. COMPUTE RAW CURVATURE FROM TELEMETRY
%  ========================================================================
%  kappa = a_lat / v^2
%
%  Two issues we need to handle:
%
%  Issue 1: At low speed, v^2 is very small, so curvature blows up to
%  infinity even for tiny lateral accelerations. This is noise, not real
%  curvature. We clamp the minimum speed to avoid division-by-nearly-zero.
%
%  Issue 2: The raw lateral g signal from iRacing is noisy (100 Hz sensor
%  noise + kerb vibrations + bumps). If we use it directly, our curvature
%  profile will be spiky and the simulator will produce unrealistic speed
%  oscillations. We need to smooth it — but carefully (see Stage 1/2 notes
%  in the header).

g = 9.81;  % [m/s^2]

% ---- STAGE 1: Median-filter the lateral g BEFORE computing curvature ----
%  Window: 15 samples (~7.5 m at 0.5 m spacing). Wide enough to span
%  a kerb hit (~1-3 m), narrow enough to preserve real corner shapes.

a_lat_raw_ms2 = ref.g_lat * g;   % [m/s^2]

median_window = 15;  % samples (~7.5 m)
if mod(median_window, 2) == 0
    median_window = median_window + 1;
end

a_lat_filtered = medfilt1(a_lat_raw_ms2, median_window);

fprintf('Median filter applied: window = %d samples\n', median_window);
fprintf('  Raw g_lat range:      %.2f to %.2f g\n', min(ref.g_lat), max(ref.g_lat));
fprintf('  Filtered g_lat range: %.2f to %.2f g\n', ...
        min(a_lat_filtered/g), max(a_lat_filtered/g));

% ---- Compute curvature from FILTERED lateral g ----
v_clamped = max(ref.v, 10);    % [m/s] minimum 10 m/s (prevents 1/0 at standstill)
kappa_raw = a_lat_filtered ./ (v_clamped.^2);   % [1/m]

fprintf('Curvature (after median filter). Range: %.6f to %.6f [1/m]\n', ...
        min(kappa_raw), max(kappa_raw));

%% ========================================================================
%  3. SMOOTH THE CURVATURE (Stage 2 — gentle moving average)
%  ========================================================================
%  After the median filter removed spikes, we apply a SMALLER moving
%  average (20 m instead of 50 m) for final cleanup. This preserves
%  much more of the real corner peaks.

ds_mean = ref.dist(end) / (length(ref.dist) - 1);   % [m/sample]
fprintf('Mean sample spacing: %.3f m\n', ds_mean);

smooth_window_m = 20;                                 % [m]
smooth_window_samples = round(smooth_window_m / ds_mean);

if mod(smooth_window_samples, 2) == 0
    smooth_window_samples = smooth_window_samples + 1;
end

fprintf('Moving average window: %d samples (~%.0f m)\n', ...
        smooth_window_samples, smooth_window_samples * ds_mean);

kappa_smooth = movmean(kappa_raw, smooth_window_samples);

fprintf('Final curvature range: %.6f to %.6f [1/m]\n', ...
        min(kappa_smooth), max(kappa_smooth));
fprintf('Peak preservation: %.0f%% (target: >85%%)\n', ...
        max(abs(kappa_smooth)) / max(abs(a_lat_raw_ms2 ./ (v_clamped.^2))) * 100);

%% ========================================================================
%  4. RESAMPLE TO UNIFORM DISTANCE SPACING
%  ========================================================================
%  The raw telemetry is sampled at uniform TIME (100 Hz), but non-uniform
%  DISTANCE (because speed varies). The simulator works in distance-domain
%  (it steps along the track meter by meter), so we resample to a uniform
%  1 m distance grid: fine enough to capture corner shapes, coarse enough
%  to keep the sim fast (~25,000 points for the full lap).

ds_target = 1.0;   % [m]
dist_uniform = (0 : ds_target : ref.dist(end))';

% Interpolate curvature onto the uniform grid
kappa_uniform = interp1(ref.dist, kappa_smooth, dist_uniform, 'linear', 'extrap');

% Reference channels — kept on the track struct for correlation plots
v_ref_uniform     = interp1(ref.dist, ref.v,       dist_uniform, 'linear', 'extrap');
g_lat_uniform     = interp1(ref.dist, ref.g_lat,   dist_uniform, 'linear', 'extrap');
g_long_uniform    = interp1(ref.dist, ref.g_long,  dist_uniform, 'linear', 'extrap');
gear_uniform      = interp1(ref.dist, ref.gear,    dist_uniform, 'nearest', 'extrap');
throttle_uniform  = interp1(ref.dist, ref.throttle, dist_uniform, 'linear', 'extrap');
brake_uniform     = interp1(ref.dist, ref.brake,   dist_uniform, 'linear', 'extrap');

fprintf('Resampled to %.1f m spacing: %d points\n', ds_target, length(dist_uniform));

%% ========================================================================
%  5. BUILD THE TRACK STRUCT
%  ========================================================================

track = struct();

% Track geometry (what the simulator uses as input)
track.dist   = dist_uniform;          % [m]   distance along track
track.kappa  = abs(kappa_uniform);    % [1/m] unsigned curvature (QSS: sign irrelevant)
track.ds     = ds_target;             % [m]   sample spacing
track.n      = length(dist_uniform);  % [-]   number of points
track.length = ref.dist(end);         % [m]   total track length

% Reference data resampled to track grid (for correlation plots)
track.ref.v         = v_ref_uniform;       % [m/s]
track.ref.g_lat     = g_lat_uniform;       % [g]
track.ref.g_long    = g_long_uniform;      % [g]
track.ref.gear      = gear_uniform;        % [-]
track.ref.throttle  = throttle_uniform;    % [%]
track.ref.brake     = brake_uniform;       % [%]

% Metadata
track.meta.name        = 'Nürburgring 24h (Nordschleife + GP combined)';
track.meta.source      = 'Derived from iRacing telemetry (racing line, not geometric centerline)';
track.meta.ref_laptime = 8*60 + 11.341;   % [s]
track.meta.smooth_m    = smooth_window_m;
track.meta.created     = datestr(now, 'yyyy-mm-dd');
track.meta.notes       = ['Curvature computed from a_lat/v^2 with a 15-sample ' ...
                          'median filter on a_lat, smoothed with ' ...
                          num2str(smooth_window_m) ' m moving average, ' ...
                          'resampled to ' num2str(ds_target) ' m uniform spacing. ' ...
                          'Unsigned curvature (left/right direction discarded for QSS). ' ...
                          'Elevation not yet included.'];

%% ========================================================================
%  6. SAVE
%  ========================================================================

outdir  = fullfile(pwd, '02_data', 'track');
outpath = fullfile(outdir, 'n24_track.mat');
save(outpath, 'track');
fprintf('Saved to: %s\n', outpath);

%% ========================================================================
%  7. VERIFICATION PLOTS
%  ========================================================================

figure('Name', 'Track Data Verification (telemetry source)', ...
       'NumberTitle', 'off', 'Position', [100, 100, 1200, 700]);

% --- Curvature vs Distance ---
subplot(3,1,1);
plot(track.dist/1000, track.kappa, 'b', 'LineWidth', 0.8);
ylabel('Curvature [1/m]');
title(sprintf('Track: %s — %.3f km', track.meta.name, track.length/1000));
grid on;
xlim([0, track.length/1000]);

% --- Equivalent corner radius vs Distance ---
%  Radius = 1/kappa. More intuitive: "this is a 50 m radius corner".
%  Clamp to max 2000 m to avoid infinity on straights.
subplot(3,1,2);
radius = min(1 ./ max(track.kappa, 1e-6), 2000);
plot(track.dist/1000, radius, 'r', 'LineWidth', 0.8);
ylabel('Corner radius [m]');
grid on;
xlim([0, track.length/1000]);
ylim([0, 2000]);

% --- Reference speed overlaid with curvature ---
subplot(3,1,3);
yyaxis left;
plot(track.dist/1000, track.ref.v * 3.6, 'b', 'LineWidth', 0.8);
ylabel('Speed [km/h]');
ylim([0, 300]);
yyaxis right;
plot(track.dist/1000, track.kappa, 'Color', [0.8 0 0 0.4], 'LineWidth', 0.5);
ylabel('Curvature [1/m]');
xlabel('Distance [km]');
grid on;
xlim([0, track.length/1000]);
title('Speed vs Curvature — high curvature should align with low speed');

fprintf('\n=== Track Build Complete (telemetry source) ===\n\n');
