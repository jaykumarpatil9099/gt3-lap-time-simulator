%% build_track_from_gps.m
%  Builds the track data struct from the geometric GPS centerline (extracted
%  from Nurburgring Combined Track.pxt), NOT from a_lat/v² telemetry.
%
%  WHY THIS SCRIPT EXISTS
%  -----------------------
%  The original build_track.m derives curvature as κ = a_lat / v², which
%  has two fundamental problems:
%
%    1. It uses the driver's RACING LINE, not the geometric centerline.
%       Racing lines clip apexes — corners appear tighter than they are.
%
%    2. The lateral-g signal is noisy (kerb strikes, bumps), so we must
%       smooth heavily, which ROUNDS OFF real corners. Even with two-stage
%       filtering we only preserved 76% of peak curvature.
%
%  This script solves both problems at once by computing κ geometrically
%  from the centerline (x, y) coordinates:
%
%      κ(s) = (x' y'' − y' x'') / (x'² + y'²)^(3/2)
%
%  with s = arc length. No speed signal, no g-sensor noise. Pure geometry.
%
%  INPUT:  02_data/track/pxt_centerline.csv  (x, y, elev, lat, lon, dist)
%          reference lap telemetry (for populating track.ref channels)
%
%  OUTPUT: 02_data/track/n24_track_gps.mat  (keeps same 'track' struct shape
%          as the old build_track.m so v01..v04 run unchanged)
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-18

%% ========================================================================
%  1. LOAD GPS CENTERLINE (extracted from .pxt GPSMapStream)
%  ========================================================================

fprintf('\n=== Building Track Data From GPS Centerline ===\n');

% Resolve paths relative to THIS SCRIPT, not the caller's cwd, because the
% build_track dispatcher calls us via run() which temporarily changes cwd
% to this folder. mfilename('fullpath') is stable against that.
script_dir = fileparts(mfilename('fullpath'));
csv_path   = fullfile(script_dir, 'pxt_centerline.csv');
if ~isfile(csv_path)
    error(['GPS centerline CSV not found: %s\n' ...
           'Run the .pxt → CSV extraction first.'], csv_path);
end

T = readtable(csv_path);
s_raw    = T.dist_m;        % [m]   cumulative distance along centerline
x_raw    = T.x_m;           % [m]   local easting (relative to first sample)
y_raw    = T.y_m;           % [m]   local northing
elev_raw = T.elev_m;        % [m]   elevation above sea level
lat_raw  = T.lat_deg;       % [deg] WGS84 latitude
lon_raw  = T.lon_deg;       % [deg] WGS84 longitude

fprintf('Loaded %d centerline points (native spacing %.2f m).\n', ...
        length(s_raw), mean(diff(s_raw)));
fprintf('Track length (from .pxt): %.1f m\n', s_raw(end));

%% ========================================================================
%  2. RESAMPLE TO 1 m UNIFORM DISTANCE
%  ========================================================================
%  The native spacing is ~3 m. We resample to 1 m so the simulator's
%  distance-domain loop doesn't have to do sub-metre bookkeeping.
%  We use linear interpolation for x, y, elev (pchip could be used later
%  if we see staircase artifacts, but linear is fine for a smooth 3-m
%  centerline).

ds_target = 1.0;                                    % [m]
dist = (0 : ds_target : s_raw(end))';               % [m]

x   = interp1(s_raw, x_raw,    dist, 'linear');
y   = interp1(s_raw, y_raw,    dist, 'linear');
z   = interp1(s_raw, elev_raw, dist, 'linear');
lat = interp1(s_raw, lat_raw,  dist, 'linear');
lon = interp1(s_raw, lon_raw,  dist, 'linear');

fprintf('Resampled to %.1f m spacing: %d points.\n', ds_target, length(dist));

%% ========================================================================
%  3. GEOMETRIC CURVATURE FROM (x, y)
%  ========================================================================
%  Use central differences. Because ds = 1 m is constant and our data
%  are noisy at the finite-difference level (interpolated from 3-m
%  samples), we apply a LIGHT smoothing on x and y BEFORE differentiation
%  — this is equivalent to smoothing κ, but keeps the geometry crisp and
%  avoids the "fake peak" problem that a big filter on κ itself creates.

% Light pre-smoothing of the coordinate series (3 m window).
% Rationale: the 3-m native spacing introduces small-scale quantisation
% noise. A 3-m moving average of x, y kills the noise without touching
% corner shapes (3 m is well inside the tightest corner radius).
smooth_xy_m = 3;
x_s = movmean(x, smooth_xy_m);
y_s = movmean(y, smooth_xy_m);

% First and second derivatives (ds = 1 m, central differences).
dx  = gradient(x_s, ds_target);
dy  = gradient(y_s, ds_target);
d2x = gradient(dx,  ds_target);
d2y = gradient(dy,  ds_target);

% κ = (x' y'' − y' x'') / (x'² + y'²)^(3/2)
v2 = dx.^2 + dy.^2;
kappa_raw = (dx .* d2y - dy .* d2x) ./ max(v2, 1e-9).^(1.5);

fprintf('Raw geometric |κ|: peak = %.5f 1/m  (R_min = %.2f m)\n', ...
        max(abs(kappa_raw)), 1/max(abs(kappa_raw)));

%% ========================================================================
%  4. FINAL SMOOTHING (small, curvature-preserving)
%  ========================================================================
%  A 5 m moving average cleans the residual noise without materially
%  changing the real corner peaks. Unlike the telemetry-derived path, we
%  do NOT need a large window — the signal is already clean.

smooth_k_m = 5;
smooth_k_samples = round(smooth_k_m / ds_target);
if mod(smooth_k_samples, 2) == 0
    smooth_k_samples = smooth_k_samples + 1;
end
kappa = movmean(kappa_raw, smooth_k_samples);

peak_raw   = max(abs(kappa_raw));
peak_final = max(abs(kappa));
fprintf('Smoothed |κ| (%d-sample MA): peak = %.5f 1/m  (R_min = %.2f m)\n', ...
        smooth_k_samples, peak_final, 1/peak_final);
fprintf('Peak preservation after smoothing: %.0f%%\n', peak_final / peak_raw * 100);

% Distribution
pct = [50 75 90 95 99 99.5 100];
fprintf('κ percentiles (1/m):\n');
for p = pct
    fprintf('  p%-5.1f : %.5f  (R = %.1f m)\n', p, prctile(abs(kappa), p), ...
            1 / max(prctile(abs(kappa), p), 1e-6));
end

%% ========================================================================
%  5. BRING REFERENCE TELEMETRY ONTO THE GPS DISTANCE GRID
%  ========================================================================
%  The 'ref' struct (from import_reference_lap) is indexed by RACING-LINE
%  distance (~25,206 m). The GPS centerline is ~25,176 m. The ~30 m gap
%  (0.12%) is the racing line being slightly shorter because it clips
%  apexes.
%
%  For correlation plots we want a single distance axis. Simplest valid
%  mapping: linearly rescale ref.dist to the GPS length. This is within
%  the finite-difference spatial resolution of the sim and preserves the
%  ordering of events (corner entries stay at corner entries).

if exist('ref', 'var')
    s_ref_scaled = ref.dist * (dist(end) / ref.dist(end));

    v_ref        = interp1(s_ref_scaled, ref.v,        dist, 'linear', 'extrap');
    g_lat_ref    = interp1(s_ref_scaled, ref.g_lat,    dist, 'linear', 'extrap');
    g_long_ref   = interp1(s_ref_scaled, ref.g_long,   dist, 'linear', 'extrap');
    gear_ref     = interp1(s_ref_scaled, ref.gear,     dist, 'nearest', 'extrap');
    throttle_ref = interp1(s_ref_scaled, ref.throttle, dist, 'linear', 'extrap');
    brake_ref    = interp1(s_ref_scaled, ref.brake,    dist, 'linear', 'extrap');

    fprintf('Reference lap telemetry resampled onto GPS grid.\n');
    fprintf('  ref length = %.1f m  →  scaled to %.1f m (scale = %.5f).\n', ...
            ref.dist(end), dist(end), dist(end)/ref.dist(end));
else
    warning(['''ref'' struct not in workspace — track.ref channels will ' ...
             'be NaN. Run import_reference_lap first for full correlation.']);
    v_ref        = nan(size(dist));
    g_lat_ref    = nan(size(dist));
    g_long_ref   = nan(size(dist));
    gear_ref     = nan(size(dist));
    throttle_ref = nan(size(dist));
    brake_ref    = nan(size(dist));
end

%% ========================================================================
%  6. BUILD THE TRACK STRUCT (schema-compatible with build_track.m)
%  ========================================================================

track = struct();

% Geometry used by the solver
track.dist   = dist;                 % [m]   distance along track
track.kappa  = abs(kappa);           % [1/m] unsigned curvature (QSS: sign irrelevant)
track.ds     = ds_target;            % [m]   sample spacing
track.n      = length(dist);         % [−]
track.length = dist(end);            % [m]   total length

% NEW: geometry channels only available because we used GPS data
track.x      = x;                    % [m]   centerline easting (local)
track.y      = y;                    % [m]   centerline northing (local)
track.z      = z;                    % [m]   centerline elevation (MSL)
track.lat    = lat;                  % [deg] WGS84 latitude
track.lon    = lon;                  % [deg] WGS84 longitude
track.kappa_signed = kappa;          % [1/m] signed curvature (left + / right −)

% Reference channels on the GPS grid
track.ref.v         = v_ref;
track.ref.g_lat     = g_lat_ref;
track.ref.g_long    = g_long_ref;
track.ref.gear      = gear_ref;
track.ref.throttle  = throttle_ref;
track.ref.brake     = brake_ref;

% Metadata
track.meta.name        = 'Nürburgring 24h (Nordschleife + GP combined) — GPS centerline';
track.meta.source      = ['Geometric centerline from ' ...
                          '02_data/track/Nurburgring Combined Track.pxt (GPSMapStream)'];
track.meta.ref_laptime = 8*60 + 11.341;
track.meta.smooth_xy_m = smooth_xy_m;
track.meta.smooth_k_m  = smooth_k_m;
track.meta.peak_pres   = peak_final / peak_raw * 100;
track.meta.created     = datestr(now, 'yyyy-mm-dd');
if exist('ref', 'var')
    ref_len = ref.dist(end);
else
    ref_len = NaN;
end
track.meta.notes = sprintf(['k computed as (x''y''''-y''x'''')/|v|^3 from GPS ' ...
                            'centerline. XY pre-smoothed with %d m MA, k ' ...
                            'post-smoothed with %d m MA. No lateral-g signal ' ...
                            'used. Reference telemetry rescaled from %.1f m ' ...
                            '(racing line) to %.1f m (centerline).'], ...
                            smooth_xy_m, smooth_k_m, ref_len, dist(end));

%% ========================================================================
%  7. SAVE
%  ========================================================================

% Use the same script-relative path resolved in §1.
outpath = fullfile(script_dir, 'n24_track_gps.mat');
save(outpath, 'track');
fprintf('Saved: %s\n', outpath);

%% ========================================================================
%  8. VERIFICATION PLOTS
%  ========================================================================

figure('Name', 'Track (GPS) verification', 'NumberTitle', 'off', ...
       'Position', [100, 100, 1300, 900]);

subplot(3,2,[1 3]);
plot(track.x, track.y, 'b', 'LineWidth', 0.7); hold on;
plot(track.x(1), track.y(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot(track.x(end), track.y(end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
axis equal; grid on;
xlabel('Easting [m]'); ylabel('Northing [m]');
title(sprintf('%s  (%.3f km)', track.meta.name, track.length/1000));
legend('Centerline', 'Start', 'End', 'Location', 'best');

subplot(3,2,2);
plot(track.dist/1000, track.kappa, 'b', 'LineWidth', 0.7);
xlabel('Distance [km]'); ylabel('|κ| [1/m]');
title('Curvature vs distance'); grid on;
xlim([0 track.length/1000]);

subplot(3,2,4);
R = min(1 ./ max(track.kappa, 1e-6), 2000);
plot(track.dist/1000, R, 'r', 'LineWidth', 0.6);
xlabel('Distance [km]'); ylabel('Radius [m]');
title('Equivalent corner radius'); grid on;
xlim([0 track.length/1000]); ylim([0 2000]);

subplot(3,2,5);
plot(track.dist/1000, track.z, 'k', 'LineWidth', 0.8);
xlabel('Distance [km]'); ylabel('Elevation [m MSL]');
title('Elevation profile'); grid on;
xlim([0 track.length/1000]);

subplot(3,2,6);
if any(~isnan(track.ref.v))
    yyaxis left;
    plot(track.dist/1000, track.ref.v*3.6, 'b', 'LineWidth', 0.7);
    ylabel('Ref speed [km/h]'); ylim([0 300]);
    yyaxis right;
    plot(track.dist/1000, track.kappa, 'Color', [0.8 0 0 0.4], 'LineWidth', 0.4);
    ylabel('|κ| [1/m]');
    xlabel('Distance [km]');
    title('Speed vs curvature (sanity check)'); grid on;
    xlim([0 track.length/1000]);
else
    text(0.1, 0.5, 'Load ref first for speed-vs-κ plot', 'Units', 'normalized');
    axis off;
end

fprintf('\n=== GPS Track Build Complete ===\n\n');
