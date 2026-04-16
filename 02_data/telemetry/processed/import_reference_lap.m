%% import_reference_lap.m
%  Reads the PI Toolbox export of the iRacing reference lap and produces
%  a clean MATLAB struct 'ref' for use by the simulator and correlation scripts.
%
%  INPUT:  reference_lap_8m11s.xlsx  (PI Toolbox export, Group 1 channels)
%  OUTPUT: ref struct in workspace + saved .mat file
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-16
%
%  USAGE:
%    Make sure you have run startup_project.m first, then:
%    >> import_reference_lap

%% ========================================================================
%  1. READ THE EXCEL FILE
%  ========================================================================

fprintf('\n=== Importing Reference Lap ===\n');

% Path to the exported file (relative to project root)
filepath = fullfile('02_data', 'telemetry', 'processed', 'reference_lap.xls');

% Read the file using readtable
% The telemetry data is on the 'Channel Data' sheet.
% Sheet 1 ('Outing Information') is just metadata — we skip it.
raw = readtable(filepath, 'Sheet', 'Channel Data');

% Display what we got
fprintf('File loaded: %s\n', filepath);
fprintf('Rows: %d | Columns: %d\n', height(raw), width(raw));
fprintf('Column names:\n');
disp(raw.Properties.VariableNames');

%% ========================================================================
%  2. MAP COLUMN NAMES TO STANDARD NAMES
%  ========================================================================
%  PI Toolbox column names vary between installations. This mapping section
%  translates whatever PI gave us into our standard names.
%  If your column names are different, edit ONLY this section.

% --- Adjust these strings to match your actual column headers ---
%  MATLAB's readtable converts spaces in headers to underscores or
%  removes them. Check raw.Properties.VariableNames output above
%  to see what MATLAB actually used, and update here if needed.
col_time      = 'Time';
col_speed     = 'Speed';
col_throttle  = 'Throttle';
col_brake     = 'Brake';
col_gear      = 'GearNumber';
col_rpm       = 'RPM';
col_g_lat     = 'AccelLateral';              % from 'Accel Lateral'
col_g_long    = 'AccelLongitudinal';         % from 'Accel Longitudinal'
col_steer     = 'SteeringWheelAngle';        % from 'Steering Wheel Angle'

% --- Extract columns into simple arrays ---
% If a column name doesn't match, MATLAB will error here.
% Fix by updating the col_xxx variable above to match your header.
t_raw       = raw.(col_time);
v_kmh       = raw.(col_speed);
throttle    = raw.(col_throttle);
brake       = raw.(col_brake);
gear        = raw.(col_gear);
rpm         = raw.(col_rpm);
g_lat       = raw.(col_g_lat);
g_long      = raw.(col_g_long);
steer       = raw.(col_steer);

fprintf('Columns mapped successfully.\n');

%% ========================================================================
%  3. CLEAN AND CONVERT UNITS
%  ========================================================================

% --- Time: zero to start of lap ---
%  PI Toolbox exports session time (time since session start), not lap time.
%  We subtract the first value so the lap starts at t = 0.
t = t_raw - t_raw(1);                   % [s] time from lap start

% --- Speed: convert km/h to m/s ---
%  All physics in SI units.
v = v_kmh / 3.6;                        % [m/s]

% --- Verify lap duration ---
lap_time = t(end) - t(1);               % [s]
fprintf('Lap duration from data: %.3f s\n', lap_time);
fprintf('Expected (8:11.341):   %.3f s\n', 8*60 + 11.341);

%% ========================================================================
%  4. COMPUTE DISTANCE (since PI export had no distance channel)
%  ========================================================================
%  Distance = cumulative integral of speed over time.
%  We use the trapezoidal rule: for each time step,
%    ds = 0.5 * (v(i) + v(i-1)) * (t(i) - t(i-1))
%  Then distance = cumulative sum of ds.
%
%  WHY TRAPEZOIDAL:
%  The car's speed changes continuously. Using just v(i) * dt would assume
%  speed is constant within each time step (rectangular rule), which
%  introduces a small systematic error. The trapezoidal rule uses the
%  average of speed at the start and end of each step, which is more
%  accurate. For 100 Hz data (dt = 0.01 s) the difference is tiny,
%  but good practice is free.

dt = diff(t);                            % time step between consecutive samples
v_avg = 0.5 * (v(1:end-1) + v(2:end));  % average speed in each interval
ds = v_avg .* dt;                        % distance covered in each interval
dist = [0; cumsum(ds)];                  % cumulative distance, starting at 0 [m]

track_length = dist(end);
fprintf('Computed track length: %.0f m (%.3f km)\n', track_length, track_length/1000);
fprintf('Expected N24 layout:  ~25378 m (25.378 km)\n');

%% ========================================================================
%  5. COMPUTE SAMPLE RATE
%  ========================================================================

dt_mean = mean(dt);
fs = 1 / dt_mean;
fprintf('Mean sample rate: %.1f Hz\n', fs);

%% ========================================================================
%  6. BUILD THE REFERENCE LAP STRUCT
%  ========================================================================
%  Everything goes into a single struct 'ref' with clean, documented fields.
%  All model versions and correlation scripts use this struct.

ref = struct();

% Metadata
ref.meta.source     = 'iRacing via PI Toolbox Pro';
ref.meta.car        = 'Mercedes-AMG GT3 Evo';
ref.meta.track      = 'Nürburgring 24h (Nordschleife + GP combined)';
ref.meta.lap_time   = 8*60 + 11.341;     % [s] official lap time from PI
ref.meta.date       = '2026-04-16';
ref.meta.export_hz  = round(fs);
ref.meta.n_samples  = length(t);

% Channels — all in SI units
ref.t         = t;           % [s]     time from lap start
ref.dist      = dist;        % [m]     cumulative distance from start/finish
ref.v         = v;           % [m/s]   speed
ref.v_kmh     = v_kmh;       % [km/h]  speed (kept for convenience in plots)
ref.throttle  = throttle;    % [%]     throttle position (0-100)
ref.brake     = brake;       % [%]     brake position/pressure (0-100)
ref.gear      = gear;        % [-]     gear number (1-6)
ref.rpm       = rpm;         % [rpm]   engine RPM
ref.g_lat     = g_lat;       % [g]     lateral acceleration (+ = right turn)
ref.g_long    = g_long;      % [g]     longitudinal acceleration (+ = accel)
ref.steer     = steer;       % [deg]   steering wheel angle

%% ========================================================================
%  7. SAVE AS .MAT FILE
%  ========================================================================
%  We keep the .xlsx as the archival format (human-readable) and save a
%  .mat file for fast loading in MATLAB. The .mat loads in milliseconds
%  vs. seconds for .xlsx.

outdir = fullfile(pwd, '02_data', 'telemetry', 'processed');
outpath = fullfile(outdir, 'reference_lap.mat');
save(outpath, 'ref');
fprintf('Saved to: %s\n', outpath);

%% ========================================================================
%  8. VERIFICATION PLOTS
%  ========================================================================
%  Quick plots to visually confirm the data looks correct.
%  You should see the Nordschleife speed profile you recognise from driving.

figure('Name', 'Reference Lap Overview', 'NumberTitle', 'off', ...
       'Position', [100, 100, 1200, 800]);

% --- Speed vs Distance ---
subplot(4,1,1);
plot(ref.dist/1000, ref.v_kmh, 'b', 'LineWidth', 0.8);
ylabel('Speed [km/h]');
title(sprintf('Reference Lap — %s — %.3f s', ref.meta.track, ref.meta.lap_time));
grid on;
xlim([0, track_length/1000]);
ylim([0, 300]);

% --- Throttle & Brake vs Distance ---
subplot(4,1,2);
hold on;
plot(ref.dist/1000, ref.throttle, 'g', 'LineWidth', 0.8);
plot(ref.dist/1000, ref.brake, 'r', 'LineWidth', 0.8);
hold off;
ylabel('[%]');
legend('Throttle', 'Brake', 'Location', 'best');
grid on;
xlim([0, track_length/1000]);

% --- G-G diagram ---
subplot(4,1,3);
plot(ref.dist/1000, ref.g_long, 'Color', [0.8 0.4 0], 'LineWidth', 0.8);
hold on;
plot(ref.dist/1000, ref.g_lat, 'Color', [0.5 0 0.8], 'LineWidth', 0.8);
hold off;
ylabel('Accel [g]');
legend('Longitudinal', 'Lateral', 'Location', 'best');
grid on;
xlim([0, track_length/1000]);

% --- Gear & RPM vs Distance ---
subplot(4,1,4);
yyaxis left;
plot(ref.dist/1000, ref.gear, 'k', 'LineWidth', 0.8);
ylabel('Gear [-]');
ylim([0, 7]);
yyaxis right;
plot(ref.dist/1000, ref.rpm, 'Color', [0.6 0.6 0.6], 'LineWidth', 0.5);
ylabel('RPM');
xlabel('Distance [km]');
grid on;
xlim([0, track_length/1000]);

fprintf('\n=== Import Complete ===\n\n');
