function laps = import_reference_laps(ibt_path)
%IMPORT_REFERENCE_LAPS  Read an IBT file and segment it into per-lap structs.
%
%   laps = import_reference_laps(ibt_path)
%
% Reads the .ibt file via read_ibt, splits the session into laps using the
% iRacing 'Lap' channel, drops the out-lap, in-lap, and any pit-road laps,
% and returns a 1xN struct array — one element per *driving* lap.
%
% Each lap struct has:
%   .lap_num        iRacing lap index (1-based, monotonic)
%   .t              [s]    time within this lap (zeroed)
%   .v              [m/s]  speed
%   .a_lat          [g]    lateral acceleration (LatAccel / 9.81)
%   .a_long         [g]    longitudinal acceleration (LongAccel / 9.81)
%   .dist           [m]    cumulative lap distance (from LapDist)
%   .gear           [-]    selected gear
%   .throttle       [%]    0..1
%   .brake          [%]    0..1
%   .steer          [rad]  steering wheel angle
%   .rpm            [rpm]
%   .yaw_rate       [rad/s]
%   .tyre_temp_lf/.rf/.lr/.rr   [°C] middle-tread temps
%   .tyre_wear_lf/.rf/.lr/.rr   [%]  remaining (1.0 = new)
%   .lap_time       [s]    duration
%   .clean          [bool] passes the basic clean-lap test (see below)
%   .meta           tickRate, source file, etc.
%
% The unit conventions match the existing single-lap import_reference_lap
% so build_track_telemetry.m and the v0X solvers can consume either source
% without changes.
%
% A lap is flagged as 'clean' when:
%   - It is not the first or last lap of the session (out/in laps).
%   - OnPitRoad is false for the entire lap.
%   - Lap distance covered is within 5 % of the median lap distance
%     (catches off-track shortcuts and resets).
%   - Throttle exceeded 0.9 at some point (sanity: actual driving).
%
% Author:  Jaykumar Patil
% Created: 2026-04-21  (Phase 6)

if ~isfile(ibt_path)
    error('import_reference_laps:NoFile', 'IBT not found: %s', ibt_path);
end

fprintf('\n=== import_reference_laps ===\n');
fprintf('Reading: %s\n', ibt_path);

[d, meta] = read_ibt(ibt_path);
fprintf('  ticks: %d  duration: %.1f s  rate: %d Hz\n', ...
        meta.numTicks, meta.duration_s, meta.tickRate);

%% ---- Find lap boundaries --------------------------------------------
%  LapDistPct rises 0→1 within a lap and drops sharply at start/finish.
%  A drop of >0.5 in one tick is a candidate lap-line crossing — but
%  iRacing also has rare one-tick glitches where LapDistPct momentarily
%  hits 0 mid-lap and snaps back to its previous value the next sample.
%  Distinguish real rollovers from glitches: a REAL rollover is followed
%  by a long stretch of small LapDistPct values; a glitch jumps back to
%  the high pre-glitch value within a few samples.
ldp = d.LapDistPct;
lap = d.Lap;

candidates = find(diff(ldp) < -0.5) + 1;
n_t        = numel(ldp);
real_roll  = false(size(candidates));
for cc = 1:numel(candidates)
    c   = candidates(cc);
    win = c : min(c+30, n_t);    % look ~0.5 s ahead at 60 Hz
    real_roll(cc) = max(ldp(win)) < 0.10;
end
n_glitch = sum(~real_roll);
if n_glitch > 0
    fprintf('  filtered out %d LapDistPct glitch(es)\n', n_glitch);
end
boundaries     = [1; candidates(real_roll); n_t + 1];
n_lap_segments = numel(boundaries) - 1;
fprintf('  raw segments (real rollovers): %d\n', n_lap_segments);

g = 9.81;

%% ---- Build per-lap struct array -------------------------------------
laps = repmat(local_empty_lap(), 1, n_lap_segments);
keep = false(1, n_lap_segments);

dist_each = zeros(1, n_lap_segments);
for i = 1:n_lap_segments
    a = boundaries(i);
    b = boundaries(i+1) - 1;
    if b - a < 60   % less than 1 second of samples -- garbage
        continue;
    end
    rng = a:b;

    L.lap_num   = mode(lap(rng));   % robust to one-tick Lap glitches
    L.t         = d.SessionTime(rng) - d.SessionTime(a);
    L.v         = d.Speed(rng);
    L.a_lat     = d.LatAccel(rng)  / g;
    L.a_long    = d.LongAccel(rng) / g;
    L.gear      = d.Gear(rng);
    L.throttle  = d.Throttle(rng);
    L.brake     = d.Brake(rng);
    L.steer     = d.SteeringWheelAngle(rng);
    L.rpm       = d.RPM(rng);
    L.yaw_rate  = d.YawRate(rng);
    L.tyre_temp_lf = d.LFtempCM(rng);
    L.tyre_temp_rf = d.RFtempCM(rng);
    L.tyre_temp_lr = d.LRtempCM(rng);
    L.tyre_temp_rr = d.RRtempCM(rng);
    L.tyre_wear_lf = d.LFwearM(rng);
    L.tyre_wear_rf = d.RFwearM(rng);
    L.tyre_wear_lr = d.LRwearM(rng);
    L.tyre_wear_rr = d.RRwearM(rng);

    % Distance: prefer LapDist (already lap-local); fall back to integration.
    L.dist = d.LapDist(rng);
    if any(diff(L.dist) < -50)   % rollover at lap line — fix monotonic
        L.dist = local_integrate_v(L.v, L.t);
    end
    L.dist = L.dist - L.dist(1);

    L.lap_time = L.t(end) - L.t(1);

    % Clean-lap flags
    on_pit  = any(d.OnPitRoad(rng));
    moved   = max(L.throttle) > 0.9 && L.lap_time > 60;
    L.clean_flags.on_pit_during_lap = on_pit;
    L.clean_flags.adequate_throttle = moved;

    L.meta.source_file = meta.ibt_path;
    L.meta.tick_rate   = meta.tickRate;
    L.meta.car         = '';   % populated from session info if needed
    L.meta.track       = '';
    L.clean            = false;   % default; final value set in post-loop

    % MATLAB requires identical field set + order for struct-array
    % assignment. Reorder L to the prototype before assigning.
    laps(i)        = orderfields(L, laps(1));
    keep(i)        = true;
    dist_each(i)   = L.dist(end);
end
laps      = laps(keep);
dist_each = dist_each(keep);
fprintf('  after dropping <1 s segments: %d laps\n', numel(laps));

%% ---- Final clean-lap classification ---------------------------------
median_d = median(dist_each);
median_t = median([laps.lap_time]);
for i = 1:numel(laps)
    not_first_or_last = (i > 1) && (i < numel(laps));
    pit_free          = ~laps(i).clean_flags.on_pit_during_lap;
    full_distance     = abs(dist_each(i) - median_d) / median_d < 0.05;
    threw             = laps(i).clean_flags.adequate_throttle;
    % Time sanity: must be within ±25 % of median lap time. Catches
    % half-laps and slow-down laps that pass the distance check on a
    % glitched LapDist signal.
    reasonable_time   = abs(laps(i).lap_time - median_t) / median_t < 0.25;
    laps(i).clean = not_first_or_last && pit_free && ...
                    full_distance && threw && reasonable_time;
end

%% ---- Report ----------------------------------------------------------
fprintf('\n  %-3s %-10s %-9s %-9s %-9s\n', 'i', 'iRacing#', 'lap_t [s]', 'dist [m]', 'clean');
for i = 1:numel(laps)
    fprintf('  %-3d %-10d %9.3f %9.0f %9s\n', ...
            i, laps(i).lap_num, laps(i).lap_time, dist_each(i), ...
            mat2str(laps(i).clean));
end
n_clean = sum([laps.clean]);
fprintf('\n  → %d clean lap(s) of %d total.\n', n_clean, numel(laps));

end

%% ---- helpers ---------------------------------------------------------
function L = local_empty_lap()
    L = struct('lap_num',[], 't',[], 'v',[], 'a_lat',[], 'a_long',[], ...
               'dist',[], 'gear',[], 'throttle',[], 'brake',[], ...
               'steer',[], 'rpm',[], 'yaw_rate',[], ...
               'tyre_temp_lf',[],'tyre_temp_rf',[], ...
               'tyre_temp_lr',[],'tyre_temp_rr',[], ...
               'tyre_wear_lf',[],'tyre_wear_rf',[], ...
               'tyre_wear_lr',[],'tyre_wear_rr',[], ...
               'lap_time',[], 'clean',false, ...
               'clean_flags',struct(), 'meta',struct());
end

function s = local_integrate_v(v, t)
    s = [0; cumsum(0.5*(v(1:end-1)+v(2:end)) .* diff(t))];
end
