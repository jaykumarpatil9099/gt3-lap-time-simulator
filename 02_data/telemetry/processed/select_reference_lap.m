function ref = select_reference_lap(laps, mode)
%SELECT_REFERENCE_LAP  Pick or synthesise a reference lap from a multi-lap set.
%
%   ref = select_reference_lap(laps)               % default mode = 'fastest_clean'
%   ref = select_reference_lap(laps, 'fastest')    % outright fastest, clean or not
%   ref = select_reference_lap(laps, 'fastest_clean')
%   ref = select_reference_lap(laps, 'median')     % median across CLEAN laps
%
% Returns a 'ref' struct that is *interface-compatible* with the existing
% single-lap import_reference_lap.m output, i.e. with fields:
%   .t, .dist, .v, .g_lat, .g_long, .gear, .throttle, .brake, .lap_time,
%   .meta.* (incl. .source_file, .source_lap, .source_mode)
%
% This means build_track_telemetry.m and the v0X solvers can be pointed at
% the multi-lap reference without code changes — only the loader changes.
%
% MODES
%   'fastest_clean' (default) — fastest lap among those flagged .clean=true
%   'fastest'                 — fastest lap regardless of clean flag
%   'median'                  — clean laps resampled onto a common 1 m grid,
%                               channel-wise median taken at each meter.
%                               Removes driver-input variance from the
%                               curvature signal, at the cost of a slightly
%                               smeared steering / throttle trace.
%
% Author:  Jaykumar Patil
% Created: 2026-04-21  (Phase 6)

if nargin < 2 || isempty(mode), mode = 'fastest_clean'; end
if isempty(laps), error('select_reference_lap:Empty', 'No laps supplied.'); end

clean_idx = find([laps.clean]);
fast_idx  = local_argmin([laps.lap_time]);

switch lower(mode)
    case 'fastest'
        ref = local_lap_to_ref(laps(fast_idx), mode, fast_idx);

    case 'fastest_clean'
        if isempty(clean_idx)
            warning('select_reference_lap:NoClean', ...
                    'No clean laps — falling back to outright fastest.');
            ref = local_lap_to_ref(laps(fast_idx), 'fastest', fast_idx);
        else
            [~, k] = min([laps(clean_idx).lap_time]);
            idx = clean_idx(k);
            ref = local_lap_to_ref(laps(idx), mode, idx);
        end

    case 'median'
        if numel(clean_idx) < 2
            warning('select_reference_lap:Median', ...
                    'Need >=2 clean laps for median; using fastest_clean.');
            ref = select_reference_lap(laps, 'fastest_clean');
            return;
        end
        ref = local_median_lap(laps(clean_idx));
        ref.meta.source_mode = 'median';
        ref.meta.source_lap  = sprintf('median of %d clean laps', numel(clean_idx));

    otherwise
        error('select_reference_lap:Mode', 'Unknown mode: %s', mode);
end

fprintf('\nReference lap selected (%s): lap_time = %.3f s, length = %.0f m\n', ...
        mode, ref.lap_time, ref.dist(end));
end

%% ---- single-lap conversion ------------------------------------------
function ref = local_lap_to_ref(L, mode, src_idx)
    ref.t        = L.t(:);
    ref.dist     = L.dist(:);
    ref.v        = L.v(:);
    ref.g_lat    = L.a_lat(:);
    ref.g_long   = L.a_long(:);
    ref.gear     = L.gear(:);
    ref.throttle = L.throttle(:);
    ref.brake    = L.brake(:);
    ref.lap_time = L.lap_time;
    ref.meta = L.meta;
    ref.meta.source_mode = mode;
    ref.meta.source_lap  = sprintf('iRacing lap #%d (set index %d)', ...
                                   L.lap_num, src_idx);
end

%% ---- median across clean laps ---------------------------------------
function ref = local_median_lap(L_clean)
    n = numel(L_clean);
    L_max = max(arrayfun(@(x) x.dist(end), L_clean));
    grid  = (0:1:L_max)';   % 1 m spacing

    fields = {'v','a_lat','a_long','gear','throttle','brake','steer','rpm', ...
              'tyre_temp_lf','tyre_temp_rf','tyre_temp_lr','tyre_temp_rr', ...
              'tyre_wear_lf','tyre_wear_rf','tyre_wear_lr','tyre_wear_rr'};
    M = struct();
    for f = 1:numel(fields)
        M.(fields{f}) = NaN(numel(grid), n);
    end

    for k = 1:n
        L = L_clean(k);
        % Force monotonic distance
        d = L.dist;
        keep = [true; diff(d) > 0];
        d = d(keep);
        for f = 1:numel(fields)
            y = L.(fields{f})(keep);
            M.(fields{f})(:,k) = interp1(d, y, grid, 'linear', NaN);
        end
    end

    median_v = median(M.v, 2, 'omitnan');
    % Reconstruct time channel from median speed
    t_med = [0; cumsum(1 ./ max(median_v(2:end), 1))];   % 1 m / v

    ref.t        = t_med;
    ref.dist     = grid;
    ref.v        = median_v;
    ref.g_lat    = median(M.a_lat,    2, 'omitnan');
    ref.g_long   = median(M.a_long,   2, 'omitnan');
    ref.gear     = round(median(M.gear, 2, 'omitnan'));
    ref.throttle = median(M.throttle, 2, 'omitnan');
    ref.brake    = median(M.brake,    2, 'omitnan');
    ref.lap_time = t_med(end);
    ref.meta.source_file  = L_clean(1).meta.source_file;
    ref.meta.tick_rate    = L_clean(1).meta.tick_rate;
    ref.meta.n_laps_used  = n;
end

function k = local_argmin(x)
    [~, k] = min(x);
end
