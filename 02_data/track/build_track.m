%% build_track.m
%  Track-data DISPATCHER. Builds the 'track' struct from one of two sources:
%
%    track_source = 'telemetry'  (default) — derive curvature from
%        reference-lap a_lat/v^2 telemetry. The resulting centerline is the
%        driver's RACING LINE, not the geometric centerline. See
%        build_track_telemetry.m for details.
%
%    track_source = 'gps'                   — derive curvature from the
%        geometric GPS centerline extracted from the .pxt file. No speed
%        signal, no lateral-g noise: pure geometry. See
%        build_track_from_gps.m for details.
%
%  WHY A DISPATCHER
%  ----------------
%  Both source scripts produce a 'track' struct with the SAME top-level
%  fields (dist, kappa, ds, n, length, ref.*, meta.*). The GPS path adds
%  bonus fields (x, y, z, lat, lon, kappa_signed) that the v01..v04 solvers
%  ignore because MATLAB structs are free-form. Keeping a single entry
%  point means every solver, every correlation script, and the logbook can
%  say "call build_track" without worrying about which underlying script
%  ran. The handshake is the variable 'track_source' in the workspace.
%
%  WORKSPACE INPUT
%  ---------------
%    ref          — from import_reference_lap.m (required for 'telemetry';
%                   used only for correlation channels in 'gps')
%    track_source — 'telemetry' (default) or 'gps'
%
%  WORKSPACE OUTPUT
%  ----------------
%    track        — the assembled struct
%    (also saved to 02_data/track/n24_track.mat [telemetry] or
%                   02_data/track/n24_track_gps.mat [gps])
%
%  USAGE
%    >> startup_project
%    >> import_reference_lap
%    >> build_track                        % uses telemetry (default)
%
%    >> track_source = 'gps';
%    >> build_track                        % uses GPS centerline
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-20  (replaces the pre-split monolithic build_track.m)

%% ------------------------------------------------------------------------
%  Resolve the source. Default to 'telemetry' so existing run scripts keep
%  working without modification.
%  ------------------------------------------------------------------------

if ~exist('track_source', 'var') || isempty(track_source)
    track_source = 'telemetry';
end

track_source = lower(string(track_source));

fprintf('\n=== build_track dispatcher ===\n');
fprintf('  track_source = ''%s''\n', track_source);

switch track_source
    case "telemetry"
        run(fullfile('02_data', 'track', 'build_track_telemetry.m'));

    case "gps"
        run(fullfile('02_data', 'track', 'build_track_from_gps.m'));

    otherwise
        error(['Unknown track_source = ''%s''. Valid options: ' ...
               '''telemetry'' (default) or ''gps''.'], track_source);
end
