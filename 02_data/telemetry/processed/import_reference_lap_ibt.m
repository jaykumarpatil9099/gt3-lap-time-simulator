%% import_reference_lap_ibt.m
%  Multi-lap IBT pipeline — drop-in replacement for import_reference_lap.m.
%
%  Reads the .ibt file in 02_data/telemetry/raw/, segments it into laps,
%  selects a reference lap, and exposes 'ref' and 'laps' in the workspace.
%  Schema-compatible with the existing single-lap pipeline, so build_track
%  and the v0X solvers run unchanged.
%
%  CONFIG (edit at the top, or set in the workspace before running)
%    ibt_file       — basename of the .ibt to load. If empty, the most
%                     recent .ibt in 02_data/telemetry/raw/ is used.
%    ref_mode       — 'fastest_clean' (default) | 'fastest' | 'median'
%
%  WORKSPACE OUTPUT
%    ref            — single-lap struct, schema matches import_reference_lap.m
%    laps           — full struct array, one element per lap (incl. dirty)
%    ibt_meta       — header / session info for traceability
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-21  (Phase 6)

if ~exist('ibt_file', 'var'),  ibt_file = '';            end
if ~exist('ref_mode','var') || isempty(ref_mode), ref_mode = 'fastest_clean'; end

raw_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'raw');
raw_dir = char(java.io.File(raw_dir).getCanonicalPath());

if isempty(ibt_file)
    listing = dir(fullfile(raw_dir, '*.ibt'));
    if isempty(listing)
        error('import_reference_lap_ibt:NoFile', ...
              'No .ibt file in %s', raw_dir);
    end
    [~, k] = max([listing.datenum]);
    ibt_file = listing(k).name;
end
ibt_path = fullfile(raw_dir, ibt_file);

fprintf('\n=== Phase 6 — Multi-lap IBT loader ===\n');
fprintf('IBT file: %s\n', ibt_file);
fprintf('Mode    : %s\n', ref_mode);

laps = import_reference_laps(ibt_path);
ref  = select_reference_lap(laps, ref_mode);

% Save .mat alongside the legacy reference_lap.mat so both pipelines remain
% reproducible. We do NOT overwrite reference_lap.mat — that file pins the
% Phase 1–5 results from the single-lap PI Toolbox export.
out_dir  = fileparts(mfilename('fullpath'));
out_path = fullfile(out_dir, 'reference_lap_ibt.mat');
ibt_meta.source_file = ibt_path;
ibt_meta.ref_mode    = ref_mode;
ibt_meta.n_laps      = numel(laps);
ibt_meta.n_clean     = sum([laps.clean]);
save(out_path, 'ref', 'laps', 'ibt_meta');
fprintf('Saved: %s\n', out_path);
fprintf('=== Done ===\n\n');
