function [data, meta] = read_ibt(ibt_path, channels)
%READ_IBT  Pure-MATLAB iRacing IBT telemetry parser.
%
%   [data, meta] = read_ibt(ibt_path)
%   [data, meta] = read_ibt(ibt_path, channels)
%
% Reads an iRacing .ibt telemetry log and returns the requested channels
% as column vectors. No external toolboxes; no Python; no PI Toolbox step.
%
% INPUTS
%   ibt_path  Full path to the .ibt file.
%   channels  (optional) cellstr of channel names to extract.
%             Default: a curated set sufficient for lap-time simulation
%             and v06 tyre-wear extension.
%
% OUTPUTS
%   data      Struct. One field per requested channel, each N_ticks x 1.
%             Char-array channels return as cellstr if encountered (rare).
%   meta      Struct with:
%               .tickRate   [Hz]
%               .numVars    [-]   total channels in file
%               .numTicks   [-]   samples per channel
%               .duration_s [s]   numTicks / tickRate
%               .channels   cellstr of returned channels
%               .session    string with the embedded YAML session info
%               .ibt_path   absolute path resolved
%
% FORMAT REFERENCE
%   iRacing SDK header `irsdk_defines.h`. The file is laid out as:
%     [header 112 B][var headers numVars*144 B][session-info YAML][data]
%   Data is a contiguous stream of fixed-size frames; each frame is bufLen
%   bytes and contains every channel at byte offset `varHeader.offset`.
%
%   Variable types: 0 char, 1 bool, 2 int (4 B), 3 bitfield (4 B),
%                   4 float (4 B), 5 double (8 B).
%
% Author:  Jaykumar Patil
% Created: 2026-04-21  (Phase 6 — multi-lap IBT pipeline)

if nargin < 2 || isempty(channels)
    channels = { ...
        'SessionTime', 'Lap', 'LapDist', 'LapDistPct', 'LapCurrentLapTime', ...
        'Speed', 'Throttle', 'Brake', 'Gear', 'RPM', 'SteeringWheelAngle', ...
        'LatAccel', 'LongAccel', 'YawRate', 'RollRate', 'PitchRate', ...
        'VelocityX', 'VelocityY', 'VelocityZ', 'OnPitRoad', 'TrackTempCrew', ...
        'LFtempCM','RFtempCM','LRtempCM','RRtempCM', ...
        'LFwearM','RFwearM','LRwearM','RRwearM' };
end

if ~isfile(ibt_path)
    error('read_ibt:NoFile', 'IBT file not found: %s', ibt_path);
end

%% ---- Read file header ------------------------------------------------
fid = fopen(ibt_path, 'rb', 'ieee-le');
cleanup = onCleanup(@() fclose(fid));

hdr_bytes = fread(fid, 112, '*uint8');
hdr = typecast(hdr_bytes, 'int32');
ver              = double(hdr(1));   %#ok<NASGU>
status           = double(hdr(2));   %#ok<NASGU>
tickRate         = double(hdr(3));
% sessInfoUpdate = double(hdr(4));
sessInfoLen      = double(hdr(5));
sessInfoOffset   = double(hdr(6));
numVars          = double(hdr(7));
varHeaderOffset  = double(hdr(8));
% numBuf         = double(hdr(9));
bufLen           = double(hdr(10));
% bufOffset of the (single) buffer is implicitly after the metadata; we
% locate data_start the same way pyirsdk does.

%% ---- Read variable headers -------------------------------------------
fseek(fid, varHeaderOffset, 'bof');
vh_bytes = fread(fid, numVars * 144, '*uint8');

types_size = [1 1 4 4 4 8];   % char bool int bitfield float double
types_kind = {'uint8','uint8','int32','uint32','single','double'};

vars = repmat(struct('type',[], 'offset',[], 'count',[], 'name','', ...
                     'unit','', 'kind','', 'sz',0), numVars, 1);
for i = 1:numVars
    o = (i-1)*144 + 1;
    vars(i).type   = double(typecast(vh_bytes(o:o+3),    'int32'));
    vars(i).offset = double(typecast(vh_bytes(o+4:o+7),  'int32'));
    vars(i).count  = double(typecast(vh_bytes(o+8:o+11), 'int32'));
    vars(i).name   = local_cstr(vh_bytes(o+16:o+47));
    vars(i).unit   = local_cstr(vh_bytes(o+112:o+143));
    t              = vars(i).type + 1;
    vars(i).kind   = types_kind{t};
    vars(i).sz     = types_size(t);
end
all_names = {vars.name};

%% ---- Locate data start -----------------------------------------------
data_start = max(varHeaderOffset + numVars*144, sessInfoOffset + sessInfoLen);
file_info  = dir(ibt_path);
file_size  = file_info.bytes;
numTicks   = floor((file_size - data_start) / bufLen);

%% ---- Read embedded session-info YAML ---------------------------------
fseek(fid, sessInfoOffset, 'bof');
session_raw = fread(fid, sessInfoLen, '*uint8');
session_str = native2unicode(session_raw(:)', 'UTF-8');
session_str = regexprep(session_str, '\x00.*$', '');   % trim NUL tail

%% ---- Read each requested channel -------------------------------------
data = struct();
for k = 1:numel(channels)
    ch = channels{k};
    idx = find(strcmp(all_names, ch), 1);
    if isempty(idx)
        warning('read_ibt:Missing', 'Channel "%s" not in file — skipped.', ch);
        continue;
    end
    v = vars(idx);
    if v.count ~= 1
        warning('read_ibt:Array', ...
                'Channel "%s" has count=%d; arrays not yet supported.', ...
                ch, v.count);
        continue;
    end
    fseek(fid, data_start + v.offset, 'bof');
    skip = bufLen - v.sz;
    raw  = fread(fid, numTicks, ['*' v.kind], skip);
    if v.type == 1               % bool
        data.(ch) = logical(raw);
    else
        data.(ch) = double(raw);
    end
end

%% ---- Meta ------------------------------------------------------------
meta.tickRate   = tickRate;
meta.numVars    = numVars;
meta.numTicks   = numTicks;
meta.duration_s = numTicks / tickRate;
meta.channels   = fieldnames(data);
meta.session    = session_str;
meta.ibt_path   = which(ibt_path);
if isempty(meta.ibt_path), meta.ibt_path = ibt_path; end

end

%% ---- helper ----------------------------------------------------------
function s = local_cstr(bytes)
    bytes = bytes(:)';
    z = find(bytes == 0, 1);
    if isempty(z), z = numel(bytes) + 1; end
    s = char(bytes(1:z-1));
end
