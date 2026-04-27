function corr = correlate_sim(sim, track, sectors, label, plot_flag)
%% correlate_sim — Sectorised sim-vs-reference correlation
%
%  corr = correlate_sim(sim, track)
%  corr = correlate_sim(sim, track, sectors)
%  corr = correlate_sim(sim, track, sectors, label)
%  corr = correlate_sim(sim, track, sectors, label, plot_flag)
%
%  INPUTS
%    sim       — any lap_sim_vXX output struct with fields .v (m/s) on
%                the track.dist grid and .lap_time (s).
%    track     — track struct with .dist, .ds, .kappa, .ref.v.
%    sectors   — [OPTIONAL] N×2 matrix of sector [start end] positions
%                in metres, OR scalar N (split into N equal-length
%                sectors). Default: 6 equal-length sectors.
%    label     — [OPTIONAL] short string shown in the report header.
%    plot_flag — [OPTIONAL] true → draw the annotated speed plot.
%                           Default: true.
%
%  OUTPUT
%    corr.per_sector — table with one row per sector:
%                        .id .start_m .end_m .length_m
%                        .v_mean_sim .v_mean_ref
%                        .v_min_sim .v_min_ref
%                        .dt_sector (sim - ref, for that sector alone)
%                        .dt_cum_end (cumulative Δt at sector exit)
%    corr.sim_laptime, .ref_laptime, .delta_lap
%    corr.dv_rms, .dv_mean, .dv_abs_max
%
%  The reference cumulative time is computed from track.ref.v on the
%  track.dist grid using trapezoidal integration (v cannot be 0 — we
%  clamp to 1 m/s to avoid divide-by-zero on the first sample).
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-21 (Phase 5 Step 2)

if nargin < 3 || isempty(sectors), sectors = 6;         end
if nargin < 4,                      label   = 'sim';    end
if nargin < 5,                      plot_flag = true;   end

%% ---- Build sector boundaries ------------------------------------------
L = track.dist(end);
if isscalar(sectors)
    edges = linspace(0, L, sectors + 1);
    sectors = [edges(1:end-1)' edges(2:end)'];
end
n_sec = size(sectors, 1);

%% ---- Reference cumulative time ----------------------------------------
v_ref  = max(track.ref.v, 1);                % clamp to avoid div/0
ds     = track.ds;
dt_ref = ds ./ v_ref;
t_ref  = [0; cumsum(dt_ref(1:end-1))];       % [s] on track.dist grid
ref_laptime = sum(dt_ref);

%% ---- Sim cumulative time (re-derive to be robust) ---------------------
v_sim  = max(sim.v, 1);
dt_sim = ds ./ v_sim;
t_sim  = [0; cumsum(dt_sim(1:end-1))];
sim_laptime = sum(dt_sim);

%% ---- Point-by-point deltas --------------------------------------------
dv    = sim.v - track.ref.v;                 % [m/s]
dt_pt = t_sim - t_ref;                       % [s] cumulative

%% ---- Per-sector table -------------------------------------------------
rows = struct('id',{}, 'start_m',{}, 'end_m',{}, 'length_m',{}, ...
              'v_mean_sim',{}, 'v_mean_ref',{}, ...
              'v_min_sim',{},  'v_min_ref',{}, ...
              'dt_sector',{},  'dt_cum_end',{});

fprintf('\n=== Correlation report — %s ===\n', label);
fprintf('Sim lap: %.3f s   Ref lap: %.3f s   Δ: %+.3f s (%+.2f%%)\n\n', ...
        sim_laptime, ref_laptime, ...
        sim_laptime - ref_laptime, ...
        100*(sim_laptime - ref_laptime)/ref_laptime);

fprintf('%-3s %8s %8s %8s %9s %9s %9s %9s %10s %10s\n', ...
        'S', 'start', 'end', 'len', 'v_mean_s', 'v_mean_r', ...
        'v_min_s', 'v_min_r', 'Δt_sec', 'Δt_cum');
fprintf('%s\n', repmat('-',1,95));

for k = 1:n_sec
    s0 = sectors(k,1);  s1 = sectors(k,2);
    idx = track.dist >= s0 & track.dist < s1;
    if k == n_sec, idx = track.dist >= s0 & track.dist <= s1; end

    sim_t_sec = sum(dt_sim(idx));
    ref_t_sec = sum(dt_ref(idx));

    idx_end = find(track.dist <= s1, 1, 'last');

    row.id         = k;
    row.start_m    = s0;
    row.end_m      = s1;
    row.length_m   = s1 - s0;
    row.v_mean_sim = mean(sim.v(idx)) * 3.6;
    row.v_mean_ref = mean(track.ref.v(idx)) * 3.6;
    row.v_min_sim  = min(sim.v(idx))  * 3.6;
    row.v_min_ref  = min(track.ref.v(idx)) * 3.6;
    row.dt_sector  = sim_t_sec - ref_t_sec;
    row.dt_cum_end = dt_pt(idx_end);
    rows(end+1) = row; %#ok<AGROW>

    fprintf('%-3d %8.0f %8.0f %8.0f %9.1f %9.1f %9.1f %9.1f %+10.3f %+10.3f\n', ...
            k, s0, s1, s1-s0, ...
            row.v_mean_sim, row.v_mean_ref, ...
            row.v_min_sim,  row.v_min_ref, ...
            row.dt_sector,  row.dt_cum_end);
end
fprintf('%s\n', repmat('-',1,95));

corr.label        = label;
corr.sim_laptime  = sim_laptime;
corr.ref_laptime  = ref_laptime;
corr.delta_lap    = sim_laptime - ref_laptime;
corr.dv_rms       = sqrt(mean(dv.^2));
corr.dv_mean      = mean(dv);
corr.dv_abs_max   = max(abs(dv));
corr.per_sector   = rows;
corr.t_sim_cum    = t_sim;
corr.t_ref_cum    = t_ref;
corr.dv           = dv;
corr.dt_cum       = dt_pt;
corr.sectors      = sectors;

fprintf('\nΔv:  mean=%+.2f m/s  rms=%.2f m/s  |max|=%.2f m/s\n', ...
        corr.dv_mean, corr.dv_rms, corr.dv_abs_max);

%% ---- Plot -------------------------------------------------------------
if plot_flag
    figure('Name', sprintf('Correlation — %s', label), ...
           'NumberTitle','off', 'Position',[100 100 1300 800]);

    % speed overlay
    subplot(3,1,1);
    plot(track.dist/1000, track.ref.v*3.6, 'k',  'LineWidth',0.8); hold on;
    plot(track.dist/1000, sim.v*3.6,       'b',  'LineWidth',0.8);
    % sector bars
    yl = ylim;
    for k = 1:n_sec
        xline(sectors(k,2)/1000, ':', sprintf('S%d',k), ...
              'LabelHorizontalAlignment','left', 'Color',[0.5 0.5 0.5]);
    end
    ylabel('Speed [km/h]'); ylim([0 320]); grid on;
    legend('ref','sim','Location','southeast');
    title(sprintf('%s — Δlap = %+.3f s (%+.2f%%)', ...
          label, corr.delta_lap, 100*corr.delta_lap/ref_laptime));

    % Δv
    subplot(3,1,2);
    plot(track.dist/1000, dv*3.6, 'Color',[0.8 0 0]); hold on;
    yline(0,'k');
    for k = 1:n_sec
        xline(sectors(k,2)/1000, ':','', 'Color',[0.5 0.5 0.5]);
    end
    ylabel('Δv = sim-ref [km/h]'); grid on;

    % Δt cumulative
    subplot(3,1,3);
    plot(track.dist/1000, dt_pt, 'Color',[0 0.5 0]); hold on;
    yline(0,'k');
    for k = 1:n_sec
        xline(sectors(k,2)/1000, ':','', 'Color',[0.5 0.5 0.5]);
    end
    xlabel('Distance [km]'); ylabel('Δt cumulative [s]'); grid on;
end

end
