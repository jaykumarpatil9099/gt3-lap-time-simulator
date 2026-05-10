%% make_one_pager_figure.m
%  Produces ONE clean, print-ready figure for a single-page project work
%  document on the Nürburgring 24h lap simulator.
%
%  WHAT IT SHOWS
%  -------------
%  Speed trace of the calibrated simulated lap (v06 Pacejka, Phase 6.5)
%  overlaid on the iRacing reference-driver telemetry, around the full
%  25.2 km Nordschleife + GP combined layout. Sector boundaries marked
%  with light grey dividers; lap times and the Δt headline shown in an
%  annotation box top-right.
%
%  This is the canonical motorsport "did your sim correlate?" figure.
%  Any race engineer reads it in five seconds.
%
%  PRE-REQUISITES (in the workspace)
%    track   from build_track
%    sim06   from lap_sim_v06   (preferred — v06 is the current calibrated head)
%    sim05   from lap_sim_v05   (fallback if v06 is not yet run)
%
%  OUTPUTS
%    06_reports/figures/fig_one_pager_n24_speed.png   (300 DPI, for print)
%    06_reports/figures/fig_one_pager_n24_speed.pdf   (vector, for scaling)
%
%  USAGE
%    >> startup_project
%    >> import_reference_lap_ibt
%    >> track_source = 'telemetry'; build_track
%    >> lap_sim_v05      % optional but recommended
%    >> lap_sim_v06      % the lead trace
%    >> run('06_reports/make_one_pager_figure.m')
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-22

if ~exist('track','var'), error('Run build_track first.'); end

% Pick the most-recent sim for the lead trace
if exist('sim06b','var')
    sim_lead   = sim06b;
    sim_label  = 'Calibrated sim — v06.0b (Pacejka MF 5.2 + G-factor envelope)';
    sim_color  = [0.85 0.20 0.10];   % motorsport red
elseif exist('sim06','var')
    sim_lead   = sim06;
    sim_label  = 'Calibrated sim — v06 (Pacejka MF 5.2)';
    sim_color  = [0.85 0.20 0.10];
elseif exist('sim05','var')
    sim_lead   = sim05;
    sim_label  = 'Calibrated sim — v05';
    sim_color  = [0.20 0.45 0.85];
else
    error('No sim struct in workspace. Run lap_sim_v0X first.');
end

ref_lap = track.meta.ref_laptime;
sim_lap = sim_lead.lap_time;
delta_s = sim_lap - ref_lap;
delta_p = 100 * delta_s / ref_lap;

%% ---- Figure setup -----------------------------------------------------
fig = figure('Name','N24 lap simulator — one-pager figure', ...
             'NumberTitle','off', 'Color','w', ...
             'Position',[100 100 1400 540]);
set(0, 'DefaultAxesFontName', 'Helvetica', 'DefaultAxesFontSize', 11);

ax = axes('Parent', fig); hold(ax, 'on'); box(ax, 'on');

% Sector dividers (6 equal-length sectors, same convention as Phase 5)
edges = linspace(0, track.dist(end), 7);
for k = 2:6
    xline(edges(k)/1000, ':', 'Color',[0.7 0.7 0.7], 'LineWidth', 0.6, ...
          'Alpha', 0.6, 'HandleVisibility','off');
end

% Reference and simulated speeds
plot(ax, track.dist/1000, track.ref.v*3.6,  'Color',[0 0 0], ...
     'LineWidth', 1.4, 'DisplayName','Reference (iRacing telemetry)');
plot(ax, track.dist/1000, sim_lead.v*3.6,   'Color',sim_color, ...
     'LineWidth', 1.4, 'DisplayName', sim_label);

xlabel(ax, 'Distance along lap [km]');
ylabel(ax, 'Speed [km/h]');
xlim(ax, [0 track.dist(end)/1000]);
ylim(ax, [0 320]);
grid(ax, 'on'); set(ax, 'GridAlpha', 0.18, 'MinorGridAlpha', 0.10);
title(ax, sprintf(['Mercedes-AMG GT3 — Nürburgring 24h (Nordschleife + GP, %.2f km) — ' ...
                   'sim vs reference'], track.dist(end)/1000), ...
      'FontWeight', 'normal', 'FontSize', 12);

% Sector labels along the top of the plot (S1..S6)
for k = 1:6
    xc = (edges(k) + edges(k+1)) / 2 / 1000;
    text(xc, 305, sprintf('S%d', k), 'Parent', ax, ...
         'HorizontalAlignment','center', 'Color',[0.5 0.5 0.5], ...
         'FontSize', 9, 'FontWeight','bold');
end

legend(ax, 'Location','south', 'Orientation','horizontal', ...
       'Box','off', 'FontSize', 11);

%% ---- Annotation box (top-right) --------------------------------------
mins  = @(t) floor(t/60);
secs  = @(t) t - floor(t/60)*60;
txt = sprintf(['Reference lap   %d:%06.3f\n', ...
               'Simulated lap   %d:%06.3f\n', ...
               'Delta              %+.3f s   (%+.2f %%)'], ...
               mins(ref_lap), secs(ref_lap), ...
               mins(sim_lap), secs(sim_lap), ...
               delta_s, delta_p);

annotation('textbox', [0.71, 0.78, 0.27, 0.18], 'String', txt, ...
           'FontName','Consolas', 'FontSize', 11, ...
           'BackgroundColor', [1 1 1], ...
           'EdgeColor', [0.7 0.7 0.7], ...
           'LineWidth', 0.7, 'Margin', 8, ...
           'FitBoxToText','off', 'VerticalAlignment','middle', ...
           'HorizontalAlignment','left');

%% ---- Export ----------------------------------------------------------
out_dir = fileparts(mfilename('fullpath'));
out_dir = fullfile(out_dir, 'figures');
if ~exist(out_dir,'dir'), mkdir(out_dir); end

png_path = fullfile(out_dir, 'fig_one_pager_n24_speed.png');
pdf_path = fullfile(out_dir, 'fig_one_pager_n24_speed.pdf');

exportgraphics(fig, png_path, 'Resolution', 300);
exportgraphics(fig, pdf_path, 'ContentType', 'vector');

fprintf('\nSaved:\n  %s\n  %s\n\n', png_path, pdf_path);
