%% export_figures.m
%  Exports the four PNGs referenced in 06_reports/n24_portfolio_summary.md.
%  Run once after Phase 5 results are in place. Re-run if anything upstream
%  changes (calibration, setup, sweeps).
%
%  Output:
%    06_reports/figures/fig_headline_speed_overlay.png
%    06_reports/figures/fig_calibration_heatmap.png
%    06_reports/figures/fig_sensitivity_tornado.png
%    06_reports/figures/fig_sector_signature.png
%
%  USAGE
%    >> startup_project
%    >> import_reference_lap
%    >> track_source = 'telemetry'; build_track
%    >> run('06_reports/export_figures.m')
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-21

if ~exist('ref','var') || ~exist('car','var') || ~exist('track','var')
    error('Load ref + car + telemetry track first.');
end

repo_root = fileparts(fileparts(mfilename('fullpath')));   % .../Lap time simulator
out_dir   = fullfile(repo_root, '06_reports', 'figures');
if ~exist(out_dir,'dir'), mkdir(out_dir); end

dpi = 200;
set(0, 'DefaultAxesFontSize', 11, 'DefaultAxesFontName', 'Helvetica');

%% ------------------------------------------------------------------------
%  1. HEADLINE — speed overlay (calibrated v05 vs reference, telemetry)
%  ------------------------------------------------------------------------
fprintf('\n[1/4] Headline speed overlay...\n');
evalc('lap_sim_v05;');

f = figure('Visible','off','Position',[100 100 1200 500]);
plot(track.dist/1000, track.ref.v*3.6, 'k', 'LineWidth',1.0); hold on;
plot(track.dist/1000, sim05.v*3.6,     'b', 'LineWidth',1.0);
xlabel('Distance [km]'); ylabel('Speed [km/h]');
ylim([0 320]); xlim([0 track.dist(end)/1000]); grid on;
legend('Reference (driver)', sprintf('Calibrated v05 (%.3f s, Δ %+.3f s)', ...
       sim05.lap_time, sim05.lap_time - track.meta.ref_laptime), ...
       'Location','southeast');
title(sprintf('N24 — calibrated v05 vs reference (telemetry source)'));
exportgraphics(f, fullfile(out_dir,'fig_headline_speed_overlay.png'), ...
               'Resolution', dpi);
close(f);

%% ------------------------------------------------------------------------
%  2. CALIBRATION HEATMAP — Step 4 sector RMS
%  ------------------------------------------------------------------------
fprintf('[2/4] Calibration heatmap...\n');
S4 = load(fullfile(repo_root,'05_studies','phase5_step4_result.mat'));
r  = S4.result;

f = figure('Visible','off','Position',[100 100 700 500]);
imagesc(r.ksk_vals, r.mu0_vals, r.secrms_grid); axis xy;
colorbar; colormap('parula');
set(gca,'XTick',r.ksk_vals,'YTick',r.mu0_vals);
xlabel('load\_sens\_k [1/N]'); ylabel('mu\_0 [-]');
title('Step 4 calibration — sector RMS Δt [s] (lower = better)');
hold on;
plot(r.best_rms.load_sens_k, r.best_rms.mu_0, 'kp', ...
     'MarkerSize',16, 'MarkerFaceColor','y', 'LineWidth',1.5);
text(r.best_rms.load_sens_k, r.best_rms.mu_0, ...
     sprintf('  best (%.3f s)', r.best_rms.rms), ...
     'Color','k','FontWeight','bold','VerticalAlignment','top');
exportgraphics(f, fullfile(out_dir,'fig_calibration_heatmap.png'), ...
               'Resolution', dpi);
close(f);

%% ------------------------------------------------------------------------
%  3. SENSITIVITY TORNADO — Step 3
%  ------------------------------------------------------------------------
fprintf('[3/4] Sensitivity tornado...\n');
S3 = load(fullfile(repo_root,'05_studies','phase5_step3_result.mat'));
[~, order] = sort([S3.result.leverage], 'descend');

f = figure('Visible','off','Position',[100 100 900 500]);
barh([S3.result(order).leverage], 'FaceColor',[0.2 0.5 0.8]);
set(gca,'YTick',1:numel(S3.result), ...
        'YTickLabel',{S3.result(order).name}, ...
        'YDir','reverse');
xlabel('Full-range Δ lap time [s]');
title(sprintf('Step 3 sensitivity — v05 GPS baseline (%.3f s)', ...
              S3.baseline_lap));
grid on;
exportgraphics(f, fullfile(out_dir,'fig_sensitivity_tornado.png'), ...
               'Resolution', dpi);
close(f);

%% ------------------------------------------------------------------------
%  4. SECTOR SIGNATURE — calibrated baseline
%  ------------------------------------------------------------------------
fprintf('[4/4] Sector signature...\n');
edges   = linspace(0, track.dist(end), 7);
sectors = [edges(1:end-1)' edges(2:end)'];
c = correlate_sim(sim05, track, sectors, 'calibrated v05', false);
sec_dt   = [c.per_sector.dt_sector];
sec_mid  = (sectors(:,1) + sectors(:,2)) / 2 / 1000;

f = figure('Visible','off','Position',[100 100 900 450]);
b = bar(sec_dt, 'FaceColor','flat');
for k = 1:numel(sec_dt)
    if sec_dt(k) < 0
        b.CData(k,:) = [0.20 0.55 0.20];   % green = sim faster
    else
        b.CData(k,:) = [0.80 0.30 0.20];   % red   = sim slower
    end
end
set(gca,'XTickLabel',arrayfun(@(k) sprintf('S%d',k), 1:numel(sec_dt), ...
                              'UniformOutput',false));
ylabel('Δt sim − ref [s]');
title(sprintf(['Sector signature — calibrated v05 (sector RMS = %.3f s, ' ...
               'Δlap = %+.3f s)'], sqrt(mean(sec_dt.^2)), c.delta_lap));
yline(0,'k'); grid on;
exportgraphics(f, fullfile(out_dir,'fig_sector_signature.png'), ...
               'Resolution', dpi);
close(f);

fprintf('\nAll figures saved to: %s\n', out_dir);
