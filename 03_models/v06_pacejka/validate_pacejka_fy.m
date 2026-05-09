%% validate_pacejka_fy.m
%  v06 Step 1 sanity check — plot the Pacejka MF 5.2 lateral curves at a
%  sweep of vertical loads, and print the key numerical landmarks (peak
%  μ, load-sensitivity, cornering stiffness). Run this before wiring
%  Pacejka into the lap sim — the physics has to look right first.
%
%  USAGE
%    >> startup_project
%    >> run('03_models/v06_pacejka/validate_pacejka_fy.m')
%
%  EXPECTED SHAPE
%  --------------
%  - Fy(α) has a peak around α ≈ 0.10..0.18 rad (6..10°), drops past it.
%  - Higher Fz pushes the peak to larger α.
%  - Peak μ_eff = max|Fy|/Fz drops with increasing Fz (load sensitivity).
%  - Cornering stiffness Ky (slope at α=0) is approximately linear in
%    Fz at low loads, saturating at higher loads.
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-22  (v06 Pacejka — Phase 6.5, Step 1)

if ~exist('car', 'var')
    error(['car struct not loaded. Run startup_project first, then ' ...
           '"run(''02_data/car/pacejka_coeffs_pirelli_dhf_gt3.m'');"']);
end
if ~isfield(car.tyre, 'pacejka')
    fprintf('Loading Pacejka coefficients...\n');
    % Script is on the MATLAB path via startup_project — call by name,
    % no relative path (relative-run fails when run() pushes cwd).
    pacejka_coeffs_pirelli_dhf_gt3;
end

p = car.tyre.pacejka.front;     % front and rear identical at v06.0

%% ---- Sweep -----------------------------------------------------------
alpha_deg = -15 : 0.1 : 15;
alpha     = alpha_deg * pi/180;

Fz_set    = [2000 4000 6000 8000 10000];
colors    = lines(numel(Fz_set));

figure('Name','v06 Pacejka MF 5.2 — lateral pure slip', ...
       'NumberTitle','off', 'Position',[100 100 1300 700]);

%% ---- Plot 1: Fy vs slip angle, multiple Fz ---------------------------
subplot(2,2,1); hold on;
peak_mu = zeros(size(Fz_set));
peak_at = zeros(size(Fz_set));
for k = 1:numel(Fz_set)
    Fz = Fz_set(k);
    Fy = pacejka_fy(alpha, Fz, p);
    plot(alpha_deg, Fy/1000, 'Color', colors(k,:), 'LineWidth', 1.4, ...
         'DisplayName', sprintf('Fz = %d N', Fz));
    [peak_F, idx] = max(abs(Fy));
    peak_mu(k) = peak_F / Fz;
    peak_at(k) = alpha_deg(idx);
end
xlabel('Slip angle α [°]');
ylabel('Lateral force F_y [kN]');
title('F_y(α) at a sweep of F_z');
grid on; legend('Location','southeast');
xlim([-15 15]);

%% ---- Plot 2: μ_eff vs Fz ---------------------------------------------
subplot(2,2,2);
plot(Fz_set/1000, peak_mu, 'o-', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'MarkerFaceColor', [0.2 0.5 0.8]);
xlabel('Vertical load F_z [kN]');
ylabel('Peak μ_{eff} = max|F_y|/F_z [-]');
title('Load sensitivity (peak μ vs F_z)');
grid on;

%% ---- Plot 3: peak slip angle vs Fz -----------------------------------
subplot(2,2,3);
plot(Fz_set/1000, peak_at, 's-', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'MarkerFaceColor', [0.8 0.4 0.2]);
xlabel('Vertical load F_z [kN]');
ylabel('α at peak F_y [°]');
title('Peak shifts to larger α at higher F_z');
grid on;

%% ---- Plot 4: cornering stiffness Ky(Fz) ------------------------------
subplot(2,2,4); hold on;
Fz_dense = linspace(500, 12000, 60);
Ky = zeros(size(Fz_dense));
for k = 1:numel(Fz_dense)
    [~, info] = pacejka_fy(0.001, Fz_dense(k), p);   % small α to get B·D
    Ky(k) = info.Ky;
end
plot(Fz_dense/1000, abs(Ky)/1000, 'LineWidth', 1.5, 'Color', [0.2 0.6 0.4]);
xlabel('Vertical load F_z [kN]');
ylabel('Cornering stiffness |K_y| [kN/rad]');
title('Cornering stiffness vs F_z');
grid on;

%% ---- Print landmarks -------------------------------------------------
fprintf('\n=== v06 Pacejka MF 5.2 — pure-lateral validation ===\n');
fprintf('Source: %s\n', p.source);
fprintf('Nominal load Fz0 = %.0f N\n\n', p.Fz0);
fprintf('  Fz [N]    peak μ_eff    α_peak [deg]\n');
fprintf('  --------  -----------   -----------\n');
for k = 1:numel(Fz_set)
    fprintf('  %6d    %.3f         %5.2f\n', ...
            Fz_set(k), peak_mu(k), peak_at(k));
end

% Load sensitivity check
mu_low  = peak_mu(1);
mu_high = peak_mu(end);
ld_pct  = 100 * (mu_low - mu_high) / mu_low;
fprintf('\nLoad sensitivity: peak μ drops from %.3f at %d N to %.3f at %d N\n', ...
        mu_low, Fz_set(1), mu_high, Fz_set(end));
fprintf('  → %.1f%% drop over a 5x load increase (typical racing slick: 8-15%%).\n', ld_pct);

% Cornering stiffness at Fz0
[~, info0] = pacejka_fy(0.001, p.Fz0, p);
fprintf('\nCornering stiffness at Fz0 = %d N: %.0f N/rad (%.0f N/deg)\n', ...
        p.Fz0, abs(info0.Ky), abs(info0.Ky)*pi/180);

fprintf('\n=== Done. Inspect the figure: peaks should be at 6..10° and shift right with load. ===\n\n');
