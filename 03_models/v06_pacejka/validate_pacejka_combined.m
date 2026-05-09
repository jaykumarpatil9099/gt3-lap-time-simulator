%% validate_pacejka_combined.m
%  v06 Step 2 sanity check — plot the combined-slip friction envelope.
%  At fixed Fz, sweep slip angle α and slip ratio κ and trace the
%  reachable (Fx, Fy) envelope. The shape should be a clean ellipse-like
%  envelope with the corners pulled in (the G-factor reduction).
%
%  USAGE
%    >> startup_project
%    >> run('03_models/v06_pacejka/validate_pacejka_combined.m')
%
%  EXPECTED SHAPE
%  --------------
%  - Pure cornering (κ=0):   Fx = 0,   |Fy| up to peak Fy0.
%  - Pure braking/drive:     Fy = 0,   Fx up to peak Fx0 (drive) / down to -Fx0 (brake).
%  - Combined:               smooth elliptical envelope with no sharp
%                             corners. Fy at peak Fx is ~30..50 % of pure-Fy peak.
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-22  (v06 Pacejka — Phase 6.5, Step 2)

if ~exist('car', 'var'),                       error('Run startup_project first.'); end
if ~isfield(car.tyre, 'pacejka'),              pacejka_coeffs_pirelli_dhf_gt3; end

p  = car.tyre.pacejka.front;
Fz = 5000;        % nominal load (= Fz0)

%% ---- Sweep grids -----------------------------------------------------
alpha_deg = -15 : 0.5 : 15;
alpha     = alpha_deg * pi/180;
kappa     = -0.30 : 0.01 : 0.30;

[A, K] = meshgrid(alpha, kappa);
A = A(:);  K = K(:);
[Fy, Fx] = pacejka_combined(A, K, Fz*ones(size(A)), p);

%% ---- Plot 1: friction circle (Fy vs Fx envelope) --------------------
figure('Name','v06 Pacejka MF 5.2 — combined slip', ...
       'NumberTitle','off', 'Position',[100 100 1300 700]);

subplot(1,2,1); hold on; box on;
% Scatter all reachable points
scatter(Fx/1000, Fy/1000, 6, abs(K), 'filled', 'MarkerEdgeColor','none');
caxis([0 0.3]); h = colorbar; ylabel(h, '|κ|');
% Overlay pure-slip cuts: κ=0 (pure α) and α=0 (pure κ)
[Fy_pure, Fx_pure_zero] = pacejka_combined(alpha(:), zeros(size(alpha(:))), Fz*ones(numel(alpha),1), p);
plot(Fx_pure_zero/1000, Fy_pure/1000, 'k-', 'LineWidth', 2.0, ...
     'DisplayName', 'pure α (κ=0)');
[Fy_pure_zero, Fx_pure] = pacejka_combined(zeros(size(kappa(:))), kappa(:), Fz*ones(numel(kappa),1), p);
plot(Fx_pure/1000, Fy_pure_zero/1000, 'r-', 'LineWidth', 2.0, ...
     'DisplayName', 'pure κ (α=0)');
xlabel('Longitudinal force F_x [kN]');
ylabel('Lateral force F_y [kN]');
title(sprintf('Friction envelope at F_z = %d N', Fz));
axis equal; grid on;
legend('Location','southeast');

%% ---- Plot 2: Fy reduction with κ at fixed α --------------------------
subplot(2,2,2); hold on;
alpha_fix = [3, 6, 9, 12] * pi/180;
colors = lines(numel(alpha_fix));
for k = 1:numel(alpha_fix)
    [Fy_red, ~] = pacejka_combined(alpha_fix(k)*ones(size(kappa(:))), ...
                                   kappa(:), Fz*ones(numel(kappa),1), p);
    plot(kappa, Fy_red/1000, 'Color', colors(k,:), 'LineWidth', 1.4, ...
         'DisplayName', sprintf('α = %d°', round(alpha_fix(k)*180/pi)));
end
xlabel('Longitudinal slip ratio κ [-]');
ylabel('F_y [kN]');
title('Fy reduction by longitudinal slip (G_{yκ} effect)');
grid on; legend('Location','south');

%% ---- Plot 3: Fx reduction with α at fixed κ --------------------------
subplot(2,2,4); hold on;
kappa_fix = [0.05, 0.10, 0.15, 0.20];
colors = lines(numel(kappa_fix));
for k = 1:numel(kappa_fix)
    [~, Fx_red] = pacejka_combined(alpha(:), kappa_fix(k)*ones(size(alpha(:))), ...
                                   Fz*ones(numel(alpha),1), p);
    plot(alpha_deg, Fx_red/1000, 'Color', colors(k,:), 'LineWidth', 1.4, ...
         'DisplayName', sprintf('κ = %.2f', kappa_fix(k)));
end
xlabel('Slip angle α [°]');
ylabel('F_x [kN]');
title('Fx reduction by slip angle (G_{xα} effect)');
grid on; legend('Location','south');

%% ---- Numerical landmarks --------------------------------------------
[Fy_peak,    iy] = max(Fy_pure);
[Fx_peak,    ix] = max(Fx_pure);
% At peak |Fx|, what's the residual |Fy|?
[Fy_at_FxPk, Fx_at_FxPk] = pacejka_combined(0.10, kappa(ix), Fz, p);  % α=6° + peak κ
% At peak |Fy|, what's the maximum |Fx| we can still produce?
peak_alpha = alpha(iy);
% Sweep κ at peak_alpha and find max |Fx|:
[~, Fx_sweep] = pacejka_combined(peak_alpha*ones(size(kappa(:))), ...
                                 kappa(:), Fz*ones(numel(kappa),1), p);
[Fx_at_aPk, ~] = max(abs(Fx_sweep));

fprintf('\n=== v06 Pacejka MF 5.2 — combined-slip validation ===\n');
fprintf('Fz = %d N (nominal)\n\n', Fz);
fprintf('  Pure α (κ=0):  peak Fy = %5.0f N  at α = %5.1f°\n', ...
        Fy_peak, alpha_deg(iy));
fprintf('  Pure κ (α=0):  peak Fx = %5.0f N  at κ = %4.2f\n', ...
        Fx_peak, kappa(ix));
fprintf('\n');
fprintf('  At α=6°, κ=peak: Fy = %5.0f N (%.0f%% of pure Fy peak)\n', ...
        Fy_at_FxPk, 100*abs(Fy_at_FxPk)/abs(Fy_peak));
fprintf('  At α=peak, max |Fx| achievable = %5.0f N (%.0f%% of pure Fx peak)\n', ...
        Fx_at_aPk, 100*Fx_at_aPk/abs(Fx_peak));

% Friction envelope sanity: max |Fx|² + |Fy|² at any point should
% never exceed (peak Fy)² + (peak Fx)². Quick check on the worst point:
totals = sqrt((Fx/1000).^2 + (Fy/1000).^2);
[max_total, ~] = max(totals);
fprintf('\n  Max combined-force magnitude: %.2f kN (envelope ceiling).\n', max_total);
fprintf('  Pure Fy peak: %.2f kN, pure Fx peak: %.2f kN.\n', Fy_peak/1000, Fx_peak/1000);
fprintf('\n=== Done. Inspect Plot 1: should be a clean ellipse-like envelope. ===\n\n');
