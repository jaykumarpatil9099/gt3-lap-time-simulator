%% v06_1_run.m
%  Phase 6.5 v06.1 — fit Pacejka MF 5.2 lateral coefficients from logged
%  iRacing slip-angle and Fy data, replacing the v05-anchored starting
%  values. Produces before/after curves, lap-time delta, and a struct of
%  fitted coefficients you can choose to lock into amg_gt3_params.m.
%
%  WORKFLOW
%    1. Extract (|α|, Fz, |F_y|) from the IBT reference lap (extract_pacejka_data).
%    2. Fit pCy1, pDy1, pDy2, pEy1, pKy1, pKy2 by minimising sum of
%       squared residuals between observed F_y_axle and the
%       per-axle Pacejka prediction (sum of two tyres).
%    3. Plot fitted curve + raw data, front and rear separately.
%    4. Print before/after coefficients side-by-side.
%    5. (Optional) inject fitted coeffs into car and rerun lap_sim_v06b.
%
%  USAGE
%    >> startup_project
%    >> import_reference_lap_ibt
%    >> track_source = 'telemetry'; build_track
%    >> lap_sim_v05; lap_sim_v06b      % baselines
%    >> run('03_models/v06_pacejka/v06_1_run.m')
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-22  (v06.1 Pacejka data-fit)

if ~exist('car','var')   || ~exist('ref','var')
    error('Run startup_project + import_reference_lap_ibt first.');
end
if ~isfield(car.tyre, 'pacejka'),  pacejka_coeffs_pirelli_dhf_gt3;  end
if ~isfield(car, 'steering_ratio')
    error(['car.steering_ratio missing — re-run startup_project to ' ...
           'pick up the v06.1 params update.']);
end

fprintf('\n==========================================================\n');
fprintf(' Phase 6.5 v06.1 — Pacejka MF 5.2 data-fit                 \n');
fprintf('==========================================================\n');

%% --- 1. Extract data from the IBT reference --------------------------
data = extract_pacejka_data(ref.meta.source_file, car);

%% --- 2. Fit coefficients via fminsearch ------------------------------
%  Free parameters: pDy1 and pKy1 only (peak grip and cornering
%  stiffness). pDy2 stays locked at the v05 anchor value (-0.275) —
%  single-lap IBT data correlates Fz with Fy through aero downforce,
%  which suppresses the load-drop signal and lets pDy2 drift to zero
%  in 3-parameter fits. With multi-lap or rig data we'd refit pDy2 too.
%  The other shape parameters (pCy1, pEy1, pKy2) are fixed at anchor
%  values throughout v06.1.

p_init = car.tyre.pacejka.front;
x0 = [p_init.pDy1, p_init.pKy1];

% Anonymous handle wrapping pacejka_fy with the candidate coefficient set
predict_axle = @(p, alpha, Fz_out, Fz_in) ...
    abs(pacejka_fy(alpha, Fz_out, p)) + abs(pacejka_fy(alpha, Fz_in, p));

cost_fn = @(x) local_cost(x, p_init, data, predict_axle);

opts_fmin = optimset('Display','iter', 'TolX',1e-5, 'TolFun',1e-5, ...
                     'MaxIter',500, 'MaxFunEvals',2000);
fprintf('\nFitting (fminsearch on pDy1, pKy1 — %d data points; pDy2 locked at %.4f)...\n', ...
        data.n_samples_kept, p_init.pDy2);
[x_fit, J_fit] = fminsearch(cost_fn, x0, opts_fmin);

% Build the fitted coefficient struct (front + rear identical at v06.1)
p_fit = p_init;
p_fit.pDy1 = x_fit(1);
% pDy2 stays at anchor
p_fit.pKy1 = x_fit(2);
p_fit.source  = 'Fitted from iRacing IBT slip-angle / F_y log (v06.1)';
p_fit.version = 'MF 5.2 (lateral data-fit) + v06.0b combined-slip';
p_fit.created = '2026-04-22-v06.1';

%% --- 3. Compare ------------------------------------------------------
fprintf('\n=== Before vs after coefficients (2 free, pDy2 locked) ===\n');
fprintf('  %-12s  %12s  %12s\n', 'param', 'v06.0b anchor', 'v06.1 fit');
fprintf('  %-12s  %12.4f  %12.4f\n', 'pDy1', p_init.pDy1, p_fit.pDy1);
fprintf('  %-12s  %12.4f  %12.4f  %s\n', 'pDy2', p_init.pDy2, p_fit.pDy2, '(locked)');
fprintf('  %-12s  %12.4f  %12.4f\n', 'pKy1', p_init.pKy1, p_fit.pKy1);
fprintf('  Final normalised cost:  %.4f\n', J_fit);

%% --- 4. Plot ----------------------------------------------------------
figure('Name','v06.1 Pacejka data-fit', 'NumberTitle','off', ...
       'Position',[100 100 1400 600]);

% Front axle: data + fitted curve at sample Fz values
subplot(1,2,1); hold on;
local_plot_axle(data.alpha_f, data.Fz_f_out, data.Fz_f_in, data.Fy_f_axle, ...
                p_init, p_fit, 'Front axle');

% Rear axle
subplot(1,2,2); hold on;
local_plot_axle(data.alpha_r, data.Fz_r_out, data.Fz_r_in, data.Fy_r_axle, ...
                p_init, p_fit, 'Rear axle');

%% --- 5. Save fitted coeffs (no auto-injection into car) -------------
out_dir  = fileparts(mfilename('fullpath'));
out_path = fullfile(out_dir, 'pacejka_fit_v06_1.mat');
save(out_path, 'p_fit', 'p_init', 'data', 'x0', 'x_fit', 'J_fit');
fprintf('\nSaved fitted coefficients to: %s\n', out_path);

fprintf('\nNEXT STEP — to test the fit in the lap solver, paste these:\n');
fprintf('   car.tyre.pacejka.front = p_fit;\n');
fprintf('   car.tyre.pacejka.rear  = p_fit;\n');
fprintf('   lap_sim_v06b\n');
fprintf('Then compare sim06b.lap_time to the v06.0b baseline.\n\n');

%% =====================================================================
%  Local helpers
%  =====================================================================

function J = local_cost(x, p_template, data, predict_axle)
    p = p_template;
    p.pDy1 = x(1);    % peak μ at Fz0
    p.pKy1 = x(2);    % cornering stiffness gradient (must be negative)
    % pDy2 stays at p_template.pDy2 (locked at anchor for v06.1)

    Fy_f_pred = predict_axle(p, data.alpha_f, data.Fz_f_out, data.Fz_f_in);
    Fy_r_pred = predict_axle(p, data.alpha_r, data.Fz_r_out, data.Fz_r_in);

    % Normalised residuals (unitless)
    Fy_scale = 8000;
    res_f = (Fy_f_pred - data.Fy_f_axle) / Fy_scale;
    res_r = (Fy_r_pred - data.Fy_r_axle) / Fy_scale;
    N = numel(res_f) + numel(res_r);
    SSR_norm = (sum(res_f.^2) + sum(res_r.^2)) / N;

    % Soft priors
    pen = 0;
    if x(1) < 1.2,  pen = pen + 100*(1.2 - x(1))^2; end
    if x(1) > 2.0,  pen = pen + 100*(x(1) - 2.0)^2; end
    if x(2) > -10,  pen = pen + 100*(x(2) + 10)^2;  end
    if x(2) < -80,  pen = pen + 100*(x(2) + 80)^2;  end

    J = SSR_norm + pen;
end

function local_plot_axle(alpha, Fz_out, Fz_in, Fy_axle_data, p_init, p_fit, ttl)
    % Scatter raw data, coloured by mean Fz
    Fz_mean = 0.5*(Fz_out + Fz_in);
    scatter(alpha*180/pi, Fy_axle_data/1000, 6, Fz_mean, 'filled', ...
            'MarkerFaceAlpha', 0.4, 'MarkerEdgeColor','none', ...
            'DisplayName','data');
    cb = colorbar; ylabel(cb, 'mean Fz [N]');

    % Reference Pacejka curves at three representative Fz values
    Fz_curve = quantile(Fz_mean, [0.25 0.50 0.75]);
    a_grid = linspace(0, 20, 200) * pi/180;
    cmap = lines(3);
    for k = 1:3
        Fy_init = pacejka_fy(a_grid', Fz_curve(k), p_init) * 2;   % both tyres
        Fy_fitc = pacejka_fy(a_grid', Fz_curve(k), p_fit ) * 2;
        plot(a_grid*180/pi, Fy_init/1000, '--', 'Color', cmap(k,:), ...
             'LineWidth', 1.0, 'DisplayName', ...
             sprintf('v06.0b @ %.0fN', Fz_curve(k)));
        plot(a_grid*180/pi, Fy_fitc/1000, '-',  'Color', cmap(k,:), ...
             'LineWidth', 1.5, 'DisplayName', ...
             sprintf('v06.1 fit @ %.0fN', Fz_curve(k)));
    end
    xlabel('|α| [deg]'); ylabel('|F_y_{axle}| [kN]');
    title(ttl); grid on; legend('Location','southeast');
    xlim([0 20]);
end
