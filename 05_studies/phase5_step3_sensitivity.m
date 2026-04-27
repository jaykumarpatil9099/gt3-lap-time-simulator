%% phase5_step3_sensitivity.m
%  Phase 5, Step 3 — Parameter sensitivity matrix on v05, GPS source.
%
%  For each parameter: sweep 5 values around the baseline, rerun v05,
%  record lap time. Produces a table of ∂t/∂param and a tornado chart
%  ranked by lap-time leverage.
%
%  Rationale for GPS source: Step 2 showed GPS has near-zero mean Δv
%  bias, so parameter effects read cleanly without line-choice noise.
%
%  USAGE
%    >> startup_project
%    >> import_reference_lap
%    >> track_source = 'gps'; build_track
%    >> run('05_studies/phase5_step3_sensitivity.m')
%
%  Author:  Jaykumar Patil
%  Created: 2026-04-21

if ~exist('ref','var') || ~exist('car','var') || ~exist('track','var')
    error('Load ref + car + GPS track first (see header usage).');
end

fprintf('\n==========================================================\n');
fprintf(' Phase 5 Step 3 — Sensitivity matrix (v05, GPS)            \n');
fprintf('==========================================================\n');

% Keep pristine copy — restore after every sweep
car_baseline = car;

% Baseline lap
lap_sim_v05;
baseline_lap = sim05.lap_time;
fprintf('\nBaseline v05 lap (GPS): %.3f s\n\n', baseline_lap);

%% ---- Parameter sweep definitions --------------------------------------
%  Each entry: { setter_fn, 5 values, short label }
%  setter_fn is a function handle that takes (car, value) and returns
%  a modified car struct. This keeps the sweep loop clean.

baseline_brake_bias = car.brakes.bias_f;   % capture before loop
baseline_Cl         = car.Cl;
baseline_mu0        = car.tyre.mu_0;
baseline_k_load     = car.tyre.load_sens_k;
baseline_aero_bal_f = car.aero_balance_f;
baseline_h_cog      = car.h_cog;
baseline_K_ARB_f    = car.suspension.K_ARB_f;
baseline_K_ARB_r    = car.suspension.K_ARB_r;
baseline_wdist_f    = car.weight_dist_f;

sweeps = {
  struct('name','h_cog [m]',         'base',baseline_h_cog,      ...
         'vals',baseline_h_cog      * [0.90 0.95 1.00 1.05 1.10], ...
         'set',@(c,v) setfield(c,'h_cog',v))
  struct('name','brake_bias_f [-]',  'base',baseline_brake_bias, ...
         'vals',[0.53 0.55 0.57 0.59 0.61],                        ...
         'set',@(c,v) bb_set(c,v))
  struct('name','Cl [-]',            'base',baseline_Cl,          ...
         'vals',baseline_Cl         * [0.90 0.95 1.00 1.05 1.10], ...
         'set',@(c,v) cl_set(c,v))
  struct('name','aero_balance_f [-]','base',baseline_aero_bal_f,  ...
         'vals',[0.39 0.41 0.43 0.45 0.47],                        ...
         'set',@(c,v) setfield(c,'aero_balance_f',v))
  struct('name','mu_0 [-]',          'base',baseline_mu0,         ...
         'vals',baseline_mu0        * [0.95 0.975 1.00 1.025 1.05],...
         'set',@(c,v) mu0_set(c,v))
  struct('name','load_sens_k [1/N]', 'base',baseline_k_load,      ...
         'vals',baseline_k_load     * [0.80 0.90 1.00 1.10 1.20], ...
         'set',@(c,v) klz_set(c,v))
  struct('name','K_ARB_f [Nm/rad]',  'base',baseline_K_ARB_f,     ...
         'vals',baseline_K_ARB_f    * [0.70 0.85 1.00 1.15 1.30], ...
         'set',@(c,v) arbf_set(c,v))
  struct('name','K_ARB_r [Nm/rad]',  'base',baseline_K_ARB_r,     ...
         'vals',baseline_K_ARB_r    * [0.70 0.85 1.00 1.15 1.30], ...
         'set',@(c,v) arbr_set(c,v))
  struct('name','weight_dist_f [-]', 'base',baseline_wdist_f,     ...
         'vals',[0.44 0.45 0.46 0.47 0.48],                        ...
         'set',@(c,v) setfield(c,'weight_dist_f',v))
};

n_params = numel(sweeps);
result   = struct();

%% ---- Run the sweeps ---------------------------------------------------
for p = 1:n_params
    S = sweeps{p};
    fprintf('--- %s (baseline = %g) ---\n', S.name, S.base);
    laps = zeros(1, numel(S.vals));
    for k = 1:numel(S.vals)
        car = S.set(car_baseline, S.vals(k));
        % v05 needs roll stiffnesses rebuilt if ARB changed
        car.suspension.K_roll_f = car.suspension.K_ARB_f + car.suspension.K_tire_f;
        car.suspension.K_roll_r = car.suspension.K_ARB_r + car.suspension.K_tire_r;
        car.suspension.roll_dist_f = car.suspension.K_roll_f / ...
            (car.suspension.K_roll_f + car.suspension.K_roll_r);
        car.suspension.roll_dist_r = 1 - car.suspension.roll_dist_f;
        evalc('lap_sim_v05;');   % suppress solver prints
        laps(k) = sim05.lap_time;
        fprintf('    %g → %.3f s  (Δ %+.3f)\n', ...
                S.vals(k), laps(k), laps(k) - baseline_lap);
    end
    result(p).name      = S.name;
    result(p).baseline  = S.base;
    result(p).values    = S.vals;
    result(p).laps      = laps;
    result(p).dlaps     = laps - baseline_lap;
    result(p).leverage  = (max(laps) - min(laps));  % full-range Δt
end
car = car_baseline;

%% ---- Rank by leverage -------------------------------------------------
[~, order] = sort([result.leverage], 'descend');
fprintf('\n=== Sensitivity ranking (full-range Δlap over the sweep) ===\n');
fprintf('%-22s %12s %12s\n', 'Parameter', 'Δt range [s]', 'baseline');
fprintf('%s\n', repmat('-',1,50));
for i = order
    fprintf('%-22s %12.3f %12g\n', ...
            result(i).name, result(i).leverage, result(i).baseline);
end

%% ---- Tornado plot -----------------------------------------------------
figure('Name','v05 sensitivity — tornado','NumberTitle','off', ...
       'Position',[100 100 900 500]);
barh([result(order).leverage], 'FaceColor',[0.2 0.5 0.8]);
set(gca, 'YTick', 1:n_params, 'YTickLabel', {result(order).name}, ...
         'YDir','reverse');
xlabel('Full-range Δlap time [s]');
title(sprintf('v05 sensitivity — GPS source — baseline %.3f s', baseline_lap));
grid on;

%% ---- Save -------------------------------------------------------------
outdir  = fileparts(mfilename('fullpath'));
outpath = fullfile(outdir, 'phase5_step3_result.mat');
save(outpath, 'result', 'baseline_lap');
fprintf('\nSaved: %s\n', outpath);
fprintf('Done.\n');

%% ---- local setter helpers --------------------------------------------
function c = bb_set(c,v),   c.brakes.bias_f      = v; end
function c = cl_set(c,v),   c.Cl                 = v;
    c.aero_df_coeff = 0.5 * c.rho * c.frontal_area * c.Cl; end
function c = mu0_set(c,v),  c.tyre.mu_0          = v; end
function c = klz_set(c,v),  c.tyre.load_sens_k   = v; end
function c = arbf_set(c,v), c.suspension.K_ARB_f = v; end
function c = arbr_set(c,v), c.suspension.K_ARB_r = v; end
