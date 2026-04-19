%% diagnose_grip.m
%  Quick diagnostic: what does the reference telemetry tell us about
%  the car's actual grip level vs. what the sim assumes?
%
%  This helps identify why v02/v03 are 36 seconds too fast.

if ~exist('ref', 'var')
    error('Run import_reference_lap first.');
end
if ~exist('car', 'var')
    error('Run startup_project first.');
end

fprintf('\n=== GRIP DIAGNOSTIC ===\n\n');

%% 1. What lateral g does the driver actually achieve?
fprintf('--- Reference lap peak lateral g ---\n');
fprintf('Peak |g_lat|:  %.2f g\n', max(abs(ref.g_lat)));
fprintf('99th pctile:   %.2f g\n', prctile(abs(ref.g_lat), 99));
fprintf('95th pctile:   %.2f g\n', prctile(abs(ref.g_lat), 95));
fprintf('90th pctile:   %.2f g\n', prctile(abs(ref.g_lat), 90));

%% 2. What does the sim assume as max grip?
fprintf('\n--- Sim grip assumptions ---\n');
fprintf('v01/v02 constant mu:   %.2f  →  max lat g = %.2f g\n', ...
        car.tyre.mu_peak, car.tyre.mu_peak);
fprintf('v03 mu at  60 km/h:    %.3f  →  max lat g = %.2f g (low-speed corner)\n', ...
        1.85 - 5.5e-5 * (car.mass*car.g + car.aero_df_coeff*(60/3.6)^2)/4, ...
        (1.85 - 5.5e-5 * (car.mass*car.g + car.aero_df_coeff*(60/3.6)^2)/4) * ...
        (car.mass*car.g + car.aero_df_coeff*(60/3.6)^2) / (car.mass*car.g));

% Simpler: what's the max lateral g at different speeds with aero + load sens?
speeds_test = [60, 100, 150, 200, 250];
fprintf('\n--- v03 max lateral g at different speeds ---\n');
for si = 1:length(speeds_test)
    v = speeds_test(si) / 3.6;
    Fz = car.mass*car.g + car.aero_df_coeff * v^2;
    mu = 1.85 - 5.5e-5 * Fz/4;
    a_lat_g = mu * Fz / (car.mass * car.g);
    fprintf('  %3d km/h:  mu = %.3f,  max lat g = %.2f g\n', ...
            speeds_test(si), mu, a_lat_g);
end

%% 3. What does the DRIVER actually achieve at different speeds?
%  Bin the reference data by speed and look at peak g in each bin
fprintf('\n--- Reference: actual lateral g achieved per speed bin ---\n');
speed_bins = [40, 80, 120, 160, 200, 240, 280];
for bi = 1:length(speed_bins)-1
    mask = ref.v_kmh >= speed_bins(bi) & ref.v_kmh < speed_bins(bi+1);
    if sum(mask) > 10
        peak_g = prctile(abs(ref.g_lat(mask)), 98);
        fprintf('  %3d-%3d km/h:  peak |g_lat| (98th pct) = %.2f g  (n=%d)\n', ...
                speed_bins(bi), speed_bins(bi+1), peak_g, sum(mask));
    end
end

%% 4. Curvature smoothing check
%  Compare cornering speed from raw vs smoothed curvature
fprintf('\n--- Curvature smoothing impact ---\n');
g_val = 9.81;
mu_check = car.tyre.mu_peak;

% Recompute raw curvature (unsmoothed)
a_lat_raw = ref.g_lat * g_val;
v_clamp = max(ref.v, 10);
kappa_raw = abs(a_lat_raw ./ (v_clamp.^2));

fprintf('Raw curvature max:      %.6f [1/m] (R = %.1f m)\n', ...
        max(kappa_raw), 1/max(kappa_raw));
fprintf('Smoothed curvature max: %.6f [1/m] (R = %.1f m)\n', ...
        max(track.kappa), 1/max(track.kappa));
fprintf('Ratio: smoothed/raw = %.2f (%.0f%% of peak preserved)\n', ...
        max(track.kappa)/max(kappa_raw), max(track.kappa)/max(kappa_raw)*100);

% What's the speed difference at the tightest corner?
v_raw_tight = sqrt(mu_check * g_val / max(kappa_raw));
v_smooth_tight = sqrt(mu_check * g_val / max(track.kappa));
fprintf('v01 cornering speed at tightest point:\n');
fprintf('  From raw curvature:      %.1f km/h\n', v_raw_tight * 3.6);
fprintf('  From smoothed curvature: %.1f km/h\n', v_smooth_tight * 3.6);
fprintf('  Difference:              %.1f km/h\n', (v_smooth_tight - v_raw_tight) * 3.6);

%% 5. Summary
fprintf('\n--- DIAGNOSIS SUMMARY ---\n');
peak_ref_g = prctile(abs(ref.g_lat), 99);
fprintf('Driver achieves ~%.2f g peak lateral (99th percentile)\n', peak_ref_g);
fprintf('Sim assumes up to %.2f g at low speed, %.2f g at 200 km/h\n', ...
        car.tyre.mu_peak * 1.05, ...  % rough v03 low-speed mu
        (1.85 - 5.5e-5 * (car.mass*car.g + car.aero_df_coeff*(200/3.6)^2)/4) * ...
        (car.mass*car.g + car.aero_df_coeff*(200/3.6)^2) / (car.mass*car.g));
fprintf('Curvature smoothing preserves %.0f%% of peak curvature\n', ...
        max(track.kappa)/max(kappa_raw)*100);
fprintf('================================================\n\n');
