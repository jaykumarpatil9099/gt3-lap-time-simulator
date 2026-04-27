%% AMG GT3 — Vehicle Parameter File
%  Project : N24 Lap Time Simulator
%  Car     : Mercedes-AMG GT3 (Evo, 2020+ homologation)
%  Config  : Nürburgring 24h, mid-downforce, qualifying fuel load
%  Author  : Jaykumar Patil
%  Created : 2026-04-16
%
%  USAGE:
%    Run this script to load the struct 'car' into the workspace.
%    All model versions (v01, v02, ...) load this same file.
%
%  DATA QUALITY FLAGS:
%    [HOMOL]   = from FIA/SRO homologation documents or official specs
%    [IRACING] = from iRacing community reverse-engineering / known sim values
%    [EST]     = engineering estimate — flagged for refinement during correlation
%    [CALC]    = calculated from other parameters
%
%  SIGN CONVENTIONS:
%    Downforce is POSITIVE (force pushing car into ground)
%    Drag is POSITIVE (force opposing motion)
%    All SI units unless noted

%% ========================================================================
%  1. MASS & INERTIA
%  ========================================================================

car.mass = 1350;                % [kg]    Total running mass (car + driver + half fuel)  [EST]
                                %         BoP minimum ~1285 kg + 75 kg driver + ~40 kg fuel (half tank)
                                %         Ref: SRO BoP bulletin, typical N24 qualifying mass

car.g = 9.81;                   % [m/s^2] Gravitational acceleration (standard)

car.weight = car.mass * car.g;  % [N]     Total weight [CALC]

%% ========================================================================
%  2. GEOMETRY
%  ========================================================================

car.wheelbase = 2.710;          % [m]     Front axle to rear axle                        [HOMOL]
                                %         Mercedes-AMG GT (R190) platform

car.track_f = 1.680;            % [m]     Front track width (contact patch to contact patch) [HOMOL]
car.track_r = 1.660;            % [m]     Rear track width                                   [HOMOL]

car.h_cog = 0.465;              % [m]     Centre of gravity height above ground            [EST]
                                %         GT3 cars with flat floor + splitter + cage: 450-480 mm typical
                                %         Conservative mid-estimate; refine during v04 correlation

car.weight_dist_f = 0.46;       % [-]     Front weight distribution (static, fraction)    [IRACING]
                                %         AMG GT3 is front-mid engine: ~46% front, 54% rear
car.weight_dist_r = 1 - car.weight_dist_f;  % [-]  Rear weight distribution [CALC]

% Static axle loads
car.Fz_f_static = car.weight * car.weight_dist_f;   % [N] Front axle normal load (static) [CALC]
car.Fz_r_static = car.weight * car.weight_dist_r;   % [N] Rear axle normal load (static)  [CALC]

%% ========================================================================
%  3. AERODYNAMICS
%  ========================================================================
%  Aero forces: F = 0.5 * rho * v^2 * A * C
%  Downforce is defined POSITIVE (pushes car down = adds to tyre Fz)
%  Drag is defined POSITIVE (opposes forward motion)

car.rho = 1.225;               % [kg/m^3] Air density at sea level, 15°C (ISA standard)
                                %          Nürburgring is ~620m ASL; rho ~ 1.16 there.
                                %          We use 1.225 for now; adjust in correlation if needed.

car.frontal_area = 2.08;        % [m^2]   Frontal area                                   [HOMOL]

car.Cd = 0.52;                  % [-]     Drag coefficient                               [EST]
                                %         GT3 cars: 0.48-0.55 depending on wing angle
                                %         Mid-downforce N24 config

car.Cl = 1.72;                  % [-]     Downforce coefficient (POSITIVE = downforce)    [EST]
                                %         Note: some references use negative Cl for downforce.
                                %         In our convention, positive Cl = downforce.
                                %         GT3 cars at N24: Cl ~ 1.60-1.85

car.aero_balance_f = 0.43;      % [-]     Fraction of total downforce on front axle       [EST]
                                %         42-45% typical for GT3 at N24

% Precompute aero constants (multiply by v^2 to get forces)
car.aero_drag_coeff = 0.5 * car.rho * car.frontal_area * car.Cd;     % [N/(m/s)^2] [CALC]
car.aero_df_coeff   = 0.5 * car.rho * car.frontal_area * car.Cl;     % [N/(m/s)^2] [CALC]

%% ========================================================================
%  4. TYRES
%  ========================================================================
%  Control tyre: Pirelli DHF (N24 spec)
%
%  For v01-v02: single peak friction coefficient (constant mu)
%  For v03+:    load-sensitive model: mu(Fz) = mu_0 - k * Fz
%               where Fz is the vertical load on that axle's tyre [N]

car.tyre.mu_peak = 1.60;        % [-]     Peak tyre friction coefficient                 [IRACING]
                                 %         Pirelli DHF GT3 compound: mu ~ 1.55-1.70
                                 %         Used in v01 and v02 as constant grip limit

car.tyre.mu_0 = 1.70;           % [-]     Zero-load extrapolated friction coefficient     [CAL]
                                 %         For load sensitivity model (v03+).
                                 %         CALIBRATED 2026-04-21 against telemetry reference lap
                                 %         in phase5_step4_calibration.m. Previous [EST] value
                                 %         was 1.85; the calibration sweep on (mu_0, load_sens_k)
                                 %         landed at mu_0 = 1.70, load_sens_k = 4.4e-5, giving
                                 %         v05 lap = 8:10.539 (Δ = -0.80 s, -0.16% — inside ±1%
                                 %         charter). Best-fit minimised the per-sector RMS Δt
                                 %         (1.49 s), not just total lap delta — the latter alone
                                 %         can hit zero with cancelling sector errors.
                                 %         Sweep boundary note: the optimum sat at the lower
                                 %         edge of the grid, so the true minimum may be slightly
                                 %         lower still. Charter is met, so we accept rather than
                                 %         refine. See logbook Entry 017.

car.tyre.load_sens_k = 4.4e-5;  % [1/N]   Load sensitivity slope                         [CAL]
                                 %         mu(Fz) = mu_0 - k * Fz
                                 %         CALIBRATED 2026-04-21 (see mu_0 above).
                                 %         At Fz = 4500 N (typical corner weight):
                                 %           mu = 1.70 - 4.4e-5 * 4500 = 1.502
                                 %         At Fz = 7000 N (heavy aero loading):
                                 %           mu = 1.70 - 4.4e-5 * 7000 = 1.392
                                 %         Lower mu_0 + flatter k vs. previous [EST] = lower
                                 %         baseline grip but slower fall-off with load.

car.tyre.rolling_radius = 0.327; % [m]     Effective rolling radius (325/705-18 GT3 spec)  [IRACING]

%% ========================================================================
%  5. POWERTRAIN
%  ========================================================================
%  Engine: Mercedes-AMG M159-derived 6.3L V8, naturally aspirated
%  BoP restricted to ~550-580 hp depending on event
%
%  Torque curve: simplified piecewise from known data
%  RPM values and corresponding torque [Nm]

car.engine.rpm =    [3000, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7200]; % [rpm]   [IRACING]
car.engine.torque = [ 480,  530,  560,  580,  585,  575,  555,  520,  500]; % [Nm]    [IRACING]
                    %  BoP-restricted curve; peak ~585 Nm around 5500 rpm
                    %  Peak power ≈ 585 * 5500 * 2*pi/60 / 1000 ≈ 337 kW ≈ 452 hp
                    %  Wait — that seems low. Let me check:
                    %  Actually at 7000 rpm: 520 * 7000 * 2*pi/60 / 1000 ≈ 381 kW ≈ 511 hp
                    %  With BoP restrictor, peak power lands ~520-550 hp region
                    %  This will be refined during correlation

car.engine.rpm_idle = 3000;      % [rpm]   Idle / minimum operating RPM                   [EST]
car.engine.rpm_max  = 7200;      % [rpm]   Rev limiter                                    [IRACING]

%% ========================================================================
%  6. TRANSMISSION
%  ========================================================================
%  6-speed sequential gearbox
%  Ratios from iRacing community data

car.gearbox.ratios = [3.40, 2.19, 1.63, 1.29, 1.05, 0.88];  % [-] Gear 1-6             [IRACING]
car.gearbox.final_drive = 3.47;                                % [-] Final drive ratio     [IRACING]
car.gearbox.n_gears = length(car.gearbox.ratios);

% Drivetrain efficiency (accounts for gearbox + diff losses)
car.gearbox.efficiency = 0.92;   % [-]     Overall drivetrain efficiency                  [EST]
                                  %         Sequential + limited-slip diff: 90-94% typical

% Precompute total reduction per gear (gear ratio * final drive)
car.gearbox.total_ratio = car.gearbox.ratios * car.gearbox.final_drive;  % [CALC]

% Compute theoretical top speed per gear [m/s]
% v = (rpm * 2*pi/60 * tyre_radius) / total_ratio
for i = 1:car.gearbox.n_gears
    car.gearbox.v_max_gear(i) = (car.engine.rpm_max * 2*pi/60 * car.tyre.rolling_radius) ...
                                 / car.gearbox.total_ratio(i);
end

%% ========================================================================
%  7. BRAKING
%  ========================================================================
%  In a QSS lap sim, braking is usually tyre-limited, not hardware-limited.
%  GT3 carbon brakes can produce more torque than the tyres can handle.
%  So we model max braking deceleration through the tyre grip limit.
%
%  Brake bias is relevant from v04+ (when we model per-axle loads).

car.brakes.bias_f = 0.57;       % [-]     Front brake bias (fraction of total braking force) [IRACING]
                                 %         57% front is a typical GT3 N24 starting point

%% ========================================================================
%  8. SUSPENSION — ANTI-ROLL BARS (ARB)  [REVISED 2026-04-21]
%  ========================================================================
%  Correction note (Entry 016): the earlier model in this file treated ARBs
%  as if they *reduced* the total lateral load transfer on each axle
%  (`dFz_eff = dFz_full * K_tire/(K_ARB+K_tire)`). That is wrong physics.
%
%  Total lateral load transfer is a rigid-body consequence of the CG being
%  above ground. At a given lateral acceleration it is fixed by geometry:
%
%      ΔFz_lat_total = m * a_lat * h_cog / t_avg
%
%  (t_avg = (track_f + track_r)/2). This total cannot be reduced by any
%  suspension choice. What ARBs *do* is set how the total is *distributed*
%  between the front and rear axles. Stiffer front ARB (relative to rear)
%  forces the front axle to absorb a larger share of the total transfer,
%  which in turn erodes more front grip → the car pushes (understeer). A
%  soft front / stiff rear setup does the opposite → loose rear (oversteer).
%
%  The roll-stiffness distribution is:
%      roll_dist_f = K_roll_f / (K_roll_f + K_roll_r)
%      K_roll_axle = K_ARB_axle + K_tire_axle
%  where K_tire is the tyre's contribution to axle roll stiffness (it acts
%  in parallel with the bar). For v05+ we use this to split the total
%  lateral transfer between axles without altering the total.
%
%  GT3 typical stiffness ranges (unchanged from earlier note):
%    - Front ARB roll stiffness: K_ARB_f ~ 100,000-200,000 N·m/rad
%    - Rear ARB roll stiffness:  K_ARB_r ~ 80,000-150,000 N·m/rad
%    - Tyre roll stiffness per axle: K_tire ~ 50,000-100,000 N·m/rad

car.suspension.K_ARB_f = 150000;  % [N·m/rad] Front anti-roll bar stiffness           [EST]
                                  %            Typical GT3: 120,000-180,000
                                  %            N24 setup is usually stiff for curb control

car.suspension.K_ARB_r = 100000;  % [N·m/rad] Rear anti-roll bar stiffness            [EST]
                                  %            Typical GT3: 80,000-130,000
                                  %            Rear is usually softer for stability

car.suspension.K_tire_f = 75000;  % [N·m/rad] Front tyre roll stiffness per axle      [EST]
                                  %            Tyre vertical-stiffness contribution to
                                  %            front axle roll stiffness (in parallel
                                  %            with K_ARB_f).

car.suspension.K_tire_r = 75000;  % [N·m/rad] Rear tyre roll stiffness per axle       [EST]

% Per-axle roll stiffness = ARB + tyre, acting in parallel.
car.suspension.K_roll_f = car.suspension.K_ARB_f + car.suspension.K_tire_f;   % [CALC]
car.suspension.K_roll_r = car.suspension.K_ARB_r + car.suspension.K_tire_r;   % [CALC]

% Roll-stiffness distribution (front share of total lateral load transfer).
% v05 uses this to split ΔFz_lat_total between axles.
car.suspension.roll_dist_f = car.suspension.K_roll_f / ...
    (car.suspension.K_roll_f + car.suspension.K_roll_r);   % [-] [CALC]
car.suspension.roll_dist_r = 1 - car.suspension.roll_dist_f;  % [-] [CALC]

%% ========================================================================
%  9. METADATA
%  ========================================================================

car.meta.name = 'Mercedes-AMG GT3 Evo';
car.meta.config = 'N24 mid-downforce, qualifying fuel';
car.meta.track = 'Nürburgring 24h (Nordschleife + GP combined)';
car.meta.created = '2026-04-16';
car.meta.author = 'Jaykumar Patil';
car.meta.notes = ['Initial parameter set. Flags: HOMOL = homologation data, ' ...
                  'IRACING = iRacing sim data, EST = estimate, CALC = calculated. ' ...
                  'All EST-flagged values are candidates for refinement during correlation.'];

%% ========================================================================
%  VERIFICATION — run this section to sanity-check the parameters
%  ========================================================================

fprintf('\n=== AMG GT3 Parameter Verification ===\n');
fprintf('Mass:              %.0f kg\n', car.mass);
fprintf('Weight:            %.0f N\n', car.weight);
fprintf('Wheelbase:         %.3f m\n', car.wheelbase);
fprintf('CoG height:        %.3f m\n', car.h_cog);
fprintf('Weight dist (F/R): %.0f%% / %.0f%%\n', car.weight_dist_f*100, car.weight_dist_r*100);
fprintf('Static Fz front:   %.0f N\n', car.Fz_f_static);
fprintf('Static Fz rear:    %.0f N\n', car.Fz_r_static);
fprintf('Frontal area:      %.2f m^2\n', car.frontal_area);
fprintf('Cd / Cl:           %.2f / %.2f\n', car.Cd, car.Cl);
fprintf('L/D ratio:         %.2f\n', car.Cl / car.Cd);
fprintf('Tyre mu (peak):    %.2f\n', car.tyre.mu_peak);
fprintf('Rolling radius:    %.3f m\n', car.tyre.rolling_radius);
fprintf('Peak engine torque: %.0f Nm @ %.0f rpm\n', max(car.engine.torque), ...
         car.engine.rpm(car.engine.torque == max(car.engine.torque)));
fprintf('Gear ratios:       '); fprintf('%.2f  ', car.gearbox.ratios); fprintf('\n');
fprintf('Final drive:       %.2f\n', car.gearbox.final_drive);
fprintf('Top speed per gear [km/h]: '); fprintf('%.0f  ', car.gearbox.v_max_gear * 3.6); fprintf('\n');
fprintf('Brake bias (front): %.0f%%\n', car.brakes.bias_f * 100);
fprintf('\n--- Suspension / ARB ---\n');
fprintf('ARB stiffness (F/R):         %.0f / %.0f N·m/rad\n', car.suspension.K_ARB_f, car.suspension.K_ARB_r);
fprintf('Tyre roll stiffness (F/R):   %.0f / %.0f N·m/rad\n', car.suspension.K_tire_f, car.suspension.K_tire_r);
fprintf('Total roll stiffness (F/R):  %.0f / %.0f N·m/rad\n', car.suspension.K_roll_f, car.suspension.K_roll_r);
fprintf('Roll distribution (F/R):     %.1f%% / %.1f%%  (front share of total lateral ΔFz)\n', ...
        car.suspension.roll_dist_f*100, car.suspension.roll_dist_r*100);
fprintf('\n--- Aero loads at 200 km/h (55.6 m/s) ---\n');
v_test = 200/3.6;
fprintf('Drag:              %.0f N (%.1f kg)\n', car.aero_drag_coeff * v_test^2, car.aero_drag_coeff * v_test^2 / car.g);
fprintf('Downforce:         %.0f N (%.1f kg)\n', car.aero_df_coeff * v_test^2, car.aero_df_coeff * v_test^2 / car.g);
fprintf('=====================================\n\n');
