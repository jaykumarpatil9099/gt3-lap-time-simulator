# Technical Reference — N24 Lap Time Simulator

**Author:** Jaykumar Patil
**Created:** 2026-04-16
**Last revised:** 2026-04-20 — added sections on the GPS centerline track source (§4A), v03 load sensitivity (§8), v04 longitudinal weight transfer (§9), and correlation diagnostics (§10); rewrote §4.4 around the two-stage curvature filter; refreshed the data-flow diagram in §5 to reflect the built v01..v04 solvers and the track-source dispatcher.
**Purpose:** Documents every calculation, equation, assumption, and design decision made in this project. This is the document you open when you ask "why did we do it this way?" or "what does this equation mean?"

---

## Table of Contents

1. Project architecture overview
2. Vehicle parameter file — every parameter explained
3. Telemetry import — how raw data becomes usable
4. Track data (telemetry source) — how we extract curvature from the racing line
4A. Track data (GPS source) — geometric curvature from the centerline
5. Data flow diagram
6. Unit conventions
7. Key equations summary
8. v03 — tyre load sensitivity
9. v04 — longitudinal weight transfer and per-axle grip
10. Correlation and diagnostics

---

## 1. Project Architecture Overview

### What is a lap time simulator?

A lap time simulator (LTS) takes two inputs — a car model and a track model — and computes the fastest possible lap time. It answers: "given this car's grip, power, aero, and mass, driven at the absolute limit around this track, what lap time does it produce?"

### Quasi-steady-state (QSS) approach

Our simulator uses the QSS method. "Quasi-steady-state" means we assume the car is in equilibrium at every point along the track. There are no dynamics — no suspension oscillations, no tyre warm-up, no transient weight transfer settling. At each point, we simply ask: "what is the maximum speed the car can sustain here?"

Why QSS works: at 100 Hz telemetry (one sample every 0.01 s), the car travels about 0.5 m per sample. Over 0.5 m, the car's state changes very little — suspension, tyres, and aero are effectively in steady state. So treating each point independently is a good approximation.

Why QSS fails: it cannot capture anything that depends on history — tyre temperature, brake fade, suspension transients over kerbs, driver reaction time. These are why we excluded them from scope.

### How the QSS solver will work (preview for Phase 3)

The solver walks along the track point by point. At each point, it knows the curvature (how tight the corner is). From the curvature and the car's grip, it calculates the maximum cornering speed. Then it checks whether the car can accelerate from the previous point's speed to this point's speed (limited by engine power and traction), or whether it must brake (limited by tyre grip). The output is a speed profile — speed at every meter of the track. Lap time = integral of 1/speed over distance.

---

## 2. Vehicle Parameter File — Every Parameter Explained

File: `02_data/car/amg_gt3_params.m`

This file defines a MATLAB struct called `car` with every physical property of the Mercedes-AMG GT3 that the simulator needs. Below is every parameter, what it physically means, why the simulator needs it, how we got the value, and how confident we are in it.

### 2.1 Mass and Weight

**car.mass = 1350 kg**

What it is: the total mass of the car as it runs on track. Includes the empty car (~1205 kg), driver in suit and helmet (~75 kg), half a tank of fuel (~40 kg), and all fluids (oil, coolant, brake fluid).

Why the simulator needs it: Newton's second law, F = m × a. Every acceleration (cornering, braking, accelerating) is force divided by mass. More mass = slower acceleration for the same force = slower lap time.

Where 1350 comes from: SRO Balance of Performance (BoP) sets the AMG GT3 minimum weight at approximately 1285 kg (this changes per event). Add 75 kg driver and ~40 kg fuel for a qualifying-representative mass of ~1350 kg. Flagged [EST] because the exact BoP weight for our reference event isn't confirmed.

**car.weight = car.mass × car.g = 1350 × 9.81 = 13,244 N**

What it is: the gravitational force pulling the car toward the ground. Measured in Newtons (the SI unit of force).

Why we precompute it: weight appears in many equations (normal loads, weight transfer). Computing it once avoids repeating "mass × g" throughout the code.

### 2.2 Geometry

**car.wheelbase = 2.710 m**

What it is: the distance from the centre of the front axle to the centre of the rear axle, measured horizontally.

Physical meaning: the wheelbase is a lever arm. When the car brakes, inertia tries to tip the car forward — the weight shifts from the rear axle to the front. How much weight shifts depends on braking force, CoG height, and wheelbase:

    Weight transfer (braking) = m × a_brake × h_cog / wheelbase

A longer wheelbase means less weight transfer for the same braking force, which is more stable. The AMG GT3's 2.710 m wheelbase comes from the road car platform (Mercedes-AMG GT, R190 chassis) and is in the homologation documents.

Needed from: v04 (longitudinal weight transfer). Collected now so we don't have to come back for it.

**car.track_f = 1.680 m, car.track_r = 1.660 m**

What it is: the distance between the left and right tyre contact patches, measured at the front axle (track_f) and rear axle (track_r).

Physical meaning: when the car corners, load transfers from the inside tyres to the outside tyres. The amount depends on lateral force, CoG height, and track width:

    Lateral load transfer = m × a_lat × h_cog / track_width

A wider track means less lateral load transfer, which preserves grip (because of tyre load sensitivity — see section 2.5).

Needed from: v05 (bicycle model with lateral load transfer). Collected now for completeness.

**car.h_cog = 0.465 m**

What it is: the height of the car's centre of gravity above the ground plane.

This is the hardest parameter to measure on a real car — teams use tilt-table rigs or pendulum tests that cost thousands of euros. We estimated 465 mm based on the fact that GT3 cars have: a flat floor very close to the ground (~50 mm ride height), a heavy engine and gearbox sitting relatively low, but a tall roll cage adding mass high up. Published values for GT3 cars range from 440-490 mm. 465 mm is a mid-estimate, flagged [EST].

Why it matters: CoG height drives both longitudinal and lateral weight transfer. A higher CoG = more weight transfer = more grip loss due to load sensitivity = slower. This is why race cars are built as low as possible.

**car.weight_dist_f = 0.46 (46% front, 54% rear)**

What it is: the fraction of the car's total weight that rests on the front axle when the car is stationary on flat ground.

The AMG GT3 has its engine behind the front axle but ahead of the driver (front-mid layout), so it's slightly rear-biased. 46/54 comes from iRacing's known values for this car.

Why it matters: determines the static load on each axle, which sets the starting point for tyre grip calculations. The front axle load is:

    Fz_f_static = weight × weight_dist_f = 13244 × 0.46 = 6092 N

The rear axle carries the rest:

    Fz_r_static = weight × (1 - weight_dist_f) = 13244 × 0.54 = 7151 N

### 2.3 Aerodynamics

**The fundamental aero equation:**

    F_aero = 0.5 × ρ × v² × A × C

Where:
- ρ (rho) = air density [kg/m³]
- v = car speed [m/s]
- A = frontal area [m²]
- C = coefficient (Cd for drag, Cl for downforce)

This equation comes from fluid dynamics. The key insight: aero force grows with the SQUARE of speed. At 200 km/h, aero force is 4× what it is at 100 km/h. At 260 km/h, it's 6.76× the force at 100 km/h. This is why aero dominates at high speed.

**car.rho = 1.225 kg/m³**

Air density at sea level, 15°C (ISA standard atmosphere). The Nürburgring sits at ~620 m elevation where rho is actually ~1.16 kg/m³. We use 1.225 for now and may adjust during correlation. This ~5% difference means our aero forces are ~5% too high at this stage.

**car.frontal_area = 2.08 m²**

The cross-sectional area of the car as seen head-on (imagine shining a light at the front of the car and measuring the shadow). From homologation data.

**car.Cd = 0.52 (drag coefficient)**

Dimensionless number describing how much aerodynamic resistance the car's shape produces. Higher Cd = more drag = slower on straights. GT3 cars at N24 mid-downforce settings: Cd ~ 0.48-0.55.

Drag force at any speed:

    F_drag = 0.5 × 1.225 × v² × 2.08 × 0.52

At 200 km/h (55.56 m/s):

    F_drag = 0.5 × 1.225 × 55.56² × 2.08 × 0.52 = 2045 N

That's equivalent to ~208 kg pushing against the car. The engine must overcome this just to maintain speed.

**car.Cl = 1.72 (downforce coefficient)**

Dimensionless number describing how much the car's shape pushes it into the ground. Higher Cl = more downforce = more grip at high speed, but usually comes with more drag too (aero trade-off).

SIGN CONVENTION: In our code, Cl is POSITIVE and means downforce (pushing down). Some textbooks use negative Cl for downforce (following the lift convention from aviation). We chose positive = downforce because it's more intuitive for racing.

Downforce at 200 km/h:

    F_downforce = 0.5 × 1.225 × 55.56² × 2.08 × 1.72 = 6763 N ≈ 689 kg

At 200 km/h, the aero pushes the car into the ground with a force equivalent to 689 kg — roughly half the car's own weight. This extra force pushes the tyres harder into the track, which increases grip.

At 260 km/h:

    F_downforce = 0.5 × 1.225 × 72.22² × 2.08 × 1.72 ≈ 11,428 N ≈ 1165 kg

Nearly doubling the effective weight of the car. This is why GT3 cars can corner at 2+ g at high speed but only ~1.6 g at low speed.

**car.aero_balance_f = 0.43 (43% front downforce)**

How the total downforce is split between front and rear axles. At 43% front, the rear gets 57% — so the rear has more aero grip than the front at high speed. This is a setup parameter teams adjust with wing angle, splitter, and diffuser.

Needed from: v05 (per-axle aero loads). Collected now.

**Precomputed aero constants:**

    car.aero_drag_coeff = 0.5 × ρ × A × Cd = 0.5 × 1.225 × 2.08 × 0.52 = 0.6627

    car.aero_df_coeff = 0.5 × ρ × A × Cl = 0.5 × 1.225 × 2.08 × 1.72 = 2.1916

To get the actual force at any speed, just multiply by v²:

    F_drag = car.aero_drag_coeff × v²
    F_downforce = car.aero_df_coeff × v²

We precompute these so the simulator doesn't redo the multiplication millions of times.

**L/D ratio = Cl / Cd = 1.72 / 0.52 = 3.31**

The lift-to-drag ratio (or more accurately, downforce-to-drag ratio in racing). This is the single most important efficiency metric in aerodynamics. It tells you how much downforce you get per unit of drag penalty. Higher is better. For reference:
- Road car: ~0.3
- GT3 car (N24): ~3.0-3.5
- LMP2: ~4.0+
- F1 car: ~5.0+

### 2.4 Engine and Transmission

**Engine torque curve: car.engine.rpm and car.engine.torque**

These are paired arrays. At 3000 RPM, the engine produces 480 Nm. At 5500 RPM, it produces 585 Nm (peak). At 7200 RPM (rev limiter), it produces 500 Nm.

Power = Torque × angular velocity:

    P = T × (RPM × 2π/60)

At peak torque (585 Nm, 5500 RPM):

    P = 585 × (5500 × 2π/60) = 585 × 575.96 = 336,936 W ≈ 337 kW ≈ 452 hp

At rev limit (500 Nm, 7200 RPM):

    P = 500 × (7200 × 2π/60) = 500 × 753.98 = 377,000 W ≈ 377 kW ≈ 505 hp

Peak power occurs near the rev limit, not at peak torque. This is normal for naturally aspirated engines.

**Gear ratios: [3.40, 2.19, 1.63, 1.29, 1.05, 0.88]**

Each number is the ratio of input RPM to output RPM for that gear. In 1st gear (ratio 3.40), the engine turns 3.40 times for every one turn of the gearbox output shaft. Higher ratio = more torque multiplication but lower top speed.

**Final drive ratio: 3.47**

An additional reduction after the gearbox, in the differential. The total reduction in any gear is:

    total_ratio = gear_ratio × final_drive

For gear 4: total_ratio = 1.29 × 3.47 = 4.4763

**How engine torque becomes wheel force:**

    F_wheel = (engine_torque × total_ratio × efficiency) / tyre_rolling_radius

In 4th gear at 5500 RPM:

    F_wheel = (585 × 4.4763 × 0.92) / 0.327 = 7366 N

This is the force pushing the car forward. Compare to drag at the same speed to see if the car is still accelerating.

**Theoretical top speed per gear:**

    v_max = (RPM_max × 2π/60 × rolling_radius) / total_ratio

6th gear:

    v_max = (7200 × 2π/60 × 0.327) / (0.88 × 3.47)
          = (7200 × 0.10472 × 0.327) / 3.0536
          = 246.64 / 3.0536
          = 80.76 m/s = 290.7 km/h

This is the THEORETICAL maximum (at rev limiter in top gear). The ACTUAL top speed is lower because drag increases with speed while engine force decreases (the engine can't produce enough force to overcome drag at this speed). The actual top speed is where:

    F_drive = F_drag  →  engine force = drag force

This equilibrium typically happens around 265-275 km/h for the AMG GT3 at N24 aero settings.

**Drivetrain efficiency: 0.92 (92%)**

Not all engine torque reaches the wheels. Friction in the gearbox bearings, gear mesh, oil churning, and the limited-slip differential absorb about 8% of the power. This is typical for a sequential racing gearbox with an LSD.

### 2.5 Tyres

**car.tyre.mu_peak = 1.60**

The peak friction coefficient. This is the maximum ratio of horizontal force to vertical force that the tyre can produce. If the tyre has 5000 N of vertical load pressing it into the track, it can produce up to:

    F_horizontal_max = μ × F_vertical = 1.60 × 5000 = 8000 N

of force in any direction (braking, cornering, or a combination).

μ = 1.60 means the tyre can produce 60% MORE horizontal force than its own vertical load. Road tyres have μ ≈ 0.8-1.0. Racing slicks achieve μ > 1.5 because of their soft compound, wide contact patch, and operating temperature.

This value is used in v01 and v02 as a constant for all four tyres.

**Tyre load sensitivity (v03+):**

In reality, μ is NOT constant. As vertical load increases, μ decreases. This is called "load sensitivity" and it's one of the most important concepts in vehicle dynamics.

Physical reason: a tyre's contact patch doesn't grow proportionally with load. At higher loads, the rubber in the centre of the contact patch is already saturated, so adding more load gives diminishing returns in grip.

We model this with a simple linear approximation:

    μ(Fz) = μ_0 - k × Fz

Where:
- μ_0 = 1.85: the extrapolated friction at zero load (theoretical, not physically real)
- k = 5.5 × 10⁻⁵ [1/N]: the rate at which friction drops with load

Example calculations:

At Fz = 4500 N (typical static corner weight):

    μ = 1.85 - 5.5e-5 × 4500 = 1.85 - 0.2475 = 1.60

This matches our mu_peak — we chose μ_0 and k so that mu_peak occurs at the static load. This is deliberate: the "peak" grip is what the tyre produces at its normal operating condition.

At Fz = 7000 N (high aero loading at 200+ km/h):

    μ = 1.85 - 5.5e-5 × 7000 = 1.85 - 0.385 = 1.47

The grip coefficient DROPPED from 1.60 to 1.47 — even though the total force increased (because Fz × 1.47 > 4500 × 1.60), the efficiency decreased. This is why simply adding downforce doesn't scale linearly with grip.

At Fz = 2500 N (inside wheel during cornering, unloaded):

    μ = 1.85 - 5.5e-5 × 2500 = 1.85 - 0.1375 = 1.71

The lightly loaded tyre has HIGHER friction coefficient. This is the fundamental reason why load transfer hurts total grip: the heavily loaded outside tyre loses more grip coefficient than the lightly loaded inside tyre gains.

### 2.6 Braking

**car.brakes.bias_f = 0.57 (57% front)**

In a QSS sim, braking is almost always tyre-limited, not hardware-limited. GT3 carbon brakes can produce far more torque than the tyres can handle. So maximum deceleration is set by tyre grip, not brake hardware.

Brake bias becomes important in v04 when we model per-axle loads: during braking, the front axle gets heavier (weight transfer forward), so it can handle more braking force. 57% front bias is a starting point that roughly matches the weight distribution under braking.

---

## 3. Telemetry Import — How Raw Data Becomes Usable

File: `02_data/telemetry/processed/import_reference_lap.m`

### 3.1 Source data

PI Toolbox exports iRacing telemetry as an Excel file with three sheets:
- Sheet 1: "Outing Information" — metadata (car name, driver, session ID)
- Sheet 2: "Channel Data" — the actual telemetry (49,134 rows × 9 columns at 100 Hz)
- Sheet 3: "Event Data" — empty

We read Sheet 2 ("Channel Data").

### 3.2 Time zeroing

PI Toolbox exports session time (time since the iRacing session started), not lap time. Our reference lap starts at session time 2152.31 s. We subtract the first timestamp so the lap starts at t = 0:

    t = t_raw - t_raw(1)

After this, t goes from 0 to ~491.33 s.

### 3.3 Speed unit conversion

PI Toolbox exported speed in km/h. All physics equations use SI units (m/s). Conversion:

    v [m/s] = v [km/h] / 3.6

Why 3.6? Because 1 km = 1000 m and 1 hour = 3600 s, so:

    1 km/h = 1000/3600 m/s = 1/3.6 m/s

### 3.4 Distance computation (trapezoidal integration)

The export had no distance channel, so we compute it from speed and time.

Distance = integral of speed over time. In discrete data, we approximate this using the trapezoidal rule.

**What is the trapezoidal rule?**

Imagine you have speed measurements at t=0, t=0.01, t=0.02, etc. Between any two consecutive measurements, the car's speed changes (it's accelerating or braking). How far did it travel in that 0.01 s interval?

Simplest method (rectangular rule): assume speed was constant during the interval.

    ds = v(i) × dt

This is inaccurate because speed was changing.

Better method (trapezoidal rule): assume speed changed linearly from v(i) to v(i+1). The average speed during the interval is:

    v_avg = 0.5 × (v(i) + v(i+1))
    ds = v_avg × dt

This is called "trapezoidal" because on a speed-vs-time plot, the area under the curve for each interval forms a trapezoid.

In MATLAB:

    dt = diff(t);                              % time step array
    v_avg = 0.5 * (v(1:end-1) + v(2:end));    % average speed in each interval
    ds = v_avg .* dt;                           % distance per interval
    dist = [0; cumsum(ds)];                     % cumulative distance

The result: dist(1) = 0 (start/finish line), dist(end) = 25,206 m (one full lap).

### 3.5 Why computed track length (25,206 m) differs from official (25,378 m)

The official track length (25.378 km) is measured along the geometric centerline of the track. But the car doesn't drive the centerline — it drives the racing line, which cuts corners (clips apexes). The racing line through a corner is shorter than the centerline arc. Over 170+ corners at the Nordschleife, these small shortcuts add up to about 172 m (0.7%). This is expected and correct.

### 3.6 The ref struct

All imported data is stored in a MATLAB struct called `ref`:

    ref.t         → time from lap start [s]
    ref.dist      → cumulative distance [m]
    ref.v         → speed [m/s]
    ref.v_kmh     → speed [km/h] (convenience copy)
    ref.throttle  → throttle position [%]
    ref.brake     → brake position [%]
    ref.gear      → gear number [-]
    ref.rpm       → engine RPM
    ref.g_lat     → lateral acceleration [g]
    ref.g_long    → longitudinal acceleration [g]
    ref.steer     → steering wheel angle [deg]

Plus metadata (ref.meta.source, ref.meta.lap_time, etc.).

Saved as both the original `.xls` (archival, human-readable) and `.mat` (fast MATLAB loading).

---

## 4. Track Data (Telemetry Source) — How We Extract Curvature from the Racing Line

Dispatcher: `02_data/track/build_track.m`
Implementation: `02_data/track/build_track_telemetry.m`

`build_track.m` is a one-screen dispatcher that reads the workspace variable `track_source` (defaulting to `'telemetry'`) and delegates to the matching builder. The telemetry path is described in this section; the alternative GPS path is §4A. Either path produces a schema-compatible `track` struct, so every solver downstream is agnostic to which source was used.

### 4.1 The core idea

A QSS lap simulator doesn't need a map of the track (GPS coordinates). It needs one thing: **curvature at every point along the track**. Curvature tells the solver "how tight is this corner?" and from that, the solver calculates the maximum cornering speed.

The telemetry source extracts curvature directly from the reference lap's lateral-acceleration and speed channels, using the fact that lateral acceleration and speed are related to corner radius by Newton's second law for circular motion. The centerline it produces is therefore the driver's **racing line**, not the geometric centerline — an important caveat for correlation.

### 4.2 The curvature equation

For a car driving in a circle of radius R at speed v:

    a_lat = v² / R        (centripetal acceleration equation)

Rearranging for curvature (κ = 1/R):

    κ = a_lat / v²

Where:
- κ (kappa) is curvature [1/m]. A straight has κ = 0. A tight hairpin has κ ≈ 0.05 [1/m] (radius 20 m).
- a_lat is lateral acceleration [m/s²] (we convert from g by multiplying by 9.81)
- v is speed [m/s]

Example: at 150 km/h (41.67 m/s) with 1.2 g lateral (11.77 m/s²):

    κ = 11.77 / 41.67² = 11.77 / 1736.1 = 0.00678 [1/m]
    R = 1/0.00678 = 147.5 m

That's a medium-speed corner with a ~148 m radius.

### 4.3 Speed clamping

At very low speeds (e.g. 5 m/s = 18 km/h), v² is only 25 m²/s². Even a tiny lateral acceleration of 0.1 m/s² would give:

    κ = 0.1 / 25 = 0.004 [1/m] → R = 250 m

That's nonsense — the car might just be drifting slightly in a pit lane. We clamp the minimum speed to 10 m/s (36 km/h) to avoid these false curvature spikes.

### 4.4 Why we smooth the curvature — a two-stage filter

**The problem.** Raw lateral acceleration from iRacing at 100 Hz contains noise of two different kinds. The first is sensor-level spikes from kerb strikes, bumps, and suspension transients — these are short-duration (1–3 m of track) and **not** steady-state cornering, so a QSS solver has no business seeing them. The second is low-amplitude sensor jitter spread across the entire signal. If we fed the raw curvature into the simulator it would predict unphysical speed oscillations and would over-estimate the tightness of every corner that contains a kerb.

**Why a single moving average is not enough.** The very first version of this script used a single 50 m moving average on the curvature. That choice had an expensive failure mode: the moving average, when asked to squash a 4 g kerb spike surrounded by a ~1.5 g real corner plateau, does so by spreading the spike's energy across 50 m of neighbouring samples. In doing so it also *rounds off* the real corner peak. On the N24 telemetry this reduced peak curvature preservation to 66% — the tightest corner (a ~14 m radius hairpin) came out as R ≈ 21 m, and v01 ran ~36 seconds faster than reference (Entry 008 in the logbook diagnoses this).

**The two-stage fix.** Instead of one aggressive filter, we use two targeted ones:

*Stage 1 — median filter on `a_lat`, 15 samples (~7.5 m).* A median filter replaces each sample with the median of its neighbours, which has the property that it kills isolated outliers but preserves step edges. That is exactly the right tool for kerb spikes: a 4 g spike is seen as an outlier and discarded, while a real step from 0 g to 1.5 g at corner entry is preserved because the neighbourhood majority is already on the new plateau. The window (~7.5 m) is wide enough to span a kerb strike but narrow enough to fit inside any real corner.

*Stage 2 — moving average on `κ`, 20 m.* Once the spikes are gone, a small moving average is enough to clean the residual jitter without rounding off the corner peaks.

With the two-stage filter, peak curvature preservation on the telemetry source comes out around 76%. That number looks low, but it has to be read in context: the racing line clips apexes and therefore over-reports peak curvature relative to the geometric centerline, so 76% of a racing-line peak is not the same as 76% of the truth. The GPS source described in §4A does not suffer from this ambiguity because it does not involve `a_lat` at all.

**Why moving average instead of a fancier filter?** Simplicity and transparency. A Butterworth or Savitzky-Golay filter would give slightly better frequency shaping, but the moving average and the median filter are completely transparent — you can explain exactly what they do in one sentence each. In correlation work, understanding your tool is more important than using the fanciest tool; anything you don't understand will eventually bite you on a setup change you can't interpret.

### 4.5 Resampling to uniform distance spacing

**The problem:** telemetry is sampled at uniform TIME (every 0.01 s). But the car's speed varies, so the distance between samples varies:
- At 270 km/h (75 m/s): samples are 0.75 m apart
- At 80 km/h (22.2 m/s): samples are 0.22 m apart

The simulator steps along the track by DISTANCE (meter by meter), so it needs data at uniform distance spacing.

**The solution:** interpolation. We create a new distance vector from 0 to 25,206 m in 1 m steps, then use MATLAB's `interp1` to compute the curvature (and all reference channels) at each of those evenly-spaced points.

    dist_uniform = (0 : 1.0 : ref.dist(end))';
    kappa_uniform = interp1(ref.dist, kappa_smooth, dist_uniform, 'linear');

`interp1` with 'linear' means: between any two known points, draw a straight line and read off the value at the desired location. This is called linear interpolation.

For gear number, we use 'nearest' instead of 'linear' because gear is an integer — interpolating between gear 3 and gear 4 to get gear 3.5 makes no sense.

### 4.6 Unsigned curvature

We take the absolute value: `kappa = abs(kappa_uniform)`.

The sign of curvature tells you whether the corner goes left or right. A QSS point-mass model doesn't care about direction — a 50 m right-hander requires the same speed as a 50 m left-hander. The car is a point with no left-right asymmetry.

Direction becomes relevant at v05 (bicycle model) where left and right tyres carry different loads. For now, we discard it.

### 4.7 The track struct

    track.dist    → distance along track [m], uniform 1 m spacing
    track.kappa   → unsigned curvature [1/m] at each distance point
    track.ds      → sample spacing (1.0 m)
    track.n       → number of points (~25,206 on the telemetry source)
    track.length  → total track length in metres
    track.ref.*   → reference telemetry resampled to the same grid (for correlation)
    track.meta.*  → name, source string, ref lap time, smoothing parameters, notes

### 4.8 Verification: what the plots should show

**Curvature vs distance:** spikes at every corner, near-zero on straights. The tallest spikes (~0.047 [1/m]) are the tightest corners (Karussell, Adenauer Forst hairpins).

**Corner radius vs distance:** the inverse of curvature. Shows ~14–21 m at the tightest points (depending on source and smoothing), 2000 m (capped) on straights.

**Speed vs curvature overlay:** this is the critical check. Every curvature peak MUST align with a speed valley. High curvature = tight corner = low speed. If you see high curvature where speed is also high, the data is corrupt. In our case they align correctly.

---

## 4A. Track Data (GPS Source) — Geometric Curvature from the Centerline

Implementation: `02_data/track/build_track_from_gps.m`
Input CSV: `02_data/track/pxt_centerline.csv` (extracted from `Nurburgring Combined Track.pxt`)

### 4A.1 Motivation

The telemetry source has two unavoidable limitations. It uses the driver's racing line rather than the geometric centerline, so the curvature it sees at each apex is biased by however aggressively the driver cut the corner. And it depends on a noisy `a_lat` signal, which forces smoothing that rounds off real corners. The GPS source was introduced to remove both of those limitations in one go — it uses no speed or g-sensor at all, and it uses the track's geometric centerline.

### 4A.2 The geometric curvature formula

Given a parametric curve `r(s) = (x(s), y(s))` arc-length-parameterised, the curvature is

    κ(s) = (x'(s) · y''(s) − y'(s) · x''(s)) / (x'(s)² + y'(s)²)^(3/2)

where primes are derivatives with respect to `s`. In practice we resample `x` and `y` to a uniform 1 m grid and take central differences; the denominator normalises out the fact that our numerical `s` is not perfectly arc-length even after resampling.

This is pure geometry. Nothing about the driver, the car, or the telemetry enters. If two different drivers lap the same track, the GPS-source curvature is identical for both; only the racing line they choose on top of it changes.

### 4A.3 Smoothing — light, and in the right place

The GPS centerline is already clean, so the filter requirements are very different from the telemetry path:

- *Pre-smooth the coordinates*: a 3 m moving average on `x` and `y` suppresses the small-scale quantisation noise that comes from interpolating the native ~3 m samples onto a 1 m grid. Three metres is well inside the tightest corner radius, so corner shapes are not touched.
- *Post-smooth the curvature*: a 5 m moving average on `κ` cleans the residual high-frequency noise from the double numerical differentiation. Anything larger than 5 m would start to flatten real corners.

With this light, two-sided smoothing the GPS source typically preserves >95% of peak curvature (the comparison plot saved at `02_data/track/pxt_curvature_comparison.png` shows this against the raw unsmoothed geometric κ).

### 4A.4 Bringing the reference telemetry onto the GPS grid

The reference lap is indexed by racing-line distance (~25,206 m) while the GPS centerline is ~25,176 m. The ~30 m difference is the racing line clipping apexes over 170+ corners — about 0.12% of total length. To put both on one distance axis for correlation we linearly rescale `ref.dist` to the GPS length. The rescale is within the sim's 1 m spatial resolution and preserves event ordering, which is all a QSS correlation requires.

### 4A.5 The track struct on the GPS source

The top-level fields (`dist`, `kappa`, `ds`, `n`, `length`, `ref.*`, `meta.*`) match the telemetry struct exactly, so every solver runs unchanged. The GPS path adds bonus fields that the solvers ignore but the plotting scripts can use:

    track.x, track.y, track.z   → centerline coordinates in metres (local projection; z is MSL elevation)
    track.lat, track.lon         → WGS84 coordinates
    track.kappa_signed           → signed curvature (positive = left-hand corner)

### 4A.6 How to switch between the two sources

In MATLAB:

    >> track_source = 'gps';   % or 'telemetry' (default)
    >> build_track

The dispatcher file is small and human-readable; anything unexpected is obvious on a single page. Running v02/v03/v04 on the GPS source quantifies the isolated contribution of curvature accuracy to the sim-vs-reference gap (see Entry 012 Next list in the logbook).

---

## 5. Data Flow Diagram

Here is how data flows through the project as of the latest revision:

    [iRacing .ibt file]                             [.pxt workbook]
         |                                               |
         v                                               v
    [PI Toolbox Pro] → exports to Excel (.xls)    [extract_pxt.py]
         |                                               |  - pulls GPSMapStream
         v                                               |  - writes pxt_centerline.csv
    [import_reference_lap.m]                             |
         |  - reads Excel Sheet 2 ("Channel Data")       |
         |  - converts units (km/h → m/s)                |
         |  - zeros time                                 |
         |  - computes distance via trapezoidal integration
         |  - saves 'ref' struct to reference_lap.mat    |
         |                                               |
         +-------------------+---------------------------+
                             |
                             v
                    [build_track.m]           ← dispatcher (track_source = 'telemetry' | 'gps')
                             |
              +--------------+--------------+
              |                             |
              v                             v
    [build_track_telemetry.m]      [build_track_from_gps.m]
      - κ = a_lat / v² on             - κ from (x, y) centerline
        racing-line data                (geometric, central-diff)
      - 15-sample median on a_lat     - 3 m pre-smooth on (x, y)
      - 20 m movmean on κ             - 5 m post-smooth on κ
      - saves n24_track.mat           - saves n24_track_gps.mat
              |                             |
              +--------------+--------------+
                             v
                    [amg_gt3_params.m]
                        - loads 'car' struct
                             |
                             v
    [03_models/v01_point_mass/lap_sim_v01.m]          (constant μ, no aero)
    [03_models/v02_aero/lap_sim_v02.m]                (+ speed-dependent downforce & drag)
    [03_models/v03_load_sens/lap_sim_v03.m]           (+ per-tyre μ(Fz))
    [03_models/v04_weight_transfer/lap_sim_v04.m]     (+ per-axle Fz, long. transfer, bias)
                             |
                             v
    [04_correlation/diagnose_grip.m]          → spot-check v03 grip arithmetic at a chosen v
    [04_correlation/diagnose_brake_v04.m]     → classify v04 brake peaks as SPIKE vs PLATEAU

---

## 6. Unit Conventions

ALL physics calculations use SI units:

| Quantity | Unit | Symbol |
|---|---|---|
| Distance | metres | m |
| Speed | metres per second | m/s |
| Time | seconds | s |
| Mass | kilograms | kg |
| Force | Newtons | N |
| Acceleration | m/s² (or g, where 1 g = 9.81 m/s²) | m/s² |
| Angle | radians (internal) or degrees (display) | rad, deg |
| Curvature | 1/metres | 1/m |
| Torque | Newton-metres | Nm |
| Power | Watts | W |
| Angular velocity | radians per second | rad/s |

Conversion factors used:
- km/h to m/s: divide by 3.6
- RPM to rad/s: multiply by 2π/60 = 0.10472
- g to m/s²: multiply by 9.81
- hp to kW: multiply by 0.7457

---

## 7. Key Equations Summary

### Circular motion (cornerning)
    a_lat = v² / R
    κ = 1/R = a_lat / v²
    v_max_corner = sqrt(μ × g × R) = sqrt(μ × g / κ)

### Aerodynamics
    F_drag = 0.5 × ρ × v² × A × Cd
    F_downforce = 0.5 × ρ × v² × A × Cl

### Engine force at wheels
    F_drive = (T_engine × gear_ratio × final_drive × efficiency) / rolling_radius

### Tyre grip (constant, v01-v02)
    F_grip_max = μ × F_normal

### Tyre grip (load sensitive, v03+)
    μ(Fz) = μ_0 - k × Fz
    F_grip_max = μ(Fz) × Fz

### Longitudinal weight transfer (v04+)
    ΔFz = m × a_long × h_cog / wheelbase        (sign: positive a_long means forward accel ⇒ transfer rearward)

### Per-axle vertical load (v04+)
    Fz_f(v, a_long) = weight_dist_f·m·g + aero_balance_f·F_downforce(v) − ΔFz(a_long)
    Fz_r(v, a_long) = (1 − weight_dist_f)·m·g + (1 − aero_balance_f)·F_downforce(v) + ΔFz(a_long)

### Per-axle grip (v04+)
    μ_axle(Fz_per_tyre) = μ_0 − k · Fz_per_tyre   (per-tyre formula; feed Fz_axle/2)
    F_grip_axle = 2 · μ_axle · (Fz_axle / 2)      (equivalently: μ_axle · Fz_axle)

### Friction circle (v04+)
    F_x_max_axle = sqrt(max(F_grip_axle² − F_y_axle², 0))

### Brake-bias constraint (v04+)
    a_brake = min( F_x_f_max / bias_f ,  F_x_r_max / (1 − bias_f) ) / m

### Trapezoidal integration (distance from speed)
    ds(i) = 0.5 × (v(i) + v(i+1)) × (t(i+1) - t(i))
    dist = cumulative sum of ds

### Lap time from speed profile
    dt(i) = ds / v(i)
    lap_time = sum of all dt(i)

---

## 8. v03 — Tyre Load Sensitivity

File: `03_models/v03_load_sens/lap_sim_v03.m`

### 8.1 What v03 adds to v02

v02 produced a speed-dependent grip budget: downforce increases `Fz`, and at constant μ the grip-force budget `F_grip = μ · Fz` grows proportionally with `Fz`. Real tyres do not behave that way. As `Fz` increases, the per-tyre friction coefficient `μ` drops, so `F_grip(Fz)` grows *sublinearly*. v03 makes the grip coefficient a function of load so the solver sees the real non-linearity.

### 8.2 The model

Linear load sensitivity, identical on all four tyres at this fidelity:

    μ(Fz_per_tyre) = μ_0 − k · Fz_per_tyre

with μ_0 = 1.85 and k = 5.5 × 10⁻⁵ [1/N]. The coefficients are chosen so that at the static per-tyre load (~3300 N) the grip coefficient evaluates to 1.60, matching the constant μ used in v01 and v02. At the heavier loaded outside tyre in a high-speed corner (Fz ≈ 7000 N) μ drops to about 1.47, and at a lightly loaded inside tyre (Fz ≈ 2500 N) μ rises to about 1.71. Section 2.5 walks through these numbers in more detail.

### 8.3 Per-tyre, not per-axle

This is the single most important implementation detail in v03 and was the seed of one of the v04 bugs diagnosed in Entry 011. The formula is calibrated for per-tyre load, i.e. `Fz_per_tyre = Fz_total / 4`. If per-axle load (`Fz_total / 2`) is passed into the same formula by mistake, μ drops twice as fast with load as it should, which is a ~20% error at high-aero speeds and enough to throw the whole lap profile off. v03 only sees a single total vertical load (no axle split yet), so this is safe here, but the same function is reused in v04 and had to be fed per-tyre loads there too.

### 8.4 Expected result

v03 was the first version that felt quantitatively sensible on N24: 7:46.382, approximately 1.0% faster than the reference lap. That small residual gap is consistent with a QSS optimum running slightly faster than a human-driven reference, and sets up v04 as a test of whether adding longitudinal weight transfer closes or widens the gap (it widens it, as expected, because weight transfer always *reduces* combined grip).

---

## 9. v04 — Longitudinal Weight Transfer and Per-Axle Grip

File: `03_models/v04_weight_transfer/lap_sim_v04.m`
Helper: the script's local `get_axle_grip_v04(v, dFz_long, car)` function (single source of truth for per-axle Fz/μ/F_grip).

### 9.1 What v04 adds to v03

v03 treated the whole car as a point mass: one total vertical load, one friction circle. v04 splits the vertical load between a front axle and a rear axle, recomputes μ per-axle using the load-sensitive formula, and lets longitudinal acceleration shift load between the two axles. This unlocks three physically important effects: the brake-bias constraint (rear can lock before front), the RWD traction limit under acceleration (only the rear axle drives), and the combined-grip penalty of weight transfer (loading one axle more than its optimum never recovers what unloading the other axle loses, thanks to tyre load sensitivity).

### 9.2 The per-axle vertical loads

At any speed `v` and longitudinal acceleration `a_long` (positive = accelerating):

    Fz_f = weight_dist_f · m · g   +   aero_balance_f · F_downforce(v)   −   ΔFz
    Fz_r = (1 − weight_dist_f) · m · g   +   (1 − aero_balance_f) · F_downforce(v)   +   ΔFz
    ΔFz = m · a_long · h_cog / wheelbase

The static split comes from `car.weight_dist_f = 0.46`; the aero split comes from `car.aero_balance_f = 0.43`. Those two numbers are separate parameters on purpose — the rear-biased downforce distribution is a setup choice independent of where the engine sits. Using the hard-coded 50/50 split in an earlier v04 draft was bug #6 in the list from Entry 011.

### 9.3 Per-axle grip and the friction circle

Per-tyre load is `Fz_axle / 2`; the load-sensitivity formula `μ(Fz_per_tyre) = μ_0 − k · Fz_per_tyre` is evaluated on each axle's per-tyre load; the axle's total lateral grip budget is `F_grip_axle = μ_axle · Fz_axle`. In any pass where both longitudinal and lateral forces are in play, the axle's available longitudinal force is what the friction circle leaves after the lateral demand has been spent:

    F_x_max_axle = sqrt(max(F_grip_axle² − F_y_axle², 0))

This is the classical friction-ellipse approximation treated as a circle; it is a standard QSS simplification and is correct provided the tyre's `F_x` and `F_y` saturation loads are similar, which they are for modern slicks.

### 9.4 Brake-bias constraint

Both axles brake simultaneously, and the driver cannot send more brake torque to one axle than its tyres can absorb without locking. With a fixed bias `bias_f`:

    F_brake_total ≤ F_x_f_max / bias_f      (front limit)
    F_brake_total ≤ F_x_r_max / (1 − bias_f) (rear limit)
    a_brake = min(F_brake_total) / m

The `min` is key. If the rear is unloaded (high speed, high forward weight transfer), `F_x_r_max` is small and the rear branch binds; the *front* is underused even though it has headroom, because the driver can't redirect rear-bound brake pressure to the front mid-corner. This is the mechanism by which bias sets peak braking — we see exactly this pattern at the end of Döttinger in Entry 014, where the rear binds at ≈2.6 g with the front still holding headroom.

### 9.5 RWD traction limit under acceleration

Only the rear axle drives. In the forward pass the longitudinal limit is whatever the rear friction circle permits after cornering:

    F_x_drive_max = sqrt(max(F_grip_r² − F_y_r², 0))
    a_drive_max   = F_x_drive_max / m

This is separate from the engine force; the solver takes `min(engine-force-at-this-gear, a_drive_max·m)` as the drive force actually delivered to the ground. At low speed traction usually binds; at high speed engine power binds.

### 9.6 Continuity iteration

v04 has an implicit coupling: `a_long` sets `ΔFz`, `ΔFz` sets per-axle grip, and per-axle grip sets the maximum achievable `a_long`. The script handles this with a small fixed-point iteration around the forward/backward passes, damped 50/50 and capped at ten iterations with a 0.01 m/s² tolerance. Convergence in one or two iterations is typical on the N24 trace.

### 9.7 Validation state

v04's validated result is 7:50.704, 20.6 s faster than the reference (−4.20%). That puts the charter target (±1%) out of reach until (a) the curvature source is improved — the GPS track lowers the optimism directly — and (b) setup parameters (`h_cog`, `brake_bias_f`) are calibrated against the reference lap. The weight-transfer cost v04 − v03 is +4.32 s, squarely inside the 3–8 s textbook expectation for a GT3 at this track, which is the strongest internal check that the new physics is behaving. Entry 014 in the logbook records the end-to-end verification, including the brake-peak investigation that resolved Entry 012's 3.65 g concern to a real-world 2.60 g.

### 9.8 What v04 is *not* yet

v04 is still a point-mass model in one respect: it has no lateral weight transfer between inside and outside tyres, because a point mass has no track width. That effect is the v05 stretch goal. Until then, v04's per-axle grip is the grip of an average tyre under the axle's average load — in a fast corner the outside tyre is overloaded relative to this average and the inside is underloaded, and those two errors do not exactly cancel because load sensitivity is non-linear.

---

## 10. Correlation and Diagnostics

Folder: `04_correlation/`

### 10.1 diagnose_grip.m

A small focused script that re-evaluates the v03 grip chain at a user-specified speed and reports every intermediate number (`Fz_total`, `Fz_per_tyre`, `μ`, `F_grip`, `a_lat_max`). It exists to catch bookkeeping errors in the grip arithmetic without having to run a whole lap — when v04 was first built, this script was used as the regression check at 200 km/h to verify that v04 collapses to v03 when `dFz_long = 0` (which it does, to rounding, per Entry 012).

### 10.2 diagnose_brake_v04.m

A classifier for v04's peak brake deceleration. Produces: a distribution summary of `a_brake` across the lap (percentiles, fraction above chosen thresholds), a connected-run analysis that separates isolated peaks from sustained plateaus, a per-axle breakdown at the top-N outliers (which axle bound, what `Fz`, `μ`, and `F_grip` were at that point), two scatter plots, and a `brake_diag` struct returned to the workspace for downstream use.

The script encodes a specific piece of race-engineering reasoning: if the peak is a single isolated point with random axle-binding, it is almost certainly an iteration artefact (numerical), while if it is a sustained run where the same axle keeps binding at similar `v` and `κ`, it is a physics artefact (usually a missing wheel-lift ceiling). The Entry 013 writeup explains the two-axis logic in detail; Entry 014 walks through the first real run against the v04 output and closes the peak-brake concern.

### 10.3 Correlation conventions

Correlation plots always use the same distance axis for simulated and reference speeds (the `track.dist` on whichever source built the track). Deltas are reported as Δt (cumulative) and Δv (point-by-point). Sector-by-sector breakdowns follow the N24 sector conventions used by the race teams rather than arbitrary length windows, so the reports read naturally against in-car radio calls.
