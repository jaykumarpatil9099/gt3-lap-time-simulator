# Technical Reference — N24 Lap Time Simulator

**Author:** Jaykumar Patil
**Created:** 2026-04-16
**Purpose:** Documents every calculation, equation, assumption, and design decision made in this project. This is the document you open when you ask "why did we do it this way?" or "what does this equation mean?"

---

## Table of Contents

1. Project architecture overview
2. Vehicle parameter file — every parameter explained
3. Telemetry import — how raw data becomes usable
4. Track data — how we extract the track from telemetry
5. Data flow diagram
6. Unit conventions
7. Key equations summary

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

## 4. Track Data — How We Extract the Track from Telemetry

File: `02_data/track/build_track.m`

### 4.1 The core idea

A QSS lap simulator doesn't need a map of the track (GPS coordinates). It needs one thing: **curvature at every point along the track**. Curvature tells the solver "how tight is this corner?" and from that, the solver calculates the maximum cornering speed.

We extract curvature directly from the telemetry, using the fact that lateral acceleration and speed are related to corner radius by Newton's second law for circular motion.

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

### 4.4 Why we smooth the curvature (and how)

**The problem:** raw lateral acceleration from iRacing at 100 Hz contains noise — kerb strikes, bumps, minor steering corrections, sensor jitter. This noise creates tiny rapid fluctuations in the computed curvature. If we feed this directly to the simulator, it would predict rapid speed oscillations that don't exist in reality.

**The solution:** smooth the curvature with a moving average filter.

**What is a moving average?** At each point, replace the value with the average of all values within a window centered on that point. With a 97-sample window:

    kappa_smooth(i) = mean(kappa_raw(i-48 : i+48))

**Why 50 m window?** This represents a physical length of track. A 50 m window:
- Is long enough to average out noise from kerbs and bumps (which happen over 1-5 m)
- Is short enough to preserve the shape of real corners (even a tight hairpin spans 30-50 m of track)

If the window were too small (5 m): noise passes through, sim speed oscillates.
If the window were too large (200 m): real corners get smeared out, sim predicts too-high speed in corners.

50 m is a starting point — we can tune this during correlation.

**Why moving average instead of a fancier filter?** Simplicity and transparency. A Butterworth or Savitzky-Golay filter would give slightly better performance, but the moving average is completely transparent — you can explain exactly what it does in one sentence. In engineering, understanding your tool is more important than using the fanciest tool.

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

    track.dist   → distance along track [m], uniform 1 m spacing
    track.kappa  → unsigned curvature [1/m] at each distance point
    track.ds     → sample spacing (1.0 m)
    track.n      → number of points (25,206)
    track.length → total track length (25,206 m)
    track.ref.*  → reference telemetry resampled to the same grid (for correlation)

### 4.8 Verification: what the plots should show

**Curvature vs distance:** spikes at every corner, near-zero on straights. The tallest spikes (~0.047 [1/m]) are the tightest corners (Karussell, Adenauer Forst hairpins).

**Corner radius vs distance:** the inverse of curvature. Shows ~21 m at the tightest points, 2000 m (capped) on straights.

**Speed vs curvature overlay:** this is the critical check. Every curvature peak MUST align with a speed valley. High curvature = tight corner = low speed. If you see high curvature where speed is also high, the data is corrupt. In our case, they align correctly.

---

## 5. Data Flow Diagram

Here is how data flows through the project:

    [iRacing .ibt file]
         |
         v
    [PI Toolbox Pro] → exports to Excel (.xls)
         |
         v
    [import_reference_lap.m]
         |  - reads Excel Sheet 2 ("Channel Data")
         |  - converts units (km/h → m/s)
         |  - zeros time
         |  - computes distance via trapezoidal integration
         |  - saves 'ref' struct to reference_lap.mat
         |
         v
    [build_track.m]
         |  - reads 'ref' struct from workspace
         |  - computes curvature: κ = a_lat / v²
         |  - smooths with 50 m moving average
         |  - resamples to 1 m uniform distance spacing
         |  - saves 'track' struct to n24_track.mat
         |
         v
    [amg_gt3_params.m]
         |  - loads 'car' struct with all vehicle parameters
         |
         v
    [lap_sim_v01.m] ← THIS IS WHAT WE BUILD NEXT
         |  - inputs: 'car' struct + 'track' struct
         |  - output: simulated speed profile + lap time
         |
         v
    [correlation scripts]
         |  - compares sim output vs track.ref.* (reference data)
         |  - produces delta plots

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
    ΔFz = m × a_long × h_cog / wheelbase

### Trapezoidal integration (distance from speed)
    ds(i) = 0.5 × (v(i) + v(i+1)) × (t(i+1) - t(i))
    dist = cumulative sum of ds

### Lap time from speed profile
    dt(i) = ds / v(i)
    lap_time = sum of all dt(i)
