# Nürburgring 24h Lap Time Simulator — Mercedes-AMG GT3

A MATLAB-based quasi-steady-state (QSS) lap time simulator for the Mercedes-AMG GT3 around the Nürburgring 24h (VLN/NLS combined) layout, validated against iRacing telemetry processed in Cosworth PI Toolbox.

**Author:** Jaykumar Patil
**Status:** Phase 4 — v01…v04 built; calibration phase beginning.
**Latest simulated lap time:** 7:50.704 (v04, −20.637 s / −4.20% vs reference 8:11.341)
**Toolchain:** MATLAB R2024b · iRacing · Cosworth PI Toolbox Pro

## Project goals

The project is a portfolio piece aimed at a professional GT3 team internship focused on N24 race support. It has three layered goals:

1. Build a correlated lap time simulator from first principles in MATLAB, hitting the charter target of ±1% simulated-vs-reference lap time on the reference lap.
2. Internalise vehicle dynamics, MATLAB engineering workflow, and the sim-to-real correlation process as they are actually practised by race engineers.
3. Produce an artefact — code, logbook, design notes, reports — that a race engineer can open cold and understand in five minutes.

See `00_admin/00_project_charter.md` for full scope and success criteria, and `00_admin/02_design_note_001_fidelity.md` for the architectural decisions (QSS vs transient, fidelity ladder v01→v05, per-version validation gates).

## What the simulator does

Given a car parameter set and a track centerline, the simulator computes the fastest physically feasible lap the car can drive. It does this in three passes over the track, discretised to 1 m steps:

1. **Cornering pass** — per-point speed cap from the lateral-grip envelope (`v_corner = √(a_lat_max / κ)`), including speed-dependent downforce, load-sensitive μ, and (v04) per-axle grip with longitudinal weight transfer set to zero.
2. **Forward pass** — integrate forward with engine torque, RWD traction limit, and the longitudinal friction-circle budget left over from the cornering grip used at each point.
3. **Backward pass** — integrate backwards under the brake-bias constraint `a = min(F_x_f_max / bias_f, F_x_r_max / (1 − bias_f)) / m`, respecting the friction circle per axle.

A small continuity iteration between passes converges the per-point longitudinal weight transfer, because `a_long` and `Fz` are mutually dependent in v04.

## Fidelity ladder

Each model version lives in its own subfolder under `03_models/` with its own solver but a shared parameter file. The ladder is deliberately incremental so every new physical effect is attributable to one commit.

| Version | Physics added | Reference target | Current result |
|---|---|---|---|
| v01 point mass | Mass, single μ, constant-radius cornering | End-to-end run | built |
| v02 aero | Speed-dependent downforce and drag | Within 5% of reference | built |
| v03 load sensitivity | μ(Fz) per-tyre grip coefficient | Within 2% | built, 7:46.382 |
| v04 longitudinal weight transfer | Per-axle Fz, friction circle per axle, brake bias | Within 1% (charter) | built, 7:50.704 |
| v05 (stretch) | Bicycle + lateral load transfer | Within 1% with better balance plausibility | not started |

## Repository layout

| Folder | Purpose |
|---|---|
| `00_admin/` | Project charter, engineering logbook, design notes |
| `01_references/` | Technical reference document, papers, datasheets |
| `02_data/` | Inputs: car parameters (`02_data/car/`), track geometry (`02_data/track/`), reference telemetry (`02_data/telemetry/`) |
| `03_models/` | Versioned MATLAB solvers — one subfolder per fidelity rung |
| `04_correlation/` | Diagnostic scripts (`diagnose_grip.m`, `diagnose_brake_v04.m`) and sim-vs-reference comparison tools |
| `05_studies/` | Engineering studies (sensitivity sweeps, setup experiments) |
| `06_reports/` | One-page study reports |
| `07_portfolio/` | Polished write-ups for the portfolio submission |

## Track data — two sources

The simulator can be driven from either of two centerline sources. The entry point `02_data/track/build_track.m` is a dispatcher; set the workspace variable `track_source` before calling it.

**`track_source = 'telemetry'`** (default) — `build_track_telemetry.m`. Curvature is derived from the reference lap as `κ = a_lat / v²`, so the centerline is the driver's racing line, not the geometric centerline. A two-stage filter (15-sample median on `a_lat`, then 20 m moving average on `κ`) denoises sensor artefacts while preserving corner shape. The racing line clips apexes, so this source *over-reports* peak curvature relative to the geometric centerline.

**`track_source = 'gps'`** — `build_track_from_gps.m`. Curvature is computed geometrically from the (x, y) centerline extracted from a PI Toolbox `.pxt` workbook, using central differences on a 1 m grid: `κ = (x'y'' − y'x'') / (x'² + y'²)^(3/2)`. Coordinates are pre-smoothed with a 3 m moving average; the resulting κ is post-smoothed with a 5 m moving average. No speed signal, no g-sensor noise. This is the source the final correlation campaign will use.

The two sources produce schema-compatible `track` structs (`dist`, `kappa`, `ds`, `n`, `length`, `ref.*`, `meta.*`); the GPS path adds bonus fields (`x`, `y`, `z`, `lat`, `lon`, `kappa_signed`).

## Typical run sequence

```matlab
>> startup_project                      % put repo folders on the MATLAB path
>> import_reference_lap                 % load reference telemetry into 'ref'
>> build_track                          % dispatcher; telemetry by default
>> lap_sim_v01; lap_sim_v02; lap_sim_v03; lap_sim_v04
>> diagnose_grip                        % spot-check v03 grip arithmetic
>> diagnose_brake_v04                   % classify brake-spike behaviour
```

To run on the GPS centerline instead:

```matlab
>> track_source = 'gps';
>> build_track
>> lap_sim_v01; ... lap_sim_v04
```

## Working agreement

The project runs in phases; there is no skipping ahead. Every working session produces an append-only entry in `00_admin/01_logbook.md` with four fixed blocks (Done, Found, Think, Next); the Next block becomes the brief for the following session. Raw telemetry is never modified; each solver version is frozen before the next one starts. Commit messages follow the `<type>: <summary> (<quantified result>)` convention — see the top of the logbook for the type list. The logbook is the single source of truth for bugs, diagnosis, design, and decisions; GitHub issues are not used.
