# Nürburgring 24h Lap Time Simulator — Mercedes-AMG GT3

A MATLAB-based quasi-steady-state (QSS) lap time simulator for the Mercedes-AMG GT3 around the Nürburgring 24h (VLN/NLS combined) layout, calibrated against iRacing telemetry processed in Cosworth PI Toolbox.

**Author:** Jaykumar Patil
**Status:** Phase 5 complete — charter PASS at −0.16 % vs reference.
**Calibrated lap time:** **8:10.539** (v05, Δ −0.80 s vs reference 8:11.341)
**Toolchain:** MATLAB R2024b · iRacing · Cosworth PI Toolbox Pro

> **Read this first:** [`06_reports/n24_portfolio_summary.md`](06_reports/n24_portfolio_summary.md) — five-page executive summary of the whole project. The same content as a polished Word doc lives at [`06_reports/n24_portfolio_summary.docx`](06_reports/n24_portfolio_summary.docx).

---

## Project goals

The project is a portfolio piece aimed at a professional GT3 team internship focused on N24 race support. It has three layered goals:

1. Build a correlated lap time simulator from first principles in MATLAB, hitting the charter target of ±1 % simulated-vs-reference lap time on the reference lap.
2. Internalise vehicle dynamics, MATLAB engineering workflow, and the sim-to-real correlation process as they are actually practised by race engineers.
3. Produce an artefact — code, logbook, design notes, reports — that a race engineer can open cold and understand in five minutes.

See `00_admin/00_project_charter.md` for full scope and success criteria, and `00_admin/02_design_note_001_fidelity.md` for the architectural decisions (QSS vs transient, fidelity ladder v01 → v05, per-version validation gates).

## What the simulator does

Given a car parameter set and a track centerline, the simulator computes the fastest physically feasible lap the car can drive. It does this in three passes over the track, discretised to 1 m steps:

1. **Cornering pass** — per-point speed cap from the lateral-grip envelope (`v_corner = √(a_lat_max / κ)`), including speed-dependent downforce, load-sensitive μ, per-axle longitudinal load (v04+), and per-tyre lateral load with ARB redistribution (v05).
2. **Forward pass** — integrate forward with engine torque, RWD traction limit, and the longitudinal friction-circle budget left over from the cornering grip used at each point.
3. **Backward pass** — integrate backwards under the brake-bias constraint `a = min(F_x_f_max / bias_f, F_x_r_max / (1 − bias_f)) / m`, respecting the friction circle per axle.

A small continuity iteration between passes converges the per-point longitudinal weight transfer, because `a_long` and `Fz` are mutually dependent from v04 onwards.

## Fidelity ladder

Each model version lives in its own subfolder under `03_models/` with its own solver but a shared parameter file. The ladder is deliberately incremental so every new physical effect is attributable to one commit.

| Version | Physics added | Reference target | Result |
|---|---|---|---|
| v01 point mass | Mass, single μ, gear-limited engine | End-to-end run | 8:24.738 |
| v02 aero | Speed-dependent downforce and drag | Within 5 % | 7:47.579 |
| v03 load sensitivity | μ(Fz) per-tyre grip coefficient | Within 2 % | 7:46.382 |
| v04 longitudinal weight transfer | Per-axle Fz, friction circle per axle, brake-bias min | Within 1 % | 7:50.704 |
| v05 lateral weight transfer | Per-tyre Fz with ARB roll-stiffness redistribution; per-tyre μ(Fz) | Within 1 % (charter) | 8:02.424 (uncalibrated) |
| **v05 calibrated** | Tyre `μ_0` and `load_sens_k` calibrated against ref lap | **±1 % charter** | **8:10.539, −0.16 %** ✓ |

## Phase 5 — calibration and analysis

Charter passed in Phase 5 via four structured studies in `05_studies/`:

| Step | Script | Output | Headline finding |
|---|---|---|---|
| 1 | `phase5_step1_gps_vs_telemetry.m` | Lap times on both track sources | Telemetry for calibration, GPS for sensitivity (peak κ preservation 76 % vs 94 %) |
| 2 | `phase5_step2_sector_analysis.m` | Sectorised Δt + Δv tables | Built `correlate_sim.m`; residual concentrated in high-speed sectors |
| 3 | `phase5_step3_sensitivity.m` | 9-parameter sensitivity tornado | `mu_0` and `load_sens_k` dominate (≈ 20 s and 16 s leverage) |
| 4 | `phase5_step4_calibration.m` | 5×5 sweep on (`mu_0`, `load_sens_k`) → calibrated tyre params | Charter PASS at −0.16 %, sector RMS 1.49 s |
| 5 | `phase5_step5_setup_study.m` | 5×5 setup heatmap (`aero_balance_f` × `roll_dist_f`) | Residual signature is **physics-bound**, not setup-bound |

The detailed write-up lives in `06_reports/n24_portfolio_summary.md` and the corresponding `.docx`.

## Repository layout

| Folder | Purpose |
|---|---|
| `00_admin/` | Project charter, append-only logbook, design notes |
| `01_references/` | Technical reference document (every equation explained) |
| `02_data/` | Inputs: car parameters (`02_data/car/`), track geometry (`02_data/track/`), reference telemetry (`02_data/telemetry/`) |
| `03_models/` | Versioned MATLAB solvers — one subfolder per fidelity rung |
| `04_correlation/` | `correlate_sim.m` (sectorised reporting) and diagnostics (`diagnose_grip.m`, `diagnose_brake_v04.m`) |
| `05_studies/` | Phase-5 studies: GPS experiment, sector analysis, sensitivity, calibration, setup study |
| `06_reports/` | Portfolio summary (`.md` + `.docx` + 4 figures) and the figure-export script |

## Track data — two sources

The simulator can be driven from either of two centerline sources. The entry point `02_data/track/build_track.m` is a dispatcher; set the workspace variable `track_source` before calling it.

**`track_source = 'telemetry'`** (default) — `build_track_telemetry.m`. Curvature is derived from the reference lap as `κ = a_lat / v²`, so the centerline is the driver's racing line. A two-stage filter (15-sample median on `a_lat`, then 20 m moving average on `κ`) denoises sensor artefacts while preserving corner shape. Peak-κ preservation: ~76 %. Used for **calibration** (matches the actual driver's lap).

**`track_source = 'gps'`** — `build_track_from_gps.m`. Curvature is computed geometrically from the (x, y) centerline extracted from a PI Toolbox `.pxt` workbook, using central differences on a 1 m grid: `κ = (x'y'' − y'x'') / (x'² + y'²)^(3/2)`. Peak-κ preservation: ~94 %. Used for **sensitivity** and **setup studies** (no driver-line bias).

The two sources produce schema-compatible `track` structs (`dist`, `kappa`, `ds`, `n`, `length`, `ref.*`, `meta.*`); the GPS path adds bonus fields (`x`, `y`, `z`, `lat`, `lon`, `kappa_signed`).

## Typical run sequence

```matlab
>> startup_project                       % put repo folders on the MATLAB path
>> import_reference_lap                  % load reference telemetry into 'ref'
>> build_track                           % dispatcher; telemetry by default
>> lap_sim_v01; lap_sim_v02; lap_sim_v03; lap_sim_v04; lap_sim_v05
>> diagnose_grip                         % spot-check v03 grip arithmetic
>> diagnose_brake_v04                    % classify brake-spike behaviour
```

To run on the GPS centerline instead:

```matlab
>> track_source = 'gps';
>> build_track
>> lap_sim_v01; ... lap_sim_v05
```

To reproduce Phase 5 end-to-end:

```matlab
>> run('05_studies/phase5_step1_gps_vs_telemetry.m')
>> run('05_studies/phase5_step2_sector_analysis.m')   % needs Step 1 result loaded
>> track_source = 'gps'; build_track
>> run('05_studies/phase5_step3_sensitivity.m')
>> track_source = 'telemetry'; build_track
>> run('05_studies/phase5_step4_calibration.m')
>> run('05_studies/phase5_step5_setup_study.m')
>> run('06_reports/export_figures.m')                 % regenerate figures
```

## Working agreement

The project runs in phases; there is no skipping ahead. Every working session produces an append-only entry in `00_admin/01_logbook.md` with four fixed blocks (Done, Found, Think, Next); the Next block becomes the brief for the following session. Raw telemetry is never modified; each solver version is frozen before the next one starts. Commit messages follow the `<type>: <summary> (<quantified result>)` convention — see the top of the logbook for the type list. The logbook is the single source of truth for bugs, diagnosis, design, and decisions; GitHub issues are not used.

## Future work (out of this project's scope)

| Direction | Why | Effort |
|---|---|---|
| Multi-lap IBT loader (replaces single-lap telemetry) | Removes biggest credibility gap; uses native iRacing IBT files via PI Toolbox or MoTeC i2 | ~1 day |
| v06 Pacejka magic-formula tyre + temperature/wear coupling | Resolves the high-speed-fast / technical-slow signature directly; turns the project into a lap-degradation simulator | 2–3 weeks |
| Transient suspension multibody add-on | Setup studies at finer resolution than ARB balance | larger |
| Differential modelling (open / Salisbury / preload) | On-throttle corner-exit fidelity | medium |
| Pipeline extension to a second car or track | Proves transferability for the portfolio | 1–2 days |

These are documented in the portfolio summary's "Known limits and future work" section and in the latest logbook entry.
