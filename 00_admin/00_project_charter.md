# Project Charter — Nürburgring 24h Lap Time Simulator

**Owner:** Jaykumar Patil
**Started:** 2026-04-15
**Version:** 0.1 (living document — update as scope evolves)

---

## 1. Objective

Build a validated lap time simulator (LTS) in MATLAB for the **Mercedes-AMG GT3** around the **Nürburgring 24h (VLN/NLS) combined layout**, correlated against iRacing telemetry, to a standard suitable for use as an internship portfolio piece with professional GT3 teams competing at the N24.

## 2. Learning goals

- MATLAB programming (scripting, functions, structures, plotting, Simulink where appropriate)
- Vehicle dynamics fundamentals: tyre modelling, weight transfer, aerodynamic load, powertrain, braking
- The correlation workflow (sim vs. real) as practiced by professional race teams
- Engineering documentation and reproducibility discipline

## 3. Scope (what IS in the project)

- Vehicle: Mercedes-AMG GT3 (Evo spec, 2020+ where data permits)
- Track: Nürburgring 24h layout (Nordschleife + GP combined), single configuration
- Reference lap: Jaykumar's own iRacing telemetry, 8:11 baseline
- Model fidelity path: point-mass QSS → + aero → + tyre load sensitivity → + longitudinal weight transfer → (stretch) bicycle model with lateral load transfer
- Deliverables: validated MATLAB model, correlation report, 3+ engineering studies, portfolio write-up

## 4. Out of scope (what is NOT in the project)

- Tyre thermal/wear modelling (too data-hungry without rig data)
- Driver-in-the-loop simulation
- Full multibody (Adams/VI-CarRealTime) — out of tooling
- Wet conditions, night conditions, traffic
- Multi-class strategy simulation

## 5. Tooling

| Purpose | Tool |
|---|---|
| Modelling & simulation | MATLAB R2024b (+ Simulink, Optimization Toolbox, Vehicle Dynamics Blockset) |
| Reference telemetry source | iRacing |
| Telemetry processing & correlation | Cosworth PI Toolbox Pro |
| Documentation | Markdown (engineering logbook, charter, reports) |
| Version control | Git (local repo, optional GitHub mirror for portfolio) |

## 6. Success criteria

The project is considered complete when ALL of the following are true:

1. MATLAB model runs end-to-end on the N24 layout and produces a lap time.
2. Simulated lap time matches the iRacing reference within **±1.0%** (≈ ±5s on an 8:11 lap).
3. Channel-by-channel correlation (speed, longitudinal g, lateral g) shows sensible agreement with no gross structural errors (validated visually in PI Toolbox overlay).
4. At least three engineering studies are completed, each documented as a one-page report in `06_reports/`.
5. Portfolio write-up exists in `07_portfolio/` with correlation plots and a clear narrative.

## 7. Milestones

| # | Milestone | Exit criteria |
|---|---|---|
| M0 | Project setup | Folder structure, charter, logbook, git repo in place |
| M1 | Data acquisition | Car parameter set + processed track file + reference telemetry in `02_data/` |
| M2 | v01 point-mass model | Runs, produces a lap time (accuracy not yet required) |
| M3 | First correlation | Delta-t and delta-v plots generated vs. reference lap |
| M4 | v02+ models | Aero, tyre, weight transfer layers added and validated |
| M5 | Target correlation met | ±1% lap time, sensible channel agreement |
| M6 | Engineering studies | 3+ studies complete with reports |
| M7 | Portfolio | Final write-up published |

## 8. Working agreement

- We work phase-by-phase. No jumping ahead.
- Every significant decision or insight is recorded in the engineering logbook (`00_admin/01_logbook.md`).
- Every model version is frozen before the next one starts.
- Raw data is never modified — only processed copies are.
- "It works on my machine" is not good enough; scripts must run from a clean state.
