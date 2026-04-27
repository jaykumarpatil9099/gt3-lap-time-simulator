# Project Charter — Nürburgring 24h Lap Time Simulator

**Owner:** Jaykumar Patil
**Started:** 2026-04-15
**Version:** 1.0 (closed — see §9 Outcome)

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

---

## 9. Outcome — 2026-04-21 (charter CLOSED)

**Charter status: PASS.**

| Success criterion | Result |
|---|---|
| MATLAB model runs end-to-end on the N24 layout | Pass — solvers v01..v05 all run from `startup_project` → `import_reference_lap` → `build_track` → `lap_sim_v0X` |
| Lap time within ±1.0 % of reference | **Pass — calibrated v05 = 8:10.539, Δ = −0.80 s (−0.16 %) vs reference 8:11.341** |
| Channel-by-channel correlation shows sensible agreement | Pass — sector-level Δt and Δv tables produced by `04_correlation/correlate_sim.m`; no gross structural errors |
| At least three engineering studies, each documented | Pass — Phase 5 Steps 1–5 are five studies in `05_studies/`, results saved as `.mat` and summarised in logbook entries 017, 018 |
| Portfolio write-up exists | Pass — `06_reports/n24_portfolio_summary.md` (markdown) + `n24_portfolio_summary.docx` (polished, 4 figures embedded). The `07_portfolio/` folder named in §6.5 was consolidated into `06_reports/` during Phase 5; the success criterion is satisfied by `06_reports/n24_portfolio_summary.*` |

### What changed from the original plan

- v05 was originally listed as a *stretch* goal. It was built and calibrated; it is the version against which the charter is evaluated. v04's pre-v05 charter pass would have required calibration on parameters with combined leverage of ~2.4 s, which Step 3 sensitivity analysis showed to be physically dishonest — see logbook Entry 017.
- Reports landed in `06_reports/` instead of a separate `07_portfolio/`. Single folder for all narrative output, easier to navigate.
- Two track sources (telemetry-derived racing line, GPS-derived geometric centreline) were built and used for different questions — telemetry for calibration, GPS for sensitivity / setup study. This was a Phase 5 decision, not foreseen in the charter.

### What is documented as future work, not charter scope

Pacejka tyre, transient suspension, differential, multi-lap IBT loader, second car/track. All listed in the portfolio summary §6 and the README "Future work" table. Not required for charter pass; queued for v06+.

**Charter version:** 1.0 (closed)
