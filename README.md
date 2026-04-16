# Nürburgring 24h Lap Time Simulator — Mercedes-AMG GT3

A MATLAB-based quasi-steady-state lap time simulator for the Mercedes-AMG GT3 around the Nürburgring 24h (VLN/NLS combined) layout, validated against iRacing telemetry using Cosworth PI Toolbox.

**Author:** Jaykumar Patil
**Status:** In development (Phase 0 — project setup)
**Toolchain:** MATLAB R2024b · iRacing · PI Toolbox Pro

## Project goals

1. Build a validated lap time simulator from first principles in MATLAB.
2. Learn vehicle dynamics, MATLAB programming, and the professional sim-to-real correlation workflow.
3. Produce a portfolio-grade artifact suitable for GT3 race team internship applications.

See `00_admin/00_project_charter.md` for full scope and success criteria.

## Repository layout

| Folder | Purpose |
|---|---|
| `00_admin/` | Project charter, engineering logbook |
| `01_references/` | Papers, datasheets, regulations |
| `02_data/` | Inputs: car parameters, track geometry, telemetry |
| `03_models/` | Versioned MATLAB models (v01_point_mass, v02_..., …) |
| `04_correlation/` | Sim vs. reference comparison scripts and plots |
| `05_studies/` | Engineering studies (sweeps, sensitivity analyses) |
| `06_reports/` | One-page study reports |
| `07_portfolio/` | Final write-ups for portfolio use |

## Working agreement

- Phase-by-phase. No skipping ahead.
- Logbook entry per session (`00_admin/01_logbook.md`).
- Raw data is never modified. Versioned models are frozen before the next version starts.
- Commit messages follow the `<type>: <summary>` convention (see logbook for types).
