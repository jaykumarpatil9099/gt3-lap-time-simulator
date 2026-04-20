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

1. *