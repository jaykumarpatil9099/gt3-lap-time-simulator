# Design Note 001 — Simulator Fidelity & Roadmap

**Date:** 2026-04-16
**Phase:** 1 (Requirements & fidelity decisions)
**Status:** Accepted
**Supersedes:** —

---

## Context

We must decide the architecture of the lap time simulator for the AMG GT3 at the Nürburgring 24h layout. Architecture on two axes:

1. Time treatment — quasi-steady-state (QSS) vs. transient.
2. Detail level — fidelity ladder from point-mass to bicycle model.

The decision drives everything downstream: data required, MATLAB structure, correlation expectations, study scope.

## Decision

**Time treatment: QSS.** Track discretized into segments; at each segment we solve for the maximum speed consistent with the car's current grip envelope. No transient integration.

**Fidelity: incremental climb through the detail ladder, one rung per model version, with validation gates between versions.**

| Model | Rung | Physics added | Validation target |
|---|---|---|---|
| v01 | Point-mass, fixed friction | Mass, single grip coefficient, constant-radius cornering limit | Runs end-to-end, produces a lap time |
| v02 | + Aero | Speed-dependent downforce and drag (g-g-*v* surface) | Within 5% of reference lap |
| v03 | + Tyre load sensitivity | Grip coefficient decreases with vertical load | Within 2% of reference lap |
| v04 | + Longitudinal weight transfer | Front/rear *F_z* redistribution during braking & traction | Within 1% — charter target |
| v05 (stretch) | Bicycle + lateral load transfer | Per-axle + lateral load transfer between inside/outside tyres | Within 1% with improved balance plausibility |

## Rationale

**QSS over transient:** QSS is the industry default for concept-level lap simulation and answers the strategic questions (wing level, gear ratios, weight distribution, brake bias) with a small fraction of the complexity and data requirements of a transient model. It also matches available tooling (no suspension kinematics or damper data modelled). Transient modelling is out of scope per the charter.

**Incremental fidelity over direct-to-v04:** Each rung introduces one named physical effect. By comparing lap time v(n) vs. v(n-1) we quantify how much each effect is worth at N24. This turns the project into a teaching instrument about vehicle dynamics at this specific track, which is a core learning goal. It also keeps debugging tractable — when v03 misbehaves, we compare to a trusted v02 and the problem is narrowed to load sensitivity specifically.

**Validation gates:** Without an exit criterion per version, we would endlessly tinker. The gates force us to move on when "good enough" is reached.

## Alternatives considered

- **Transient bicycle model from day one.** Rejected — disproportionate complexity for our question, requires suspension data we do not have, and out-of-scope per charter.
- **Point-mass only (stop at v01).** Rejected — cannot correlate at Nordschleife where aero dominates above 250 km/h. Would not reach charter's ±1% target.
- **Jump straight to v04.** Rejected — obscures the learning value of understanding each physical effect in isolation and makes debugging harder.

## Consequences

- **Data to collect:** mass, wheelbase, CoG height (longitudinal), frontal area, drag & lift coefficients (or a downforce/drag map), peak tyre grip coefficient, a simple load-sensitivity curve, engine torque curve, gear ratios, final drive, brake system torque capacity. Detailed suspension, damper, and tyre thermal data are NOT required.
- **MATLAB structure:** each model version lives in its own subfolder under `03_models/`; parameters are loaded from a single shared `02_data/car/` parameter file so only the solver changes between versions.
- **Correlation scope:** after each version we generate delta-t and delta-v plots vs. the reference iRacing lap and log the result in the engineering logbook.
