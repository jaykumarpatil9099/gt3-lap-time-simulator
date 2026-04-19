# Engineering Logbook — N24 Lap Time Simulator

**Owner:** Jaykumar Patil
**Rule:** Append-only. One entry per working session. Never delete entries — if something was wrong, add a correction entry referring back to it.

**Entry format:**

- **Done:** what I actually did this session
- **Found:** observations, numbers, screenshots, surprises
- **Think:** my interpretation — what it means, why it matters
- **Next:** what comes in the next session

---

## Entry 012 — 2026-04-19 — v04 rewrite validated: 7:50.704, weight-transfer cost +4.3 s

**Phase:** 4 (Model build — v04 verified)

**Done:**
- Rewrote `lap_sim_v04.m` as "v03 grip model + per-axle loads + longitudinal transfer". Buggy version preserved as `lap_sim_v04_buggy_2026-04-18.m` for A/B reference.
- Added single-source-of-truth helper `get_axle_grip_v04(v, dFz_long, car)` returning per-axle `Fz`, `μ`, `F_grip`. All three passes call it — no duplicated load equations that can drift out of sync.
- Implemented per-axle friction circle in forward (RWD: rear only, `F_x_r_max = √(F_grip_r² − F_y_r²)`) and backward (both axles under brake-bias constraint `a = min(F_x_f_max/bias, F_x_r_max/(1−bias))/m`) passes.
- Ran v04 after `startup_project; import_reference_lap; build_track; lap_sim_v03`. Continuity iteration converged in 1 iter.

**Found:**
- **v04 lap time: 7:50.704** (Δref = −20.637 s, −4.20%). v03 was 7:46.382. **Weight-transfer cost: +4.32 s** (positive = v04 slower = transfer reduces combined grip, as expected).
- **Regression check passes.** Grip diagnostic at 200 km/h: v04 reports `a_grip = 23.30 m/s²`; v03 gives 23.35 m/s². Matches within rounding, confirming v04 collapses to v03 when `dFz_long = 0`.
- μ split at 200 km/h: `μ_f = 1.602`, `μ_r = 1.547`. Rear per-tyre load is ~22% higher than front (`Fz_r/2 = 5504 N` vs `Fz_f/2 = 4500 N`), producing a 3.5% μ split. That split is the mechanism by which weight transfer costs lap time.
- Max `a_long = 0.91 g` — realistic for a GT3 out of slow corners. `dFz` at 0.91 g = 2074 N matches `m·a·h/L` to the newton, confirming the implicit forward-pass solver is self-consistent.
- **Red flag: max `a_brake = 3.65 g` / max |dFz| = 8301 N.** GT3 at N24 peaks around 2.5–2.8 g; 3.65 g is too high. The lever-arm math is internally consistent (8301 N matches 3.65 g transfer), so it is not a bookkeeping bug — likely either (a) iteration artefact at one outlier track point, or (b) a high-speed bias-constraint corner case where the front's aero-loaded grip headroom lets the formula over-deliver, with no wheel-lift ceiling to cap it.

**Think:**
- +4.32 s weight-transfer cost is physically sensible. Textbook expectation for a GT3 at N24 is 3–8 s; we're right in the middle.
- The v04 rewrite has *uncovered* the curvature problem rather than v04-the-model hiding it. The remaining 20.6 s gap to reference decomposes roughly as: curvature under-reporting (10–15 s; still on 76% peak preservation from telemetry), driver pace (QSS optimum beats a real lap by 2–5 s), missing physics (tyre temp, fuel burn, shift time — a few s combined).
- Entry 011's diagnosis is now fully validated. The 2026-04-18 "curvature is the bottleneck" theory was wrong; curvature improvement is still worth pursuing, but as a separate, additive axis of error — not the cause of v04's oscillation.

**Next:**
- Inspect the 3.65 g brake peak using the bonus diagnostic figure the v04 script already produces (`a_long` and `−a_brake` vs distance). Spike at one point → numerical outlier, note and move on. Plateau across several points → bias-constraint issue, consider capping `a_brake` physically or modelling rear wheel-lift.
- Wire GPS-derived centerline into `build_track.m` as optional source (`track_source = 'gps' | 'telemetry'`). Re-run v02/v03/v04 on GPS track to quantify curvature's isolated contribution.
- After both experiments, begin calibration: sweep `h_cog` ∈ [0.40, 0.52] m and `brake_bias_f` ∈ [0.53, 0.61] against reference lap, minimising lap-time delta and sector-by-sector Δspeed.

---

## Entry 011 — 2026-04-19 — v04 diagnosis: six physics bugs identified (retracts Entry 010 conclusion)

**Phase:** 4 (Model build — v04 diagnosis)

**Done:**
- Diagnosed v04 `lap_sim_v04.m` line-by-line against v03 `lap_sim_v03.m`. Identified six distinct bugs in v04's grip and force physics.
- Retracts Entry 010's conclusion that *"v04 code is correct; the issue is input quality"*. The +49.7 s penalty is not masked by curvature error — v04 has structural physics errors that would produce a large unphysical delta on any track.

**Found — Bug list ranked by impact:**

  1. **Cornering pass ignores aero downforce in lateral grip.** v04 line 131: `v_corner = sqrt(mu_eff * g / kappa)` uses `a_lat = μ·g`, i.e. v01-era physics. v03 uses `a_lat = μ·(m·g + F_df)/m`. At 200 km/h, downforce adds ~51% to static weight → v04 underestimates a_lat by ~34%, v_corner by ~18% in fast corners. **This alone explains the bulk of the 49.7 s.**
  2. **Per-axle Fz used with per-tyre load-sensitivity coefficient.** v04 lines 123/126 feed `Fz_per_axle` (= Fz_total/2) into `k_load_sens`, which is calibrated for `Fz_per_tyre` (= Fz_total/4) in v03. μ drops 2× too fast with load. At 200 km/h: v03 μ = 1.575, v04 μ = 1.300.
  3. **Forward-pass traction uses `a = μ·g` instead of `a = μ·Fz/m`.** v04 line 230. Correct RWD traction at rest: `a = μ · 0.54 · g`. v04 formula gives ~2× that figure; also loses downforce contribution at speed. Engine power dominates at medium/high v, limiting impact — but wrong at slow corner exits.
  4. **Brake formula `min(μ_f·g, μ_r·g)` is nonphysical.** v04 line 313. Both axles brake simultaneously; correct limit is `(μ_f·Fz_f + μ_r·Fz_r)/m` subject to `brake_bias_f = 0.57`. Current formula throws away ~half of total brake force.
  5. **No friction circle in forward/backward passes.** v03 uses `a_long = sqrt(a_total² − a_lat²)`. v04 treats lateral and longitudinal grip as independent, over-estimating grip on entry/exit of corners.
  6. **Hard-coded 50/50 static and aero splits** ignore `car.weight_dist_f = 0.46` and `car.aero_balance_f = 0.43` from the params file.

**Think:**
- Root cause is architectural, not algebraic: v04 was written as a ground-up rewrite rather than "v03 physics + per-axle loads + longitudinal transfer". Several pieces of v03's correct grip calculation got dropped in the rewrite (aero-inclusive Fz, per-tyre k scaling, friction circle).
- Entry 010's "curvature data is critical" hypothesis is not supported by the code. Fixing curvature will improve absolute accuracy across all models, but will NOT close the v03→v04 gap because the gap is caused by code-level physics errors in v04. GPS centerline work from 2026-04-19 remains independently valuable.
- Expected v04 result after fix: **7:50 – 7:58** (i.e. 4–12 s slower than v03, not 50 s slower). Weight transfer should reduce combined grip modestly on corner entry/exit; a 50-second penalty implies the model is leaving nearly a third of total grip on the table, which is non-physical.

**Next:**
- File a GitHub issue capturing the six bugs (draft in `00_admin/v04_github_issue.md`).
- Rewrite `lap_sim_v04.m` as "v03 + per-axle loads + longitudinal transfer" preserving v03's grip calculation. Ordered fix plan:
  1. Per-axle grip function returning `(a_f, a_r)` using correct `weight_dist_f` / `aero_balance_f` and per-tyre k.
  2. Cornering pass: use `a_lat = (F_grip_f + F_grip_r)/m` with zero longitudinal transfer.
  3. Forward pass: iterative solve with `dFz = m·a·h/L`, friction circle, RWD traction `a = μ_r·Fz_r/m`.
  4. Backward pass: same but with brake-bias-aware combined braking force.
- Confirm expected result lands in 7:50–7:58 range before proceeding to v05.

---

## Entry 010 — 2026-04-18 — v04 weight transfer: 49.7 s penalty; curvature data critical
> **⚠ Superseded by Entry 011 (2026-04-19):** the "v04 code is correct, issue is curvature data" conclusion below is wrong. v04 has six code-level physics bugs — see Entry 011.


**Phase:** 4 (Model build — v04)

**Done:**
- Built v04 simulator (`03_models/v04_weight_transfer/lap_sim_v04.m`). Added longitudinal weight transfer: during acceleration, weight shifts to rear (increases rear grip, decreases front grip); during braking, weight shifts to front (increases front grip, decreases rear grip).
- Physics: dFz = m × a_long × h_cog / wheelbase. Front and rear loads computed separately. Load sensitivity applied per-axle.
- Braking model checks BOTH front and rear axle limits; takes minimum (whichever reaches grip limit first).
- v04 result: **8:36.089** vs reference 8:11.341 = **+24.7 s (+5.0%)**.
- v04 vs v03: **+49.7 s** penalty (weight transfer cost).

**Found:**
- v04 swung from v03's −25.0 s to +24.7 s — a 50-second swing, which is huge and unphysical.
- The progression v01 (+2.4) → v02 (−35.4) → v03 (−25.0) → v04 (+24.7) oscillates instead of converging, suggesting input data error, not physics error.
- Curvature data at 76% peak preservation is now identified as the critical bottleneck. The differences between v02, v03, v04 are so large that systematic curvature error overwhelms the model differences.
- When corner tightness is mis-reported (corners smoothed to appear gentler), adding grip-reducing physics (load sens, weight transfer) compounds the error, making the car appear much slower than it should be.

**Think:**
- Weight transfer is a real physical effect (~50 seconds worth at N24 if the model is correct). But we can't validate the magnitude with curvature error masking it.
- The correct approach: fix curvature data FIRST (via GPS or clean lap → 90%+ preservation), THEN tune load sensitivity and weight transfer coefficients to match reference lap.
- We've been building models in the right order (v01 → v02 → v03 → v04), but on bad input data. Like trying to tune a car's setup with a broken load cell.
- v04 code is correct (physics equations are sound, solver structure works). The issue is input quality, not the model.

**Next:**
- CRITICAL: Extract GPS geometric curvature from .pxt file or do clean mapping lap to reach 90%+ peak preservation.
- Once curvature is fixed, re-run v02, v03, v04 to see realistic deltas for each physics addition.
- Proceed to v05 (bicycle model, lateral load transfer) only after curvature is validated.

---

## Entry 008 — 2026-04-16 — Root cause found: curvature smoothing destroys 34% of peak

**Phase:** 4 (Correlation diagnosis)

**Done:**
- Built diagnostic script (`04_correlation/diagnose_grip.m`) to compare reference telemetry grip vs sim assumptions at every speed range.
- Identified root cause of 36-second gap: the 50 m moving average on curvature was destroying 34% of peak curvature. The tightest corner went from R = 13.9 m (raw) to R = 21.2 m (smoothed) — a 52% increase in radius. The sim then computed much higher cornering speeds for every tight corner.
- Root cause behind the root cause: iRacing's lateral g signal contains kerb/bump spikes up to 4.44 g (peak raw). These spikes forced us to use a wide 50 m smoothing window, which was too aggressive and rounded off real corner shapes.
- Implemented fix in `build_track.m`: replaced single-stage 50 m moving average with two-stage filtering:
  - Stage 1: Median filter (15 samples, ~7.5 m) on raw lateral g BEFORE computing curvature. Median filter kills spikes while preserving step edges (corner entries/exits).
  - Stage 2: Smaller 20 m moving average on the resulting curvature for final cleanup.
- Expected improvement: peak curvature preservation from 66% → 85–90%.

**Found:**
- v03 result with old smoothing: 7:35.042 (−36.3 s, −7.4%). Only 0.9 s slower than v02 because load sensitivity barely matters when curvature is under-reported — the sim never reaches the high-load regime where load sensitivity bites.
- Reference telemetry grip levels: peak |g_lat| = 4.44 g (kerb spike, not real grip), 99th pctile = 2.77 g (real cornering), 95th pctile = 2.09 g.
- Sim v03 assumed up to ~1.65 g at low speed, ~3.3 g at 200 km/h (with aero). The 3.3 g at high speed is plausible (matches 99th pctile at those speeds), but the curvature under-reporting meant the sim never needed that much grip.
- The v01→v02→v03 lap time progression (8:13 → 7:35 → 7:35) was suspicious: load sensitivity should have added ~10–15 s back onto v02. It only added 0.9 s because the curvature error dominated everything.

**Think:**
- This is the classic "garbage in, garbage out" problem. The physics model (load sensitivity) was correct, but the input data (curvature) was wrong. No amount of model refinement can fix bad input data — the curvature fix must come first before we can trust v02/v03 deltas.
- The two-stage filter is the right engineering approach: use the right tool for each problem. Median filter for impulsive noise (kerbs), moving average for continuous noise (sensor/bumps). Professional teams do exactly this — you never just throw a big moving average at noisy telemetry.
- After the fix, we should see v03 produce a significantly different (slower) time than v02, because the tighter corners will now demand more grip → more load → more load sensitivity loss. That's the signal we expect.

**Next:**
- Rebuild track data with updated build_track.m (two-stage filtering).
- Re-run v02 and v03 to verify improvement.
- If the gap drops to ±5% or better, proceed to v04 (longitudinal weight transfer). If still too fast, investigate geometric curvature from GPS as an alternative.

---

## Entry 009 — 2026-04-18 — Two-stage filter fix: recovered 11 seconds, load sensitivity now visible

**Phase:** 4 (Correlation — curvature fix validation)

**Done:**
- Implemented two-stage filtering in `build_track.m`: median filter (15 samples, ~7.5 m) on raw lateral g, then 20 m moving average on curvature. Replaced old single-stage 50 m smoothing.
- Rebuilt track data with updated script.
- Re-ran v03 simulator with corrected curvature input.
- Peak preservation improved from 66% → 76% (target was >85%, still need GPS or clean lap).

**Found:**
- v03 lap time improved dramatically: 7:35.042 → **7:46.382** (−25.0 s vs reference, −5.1% instead of −7.4%).
- Recovered 11 seconds just from better curvature input.
- Load sensitivity effect now visible: **10.5 s cost** (v03 vs v02), vs. negligible 0.9 s before. This validates that the curvature fix is working — the model now reaches tight corners where load sensitivity matters.
- μ drop with load: 8.6% (1.653 g at 80 km/h → 1.511 g at 260 km/h). Physically sensible.
- Median filter reduced peak lateral g from 4.44 g (kerb spike) to 3.69 g, but 3.69 g is still a kerb strike — 15-sample window not quite wide enough.

**Think:**
- Input data quality is the limiting factor now, not the physics model. The load sensitivity model is correct; it was just invisible with bad curvature data. This is a key lesson: garbage in = garbage out, no matter how good your model is.
- Remaining 25-second gap (−5.1%) comes from: (1) curvature still at 76% preservation, not 85%+, and (2) missing v04 (weight transfer). Both will be addressed next.
- The version progression now makes physical sense: v01→v02 (+38 s aero) → v03 (+10.5 s load sensitivity) → v04 (+ weight transfer, est. +5–10 s).

**Next:**
- Extract GPS position channels from iRacing telemetry (x, y coordinates) or use .pxt track map file to compute curvature geometrically. This bypasses lateral-g noise completely and should reach 90%+ peak preservation.
- Re-run v03 with GPS-derived curvature.
- If gap drops below −3%, proceed to v04. Otherwise, do clean mapping lap as backup.

---

## Entry 007 — 2026-04-16 — v03 load sensitivity: only 0.9 s slower than v02

**Phase:** 4 (Model build — v03)

**Done:**
- Built v03 simulator (`03_models/v03_load_sens/lap_sim_v03.m`). Added tyre load sensitivity: μ_eff = μ_0 − k × Fz_per_tyre, where Fz_per_tyre = (m×g + aero_df_coeff × v²) / 4.
- Cornering speed equation becomes implicit (μ depends on Fz, Fz depends on v through aero) — solved iteratively with fixed-point iteration (converges in 3–5 steps).
- v03 result: **7:35.042** vs reference 8:11.341 = **−36.3 s (−7.4%)**.
- v03 vs v02: only **−0.9 s** difference. Load sensitivity barely changed the lap time.

**Found:**
- At low speed (no aero), μ_eff = μ_0 − k × (m×g/4) = 1.85 − 5.5e-5 × 3312 = 1.668. This is HIGHER than v01/v02's constant μ = 1.60. So v03 is actually faster in slow corners.
- At high speed (200 km/h), aero doubles the tyre load, but μ only drops to ~1.45. The extra downforce still provides more grip than it costs in reduced μ.
- Net effect: low-speed gain (higher μ than 1.60) nearly cancels high-speed loss (load sensitivity). This explains the tiny 0.9 s delta.
- The real problem is not the grip model — it's the curvature input. Diagnosed separately in diagnose_grip.m.

**Think:**
- The 0.9 s v02→v03 delta is suspiciously small. In professional lap sim work, load sensitivity typically costs 3–8% of lap time at a high-downforce circuit like N24. The near-zero impact here is a red flag that something upstream (curvature data) is masking the effect.
- The choice of μ_0 = 1.85 and k = 5.5e-5 came from literature estimates [EST]. These will need tuning during correlation, but only after the curvature issue is fixed — no point tuning a model against bad data.

**Next:**
- Diagnose the 36-second gap: is it curvature smoothing, grip overestimate, or both?

---

## Entry 006 — 2026-04-16 — v02 aero downforce: 38 s gain, 7.2% too fast

**Phase:** 4 (Model build — v02)

**Done:**
- Built v02 simulator (`03_models/v02_aero/lap_sim_v02.m`). Added speed-dependent grip: F_grip = μ × (m×g + aero_df_coeff × v²). Solver structure unchanged from v01.
- Derived new closed-form cornering speed equation with downforce. Discovered critical curvature threshold (κ_crit = 0.0026, R = 385 m): corners gentler than this have no grip limit — downforce grows faster than cornering demand.
- v02 result: **7:35.919** vs reference 8:11.341 = **-35.4 s (-7.2%)**.
- v02 vs v01: **-37.8 s** (downforce value at N24).

**Found:**
- Downforce is worth ~38 seconds at N24 — by far the largest single physics effect in the model. No other parameter comes close.
- 17,184 of 25,206 track points (68%) are aero-dominated (grip limit set by downforce, not base grip).
- Max speed 289.6 km/h (realistic — drag-limited equilibrium working correctly).
- Model is 7.2% too fast because constant μ overestimates grip at high aero loads. At 260 km/h, downforce adds ~11,400 N, nearly doubling the tyre load, but real μ drops from ~1.60 to ~1.45 at that load (load sensitivity). This overestimate compounds across every fast corner.

**Think:**
- v02 confirms that aero-without-load-sensitivity is dangerously optimistic. Real teams ALWAYS model load sensitivity alongside aero — this result shows exactly why. An engineer presenting a -7.2% prediction would lose credibility immediately.
- The v01→v02 delta (+38 s for downforce) is a genuinely useful engineering number. It quantifies the aero's contribution to the lap and could inform wing-level trade-off decisions.
- v03 (load sensitivity) should bring the time back up significantly, possibly close to the ±1% target. The question is whether it overshoots or undershoots.

**Next:**
- v03 — add tyre load sensitivity: μ(Fz) = μ_0 - k × Fz. This directly addresses the dominant error in v02.

---

## Entry 005 — 2026-04-16 — Phase 3 complete: v01 point-mass simulator runs

**Phase:** 3 (Model build — v01)

**Done:**
- Built the v01 point-mass QSS lap simulator (`03_models/v01_point_mass/lap_sim_v01.m`).
- Model: fixed friction circle (μ = 1.60), aero drag (no downforce), engine torque curve with auto gear selection, three-pass solver (cornering → forward → backward), lap continuity iteration.
- Debugged forward pass: initial code clamped `a_forward` to zero, preventing drag deceleration above equilibrium speed. Fixed to allow drag (an aero force, not a tyre force) to decelerate the car independently of the friction circle.
- First sim result: **8:13.730** vs reference **8:11.341** = **+2.389 s (+0.5%)**.

**Found:**
- v01 hits ±1% target on first attempt. This is likely due to compensating errors: no downforce makes fast corners too slow (sim underestimates grip at high speed), while constant μ overestimates grip at high load (ignores load sensitivity). These errors partially cancel, giving a misleadingly close lap time.
- Lap continuity iteration converges in 1 pass. Start/finish speed settles at ~233 km/h.
- Min cornering speed: 65.7 km/h (tightest Nordschleife corner, ~21 m radius).
- The speed comparison plot shows the sim tracks the reference shape well overall, but will diverge in specific zones once we look more closely during proper correlation analysis.

**Think:**
- The +0.5% headline number is encouraging but should not be over-interpreted. Channel-by-channel correlation (speed vs. distance overlay, g-g scatter) will reveal where the model is honest and where it's getting lucky. That's the real validation, not just the total lap time.
- Adding downforce in v02 will change the picture significantly: fast-corner speeds will increase (more grip), but straight-line speed may drop (more drag from increased Cl). The net lap time effect is unclear until we run it.
- The three-pass solver structure is clean and extensible — v02 only needs to modify how grip is computed at each point (make it speed-dependent), not the solver logic itself.

**Next:**
- Phase 4 — build v02 (add aerodynamic downforce to the grip model). This is the first model version where grip becomes speed-dependent.

---

## Entry 004 — 2026-04-16 — Phase 2 complete: data acquisition done

**Phase:** 2 (Data acquisition)

**Done:**
- Created vehicle parameter file (`02_data/car/amg_gt3_params.m`). All parameters loaded as a single `car` struct with data-quality flags: [HOMOL], [IRACING], [EST], [CALC]. Verification printout confirms sane values.
- Created MATLAB startup script (`startup_project.m`) — sets project root as working directory and loads car parameters automatically.
- Exported reference lap from iRacing via PI Toolbox Pro: 8:11.341 at N24 in AMG GT3. Group 1 channels (speed, accel, throttle, brake, gear, RPM, steering). Saved as `reference_lap.xls`.
- Built import script (`02_data/telemetry/processed/import_reference_lap.m`): reads PI Toolbox export, converts units to SI, computes distance via trapezoidal integration of speed, saves clean `ref` struct as `.mat`.
- Built track data script (`02_data/track/build_track.m`): computes curvature from lateral g and speed (kappa = a_lat/v²), smooths with 50 m moving average, resamples to 1 m uniform distance spacing.

**Found:**
- PI Toolbox export is at 100 Hz (interpolated from iRacing's native 60 Hz). Adequate resolution.
- Computed track length: 25,206 m vs. official 25,378 m (0.7% shorter — expected, since racing line clips apexes vs. geometric centerline).
- Lap duration from data: 491.330 s vs. stated 491.341 s (11 ms difference from last sample boundary — negligible).
- Peak curvature ~0.047 [1/m] = ~21 m radius. Consistent with tightest Nordschleife corners (Karussell, Adenauer Forst).
- Top speed per gear from params: 75, 117, 157, 198, 244, 291 km/h. 6th gear theoretical max (291) is above aero-limited top speed (~270), meaning car runs out of power before gears. Correct for N24 aero config.
- Aero at 200 km/h: 6763 N downforce (~689 kg, about half car mass). L/D = 3.31. Both in expected GT3 range.
- MATLAB `save` function does not resolve relative paths the same way `readtable` does — must use absolute paths via `fullfile(pwd, ...)`. Learned the hard way.

**Think:**
- All three data inputs for v01 are in place: car params, reference telemetry, track curvature.
- Groups 2–4 telemetry (parameter extraction, GPS, extras) deferred — not needed until v02+ correlation when we refine [EST] parameters. GPS data available when needed for elevation profile.
- The verification plots (speed trace, curvature, g-g) all look physically consistent. No gross data errors detected.
- 50 m smoothing window for curvature is a tuneable parameter — may need refinement during correlation if sim speed oscillates or corners are over-smoothed.

**Next:**
- Phase 3 — build the v01 point-mass QSS simulator in MATLAB. First real model code.

---

## Entry 003 — 2026-04-16 — Phase 1 complete: fidelity ladder accepted

**Phase:** 1 (Requirements & fidelity decisions)

**Done:**
- Reviewed the fidelity ladder from Rung 1 (point-mass, fixed friction) through Rung 5 (bicycle + lateral load transfer). Studied what each rung adds and what it still misses.
- Accepted the incremental roadmap: v01 → v02 → v03 → v04, with v05 as a stretch goal.
- Wrote and committed Design Note 001 (`00_admin/02_design_note_001_fidelity.md`) capturing the architecture decision in ADR format: context, decision, rationale, alternatives considered, consequences.
- Chose QSS (quasi-steady-state) time treatment over transient. Track will be discretized into segments; at each segment we solve for maximum speed given the current grip envelope.

**Found:**
- The core question in simulation engineering is not "is my model correct?" but "is my model correct enough for the question I'm asking?" — fidelity is a deliberate design choice, not a default.
- The g-g diagram (friction circle) is the central concept: it represents the car's acceleration capability at any instant. For v01 it's a fixed circle; for v02+ it becomes speed-dependent (g-g-*v* surface) because aero downforce grows with speed.
- Tyre load sensitivity (Rung 3) is where correlation typically improves the most: real tyres lose grip coefficient as vertical load increases, so ignoring this overestimates high-speed cornering.
- Climbing rung-by-rung lets us quantify how much lap time each physical effect is worth at N24 — that's a learning output, not just an intermediate.

**Think:**
- The incremental approach is slower to build but dramatically easier to debug and learn from. If v03 produces a bad number, the diff against a trusted v02 isolates the problem to load sensitivity specifically. Jumping to v04 directly would make root-cause analysis nearly impossible.
- QSS is the right choice: ~80% of professional setup studies are done QSS. Transient would require suspension/damper data we don't have and would add complexity without teaching the core lessons better.
- v04 (+ longitudinal weight transfer) is the realistic target for the charter's ±1% lap time correlation. v05 is a bonus if time allows.

**Next:**
- Phase 2 — data acquisition. Collect AMG GT3 vehicle parameters (mass, CoG, wheelbase, aero map, tyre grip, engine curve, gearing) and process iRacing telemetry + track map through PI Toolbox into usable inputs.

---

## Entry 002 — 2026-04-16 — Phase 0 complete: Git repo live

**Phase:** 0 (Project setup)

**Done:**
- Installed Git for Windows and configured global `user.name` and `user.email`.
- Cleaned up leftover `.git/` folder from failed sandbox attempt, then ran `git init -b main`, `git add .`, `git commit -m "setup: ..."` in Git Bash at the project root.
- Verified first commit with `git log --oneline`. Hash on `main`: `9920ffb`.

**Found:**
- `git init` printed `warning: re-init: ignored --initial-branch=main` because a prior partial `.git/` still contained a valid commit. Subsequent `git add`/`git commit` returned "nothing to commit, working tree clean" — i.e. the state was already what we wanted. No harm done; just a quirk of reusing an existing `.git`.
- Learned the core six Git commands: `init`, `status`, `add`, `commit`, `log`, `diff`.
- Learned our commit-message convention: `<type>: <summary>`, types being `setup | data | model | corr | study | docs | fix`.

**Think:**
- Repo is correctly initialized with `main` as the primary branch. `.gitignore` is in place so raw telemetry and MATLAB clutter won't pollute history.
- The working loop from now on is: edit → `git status` → `git add <files>` → `git commit -m "..."`. Every session ends with at least one commit.
- The "re-init" warning is benign but worth remembering — Git treats `.git/` as sacred and won't overwrite it, so if something is wrong with a repo you delete `.git/` and start fresh.

**Next:**
- Begin Phase 1 — requirements & fidelity decisions. Pick the model architecture (point-mass QSS → what extensions, in what order) and write it up as a short design note in `00_admin/`.

---

## Entry 001 — 2026-04-15 — Project kickoff

**Phase:** 0 (Project setup)

**Done:**
- Defined project scope with technical lead: AMG GT3 on N24 24h layout, MATLAB R2024b + iRacing + PI Toolbox Pro only.
- Created folder structure under `Lap time simulator/` with phase-numbered directories (00_admin through 07_portfolio).
- Wrote and reviewed project charter (`00_admin/00_project_charter.md`). Accepted as-is.
- Confirmed reference lap: own iRacing telemetry, 8:11 baseline in AMG GT3 at N24 layout.

**Found:**
- MATLAB R2024b installed with full toolbox suite (incl. Simulink, Optimization, Vehicle Dynamics Blockset).
- PI Toolbox Pro licensed — math channels and exports available.
- No Git experience yet; crash course scheduled for Step 0.3.

**Think:**
- Tooling is sufficient to hit the charter's correlation target (±1% lap time on QSS model).
- Scope deliberately excludes tyre thermal/wear — defensible given no rig data access.
- Reference lap at 8:11 is clean enough for correlation (doesn't need to be a record lap; needs to be consistent and representative).

**Next:**
- Complete Step 0.3 — Git setup.
- Begin Phase 1 — requirements and fidelity decisions.

---

<!-- Add new entries ABOVE this line, most-recent-first ordering -->
<!-- Template:

## Entry NNN — YYYY-MM-DD — short title

**Phase:** X (name)

**Done:**
-

**Found:**
-

**Think:**
-

**Next:**
-

---

-->
