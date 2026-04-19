# GitHub issue — ready to file

This file contains a ready-to-paste GitHub issue for the v04 physics bugs.
The repo has no remote configured yet and `gh` CLI isn't available in the
Cowork sandbox, so file it manually once the remote is set up.

---

## How to file

### Option A — via GitHub web UI
1. Create a repo on GitHub (e.g. `github.com/jaykumarpatil/n24-lap-sim`).
2. Wire up the remote locally:
   ```bash
   cd "C:\Users\jayku\Documents\Claude\Projects\Lap time simulator"
   git remote add origin git@github.com:<your-username>/<repo>.git
   git push -u origin main
   ```
3. Open `https://github.com/<your-username>/<repo>/issues/new`.
4. Paste the **Title** and **Body** sections below. Apply the suggested labels.

### Option B — via `gh` CLI (once installed)
```bash
gh auth login
gh issue create \
  --title "v04: six physics bugs cause +49.7 s unphysical penalty vs v03" \
  --body-file 00_admin/v04_github_issue_body.md \
  --label "bug,physics,v04"
```
(Save the body section below to `00_admin/v04_github_issue_body.md` first, or
pass the body inline with `--body "..."`.)

---

## Title

```
v04: six physics bugs cause +49.7 s unphysical penalty vs v03
```

## Labels

`bug`, `physics`, `v04`

## Body

```markdown
## Summary

`lap_sim_v04.m` currently produces **8:36.089** vs reference **8:11.341**
(+24.7 s / +5.0%). More tellingly, it is **+49.7 s slower than v03**, which
is non-physical — adding longitudinal weight transfer should cost at most
a few seconds on combined grip, not 50. Root cause is **architectural**,
not a coefficient tweak: v04 was rewritten from scratch rather than
extending v03, and in doing so dropped several pieces of v03's correct
physics.

## Observed

| Version | Lap time   | Δ vs ref | Δ vs prior |
|---------|------------|----------|------------|
| v01     | 8:13.730   | +2.4 s   | —          |
| v02     | 7:35.919   | −35.4 s  | −37.8 s    |
| v03     | 7:46.382   | −25.0 s  | +10.5 s    |
| **v04** | **8:36.089** | **+24.7 s** | **+49.7 s** |
| Ref     | 8:11.341   | —        | —          |

## Expected

v04 should sit between v03 and the reference, roughly **7:50–7:58**
(4–12 s slower than v03). Weight transfer reduces combined grip modestly
on entry/exit; 50 s implies the model is leaving ~⅓ of total grip on the
table.

## Bugs, ranked by impact

### 1. Cornering pass ignores aero downforce in lateral grip (HIGH)

`lap_sim_v04.m:131`

```matlab
v_corner_iter = sqrt(mu_eff * g / kappa);    % a_lat = μ·g  (v01 physics)
```

Correct (v03): `a_lat = μ · (m·g + F_downforce) / m`. At 200 km/h
downforce adds ~51% to static weight → v04 underestimates `a_lat` by
~34%, `v_corner` by ~18% in every fast corner. Accounts for most of the
49.7 s.

### 2. Per-axle Fz used with per-tyre `k` (HIGH)

`lap_sim_v04.m:123, 126`

```matlab
Fz_total_per_axle = Fz_static + Fz_aero_per_axle;     % = Fz_total / 2
mu_eff = mu_0 - k_load_sens * Fz_total_per_axle;
```

`car.tyre.load_sens_k = 5.5e-5` is calibrated in v03 against
`Fz_per_tyre` (= `Fz_total / 4`). Feeding it `Fz_per_axle` (2× larger)
makes μ drop twice as fast with load. At 200 km/h: v03 μ = 1.575,
v04 μ = 1.300.

### 3. Forward-pass traction uses `a = μ·g` instead of `a = μ·Fz/m` (MEDIUM)

`lap_sim_v04.m:230`

```matlab
a_traction_max = mu_r * g;
```

Correct RWD traction: `a_max = μ_r · Fz_r / m`. With `Fz_r ≈ 0.54·m·g`
at rest, the correct figure is `≈ 0.54·μ·g` — roughly half of what v04
computes. Power limit dominates at medium/high speed so real impact is
smaller than bugs 1 and 2, but the formula is wrong at slow corner
exits and loses the downforce contribution at speed.

### 4. Brake formula `min(μ_f·g, μ_r·g)` is nonphysical (MEDIUM)

`lap_sim_v04.m:313`

```matlab
a_brake = min(a_brake_f, a_brake_r);
```

Both axles brake simultaneously. Correct limit:
`a_brake = (μ_f·Fz_f + μ_r·Fz_r) / m`, subject to `car.brakes.bias_f =
0.57` constraining the front/rear split. Current formula throws away
roughly half the available brake force.

### 5. No friction circle in forward/backward passes (MEDIUM)

v03 has:

```matlab
a_grip_long = sqrt(a_grip_total^2 - a_lat_used^2);
```

v04 omits this entirely — lateral and longitudinal grip are treated as
independent, over-estimating grip on corner entry/exit where both are
in use.

### 6. Hard-coded 50/50 static and aero splits (LOW)

`lap_sim_v04.m:111, 120, 299–300`

```matlab
Fz_static = (car.mass * g) / 2;
Fz_aero_f = aero_df_coeff * v_next^2 / 2;    % should use aero_balance_f
Fz_aero_r = aero_df_coeff * v_next^2 / 2;
```

Ignores `car.weight_dist_f = 0.46` and `car.aero_balance_f = 0.43` from
`amg_gt3_params.m`. Smaller effect than bugs 1–5 but still wrong.

## Recommended fix — rewrite, not patch

No one-line fix exists. Rewrite `lap_sim_v04.m` as "**v03 grip
calculation + per-axle loads + longitudinal transfer**":

1. Build a per-axle grip function returning `(F_f, F_r)` using correct
   `weight_dist_f` / `aero_balance_f` and per-**tyre** `k`.
2. **Cornering pass:** `a_lat_max = (F_f + F_r) / m` with zero long
   transfer.
3. **Forward pass:** iterative solve with
   `ΔFz = m·a·h_cog / wheelbase`, friction circle,
   RWD traction `a = μ_r · Fz_r / m`.
4. **Backward pass:** same skeleton, combined braking force
   `a = (μ_f·Fz_f + μ_r·Fz_r) / m`, subject to `brake_bias_f`.

## Acceptance

- [ ] v04 lap time lands in 7:50–7:58 range.
- [ ] v04 − v03 Δ is between +4 s and +12 s (not +49.7 s).
- [ ] v04 lap time responds sensibly to sweeping `h_cog` from 0.40 m to
      0.52 m (higher CoG → slower lap, monotonic).
- [ ] All six bugs listed above are closed.

## Related

- `03_models/v04_weight_transfer/lap_sim_v04.m` (buggy)
- `03_models/v03_load_sens/lap_sim_v03.m` (reference implementation)
- `02_data/car/amg_gt3_params.m` (parameters being ignored)
- Logbook: `00_admin/01_logbook.md` — Entry 011 (2026-04-19)
```
