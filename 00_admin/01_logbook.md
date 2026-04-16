# Engineering Logbook — N24 Lap Time Simulator

**Owner:** Jaykumar Patil
**Rule:** Append-only. One entry per working session. Never delete entries — if something was wrong, add a correction entry referring back to it.

**Entry format:**

- **Done:** what I actually did this session
- **Found:** observations, numbers, screenshots, surprises
- **Think:** my interpretation — what it means, why it matters
- **Next:** what comes in the next session

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
