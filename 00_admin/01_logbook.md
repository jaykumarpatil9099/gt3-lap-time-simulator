# Engineering Logbook — N24 Lap Time Simulator

**Owner:** Jaykumar Patil
**Rule:** Append-only. One entry per working session. Never delete entries — if something was wrong, add a correction entry referring back to it.

**Entry format:**

- **Done:** what I actually did this session
- **Found:** observations, numbers, screenshots, surprises
- **Think:** my interpretation — what it means, why it matters
- **Next:** what comes in the next session

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
