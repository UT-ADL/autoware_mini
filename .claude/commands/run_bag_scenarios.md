---
allowed-tools: Bash, Read, Edit, Glob, Grep
description: Run bag scenario tests and analyze result changes
---

# Run Bag Scenario Tests

Run bag scenario tests and report rating changes vs `main`.

**Arguments**: `$ARGUMENTS` — optional test set name (`yielding`, `crosswalks`, `tartu_demo`, `lexus`, `swerving`, `give_way_bus`, `planner`, `objects_behind`). If omitted, runs ALL.

## Gotchas (read before running)

- **Never parallel.** Each `*_tests.sh` starts its own `roslaunch`. Two concurrent launches on port 11311 crash on roscore conflict; even with separate `ROS_MASTER_URI` they fight over CPU/GPU and make the metrics non-comparable. No backgrounding, no `&&`-chains, no second terminal — one suite, wait, verify, next.
- **Source the ROS env in every Bash call.** The harness hands a fresh shell per invocation. Without `source $WORKSPACE/devel/setup.bash` the scripts' `rospack find autoware_mini` returns empty, they silently write CSVs under `/data/bag_scenarios/…` (nonexistent), and still exit 0. Verify log tail + CSV mtime, never trust exit code.
- **`--no_rviz` only on neuron.** Neuron has `/data/bag_cache` and no display — pass `--no_rviz`. Laptop has a display and no `/data/bag_cache` — leave RViz on (primary debugging aid). Quick check: `[ -d /data/bag_cache ] && echo neuron || echo laptop`.
- **Diff vs `main`, not HEAD.** A feature branch may already have committed CSV regenerations, so diffing vs HEAD shows only rerun noise. Always `git diff main -- 'data/bag_scenarios/*/results/*_results.csv'`.
- **`run_single_test.sh` rewrites the scenario's row in the suite CSV in-place** (and recomputes the AVERAGE row). Stash the full-suite CSVs before reruns, grep each rerun's row immediately, restore CSVs when done — see step 4 below.
- **Rerun batching.** Full-stack relaunch ≈35 s/scenario; Bash timeout caps at 10 min. Keep each background batch to ≤4 scenarios × 3 runs (~7 min).

## Process

### 1. Check environment

```bash
source $(catkin locate)/devel/setup.bash
rospack find autoware_mini
cd $(rospack find autoware_mini)/scripts/bag_scenarios
```

### 2. Run suites

Append `--no_rviz` on neuron, omit on laptop.

**Single suite** (`$ARGUMENTS` given):
```bash
./<testset>_tests.sh [--no_rviz]
```

**All suites** (sequentially; `tartu_demo` first as a fast env sanity check):
```bash
./tartu_demo_tests.sh        # fast sanity check
./give_way_bus_tests.sh
./objects_behind_tests.sh
./lexus_tests.sh
./planner_tests.sh
./swerving_tests.sh
./crosswalks_tests.sh
./yielding_tests.sh
```

**After every suite, before the next**:
1. Tail the log — must end with `Average ADE / collision / max deceleration` and their success rates. Missing summary = crash; stop.
2. Confirm the CSV was touched (`git status`).
3. `git diff main -- <suite_csv>` and sanity-check. Stop and investigate if:
   - all / most scenarios flip SUCCESS → FAILURE,
   - values are nonsensical (NaN, negative ADE, zeroed metrics),
   - CSV shape changed (rows missing, columns reordered),
   - a previously-passing suite now has 0 % success.

### 3. Identify rating flips vs main

```bash
git diff main -- 'data/bag_scenarios/*/results/*_results.csv'
```

List scenarios where the **ADE rating** or **collision rating** changed SUCCESS ↔ FAILURE.

### 4. Verify flips with 3× reruns

Snapshot the full-suite CSVs once so per-scenario reruns don't lose them:
```bash
git stash push -m "full-suite CSVs" -- 'data/bag_scenarios/*/results/*_results.csv'
git stash apply    # keep snapshot AND working copy
```

For each flipped scenario, 3 runs:
```bash
./run_single_test.sh <map_name> <scenario_name> [--no_rviz]
grep "^<scenario_name>," <results_csv>    # record row before the next run overwrites it
```

`<map_name>` is `tartu_large` for most scenarios, `tartu_demo` for `tartu_demo_*`.

Restore clean full-suite CSVs when done:
```bash
git checkout -- 'data/bag_scenarios/*/results/*_results.csv'
git stash drop
```

### 5. Report

A flip is **confirmed** if 2/3 or 3/3 reruns differ from `main`; otherwise **flaky**.

```
## Scenarios with changed outcomes

### ADE rating changes
| Scenario | main | Majority (3 runs) | Confirmed? |

### Collision rating changes
| Scenario | main | Majority (3 runs) | Confirmed? |

## Summary
- Confirmed improvements: N
- Confirmed regressions: N
- Flaky: N
```

### 6. Commit (only when user asks)

One commit for the entire `data/bag_scenarios/*/results/` folder — CSVs and PNG plots together. Summarise confirmed improvements, confirmed regressions, and flaky findings vs `main` in the message. (The bag-vs-results commit split only applies to `/regenerate_bag_scenarios`.)
