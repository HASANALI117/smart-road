# Context Log

This file is a running record of major repository changes so future agents can recover project state quickly.

## How to use
- Append a new entry for any significant code change, refactor, temporary workaround, build fix, or architectural shift.
- Keep each entry short and factual.
- Include the date, what changed, why it changed, important files, and any verification status.
- If a temporary workaround is added, note the follow-up needed to remove it later.

## Current State

### 2026-04-13
- Replaced `src/intersection.rs` with a new intersection model that defines lane centers, spawn positions, stop-line positions, intersection bounds, and routing helpers.
- Updated `src/vehicle.rs` so vehicle angle initialization uses the SDL2-friendly angle convention, and added `stop_line_x` / `stop_line_y` fields initialized from `stop_line_pos(...)`.
- Added a temporary `src/statistics.rs` module earlier to satisfy the missing-module compile error.
- Removed the SDL2_image dependency path and switched rendering to a fallback path so the project can build and run in the current environment without the missing system library.
- Verification: `cargo check` passed after the changes.

### 2026-04-13 - Car sprites
- Added recursive loading for `assets/cars/**/*-top.png` and ignored the `-front` / `-back` variants.
- Vehicles now store a random sprite index chosen at spawn time so each car keeps its selected PNG.
- Textured rendering compensates for the left-facing source orientation by adding a 90° render offset.
- Follow-up: if new car folders are added, they are picked up automatically as long as the file name ends in `-top.png`.
- Implementation detail: PNGs are loaded through the `image` crate into SDL textures, so no SDL2_image system library is required for this path.
- Fix: some `*-top.png` files are actually WebP binaries; loader now decodes by file signature (`image::load_from_memory`) instead of trusting file extension.
- Fix: textured car rendering now preserves each sprite's aspect ratio and scales by length, preventing squeezed cars.

### 2026-04-14 - Turn routing fix
- Reworked the intersection turn path in `src/algorithm.rs` to use a directional cubic curve between the lane entry point and the correct exit lane instead of the old mirrored arc tables.
- Updated `src/intersection.rs` turn helpers so exit positions are derived from the actual exit road lane, preserving the existing lane convention used by the road markings.
- Removed the obsolete arc-only turning fields from `src/vehicle.rs` and stored explicit turn start/end points for curve interpolation.
- Corrected right/left semantics in `src/intersection.rs`: `exit_direction(...)` now matches each vehicle's travel heading, and north/south lane-center mapping now places right-turn traffic on the actual right-side lane.
- Added regression tests in `src/intersection.rs` for lane-center ordering, right/left exit mapping, and turn-exit lane alignment.
- Verification: `cargo check` passes; the only remaining warning is the pre-existing unused `stop_line_x` / `stop_line_y` fields in `src/vehicle.rs`.

### 2026-04-18 - ETA-based intersection priority
- Change: Replaced the FCFS `approach_time` queue with an ETA-based priority system in `src/algorithm.rs`. Two new helpers — `time_to_stop_line` and `lower_priority` — replace the old `approach_time` comparisons in both the trajectory conflict check and `should_yield`.
- Why: The old system gave permanent priority to whichever car entered the 160px detection zone first, regardless of actual position or speed, and used an arbitrary ID tiebreaker. The new system grants right-of-way to whichever car will physically reach the stop line soonest (`dist / max(velocity, base_velocity × 0.15)`). The speed floor prevents stopped cars near the line from having infinite ETA. `approach_time` is retained as a secondary tiebreaker when two ETAs are within 0.1s of each other, preventing frame-to-frame oscillation between similarly-positioned vehicles. ID remains the last-resort tiebreaker.
- Files: `src/algorithm.rs`
- Verification: `cargo check` passes; only pre-existing `stop_line_x` / `stop_line_y` dead-code warning remains.
- Follow-up: `stop_line_x` / `stop_line_y` fields on `Vehicle` are now fully superseded by `time_to_stop_line`; they can be removed when convenient.

## Entry Template

### YYYY-MM-DD
- Change:
- Why:
- Files:
- Verification:
- Follow-up:
