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

## Entry Template

### YYYY-MM-DD
- Change:
- Why:
- Files:
- Verification:
- Follow-up:
