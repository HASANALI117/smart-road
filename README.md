# Smart Road — Intersection Simulator

A real-time traffic intersection simulator built with Rust and SDL2. Vehicles navigate a 4-way intersection using an ETA-based priority algorithm that prevents collisions without traffic lights.

## Features

- 6-lane cross intersection (3 lanes per direction)
- Vehicles choose Right, Straight, or Left routes
- ETA-based right-of-way algorithm with safe-distance enforcement
- Collision avoidance via velocity reduction and creep logic
- 3 vehicle speed classes: slow (120 px/s), normal (200 px/s), fast (240 px/s)
- Textured road rendering (kerb tiles, grass, rounded corners)
- Post-simulation statistics screen
- Trajectory and hitbox debug overlays

## Controls

| Key | Action |
|-----|--------|
| `↑` | Spawn vehicle from **South** (moving north) |
| `↓` | Spawn vehicle from **North** (moving south) |
| `→` | Spawn vehicle from **West** (moving east) |
| `←` | Spawn vehicle from **East** (moving west) |
| `R` | Toggle **random** auto-spawn mode |
| `T` | Toggle **trajectory** overlay |
| `H` | Toggle **hitbox** overlay |
| `ESC` | Open statistics screen / exit |

## Statistics

After pressing ESC the simulator displays:

- Total vehicles passed
- Max / Min velocity
- Max / Min transit time
- Average wait to enter intersection
- Average transit time
- Close calls

## Building

**Requirements:** Rust toolchain, SDL2 development libraries.

```bash
cargo build --release
cargo run --release
```

## Assets

Road tiles and grass textures are located in `assets/`. Car sprites are in `assets/cars/`, one subdirectory per model (top-view PNG used for rendering).
