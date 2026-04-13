use crate::route::{Cardinal, Route};

pub const WINDOW_WIDTH: u32 = 1000;
pub const WINDOW_HEIGHT: u32 = 800;

pub const ROAD_WIDTH: f64 = 360.0;
pub const LANE_WIDTH: f64 = 60.0;

pub const CENTER_X: f64 = WINDOW_WIDTH as f64 / 2.0;
pub const CENTER_Y: f64 = WINDOW_HEIGHT as f64 / 2.0;

pub const INTERSECTION_LEFT: f64   = CENTER_X - ROAD_WIDTH / 2.0;
pub const INTERSECTION_RIGHT: f64  = CENTER_X + ROAD_WIDTH / 2.0;
pub const INTERSECTION_TOP: f64    = CENTER_Y - ROAD_WIDTH / 2.0;
pub const INTERSECTION_BOTTOM: f64 = CENTER_Y + ROAD_WIDTH / 2.0;

// How far off-screen vehicles spawn
const SPAWN_MARGIN: f64 = 60.0;

// ─────────────────────────────────────────────────────────────
//  Lane centre positions
//  Each approach has 3 lanes (Right, Straight, Left).
//  Lanes are numbered 0-2 from the kerb inward.
//
//  South approach (vehicles move ↑, occupy east half of vertical road):
//    Right    → x = CENTER_X + LANE_WIDTH * 0.5   (outermost / rightmost)
//    Straight → x = CENTER_X + LANE_WIDTH * 1.5
//    Left     → x = CENTER_X + LANE_WIDTH * 2.5   (innermost)
//
//  North approach (vehicles move ↓, occupy west half of vertical road):
//    Right    → x = CENTER_X - LANE_WIDTH * 0.5
//    Straight → x = CENTER_X - LANE_WIDTH * 1.5
//    Left     → x = CENTER_X - LANE_WIDTH * 2.5
//
//  West approach (vehicles move →, occupy north half of horizontal road):
//    Right    → y = CENTER_Y - LANE_WIDTH * 0.5
//    Straight → y = CENTER_Y - LANE_WIDTH * 1.5
//    Left     → y = CENTER_Y - LANE_WIDTH * 2.5
//
//  East approach (vehicles move ←, occupy south half of horizontal road):
//    Right    → y = CENTER_Y + LANE_WIDTH * 0.5
//    Straight → y = CENTER_Y + LANE_WIDTH * 1.5
//    Left     → y = CENTER_Y + LANE_WIDTH * 2.5
// ─────────────────────────────────────────────────────────────
pub fn lane_center(origin: Cardinal, route: Route) -> (f64, f64) {
    let offset = match route {
        Route::Right    => LANE_WIDTH * 0.5,
        Route::Straight => LANE_WIDTH * 1.5,
        Route::Left     => LANE_WIDTH * 2.5,
    };

    match origin {
        Cardinal::South => (CENTER_X + offset, 0.0),   // y unused for vertical
        Cardinal::North => (CENTER_X - offset, 0.0),
        Cardinal::West  => (0.0, CENTER_Y - offset),   // x unused for horizontal
        Cardinal::East  => (0.0, CENTER_Y + offset),
    }
}

/// Returns the (x, y) spawn point — off screen, in the correct lane.
pub fn spawn_position(origin: Cardinal, route: Route) -> (f64, f64) {
    let (cx, cy) = lane_center(origin, route);
    match origin {
        Cardinal::South => (cx, WINDOW_HEIGHT as f64 + SPAWN_MARGIN),
        Cardinal::North => (cx, -SPAWN_MARGIN),
        Cardinal::West  => (-SPAWN_MARGIN,                    cy),
        Cardinal::East  => (WINDOW_WIDTH as f64 + SPAWN_MARGIN, cy),
    }
}

/// Stop-line position — where a vehicle waits before entering the intersection.
/// The vehicle centre should stop here.
pub fn stop_line_pos(origin: Cardinal, route: Route) -> (f64, f64) {
    let (cx, cy) = lane_center(origin, route);
    // One vehicle-length outside the intersection boundary
    let gap = 40.0; // px past the intersection edge
    match origin {
        Cardinal::South => (cx, INTERSECTION_BOTTOM + gap),
        Cardinal::North => (cx, INTERSECTION_TOP    - gap),
        Cardinal::West  => (INTERSECTION_LEFT  - gap, cy),
        Cardinal::East  => (INTERSECTION_RIGHT + gap, cy),
    }
}

// ─────────────────────────────────────────────────────────────
//  Spatial predicates
// ─────────────────────────────────────────────────────────────

pub fn is_in_intersection(x: f64, y: f64) -> bool {
    x >= INTERSECTION_LEFT  && x <= INTERSECTION_RIGHT
        && y >= INTERSECTION_TOP && y <= INTERSECTION_BOTTOM
}

/// True when the vehicle is within `margin` px of its stop line
/// (used to trigger reservation requests).
pub fn is_approaching_intersection(x: f64, y: f64, origin: Cardinal) -> bool {
    let margin = 160.0;
    match origin {
        Cardinal::South => y > INTERSECTION_BOTTOM && y < INTERSECTION_BOTTOM + margin,
        Cardinal::North => y < INTERSECTION_TOP    && y > INTERSECTION_TOP    - margin,
        Cardinal::West  => x < INTERSECTION_LEFT   && x > INTERSECTION_LEFT   - margin,
        Cardinal::East  => x > INTERSECTION_RIGHT  && x < INTERSECTION_RIGHT  + margin,
    }
}

/// True when the vehicle is very close to the intersection entry
/// (last chance to stop before entering).
pub fn is_near_intersection_entry(x: f64, y: f64, origin: Cardinal) -> bool {
    let margin = 50.0;
    match origin {
        Cardinal::South => y > INTERSECTION_BOTTOM && y < INTERSECTION_BOTTOM + margin,
        Cardinal::North => y < INTERSECTION_TOP    && y > INTERSECTION_TOP    - margin,
        Cardinal::West  => x < INTERSECTION_LEFT   && x > INTERSECTION_LEFT   - margin,
        Cardinal::East  => x > INTERSECTION_RIGHT  && x < INTERSECTION_RIGHT  + margin,
    }
}

pub fn has_left_screen(x: f64, y: f64) -> bool {
    let margin = 80.0;
    x < -margin
        || x > WINDOW_WIDTH  as f64 + margin
        || y < -margin
        || y > WINDOW_HEIGHT as f64 + margin
}

// ─────────────────────────────────────────────────────────────
//  Routing helpers
// ─────────────────────────────────────────────────────────────

/// Which cardinal direction a vehicle exits toward.
pub fn exit_direction(origin: Cardinal, route: Route) -> Cardinal {
    match (origin, route) {
        (Cardinal::South, Route::Straight) => Cardinal::North,
        (Cardinal::South, Route::Right)    => Cardinal::West,
        (Cardinal::South, Route::Left)     => Cardinal::East,

        (Cardinal::North, Route::Straight) => Cardinal::South,
        (Cardinal::North, Route::Right)    => Cardinal::East,
        (Cardinal::North, Route::Left)     => Cardinal::West,

        (Cardinal::West,  Route::Straight) => Cardinal::East,
        (Cardinal::West,  Route::Right)    => Cardinal::North,
        (Cardinal::West,  Route::Left)     => Cardinal::South,

        (Cardinal::East,  Route::Straight) => Cardinal::West,
        (Cardinal::East,  Route::Right)    => Cardinal::South,
        (Cardinal::East,  Route::Left)     => Cardinal::North,
    }
}

/// The precise (x, y) where a vehicle should begin its turning arc.
/// This is the point on the intersection boundary in the vehicle's lane.
pub fn turn_entry_point(origin: Cardinal, route: Route) -> (f64, f64) {
    let (cx, cy) = lane_center(origin, route);
    match origin {
        Cardinal::South => (cx, INTERSECTION_BOTTOM),
        Cardinal::North => (cx, INTERSECTION_TOP),
        Cardinal::West  => (INTERSECTION_LEFT,  cy),
        Cardinal::East  => (INTERSECTION_RIGHT, cy),
    }
}

/// The precise (x, y) where a turning arc ends (exit boundary point).
pub fn turn_exit_point(origin: Cardinal, route: Route) -> (f64, f64) {
    let exit = exit_direction(origin, route);
    // The exit lane centre is the entry lane centre of the corresponding
    // opposite-direction approach in that exit direction.
    // Derive it from the exit boundary:
    let exit_route_offset = match route {
        // Right turn is a short arc → exits near the kerb of the exit road
        Route::Right => LANE_WIDTH * 0.5,
        // Straight exits in the mirror lane
        Route::Straight => LANE_WIDTH * 1.5,
        // Left turn is a wide arc → exits near the centre line of the exit road
        Route::Left => LANE_WIDTH * 0.5,
    };

    match exit {
        Cardinal::North => (CENTER_X - exit_route_offset, INTERSECTION_TOP),
        Cardinal::South => (CENTER_X + exit_route_offset, INTERSECTION_BOTTOM),
        Cardinal::West  => (INTERSECTION_LEFT,  CENTER_Y + exit_route_offset),
        Cardinal::East  => (INTERSECTION_RIGHT, CENTER_Y - exit_route_offset),
    }
}
