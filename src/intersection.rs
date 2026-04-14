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
//    Right    → x = CENTER_X + LANE_WIDTH * 2.5   (outermost / rightmost)
//    Straight → x = CENTER_X + LANE_WIDTH * 1.5
//    Left     → x = CENTER_X + LANE_WIDTH * 0.5   (innermost)
//
//  North approach (vehicles move ↓, occupy west half of vertical road):
//    Right    → x = CENTER_X - LANE_WIDTH * 2.5
//    Straight → x = CENTER_X - LANE_WIDTH * 1.5
//    Left     → x = CENTER_X - LANE_WIDTH * 0.5
//
//  West approach (vehicles move →, occupy south half of horizontal road):
//    Right    → y = CENTER_Y + LANE_WIDTH * 2.5
//    Straight → y = CENTER_Y + LANE_WIDTH * 1.5
//    Left     → y = CENTER_Y + LANE_WIDTH * 0.5
//
//  East approach (vehicles move ←, occupy north half of horizontal road):
//    Right    → y = CENTER_Y - LANE_WIDTH * 2.5
//    Straight → y = CENTER_Y - LANE_WIDTH * 1.5
//    Left     → y = CENTER_Y - LANE_WIDTH * 0.5
// ─────────────────────────────────────────────────────────────
pub fn lane_center(origin: Cardinal, route: Route) -> (f64, f64) {
    match origin {
        // Vertical approaches: right lane is farther from the center line.
        Cardinal::South => {
            let offset = match route {
                Route::Right => LANE_WIDTH * 2.5,
                Route::Straight => LANE_WIDTH * 1.5,
                Route::Left => LANE_WIDTH * 0.5,
            };
            (CENTER_X + offset, 0.0)
        }
        Cardinal::North => {
            let offset = match route {
                Route::Right => LANE_WIDTH * 2.5,
                Route::Straight => LANE_WIDTH * 1.5,
                Route::Left => LANE_WIDTH * 0.5,
            };
            (CENTER_X - offset, 0.0)
        }
        Cardinal::West => {
            let offset = match route {
                Route::Right => LANE_WIDTH * 2.5,
                Route::Straight => LANE_WIDTH * 1.5,
                Route::Left => LANE_WIDTH * 0.5,
            };
            (0.0, CENTER_Y + offset)
        }
        Cardinal::East => {
            let offset = match route {
                Route::Right => LANE_WIDTH * 2.5,
                Route::Straight => LANE_WIDTH * 1.5,
                Route::Left => LANE_WIDTH * 0.5,
            };
            (0.0, CENTER_Y - offset)
        }
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
        (Cardinal::South, Route::Right)    => Cardinal::East,
        (Cardinal::South, Route::Left)     => Cardinal::West,

        (Cardinal::North, Route::Straight) => Cardinal::South,
        (Cardinal::North, Route::Right)    => Cardinal::West,
        (Cardinal::North, Route::Left)     => Cardinal::East,

        (Cardinal::West,  Route::Straight) => Cardinal::East,
        (Cardinal::West,  Route::Right)    => Cardinal::South,
        (Cardinal::West,  Route::Left)     => Cardinal::North,

        (Cardinal::East,  Route::Straight) => Cardinal::West,
        (Cardinal::East,  Route::Right)    => Cardinal::North,
        (Cardinal::East,  Route::Left)     => Cardinal::South,
    }
}

/// Which cardinal direction a vehicle travels while approaching the intersection.
pub fn travel_direction(origin: Cardinal) -> Cardinal {
    match origin {
        Cardinal::South => Cardinal::North,
        Cardinal::North => Cardinal::South,
        Cardinal::West => Cardinal::East,
        Cardinal::East => Cardinal::West,
    }
}

fn opposite_cardinal(direction: Cardinal) -> Cardinal {
    match direction {
        Cardinal::North => Cardinal::South,
        Cardinal::South => Cardinal::North,
        Cardinal::East => Cardinal::West,
        Cardinal::West => Cardinal::East,
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
    let exit_origin = opposite_cardinal(exit);
    let (cx, cy) = lane_center(exit_origin, route);

    match exit {
        Cardinal::North => (cx, INTERSECTION_TOP),
        Cardinal::South => (cx, INTERSECTION_BOTTOM),
        Cardinal::West  => (INTERSECTION_LEFT, cy),
        Cardinal::East  => (INTERSECTION_RIGHT, cy),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn north_south_lane_centers_match_turn_intent() {
        let (sx_r, _) = lane_center(Cardinal::South, Route::Right);
        let (sx_l, _) = lane_center(Cardinal::South, Route::Left);
        let (nx_r, _) = lane_center(Cardinal::North, Route::Right);
        let (nx_l, _) = lane_center(Cardinal::North, Route::Left);
        let (_, wy_r) = lane_center(Cardinal::West, Route::Right);
        let (_, wy_l) = lane_center(Cardinal::West, Route::Left);
        let (_, ey_r) = lane_center(Cardinal::East, Route::Right);
        let (_, ey_l) = lane_center(Cardinal::East, Route::Left);

        assert!(sx_r > sx_l);
        assert!(nx_r < nx_l);
        assert!(wy_r > wy_l);
        assert!(ey_r < ey_l);
    }

    #[test]
    fn exit_direction_matches_vehicle_heading_right_left() {
        assert_eq!(exit_direction(Cardinal::South, Route::Right), Cardinal::East);
        assert_eq!(exit_direction(Cardinal::South, Route::Left), Cardinal::West);

        assert_eq!(exit_direction(Cardinal::North, Route::Right), Cardinal::West);
        assert_eq!(exit_direction(Cardinal::North, Route::Left), Cardinal::East);

        assert_eq!(exit_direction(Cardinal::West, Route::Right), Cardinal::South);
        assert_eq!(exit_direction(Cardinal::West, Route::Left), Cardinal::North);

        assert_eq!(exit_direction(Cardinal::East, Route::Right), Cardinal::North);
        assert_eq!(exit_direction(Cardinal::East, Route::Left), Cardinal::South);
    }

    #[test]
    fn turn_exit_point_uses_matching_exit_lane_center() {
        let (x, y) = turn_exit_point(Cardinal::North, Route::Right);
        let (_, expected_y) = lane_center(Cardinal::East, Route::Right);

        assert!(approx_eq(x, INTERSECTION_LEFT));
        assert!(approx_eq(y, expected_y));
    }
}
