use crate::route::{Cardinal, Route};

pub const WINDOW_WIDTH: u32 = 1000;
pub const WINDOW_HEIGHT: u32 = 800;

pub const ROAD_WIDTH: f64 = 360.0;
pub const LANE_WIDTH: f64 = 60.0;

pub const CENTER_X: f64 = WINDOW_WIDTH as f64 / 2.0;
pub const CENTER_Y: f64 = WINDOW_HEIGHT as f64 / 2.0;

pub const INTERSECTION_LEFT: f64 = CENTER_X - ROAD_WIDTH / 2.0;
pub const INTERSECTION_RIGHT: f64 = CENTER_X + ROAD_WIDTH / 2.0;
pub const INTERSECTION_TOP: f64 = CENTER_Y - ROAD_WIDTH / 2.0;
pub const INTERSECTION_BOTTOM: f64 = CENTER_Y + ROAD_WIDTH / 2.0;

pub fn spawn_position(origin: Cardinal, route: Route) -> (f64, f64) {
    let (lane_x, lane_y) = lane_position(origin, route);
    match origin {
        Cardinal::South => (lane_x, WINDOW_HEIGHT as f64 + 40.0),
        Cardinal::North => (lane_x, -40.0),
        Cardinal::West => (-40.0, lane_y),
        Cardinal::East => (WINDOW_WIDTH as f64 + 40.0, lane_y),
    }
}

pub fn lane_position(origin: Cardinal, route: Route) -> (f64, f64) {
    match origin {
        Cardinal::South => {
            let base_x = CENTER_X + LANE_WIDTH / 2.0;
            let x = match route {
                Route::Right => base_x,
                Route::Straight => base_x + LANE_WIDTH,
                Route::Left => base_x + LANE_WIDTH * 2.0,
            };
            (x, CENTER_Y)
        }
        Cardinal::North => {
            let base_x = CENTER_X - LANE_WIDTH / 2.0;
            let x = match route {
                Route::Right => base_x,
                Route::Straight => base_x - LANE_WIDTH,
                Route::Left => base_x - LANE_WIDTH * 2.0,
            };
            (x, CENTER_Y)
        }
        Cardinal::West => {
            let base_y = CENTER_Y - LANE_WIDTH / 2.0;
            let y = match route {
                Route::Right => base_y,
                Route::Straight => base_y - LANE_WIDTH,
                Route::Left => base_y - LANE_WIDTH * 2.0,
            };
            (CENTER_X, y)
        }
        Cardinal::East => {
            let base_y = CENTER_Y + LANE_WIDTH / 2.0;
            let y = match route {
                Route::Right => base_y,
                Route::Straight => base_y + LANE_WIDTH,
                Route::Left => base_y + LANE_WIDTH * 2.0,
            };
            (CENTER_X, y)
        }
    }
}

pub fn is_in_intersection(x: f64, y: f64) -> bool {
    x >= INTERSECTION_LEFT && x <= INTERSECTION_RIGHT
        && y >= INTERSECTION_TOP && y <= INTERSECTION_BOTTOM
}

pub fn is_approaching_intersection(x: f64, y: f64, origin: Cardinal) -> bool {
    let margin = 120.0;
    match origin {
        Cardinal::South => y > INTERSECTION_BOTTOM && y < INTERSECTION_BOTTOM + margin,
        Cardinal::North => y < INTERSECTION_TOP && y > INTERSECTION_TOP - margin,
        Cardinal::West => x < INTERSECTION_LEFT && x > INTERSECTION_LEFT - margin,
        Cardinal::East => x > INTERSECTION_RIGHT && x < INTERSECTION_RIGHT + margin,
    }
}

pub fn has_left_screen(x: f64, y: f64) -> bool {
    let margin = 60.0;
    x < -margin || x > WINDOW_WIDTH as f64 + margin
        || y < -margin || y > WINDOW_HEIGHT as f64 + margin
}

pub fn exit_direction(origin: Cardinal, route: Route) -> Cardinal {
    match (origin, route) {
        (Cardinal::South, Route::Straight) => Cardinal::North,
        (Cardinal::South, Route::Right) => Cardinal::West,
        (Cardinal::South, Route::Left) => Cardinal::East,
        (Cardinal::North, Route::Straight) => Cardinal::South,
        (Cardinal::North, Route::Right) => Cardinal::East,
        (Cardinal::North, Route::Left) => Cardinal::West,
        (Cardinal::West, Route::Straight) => Cardinal::East,
        (Cardinal::West, Route::Right) => Cardinal::North,
        (Cardinal::West, Route::Left) => Cardinal::South,
        (Cardinal::East, Route::Straight) => Cardinal::West,
        (Cardinal::East, Route::Right) => Cardinal::South,
        (Cardinal::East, Route::Left) => Cardinal::North,
    }
}
