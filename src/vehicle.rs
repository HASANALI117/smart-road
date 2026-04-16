use crate::intersection::stop_line_pos;
use crate::route::{Cardinal, Route};

#[derive(Clone)]
pub struct Vehicle {
    pub id: u64,
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
    pub angle: f64,
    pub origin: Cardinal,
    pub route: Route,
    pub velocity: f64,
    pub max_velocity: f64,
    pub min_velocity: f64,
    pub base_velocity: f64,
    pub distance_traveled: f64,
    pub time_in_intersection: f64,
    pub entered_intersection: bool,
    pub passed_intersection: bool,
    pub removed: bool,
    pub turning: bool,
    pub turn_progress: f64,
    pub turn_radius: f64,
    pub turn_start_x: f64,
    pub turn_start_y: f64,
    pub turn_end_x: f64,
    pub turn_end_y: f64,
    pub close_call: bool,
    pub approach_time: f64,
    pub stop_line_x: f64,
    pub stop_line_y: f64,
    pub sprite_index: usize,
    pub color_index: usize,
}

pub const VEHICLE_W: f64 = 26.0;
pub const VEHICLE_H: f64 = 52.0;

pub const CREEP_SPEED: f64 = 20.0;   // inches forward when a collision is predicted very soon
pub const SLOW_SPEED: f64 = 80.0;
pub const NORMAL_SPEED: f64 = 160.0;
pub const FAST_SPEED: f64 = 240.0;

pub const SAFE_DISTANCE: f64 = 90.0;

impl Vehicle {
    pub fn new(
        id: u64,
        origin: Cardinal,
        route: Route,
        lane_x: f64,
        lane_y: f64,
        sprite_index: usize,
        color_index: usize,
    ) -> Self {
        let speeds = [SLOW_SPEED, NORMAL_SPEED, FAST_SPEED];
        let base = speeds[id as usize % 3];

        // Angle convention: 0° = facing up (north), clockwise positive (SDL2).
        // South-origin vehicles travel north (up)  → 0°
        // North-origin vehicles travel south (down)→ 180°
        // West-origin  vehicles travel east (right)→ 90°
        // East-origin  vehicles travel west (left) → 270°
        let angle = match origin {
            Cardinal::South => 0.0,
            Cardinal::North => 180.0,
            Cardinal::West  => 90.0,
            Cardinal::East  => 270.0,
        };
        let (x, y) = (lane_x, lane_y);
        let (slx, sly) = stop_line_pos(origin, route);

        Vehicle {
            id,
            x,
            y,
            width: VEHICLE_W,
            height: VEHICLE_H,
            angle,
            origin,
            route,
            velocity: base,
            max_velocity: base,
            min_velocity: f64::MAX,
            base_velocity: base,
            distance_traveled: 0.0,
            time_in_intersection: 0.0,
            entered_intersection: false,
            passed_intersection: false,
            removed: false,
            turning: false,
            turn_progress: 0.0,
            turn_radius: 0.0,
            turn_start_x: 0.0,
            turn_start_y: 0.0,
            turn_end_x: 0.0,
            turn_end_y: 0.0,
            close_call: false,
            approach_time: f64::MAX,
            stop_line_x: slx,
            stop_line_y: sly,
            sprite_index,
            color_index,
        }
    }

    pub fn distance_to(&self, other: &Vehicle) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn is_ahead_of(&self, other: &Vehicle) -> bool {
        match self.origin {
            Cardinal::South => self.y < other.y,
            Cardinal::North => self.y > other.y,
            Cardinal::West => self.x > other.x,
            Cardinal::East => self.x < other.x,
        }
    }

    pub fn is_same_lane(&self, other: &Vehicle) -> bool {
        self.origin == other.origin && self.route == other.route
    }
}
