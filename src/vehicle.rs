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
    pub turn_center_x: f64,
    pub turn_center_y: f64,
    pub turn_start_angle: f64,
    pub turn_end_angle: f64,
    pub turn_radius: f64,
    pub close_call: bool,
    pub color_index: usize,
}

pub const VEHICLE_W: f64 = 30.0;
pub const VEHICLE_H: f64 = 50.0;

pub const SLOW_SPEED: f64 = 80.0;
pub const NORMAL_SPEED: f64 = 160.0;
pub const FAST_SPEED: f64 = 240.0;

pub const SAFE_DISTANCE: f64 = 60.0;

impl Vehicle {
    pub fn new(id: u64, origin: Cardinal, route: Route, lane_x: f64, lane_y: f64, color_index: usize) -> Self {
        let speeds = [SLOW_SPEED, NORMAL_SPEED, FAST_SPEED];
        let base = speeds[id as usize % 3];

        let (x, y, angle) = match origin {
            Cardinal::South => (lane_x, lane_y, 0.0),
            Cardinal::North => (lane_x, lane_y, 180.0),
            Cardinal::West => (lane_x, lane_y, 90.0),
            Cardinal::East => (lane_x, lane_y, 270.0),
        };

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
            min_velocity: base,
            base_velocity: base,
            distance_traveled: 0.0,
            time_in_intersection: 0.0,
            entered_intersection: false,
            passed_intersection: false,
            removed: false,
            turning: false,
            turn_progress: 0.0,
            turn_center_x: 0.0,
            turn_center_y: 0.0,
            turn_start_angle: 0.0,
            turn_end_angle: 0.0,
            turn_radius: 0.0,
            close_call: false,
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
