use crate::intersection::*;
use crate::route::{Cardinal, Route};
use crate::vehicle::{Vehicle, SAFE_DISTANCE, SLOW_SPEED};

pub struct SmartIntersection {
    pub close_calls: u32,
}

impl SmartIntersection {
    pub fn new() -> Self {
        SmartIntersection { close_calls: 0 }
    }

    pub fn update(&mut self, vehicles: &mut Vec<Vehicle>, dt: f64) {
        let snapshot: Vec<Vehicle> = vehicles.clone();
        let len = vehicles.len();

        for i in 0..len {
            if vehicles[i].removed {
                continue;
            }

            let mut should_slow = false;
            let mut should_stop = false;

            // Check same-lane vehicles ahead
            for j in 0..len {
                if i == j || vehicles[j].removed {
                    continue;
                }

                let dist = vehicles[i].distance_to(&snapshot[j]);

                // Same lane: maintain safe distance behind vehicle ahead
                if vehicles[i].is_same_lane(&snapshot[j]) && snapshot[j].is_ahead_of(&vehicles[i]) {
                    if dist < SAFE_DISTANCE {
                        should_stop = true;
                    } else if dist < SAFE_DISTANCE * 2.0 {
                        should_slow = true;
                    }
                }

                // Close call detection
                if dist < SAFE_DISTANCE * 0.7 && dist > 5.0
                    && !vehicles[i].is_same_lane(&snapshot[j])
                {
                    if !vehicles[i].close_call {
                        vehicles[i].close_call = true;
                        self.close_calls += 1;
                    }
                }
            }

            // Intersection collision avoidance
            if !vehicles[i].turning && !vehicles[i].passed_intersection {
                if is_approaching_intersection(vehicles[i].x, vehicles[i].y, vehicles[i].origin)
                    || is_near_intersection_entry(vehicles[i].x, vehicles[i].y, vehicles[i].origin)
                {
                    if should_yield(&vehicles[i], &snapshot) {
                        should_slow = true;
                        if is_near_intersection_entry(vehicles[i].x, vehicles[i].y, vehicles[i].origin) {
                            let has_conflict = has_conflicting_vehicle_in_intersection(&vehicles[i], &snapshot);
                            if has_conflict {
                                should_stop = true;
                            }
                        }
                    }
                }
            }

            // Apply velocity changes
            if should_stop {
                vehicles[i].velocity = 0.0;
            } else if should_slow {
                vehicles[i].velocity = SLOW_SPEED * 0.5;
            } else {
                vehicles[i].velocity = vehicles[i].base_velocity;
            }

            // Track velocity extremes after setting the current velocity.
            if vehicles[i].velocity < vehicles[i].min_velocity {
                vehicles[i].min_velocity = vehicles[i].velocity;
            }
            if vehicles[i].velocity > vehicles[i].max_velocity {
                vehicles[i].max_velocity = vehicles[i].velocity;
            }

            // Start timing when the vehicle is detected by the algorithm (approaching)
            let is_detected = vehicles[i].entered_intersection
                || is_approaching_intersection(vehicles[i].x, vehicles[i].y, vehicles[i].origin)
                || is_near_intersection_entry(vehicles[i].x, vehicles[i].y, vehicles[i].origin);

            if is_detected && !vehicles[i].passed_intersection {
                vehicles[i].time_in_intersection += dt;
            }

            // Move the vehicle
            move_vehicle(&mut vehicles[i], dt);
        }
    }
}

fn is_near_intersection_entry(x: f64, y: f64, origin: Cardinal) -> bool {
    let margin = 40.0;
    match origin {
        Cardinal::South => y > INTERSECTION_BOTTOM && y < INTERSECTION_BOTTOM + margin,
        Cardinal::North => y < INTERSECTION_TOP && y > INTERSECTION_TOP - margin,
        Cardinal::West => x < INTERSECTION_LEFT && x > INTERSECTION_LEFT - margin,
        Cardinal::East => x > INTERSECTION_RIGHT && x < INTERSECTION_RIGHT + margin,
    }
}

fn should_yield(vehicle: &Vehicle, others: &[Vehicle]) -> bool {
    for other in others {
        if other.id == vehicle.id || other.removed {
            continue;
        }
        if other.origin == vehicle.origin {
            continue;
        }

        let other_in_intersection = is_in_intersection(other.x, other.y) || other.turning;
        let other_approaching = is_approaching_intersection(other.x, other.y, other.origin)
            || is_near_intersection_entry(other.x, other.y, other.origin);

        if !other_in_intersection && !other_approaching {
            continue;
        }

        if paths_conflict(vehicle, other) {
            // Priority: vehicle already in intersection goes first
            if other_in_intersection && !is_in_intersection(vehicle.x, vehicle.y) {
                return true;
            }
            // If both approaching, use a deterministic priority based on origin and id
            if other_approaching && !other_in_intersection {
                let my_priority = origin_priority(vehicle.origin);
                let their_priority = origin_priority(other.origin);
                if my_priority == their_priority {
                    if vehicle.id > other.id {
                        return true;
                    }
                } else if my_priority > their_priority {
                    return true;
                }
            }
        }
    }
    false
}

fn origin_priority(origin: Cardinal) -> u8 {
    // Right-hand rule inspired priority
    match origin {
        Cardinal::South => 0,
        Cardinal::West => 1,
        Cardinal::North => 2,
        Cardinal::East => 3,
    }
}

fn paths_conflict(a: &Vehicle, b: &Vehicle) -> bool {
    let a_exit = exit_direction(a.origin, a.route);
    let b_exit = exit_direction(b.origin, b.route);

    // Same entry or exit means potential conflict
    if a.origin == b.origin {
        return false; // Same direction, no crossing
    }

    // Check if paths cross inside the intersection
    // Opposite directions with crossing routes
    if are_opposite(a.origin, b.origin) {
        // Straight vs straight: no conflict (different lanes)
        if a.route == Route::Straight && b.route == Route::Straight {
            return false;
        }
        // Right turns typically don't conflict with opposite right turns
        if a.route == Route::Right && b.route == Route::Right {
            return false;
        }
        return true;
    }

    // Perpendicular: almost always conflict unless both turning right away from each other
    if a.route == Route::Right && a_exit != b.origin && b.route == Route::Right && b_exit != a.origin {
        return false;
    }

    true
}

fn are_opposite(a: Cardinal, b: Cardinal) -> bool {
    matches!(
        (a, b),
        (Cardinal::North, Cardinal::South)
            | (Cardinal::South, Cardinal::North)
            | (Cardinal::East, Cardinal::West)
            | (Cardinal::West, Cardinal::East)
    )
}

fn has_conflicting_vehicle_in_intersection(vehicle: &Vehicle, others: &[Vehicle]) -> bool {
    for other in others {
        if other.id == vehicle.id || other.removed {
            continue;
        }
        if (is_in_intersection(other.x, other.y) || other.turning) && paths_conflict(vehicle, other)
        {
            return true;
        }
    }
    false
}

fn move_vehicle(vehicle: &mut Vehicle, dt: f64) {
    if vehicle.removed || vehicle.velocity == 0.0 {
        return;
    }

    let speed = vehicle.velocity * dt;
    vehicle.distance_traveled += speed;

    let in_intersection = is_in_intersection(vehicle.x, vehicle.y);
    if in_intersection && !vehicle.entered_intersection {
        vehicle.entered_intersection = true;
    }

    // Determine if vehicle should begin turning
    if in_intersection && !vehicle.turning && !vehicle.passed_intersection {
        match vehicle.route {
            Route::Straight => {}
            Route::Right | Route::Left => setup_turn(vehicle),
        }
    }

    if vehicle.turning {
        update_turn(vehicle, dt);
    } else if vehicle.passed_intersection {
        // Move in exit direction after completing turn or passing straight through
        let exit_dir = exit_direction(vehicle.origin, vehicle.route);
        match exit_dir {
            Cardinal::North => vehicle.y -= speed,
            Cardinal::South => vehicle.y += speed,
            Cardinal::East => vehicle.x += speed,
            Cardinal::West => vehicle.x -= speed,
        }
    } else {
        // Move straight in entry direction (approaching or inside intersection)
        match vehicle.origin {
            Cardinal::South => vehicle.y -= speed,
            Cardinal::North => vehicle.y += speed,
            Cardinal::West => vehicle.x += speed,
            Cardinal::East => vehicle.x -= speed,
        }

        // For straight route: mark passed once exiting the intersection box
        if vehicle.entered_intersection && vehicle.route == Route::Straight {
            if !is_in_intersection(vehicle.x, vehicle.y) {
                vehicle.passed_intersection = true;
            }
        }
    }

    if has_left_screen(vehicle.x, vehicle.y) {
        vehicle.removed = true;
        vehicle.passed_intersection = true;
    }
}

fn setup_turn(vehicle: &mut Vehicle) {
    vehicle.turning = true;
    vehicle.turn_progress = 0.0;

    let (start_x, start_y) = turn_entry_point(vehicle.origin, vehicle.route);
    vehicle.turn_start_x = start_x;
    vehicle.turn_start_y = start_y;
    vehicle.x = start_x;
    vehicle.y = start_y;

    let (end_x, end_y) = turn_exit_point(vehicle.origin, vehicle.route);
    vehicle.turn_end_x = end_x;
    vehicle.turn_end_y = end_y;
    vehicle.turn_radius = LANE_WIDTH * 1.1;
}

fn update_turn(vehicle: &mut Vehicle, dt: f64) {
    let speed = vehicle.velocity * dt;
    let entry_dir = travel_direction(vehicle.origin);
    let exit_dir = exit_direction(vehicle.origin, vehicle.route);
    let (entry_dx, entry_dy) = cardinal_vector(entry_dir);
    let (exit_dx, exit_dy) = cardinal_vector(exit_dir);

    let start = (vehicle.turn_start_x, vehicle.turn_start_y);
    let end = (vehicle.turn_end_x, vehicle.turn_end_y);
    let control_len = vehicle.turn_radius.max(LANE_WIDTH * 0.75);

    let control_1 = (
        start.0 + entry_dx * control_len,
        start.1 + entry_dy * control_len,
    );
    let control_2 = (
        end.0 - exit_dx * control_len,
        end.1 - exit_dy * control_len,
    );

    let path_length = distance(start.0, start.1, end.0, end.1).max(control_len * 2.0);
    if path_length <= 0.0 {
        vehicle.turning = false;
        vehicle.passed_intersection = true;
        return;
    }

    let progress_increment = speed / path_length;
    vehicle.turn_progress += progress_increment;

    if vehicle.turn_progress >= 1.0 {
        vehicle.turn_progress = 1.0;
        vehicle.turning = false;
        vehicle.passed_intersection = true;

        vehicle.x = end.0;
        vehicle.y = end.1;
        vehicle.angle = heading_from_vector(exit_dx, exit_dy);
        return;
    }

    let t = vehicle.turn_progress;
    let (x, y) = cubic_bezier_point(start, control_1, control_2, end, t);
    let (dx, dy) = cubic_bezier_derivative(start, control_1, control_2, end, t);
    vehicle.x = x;
    vehicle.y = y;
    vehicle.angle = heading_from_vector(dx, dy);
}

fn cardinal_vector(direction: Cardinal) -> (f64, f64) {
    match direction {
        Cardinal::North => (0.0, -1.0),
        Cardinal::South => (0.0, 1.0),
        Cardinal::East => (1.0, 0.0),
        Cardinal::West => (-1.0, 0.0),
    }
}

fn heading_from_vector(dx: f64, dy: f64) -> f64 {
    if dx.abs() < f64::EPSILON && dy.abs() < f64::EPSILON {
        return 0.0;
    }

    dx.atan2(-dy).to_degrees().rem_euclid(360.0)
}

fn cubic_bezier_point(
    start: (f64, f64),
    control_1: (f64, f64),
    control_2: (f64, f64),
    end: (f64, f64),
    t: f64,
) -> (f64, f64) {
    let one_minus_t = 1.0 - t;
    let one_minus_t_sq = one_minus_t * one_minus_t;
    let t_sq = t * t;

    let x = one_minus_t_sq * one_minus_t * start.0
        + 3.0 * one_minus_t_sq * t * control_1.0
        + 3.0 * one_minus_t * t_sq * control_2.0
        + t_sq * t * end.0;
    let y = one_minus_t_sq * one_minus_t * start.1
        + 3.0 * one_minus_t_sq * t * control_1.1
        + 3.0 * one_minus_t * t_sq * control_2.1
        + t_sq * t * end.1;

    (x, y)
}

fn cubic_bezier_derivative(
    start: (f64, f64),
    control_1: (f64, f64),
    control_2: (f64, f64),
    end: (f64, f64),
    t: f64,
) -> (f64, f64) {
    let one_minus_t = 1.0 - t;

    let x = 3.0 * one_minus_t * one_minus_t * (control_1.0 - start.0)
        + 6.0 * one_minus_t * t * (control_2.0 - control_1.0)
        + 3.0 * t * t * (end.0 - control_2.0);
    let y = 3.0 * one_minus_t * one_minus_t * (control_1.1 - start.1)
        + 6.0 * one_minus_t * t * (control_2.1 - control_1.1)
        + 3.0 * t * t * (end.1 - control_2.1);

    (x, y)
}

fn distance(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    let dx = x2 - x1;
    let dy = y2 - y1;
    (dx * dx + dy * dy).sqrt()
}
