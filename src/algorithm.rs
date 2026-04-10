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
                if vehicles[i].velocity < vehicles[i].min_velocity || vehicles[i].min_velocity == vehicles[i].base_velocity {
                    vehicles[i].min_velocity = 0.0;
                }
            } else if should_slow {
                vehicles[i].velocity = SLOW_SPEED * 0.5;
                if vehicles[i].velocity < vehicles[i].min_velocity {
                    vehicles[i].min_velocity = vehicles[i].velocity;
                }
            } else {
                // Restore to base velocity
                vehicles[i].velocity = vehicles[i].base_velocity;
            }

            // Track max velocity
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
            Route::Straight => {
                // Continue straight
            }
            Route::Right => {
                setup_right_turn(vehicle);
            }
            Route::Left => {
                setup_left_turn(vehicle);
            }
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

fn setup_right_turn(vehicle: &mut Vehicle) {
    vehicle.turning = true;
    vehicle.turn_progress = 0.0;

    let r = LANE_WIDTH * 0.8;
    vehicle.turn_radius = r;

    match vehicle.origin {
        Cardinal::South => {
            vehicle.turn_center_x = vehicle.x - r;
            vehicle.turn_center_y = vehicle.y;
            vehicle.turn_start_angle = 0.0_f64.to_radians();
            vehicle.turn_end_angle = -90.0_f64.to_radians();
        }
        Cardinal::North => {
            vehicle.turn_center_x = vehicle.x + r;
            vehicle.turn_center_y = vehicle.y;
            vehicle.turn_start_angle = 180.0_f64.to_radians();
            vehicle.turn_end_angle = 90.0_f64.to_radians();
        }
        Cardinal::West => {
            vehicle.turn_center_x = vehicle.x;
            vehicle.turn_center_y = vehicle.y - r;
            vehicle.turn_start_angle = 90.0_f64.to_radians();
            vehicle.turn_end_angle = 0.0_f64.to_radians();
        }
        Cardinal::East => {
            vehicle.turn_center_x = vehicle.x;
            vehicle.turn_center_y = vehicle.y + r;
            vehicle.turn_start_angle = 270.0_f64.to_radians();
            vehicle.turn_end_angle = 180.0_f64.to_radians();
        }
    }
}

fn setup_left_turn(vehicle: &mut Vehicle) {
    vehicle.turning = true;
    vehicle.turn_progress = 0.0;

    let r = LANE_WIDTH * 2.0;
    vehicle.turn_radius = r;

    match vehicle.origin {
        Cardinal::South => {
            vehicle.turn_center_x = vehicle.x + r;
            vehicle.turn_center_y = vehicle.y;
            vehicle.turn_start_angle = 180.0_f64.to_radians();
            vehicle.turn_end_angle = 270.0_f64.to_radians();
        }
        Cardinal::North => {
            vehicle.turn_center_x = vehicle.x - r;
            vehicle.turn_center_y = vehicle.y;
            vehicle.turn_start_angle = 0.0_f64.to_radians();
            vehicle.turn_end_angle = 90.0_f64.to_radians();
        }
        Cardinal::West => {
            vehicle.turn_center_x = vehicle.x;
            vehicle.turn_center_y = vehicle.y + r;
            vehicle.turn_start_angle = 270.0_f64.to_radians();
            vehicle.turn_end_angle = 360.0_f64.to_radians();
        }
        Cardinal::East => {
            vehicle.turn_center_x = vehicle.x;
            vehicle.turn_center_y = vehicle.y - r;
            vehicle.turn_start_angle = 90.0_f64.to_radians();
            vehicle.turn_end_angle = 0.0_f64.to_radians();
        }
    }
}

fn update_turn(vehicle: &mut Vehicle, dt: f64) {
    let speed = vehicle.velocity * dt;
    let arc_length = vehicle.turn_radius * (vehicle.turn_end_angle - vehicle.turn_start_angle).abs();
    if arc_length <= 0.0 {
        vehicle.turning = false;
        vehicle.passed_intersection = true;
        return;
    }
    let progress_increment = speed / arc_length;
    vehicle.turn_progress += progress_increment;

    if vehicle.turn_progress >= 1.0 {
        vehicle.turn_progress = 1.0;
        vehicle.turning = false;
        vehicle.passed_intersection = true;

        // Set final direction angle
        let exit_dir = exit_direction(vehicle.origin, vehicle.route);
        vehicle.angle = match exit_dir {
            Cardinal::North => 0.0,
            Cardinal::South => 180.0,
            Cardinal::East => 270.0,
            Cardinal::West => 90.0,
        };

        // Position at turn end
        let t = vehicle.turn_progress;
        let current_angle = vehicle.turn_start_angle + t * (vehicle.turn_end_angle - vehicle.turn_start_angle);
        vehicle.x = vehicle.turn_center_x + vehicle.turn_radius * current_angle.cos();
        vehicle.y = vehicle.turn_center_y - vehicle.turn_radius * current_angle.sin();
        return;
    }

    let t = vehicle.turn_progress;
    let current_angle = vehicle.turn_start_angle + t * (vehicle.turn_end_angle - vehicle.turn_start_angle);
    vehicle.x = vehicle.turn_center_x + vehicle.turn_radius * current_angle.cos();
    vehicle.y = vehicle.turn_center_y - vehicle.turn_radius * current_angle.sin();

    // Update visual rotation angle
    let tangent_angle = if vehicle.turn_end_angle > vehicle.turn_start_angle {
        current_angle + std::f64::consts::FRAC_PI_2
    } else {
        current_angle - std::f64::consts::FRAC_PI_2
    };
    vehicle.angle = -tangent_angle.to_degrees() + 90.0;
}
