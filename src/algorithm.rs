use crate::intersection::*;
use crate::route::{Cardinal, Route};
use crate::vehicle::{Vehicle, CREEP_SPEED, SAFE_DISTANCE, SLOW_SPEED};

const LOOKAHEAD_SECS: f64 = 1.5;
const TRAJECTORY_STEPS: usize = 20;

// How close two vehicle centres must be (px) for the trajectory check to consider it a collision.
// Lower = cars pass closer before braking (more close calls, more efficient).
// Raise = cars give more clearance (safer but slower throughput).
const COLLISION_RADIUS: f64 = 40.0;

// Time-to-collision thresholds (seconds).
// TTC below STOP_TTC  → hard stop.
// TTC below CREEP_TTC → creep forward (car anticipates path clearing and stays ready to go).
// TTC below SLOW_TTC  → slow down.
// Lower values = more aggressive / more close calls. Raise for more conservative behaviour.
const TTC_STOP: f64  = 0.35;
const TTC_CREEP: f64 = 0.60;
const TTC_SLOW: f64  = 1.00;

pub struct SmartIntersection {
    pub close_calls: u32,
}

impl SmartIntersection {
    pub fn new() -> Self {
        SmartIntersection { close_calls: 0 }
    }

    pub fn update(&mut self, vehicles: &mut Vec<Vehicle>, dt: f64, elapsed: f64) {
        let snapshot: Vec<Vehicle> = vehicles.clone();
        let len = vehicles.len();

        for i in 0..len {
            if vehicles[i].removed {
                continue;
            }

            let mut should_slow = false;
            let mut should_creep = false;
            let mut should_stop = false;

            let traj_i = project_trajectory(&vehicles[i]);

            for j in 0..len {
                if i == j || vehicles[j].removed {
                    continue;
                }

                let dist = vehicles[i].distance_to(&snapshot[j]);

                // Same lane: maintain safe distance behind vehicle ahead (approach only)
                if vehicles[i].is_same_lane(&snapshot[j])
                    && !snapshot[j].passed_intersection
                    && !vehicles[i].passed_intersection
                    && snapshot[j].is_ahead_of(&vehicles[i])
                {
                    if dist < SAFE_DISTANCE {
                        should_stop = true;
                    } else if dist < SAFE_DISTANCE * 1.5 {
                        should_slow = true;
                    }
                }

                // Trajectory projection — one-sided: only the lower-priority vehicle yields,
                // the higher-priority one continues unaffected (prevents deadlocks)
                if vehicles[i].entered_intersection
                    && !vehicles[i].passed_intersection
                    && snapshot[j].entered_intersection
                    && !snapshot[j].passed_intersection
                    && !vehicles[i].is_same_lane(&snapshot[j])
                    && paths_conflict(&vehicles[i], &snapshot[j])
                {
                    let i_yields = lower_priority(&vehicles[i], &snapshot[j]);
                    if i_yields {
                        let traj_j = project_trajectory(&snapshot[j]);
                        if let Some(step) = first_collision_step(&traj_i, &traj_j, COLLISION_RADIUS) {
                            let ttc = step as f64 * (LOOKAHEAD_SECS / TRAJECTORY_STEPS as f64);
                            if ttc < TTC_STOP {
                                should_stop = true;
                            } else if ttc < TTC_CREEP {
                                should_creep = true;
                            } else if ttc < TTC_SLOW {
                                should_slow = true;
                            }
                        }
                    }
                }

                // Exit-lane following — one-sided: only the vehicle behind yields to the one ahead.
                // Also checks lateral alignment so vehicles in adjacent exit lanes (different x/y)
                // don't incorrectly slow each other down.
                if vehicles[i].entered_intersection && snapshot[j].passed_intersection {
                    let exit_i = exit_direction(vehicles[i].origin, vehicles[i].route);
                    let exit_j = exit_direction(snapshot[j].origin, snapshot[j].route);
                    if exit_i == exit_j {
                        let j_is_ahead = match exit_i {
                            Cardinal::North => snapshot[j].y < vehicles[i].y,
                            Cardinal::South => snapshot[j].y > vehicles[i].y,
                            Cardinal::East  => snapshot[j].x > vehicles[i].x,
                            Cardinal::West  => snapshot[j].x < vehicles[i].x,
                        };
                        // Same lane = within one lane-width laterally on the exit road
                        let same_lane = match exit_i {
                            Cardinal::North | Cardinal::South =>
                                (snapshot[j].x - vehicles[i].x).abs() < LANE_WIDTH,
                            Cardinal::East | Cardinal::West =>
                                (snapshot[j].y - vehicles[i].y).abs() < LANE_WIDTH,
                        };
                        if j_is_ahead && same_lane && dist < SAFE_DISTANCE * 2.0 {
                            if dist < SAFE_DISTANCE {
                                should_stop = true;
                            } else {
                                should_slow = true;
                            }
                        }
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
            if !vehicles[i].turning && !vehicles[i].passed_intersection
                && vehicles[i].route != Route::Right
            {
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
            } else if should_creep {
                vehicles[i].velocity = CREEP_SPEED;
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

            // Stamp arrival time the first time a vehicle enters the detection zone
            let is_detected = vehicles[i].entered_intersection
                || is_approaching_intersection(vehicles[i].x, vehicles[i].y, vehicles[i].origin)
                || is_near_intersection_entry(vehicles[i].x, vehicles[i].y, vehicles[i].origin);

            if is_detected {
                if vehicles[i].approach_time == f64::MAX {
                    vehicles[i].approach_time = elapsed;
                }
                if !vehicles[i].passed_intersection {
                    vehicles[i].time_in_intersection += dt;
                }
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

// Estimated seconds until this vehicle reaches the stop line.
// Uses a speed floor so a stopped car near the line still beats a distant mover.
fn time_to_stop_line(v: &Vehicle) -> f64 {
    let dist = match v.origin {
        Cardinal::South => (v.y - (INTERSECTION_BOTTOM + 40.0)).max(0.0),
        Cardinal::North => ((INTERSECTION_TOP    - 40.0) - v.y).max(0.0),
        Cardinal::West  => ((INTERSECTION_LEFT   - 40.0) - v.x).max(0.0),
        Cardinal::East  => (v.x - (INTERSECTION_RIGHT + 40.0)).max(0.0),
    };
    let speed = v.velocity.max(v.base_velocity * 0.15);
    dist / speed
}

// True when `a` should yield to `b`.
// Primary: whoever arrives at the stop line sooner has right of way.
// Secondary (ETAs within 0.1 s): earlier approach_time wins to avoid oscillation.
// Tertiary: lower ID.
fn lower_priority(a: &Vehicle, b: &Vehicle) -> bool {
    let eta_a = time_to_stop_line(a);
    let eta_b = time_to_stop_line(b);
    if (eta_a - eta_b).abs() > 0.1 {
        eta_a > eta_b
    } else if a.approach_time != b.approach_time {
        a.approach_time > b.approach_time
    } else {
        a.id > b.id
    }
}

fn should_yield(vehicle: &Vehicle, others: &[Vehicle]) -> bool {
    for other in others {
        if other.id == vehicle.id || other.removed { continue; }
        if other.origin == vehicle.origin { continue; }
        if !paths_conflict(vehicle, other) { continue; }

        let other_in = is_in_intersection(other.x, other.y) || other.turning;
        let i_in = is_in_intersection(vehicle.x, vehicle.y);

        if other_in && !i_in { return true; }

        let other_approaching = is_approaching_intersection(other.x, other.y, other.origin)
            || is_near_intersection_entry(other.x, other.y, other.origin);

        if !other_in && other_approaching {
            if lower_priority(vehicle, other) { return true; }
        }
    }
    false
}

fn paths_conflict(a: &Vehicle, b: &Vehicle) -> bool {
    if a.origin == b.origin { return false; }

    let a_exit = exit_direction(a.origin, a.route);
    let b_exit = exit_direction(b.origin, b.route);

    // Vehicles merging onto the same exit road always conflict
    if a_exit == b_exit { return true; }

    if are_opposite(a.origin, b.origin) {
        // Both straight: travel parallel on opposite sides, no crossing
        if a.route == Route::Straight && b.route == Route::Straight { return false; }
        // Both right: short arcs that don't reach center
        if a.route == Route::Right && b.route == Route::Right { return false; }
        // Everything else (one left, mixed straight/right, any left) crosses
        return true;
    }

    // Perpendicular approaches from here down
    // Both right: each arc stays in its own quadrant
    if a.route == Route::Right && b.route == Route::Right { return false; }

    // One right, one straight: no conflict only if the right-turner
    // exits away from the straight vehicle's path
    if a.route == Route::Right && b.route == Route::Straight {
        if a_exit != b.origin { return false; }
    }
    if b.route == Route::Right && a.route == Route::Straight {
        if b_exit != a.origin { return false; }
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

    let start = (vehicle.turn_start_x, vehicle.turn_start_y);
    let end   = (vehicle.turn_end_x,   vehicle.turn_end_y);

    let (control_1, control_2, path_length) =
        turn_bezier_params(start, end, vehicle.route, entry_dir, exit_dir, vehicle.turn_radius);
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
        vehicle.angle = heading_for_cardinal(exit_dir);
        return;
    }

    let t = vehicle.turn_progress;
    let (x, y) = cubic_bezier_point(start, control_1, control_2, end, t);
    vehicle.x = x;
    vehicle.y = y;

    let start_heading = heading_for_cardinal(entry_dir);
    let turn_delta = match vehicle.route {
        Route::Right => 90.0,
        Route::Left => -90.0,
        Route::Straight => 0.0,
    };
    vehicle.angle = (start_heading + turn_delta * t).rem_euclid(360.0);
}

fn cardinal_vector(direction: Cardinal) -> (f64, f64) {
    match direction {
        Cardinal::North => (0.0, -1.0),
        Cardinal::South => (0.0, 1.0),
        Cardinal::East => (1.0, 0.0),
        Cardinal::West => (-1.0, 0.0),
    }
}

fn heading_for_cardinal(direction: Cardinal) -> f64 {
    match direction {
        Cardinal::North => 0.0,
        Cardinal::East => 90.0,
        Cardinal::South => 180.0,
        Cardinal::West => 270.0,
    }
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

fn distance(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    let dx = x2 - x1;
    let dy = y2 - y1;
    (dx * dx + dy * dy).sqrt()
}

// Returns (control_1, control_2, path_length) for a turning arc.
//
// Right turns use corner-based control points (the geometric corner where the entry
// and exit lane lines meet).  This produces an outward quarter-circle arc that looks
// natural.  The standard "start + entry_dir * r / end - exit_dir * r" formula works
// for left turns but creates an inward arc for the short right turns in the rightmost
// lane, so they get separate treatment.
fn turn_bezier_params(
    start: (f64, f64),
    end: (f64, f64),
    route: Route,
    entry_dir: Cardinal,
    exit_dir: Cardinal,
    turn_radius: f64,
) -> ((f64, f64), (f64, f64), f64) {
    if route == Route::Right {
        // Corner = intersection of the entry-lane line and the exit-lane line.
        // For a vertical entry (North/South) the entry line is x = start.0;
        // the exit line is y = end.1.  Horizontal entry is the mirror.
        let (corner_x, corner_y) = match entry_dir {
            Cardinal::North | Cardinal::South => (start.0, end.1),
            Cardinal::East  | Cardinal::West  => (end.0,   start.1),
        };
        // alpha = 0.5523 is the exact cubic-Bezier quarter-circle approximation;
        // raising it slightly (0.65) makes the arc look rounder and more natural.
        let alpha = 0.65_f64;
        let c1 = (
            start.0 + alpha * (corner_x - start.0),
            start.1 + alpha * (corner_y - start.1),
        );
        let c2 = (
            end.0 + alpha * (corner_x - end.0),
            end.1 + alpha * (corner_y - end.1),
        );
        // Arc length of a quarter-circle ≈ (π/2) * radius, where radius is the
        // leg from the corner to either endpoint.
        let radius = distance(start.0, start.1, corner_x, corner_y);
        let path_length = (std::f64::consts::FRAC_PI_2 * radius).max(20.0);
        (c1, c2, path_length)
    } else {
        let (entry_dx, entry_dy) = cardinal_vector(entry_dir);
        let (exit_dx, exit_dy) = cardinal_vector(exit_dir);
        let control_len = turn_radius.max(LANE_WIDTH * 0.75);
        let c1 = (start.0 + entry_dx * control_len, start.1 + entry_dy * control_len);
        let c2 = (end.0   - exit_dx  * control_len, end.1   - exit_dy  * control_len);
        let path_length = distance(start.0, start.1, end.0, end.1).max(control_len * 2.0);
        (c1, c2, path_length)
    }
}

fn project_trajectory(vehicle: &Vehicle) -> Vec<(f64, f64)> {
    let step_dt = LOOKAHEAD_SECS / TRAJECTORY_STEPS as f64;
    let speed = vehicle.velocity;

    if speed <= 0.0 {
        return vec![(vehicle.x, vehicle.y); TRAJECTORY_STEPS];
    }

    let mut points = Vec::with_capacity(TRAJECTORY_STEPS);

    if vehicle.turning {
        let entry_dir = travel_direction(vehicle.origin);
        let exit_dir = exit_direction(vehicle.origin, vehicle.route);

        let start = (vehicle.turn_start_x, vehicle.turn_start_y);
        let end   = (vehicle.turn_end_x,   vehicle.turn_end_y);

        let (c1, c2, path_length) =
            turn_bezier_params(start, end, vehicle.route, entry_dir, exit_dir, vehicle.turn_radius);

        let mut progress = vehicle.turn_progress;
        let progress_per_step = speed * step_dt / path_length;

        for _ in 0..TRAJECTORY_STEPS {
            progress += progress_per_step;
            if progress >= 1.0 {
                let overshoot = (progress - 1.0) * path_length;
                let (dx, dy) = cardinal_vector(exit_dir);
                points.push((end.0 + dx * overshoot, end.1 + dy * overshoot));
            } else {
                points.push(cubic_bezier_point(start, c1, c2, end, progress));
            }
        }
    } else if vehicle.passed_intersection {
        let exit_dir = exit_direction(vehicle.origin, vehicle.route);
        let (dx, dy) = cardinal_vector(exit_dir);
        for step in 1..=TRAJECTORY_STEPS {
            let t = step as f64 * step_dt;
            points.push((vehicle.x + dx * speed * t, vehicle.y + dy * speed * t));
        }
    } else {
        // Approaching or going straight through the intersection
        let dir = travel_direction(vehicle.origin);
        let (dx, dy) = cardinal_vector(dir);
        for step in 1..=TRAJECTORY_STEPS {
            let t = step as f64 * step_dt;
            points.push((vehicle.x + dx * speed * t, vehicle.y + dy * speed * t));
        }
    }

    points
}

fn first_collision_step(a: &[(f64, f64)], b: &[(f64, f64)], radius: f64) -> Option<usize> {
    let threshold_sq = (radius * 2.0).powi(2);
    for (step, ((ax, ay), (bx, by))) in a.iter().zip(b.iter()).enumerate() {
        let dist_sq = (ax - bx).powi(2) + (ay - by).powi(2);
        if dist_sq < threshold_sq {
            return Some(step);
        }
    }
    None
}
