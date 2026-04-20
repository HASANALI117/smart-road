use crate::intersection::*;
use crate::route::{Cardinal, Route};
use crate::vehicle::{Vehicle, CREEP_SPEED, SAFE_DISTANCE, SLOW_SPEED};

const LOOKAHEAD_SECS: f64 = 1.5;
const TRAJECTORY_STEPS: usize = 20;

// Longer lookahead used when checking whether the stop line is clear to enter.
// Needs to cover the full transit time of a slow left-turner (~2.5 s at min speed).
const CLEAR_LOOKAHEAD_SECS: f64 = 3.0;
const CLEAR_TRAJECTORY_STEPS: usize = 30;

// Lookahead used for the visual trajectory overlay — long enough to show the
// complete route through the intersection and onto the exit road.
const VIS_LOOKAHEAD_SECS: f64 = 4.5;
const VIS_TRAJECTORY_STEPS: usize = 45;

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
            let mut follow_velocity: Option<f64> = None;

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
                    } else if dist < SAFE_DISTANCE * 1.8 {
                        // Blend from leader's speed (at SAFE_DISTANCE) up to base_velocity
                        // (at 1.8× SAFE_DISTANCE) so the follower never hard-stops and
                        // immediately snaps back to full speed — eliminating jitter.
                        let t = (dist - SAFE_DISTANCE) / (SAFE_DISTANCE * 0.8);
                        let blended = snapshot[j].velocity
                            + (vehicles[i].base_velocity - snapshot[j].velocity)
                                * t.clamp(0.0, 1.0);
                        let candidate = blended.max(0.0).min(vehicles[i].base_velocity);
                        follow_velocity = Some(match follow_velocity {
                            Some(prev) => prev.min(candidate),
                            None => candidate,
                        });
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
                    let i_yields = in_intersection_lower_priority(&vehicles[i], &snapshot[j]);
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
                        if j_is_ahead && same_lane {
                            if dist < SAFE_DISTANCE {
                                should_stop = true;
                            } else if dist < SAFE_DISTANCE * 1.8 {
                                let t = (dist - SAFE_DISTANCE) / (SAFE_DISTANCE * 0.8);
                                let blended = snapshot[j].velocity
                                    + (vehicles[i].base_velocity - snapshot[j].velocity)
                                        * t.clamp(0.0, 1.0);
                                let candidate = blended.max(0.0).min(vehicles[i].base_velocity);
                                follow_velocity = Some(match follow_velocity {
                                    Some(prev) => prev.min(candidate),
                                    None => candidate,
                                });
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
            } else if let Some(fv) = follow_velocity {
                vehicles[i].velocity = fv;
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
                if vehicles[i].entered_intersection && vehicles[i].entry_time == f64::MAX {
                    vehicles[i].entry_time = elapsed;
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

// True when `a` should yield to `b` — used at the stop line / approach zone.
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

// True when `a` should yield to `b` — used for cars already inside the intersection.
// Once inside, whoever entered first keeps right of way regardless of how long
// the other car has been waiting outside. This prevents a nearly-done turning car
// from being forced to stop mid-turn because the car that just entered has an
// earlier approach_time (was waiting longer outside).
fn in_intersection_lower_priority(a: &Vehicle, b: &Vehicle) -> bool {
    let at = if a.entry_time != f64::MAX { a.entry_time } else { a.approach_time };
    let bt = if b.entry_time != f64::MAX { b.entry_time } else { b.approach_time };
    if (at - bt).abs() > 0.05 {
        at > bt  // later entry = lower priority
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

// True when a vehicle has crossed the centre of the intersection and is committed
// to its exit. A car in this state is unlikely to stop in a position that blocks
// a newly entering vehicle, so it is safe to let entry happen early.
fn past_intersection_centre(v: &Vehicle) -> bool {
    if v.passed_intersection {
        return true;
    }
    if v.turning {
        return v.turn_progress >= 0.55;
    }
    // Straight-through: check which side of the centre the car is on.
    match travel_direction(v.origin) {
        Cardinal::North => v.y < CENTER_Y,
        Cardinal::South => v.y > CENTER_Y,
        Cardinal::East  => v.x > CENTER_X,
        Cardinal::West  => v.x < CENTER_X,
    }
}

fn has_conflicting_vehicle_in_intersection(vehicle: &Vehicle, others: &[Vehicle]) -> bool {
    // Project the waiting vehicle at base speed to see where it would go if it moved now.
    let mut entry_ghost = vehicle.clone();
    entry_ghost.velocity = entry_ghost.base_velocity;
    let traj_entry = project_trajectory_n(&entry_ghost, CLEAR_LOOKAHEAD_SECS, CLEAR_TRAJECTORY_STEPS);

    for other in others {
        if other.id == vehicle.id || other.removed {
            continue;
        }
        if (is_in_intersection(other.x, other.y) || other.turning) && paths_conflict(vehicle, other) {
            // Guard: if the conflicting car hasn't crossed the intersection centre yet it
            // could still slow down or stop in a blocking position. Always hold until it
            // is committed to the second half of its transit.
            if !past_intersection_centre(other) {
                return true;
            }
            // Car is past centre — use trajectory to check if it clears before we arrive.
            // Use a larger radius than normal TTC checks to stay conservative.
            let traj_other = project_trajectory_n(other, CLEAR_LOOKAHEAD_SECS, CLEAR_TRAJECTORY_STEPS);
            if first_collision_step(&traj_entry, &traj_other, COLLISION_RADIUS * 1.3).is_some() {
                return true;
            }
        }
    }
    false
}

fn right_turn_entry_reached(v: &Vehicle) -> bool {
    let (ex, ey) = turn_entry_point(v.origin, v.route);
    match v.origin {
        Cardinal::South => v.y <= ey,
        Cardinal::North => v.y >= ey,
        Cardinal::West  => v.x >= ex,
        Cardinal::East  => v.x <= ex,
    }
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

    // Determine if vehicle should begin turning.
    // Right turns start before the intersection boundary (at their offset entry point)
    // so the arc uses the outer road-corner space.  Left turns wait until inside.
    if !vehicle.turning && !vehicle.passed_intersection {
        match vehicle.route {
            Route::Straight => {}
            Route::Right => {
                if right_turn_entry_reached(vehicle) {
                    if !vehicle.entered_intersection {
                        vehicle.entered_intersection = true;
                    }
                    setup_turn(vehicle);
                }
            }
            Route::Left => {
                if in_intersection {
                    setup_turn(vehicle);
                }
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
fn turn_bezier_params(
    start: (f64, f64),
    end: (f64, f64),
    route: Route,
    entry_dir: Cardinal,
    exit_dir: Cardinal,
    turn_radius: f64,
) -> ((f64, f64), (f64, f64), f64) {
    if route == Route::Right {
        // Align control points with the actual travel directions so the arc
        // starts/ends tangent to the road instead of sliding sideways.
        // c1 extends along the entry heading; c2 backs off along the exit heading.
        // Control length = shorter leg of the turn so the arc stays in the corner space.
        let (entry_dx, entry_dy) = cardinal_vector(entry_dir);
        let (exit_dx, exit_dy)   = cardinal_vector(exit_dir);
        let leg_x = (end.0 - start.0).abs();
        let leg_y = (end.1 - start.1).abs();
        // Each arm uses the leg that runs along its own axis so the curve
        // fills the full corner space rather than being limited to the shorter leg.
        let c1_len = if entry_dy != 0.0 { leg_y } else { leg_x } * 0.75;
        let c2_len = if exit_dy  != 0.0 { leg_y } else { leg_x } * 0.75;
        let c1 = (start.0 + entry_dx * c1_len, start.1 + entry_dy * c1_len);
        let c2 = (end.0   - exit_dx  * c2_len, end.1   - exit_dy  * c2_len);
        let path_length = (distance(start.0, start.1, end.0, end.1) * 1.15).max(20.0);
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

/// Trajectory for visualization — simulates a ghost vehicle at base_velocity so
/// the dots trace the full intended route including upcoming turns.
pub fn project_trajectory_pub(vehicle: &Vehicle) -> Vec<(f64, f64)> {
    let step_dt = VIS_LOOKAHEAD_SECS / VIS_TRAJECTORY_STEPS as f64;
    let mut ghost = vehicle.clone();
    ghost.velocity = ghost.base_velocity;
    let mut points = Vec::with_capacity(VIS_TRAJECTORY_STEPS);
    for _ in 0..VIS_TRAJECTORY_STEPS {
        if ghost.removed {
            break;
        }
        move_vehicle(&mut ghost, step_dt);
        points.push((ghost.x, ghost.y));
    }
    points
}

fn project_trajectory(vehicle: &Vehicle) -> Vec<(f64, f64)> {
    project_trajectory_n(vehicle, LOOKAHEAD_SECS, TRAJECTORY_STEPS)
}

fn project_trajectory_n(vehicle: &Vehicle, lookahead_secs: f64, steps: usize) -> Vec<(f64, f64)> {
    let step_dt = lookahead_secs / steps as f64;
    let speed = vehicle.velocity;

    if speed <= 0.0 {
        return vec![(vehicle.x, vehicle.y); steps];
    }

    let mut points = Vec::with_capacity(steps);

    if vehicle.turning {
        let entry_dir = travel_direction(vehicle.origin);
        let exit_dir = exit_direction(vehicle.origin, vehicle.route);

        let start = (vehicle.turn_start_x, vehicle.turn_start_y);
        let end   = (vehicle.turn_end_x,   vehicle.turn_end_y);

        let (c1, c2, path_length) =
            turn_bezier_params(start, end, vehicle.route, entry_dir, exit_dir, vehicle.turn_radius);

        let mut progress = vehicle.turn_progress;
        let progress_per_step = speed * step_dt / path_length;

        for _ in 0..steps {
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
        for step in 1..=steps {
            let t = step as f64 * step_dt;
            points.push((vehicle.x + dx * speed * t, vehicle.y + dy * speed * t));
        }
    } else {
        // Approaching or going straight through the intersection
        let dir = travel_direction(vehicle.origin);
        let (dx, dy) = cardinal_vector(dir);
        for step in 1..=steps {
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
