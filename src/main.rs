mod algorithm;
mod intersection;
mod renderer;
mod route;
mod statistics;
mod vehicle;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::time::Instant;

use rand::Rng;

use algorithm::SmartIntersection;
use intersection::*;
use route::{Cardinal, Route};
use statistics::Statistics;
use vehicle::Vehicle;

const FPS: u32 = 60;
const FRAME_DELAY: u32 = 1000 / FPS;
const SPAWN_COOLDOWN: f64 = 0.3;
const RANDOM_SPAWN_INTERVAL: f64 = 0.5;

// Maximum vehicles queued per approach direction (not yet passed the intersection).
// Raise to allow longer queues, lower to keep lanes cleaner.
const MAX_VEHICLES_PER_APPROACH: usize = 4;

fn main() {
    let sdl_context = sdl2::init().expect("Failed to initialize SDL2");
    let video_subsystem = sdl_context.video().expect("Failed to init video");

    let window = video_subsystem
        .window("Smart Road - Intersection Simulator", WINDOW_WIDTH, WINDOW_HEIGHT)
        .position_centered()
        .build()
        .expect("Failed to create window");

    let mut canvas = window
        .into_canvas()
        .software()
        .build()
        .expect("Failed to create canvas");

    let texture_creator = canvas.texture_creator();
    let textures = renderer::GameTextures::load(&texture_creator);

    let mut event_pump = sdl_context.event_pump().expect("Failed to get event pump");

    let mut vehicles: Vec<Vehicle> = Vec::new();
    let mut smart = SmartIntersection::new();
    let mut next_id: u64 = 0;
    let mut all_vehicles: Vec<Vehicle> = Vec::new();

    let mut last_spawn: [f64; 4] = [0.0; 4];
    let mut show_hitbox = false;
    let mut show_trajectory = false;
    let mut random_mode = false;
    let mut random_timer = 0.0;
    let mut running = true;
    let mut show_stats = false;

    let start_time = Instant::now();
    let mut last_frame = Instant::now();

    'mainloop: loop {
        let now = Instant::now();
        let dt = now.duration_since(last_frame).as_secs_f64();
        last_frame = now;
        let elapsed = now.duration_since(start_time).as_secs_f64();

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. } => break 'mainloop,

                Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => {
                    if show_stats {
                        break 'mainloop;
                    }
                    show_stats = true;
                    running = false;
                }

                Event::KeyDown {
                    keycode: Some(Keycode::R),
                    ..
                } if !show_stats => {
                    random_mode = !random_mode;
                }

                Event::KeyDown {
                    keycode: Some(Keycode::H),
                    ..
                } if !show_stats => {
                    show_hitbox = !show_hitbox;
                }

                Event::KeyDown {
                    keycode: Some(Keycode::T),
                    ..
                } if !show_stats => {
                    show_trajectory = !show_trajectory;
                }

                Event::KeyDown {
                    keycode: Some(Keycode::Up),
                    ..
                } if !show_stats => {
                    try_spawn_vehicle(
                        Cardinal::South,
                        &mut vehicles,
                        &mut next_id,
                        &mut last_spawn,
                        elapsed,
                        textures.car_textures.len(),
                    );
                }

                Event::KeyDown {
                    keycode: Some(Keycode::Down),
                    ..
                } if !show_stats => {
                    try_spawn_vehicle(
                        Cardinal::North,
                        &mut vehicles,
                        &mut next_id,
                        &mut last_spawn,
                        elapsed,
                        textures.car_textures.len(),
                    );
                }

                Event::KeyDown {
                    keycode: Some(Keycode::Right),
                    ..
                } if !show_stats => {
                    try_spawn_vehicle(
                        Cardinal::West,
                        &mut vehicles,
                        &mut next_id,
                        &mut last_spawn,
                        elapsed,
                        textures.car_textures.len(),
                    );
                }

                Event::KeyDown {
                    keycode: Some(Keycode::Left),
                    ..
                } if !show_stats => {
                    try_spawn_vehicle(
                        Cardinal::East,
                        &mut vehicles,
                        &mut next_id,
                        &mut last_spawn,
                        elapsed,
                        textures.car_textures.len(),
                    );
                }

                Event::KeyDown { .. } if show_stats => {
                    break 'mainloop;
                }

                _ => {}
            }
        }

        if running {
            if random_mode {
                random_timer += dt;
                if random_timer >= RANDOM_SPAWN_INTERVAL {
                    random_timer = 0.0;
                    let origin = Cardinal::random();
                    try_spawn_vehicle(
                        origin,
                        &mut vehicles,
                        &mut next_id,
                        &mut last_spawn,
                        elapsed,
                        textures.car_textures.len(),
                    );
                }
            }

            smart.update(&mut vehicles, dt, elapsed);

            let mut i = 0;
            while i < vehicles.len() {
                if vehicles[i].removed {
                    let v = vehicles.remove(i);
                    all_vehicles.push(v);
                } else {
                    i += 1;
                }
            }

            renderer::render_scene(&mut canvas, &vehicles, &textures, show_hitbox, show_trajectory, random_mode);
        } else if show_stats {
            let mut combined = all_vehicles.clone();
            combined.extend(vehicles.iter().cloned());
            let stats = Statistics::compute(&combined, smart.close_calls);
            renderer::render_statistics(&mut canvas, &stats);
        }

        canvas.present();

        let frame_time = now.elapsed().as_millis() as u32;
        if frame_time < FRAME_DELAY {
            std::thread::sleep(std::time::Duration::from_millis(
                (FRAME_DELAY - frame_time) as u64,
            ));
        }
    }
}

fn try_spawn_vehicle(
    origin: Cardinal,
    vehicles: &mut Vec<Vehicle>,
    next_id: &mut u64,
    last_spawn: &mut [f64; 4],
    elapsed: f64,
    car_count: usize,
) {
    let idx = match origin {
        Cardinal::North => 0,
        Cardinal::South => 1,
        Cardinal::East => 2,
        Cardinal::West => 3,
    };

    if elapsed - last_spawn[idx] < SPAWN_COOLDOWN {
        return;
    }

    // Reject spawn if too many vehicles from this direction are still approaching
    let queued = vehicles
        .iter()
        .filter(|v| v.origin == origin && !v.passed_intersection)
        .count();
    if queued >= MAX_VEHICLES_PER_APPROACH {
        return;
    }

    let route = Route::random();
    let (sx, sy) = spawn_position(origin, route);

    for v in vehicles.iter() {
        if v.origin == origin {
            let dist = ((v.x - sx).powi(2) + (v.y - sy).powi(2)).sqrt();
            if dist < 70.0 {
                return;
            }
        }
    }

    let color_index = *next_id as usize % 6;
    let sprite_index = if car_count == 0 {
        0
    } else {
        rand::thread_rng().gen_range(0..car_count)
    };
    let v = Vehicle::new(*next_id, origin, route, sx, sy, sprite_index, color_index);
    vehicles.push(v);
    *next_id += 1;
    last_spawn[idx] = elapsed;
}
