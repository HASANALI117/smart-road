use sdl2::image::LoadTexture;
use sdl2::pixels::Color;
use sdl2::rect::{Point, Rect};
use sdl2::render::{Texture, TextureCreator, WindowCanvas};
use sdl2::video::WindowContext;

use crate::intersection::*;
use crate::statistics::Statistics;
use crate::vehicle::Vehicle;

const GRASS_COLOR: Color = Color::RGB(76, 153, 0);
const MARKING_COLOR: Color = Color::RGB(255, 255, 255);
const LINE_COLOR: Color = Color::RGB(200, 200, 200);

// Car sprite default orientation: facing UP (north).
// vehicle.angle 0 = up, 90 = right, 180 = down, 270 = left.
// SDL2 copy_ex rotates clockwise — matches vehicle.angle directly.

pub struct GameTextures<'a> {
    pub car: Option<Texture<'a>>,
    pub road_left: Option<Texture<'a>>,
    pub road_mid: Option<Texture<'a>>,
    pub road_right: Option<Texture<'a>>,
}

impl<'a> GameTextures<'a> {
    pub fn load(creator: &'a TextureCreator<WindowContext>) -> Self {
        let load = |path: &str| match creator.load_texture(path) {
            Ok(t) => { println!("Loaded {}", path); Some(t) }
            Err(e) => { eprintln!("Warning: could not load {}: {}", path, e); None }
        };
        GameTextures {
            car:        load("assets/car_black.png"),
            road_left:  load("assets/left.png"),
            road_mid:   load("assets/mid.png"),
            road_right: load("assets/right.png"),
        }
    }
}

// ─────────────────────────────────────────────────────────
//  Scene entry point
// ─────────────────────────────────────────────────────────

pub fn render_scene(canvas: &mut WindowCanvas, vehicles: &[Vehicle], textures: &GameTextures) {
    canvas.set_draw_color(GRASS_COLOR);
    canvas.clear();

    draw_roads(canvas, textures);
    draw_lane_markings(canvas);

    for v in vehicles {
        if !v.removed {
            draw_vehicle(canvas, v, textures);
        }
    }
}

// ─────────────────────────────────────────────────────────
//  Road surfaces  (left | mid | mid | right, 4 tiles wide)
// ─────────────────────────────────────────────────────────

/// Width of one tile column on-screen.
/// The road is exactly 4 of these wide.
const TILE_COLS: i32 = 4;

fn tile_w() -> i32 {
    ROAD_WIDTH as i32 / TILE_COLS
}

fn draw_roads(canvas: &mut WindowCanvas, tex: &GameTextures) {
    let tw = tile_w();
    let il = INTERSECTION_LEFT as i32;
    let it = INTERSECTION_TOP as i32;

    // ── Vertical road (full window height) ──────────────────────────
    //   columns (left→right): left | mid | mid | right
    let v_strips: [(usize, i32); 4] = [(0, il), (1, il+tw), (1, il+tw*2), (2, il+tw*3)];
    let road_textures = [&tex.road_left, &tex.road_mid, &tex.road_right];
    for (ti, x) in v_strips {
        tile_strip_v(canvas, road_textures[ti], x, tw, 0, WINDOW_HEIGHT as i32);
    }

    // ── Horizontal road (full window width) ─────────────────────────
    //   rows (top→bottom): left→(rotated = top kerb) | mid | mid | right→(bottom kerb)
    let h_strips: [(usize, i32, f64); 4] = [
        (0, it,        90.0),
        (1, it+tw,      0.0),
        (1, it+tw*2,    0.0),
        (2, it+tw*3,   90.0),
    ];
    for (ti, y, angle) in h_strips {
        tile_strip_h(canvas, road_textures[ti], y, tw, 0, WINDOW_WIDTH as i32, angle);
    }

    // ── Fill intersection centre with plain asphalt ──────────────────
    // Overwrite the kerb strips that bleed in from both roads.
    let ib = INTERSECTION_BOTTOM as i32;
    for col in 0..TILE_COLS {
        tile_strip_v(canvas, &tex.road_mid, il + tw * col, tw, it, ib);
    }
}

/// Tile a texture as a vertical strip of `tw`x`tw` squares.
/// `strip_x` = left edge, tiles repeat from `y_start` to `y_end`.
fn tile_strip_v(
    canvas: &mut WindowCanvas,
    tex: &Option<Texture>,
    strip_x: i32,
    tw: i32,
    y_start: i32,
    y_end: i32,
) {
    let t = match tex { Some(t) => t, None => return };
    let mut y = y_start;
    while y < y_end {
        let dst = Rect::new(strip_x, y, tw as u32, tw as u32);
        canvas.copy(t, None, Some(dst)).ok();
        y += tw;
    }
}

/// Tile a texture as a horizontal strip of `tw`x`tw` squares.
/// `strip_y` = top edge, tiles repeat from `x_start` to `x_end`.
/// `angle` is the clockwise rotation (degrees) applied to each tile cell.
fn tile_strip_h(
    canvas: &mut WindowCanvas,
    tex: &Option<Texture>,
    strip_y: i32,
    tw: i32,
    x_start: i32,
    x_end: i32,
    angle: f64,
) {
    let t = match tex { Some(t) => t, None => return };
    let mut x = x_start;
    while x < x_end {
        let dst = Rect::new(x, strip_y, tw as u32, tw as u32);
        if angle == 0.0 {
            canvas.copy(t, None, Some(dst)).ok();
        } else {
            canvas.copy_ex(t, None, Some(dst), angle, None, false, false).ok();
        }
        x += tw;
    }
}

// ─────────────────────────────────────────────────────────
//  Lane markings & arrows
// ─────────────────────────────────────────────────────────

fn draw_lane_markings(canvas: &mut WindowCanvas) {
    canvas.set_draw_color(LINE_COLOR);

    let il = INTERSECTION_LEFT as i32;
    let it = INTERSECTION_TOP as i32;
    let ir = INTERSECTION_RIGHT as i32;
    let ib = INTERSECTION_BOTTOM as i32;
    let lane_w = LANE_WIDTH as i32;

    // Dashed lane dividers – vertical road (above & below intersection)
    // 6 lanes → 5 interior dividers; skip i=3 (the solid centre line)
    for i in 1..6 {
        if i == 3 { continue; }
        let x = il + lane_w * i;
        draw_dashed_line_v(canvas, x, 0, it);
        draw_dashed_line_v(canvas, x, ib, WINDOW_HEIGHT as i32);
    }

    // Dashed lane dividers – horizontal road (left & right of intersection)
    for i in 1..6 {
        if i == 3 { continue; }
        let y = it + lane_w * i;
        draw_dashed_line_h(canvas, y, 0, il);
        draw_dashed_line_h(canvas, y, ir, WINDOW_WIDTH as i32);
    }

    // Solid centre dividers
    canvas.set_draw_color(MARKING_COLOR);
    let cx = CENTER_X as i32;
    draw_solid_line_v(canvas, cx, 0, it);
    draw_solid_line_v(canvas, cx, ib, WINDOW_HEIGHT as i32);

    let cy = CENTER_Y as i32;
    draw_solid_line_h(canvas, cy, 0, il);
    draw_solid_line_h(canvas, cy, ir, WINDOW_WIDTH as i32);

    // Corner squares at intersection entry
    canvas.set_draw_color(MARKING_COLOR);
    let cs: i32 = 6;
    canvas.fill_rect(Rect::new(il - cs, it - cs, (cs * 2) as u32, (cs * 2) as u32)).ok();
    canvas.fill_rect(Rect::new(ir - cs, it - cs, (cs * 2) as u32, (cs * 2) as u32)).ok();
    canvas.fill_rect(Rect::new(il - cs, ib - cs, (cs * 2) as u32, (cs * 2) as u32)).ok();
    canvas.fill_rect(Rect::new(ir - cs, ib - cs, (cs * 2) as u32, (cs * 2) as u32)).ok();

    draw_lane_arrows(canvas);
}

fn draw_dashed_line_v(canvas: &mut WindowCanvas, x: i32, y_start: i32, y_end: i32) {
    let dash = 20; let gap = 15;
    let mut y = y_start;
    while y < y_end {
        let end = (y + dash).min(y_end);
        canvas.fill_rect(Rect::new(x - 1, y, 2, (end - y) as u32)).ok();
        y += dash + gap;
    }
}

fn draw_dashed_line_h(canvas: &mut WindowCanvas, y: i32, x_start: i32, x_end: i32) {
    let dash = 20; let gap = 15;
    let mut x = x_start;
    while x < x_end {
        let end = (x + dash).min(x_end);
        canvas.fill_rect(Rect::new(x, y - 1, (end - x) as u32, 2)).ok();
        x += dash + gap;
    }
}

fn draw_solid_line_v(canvas: &mut WindowCanvas, x: i32, y_start: i32, y_end: i32) {
    canvas.fill_rect(Rect::new(x - 1, y_start, 3, (y_end - y_start) as u32)).ok();
}

fn draw_solid_line_h(canvas: &mut WindowCanvas, y: i32, x_start: i32, x_end: i32) {
    canvas.fill_rect(Rect::new(x_start, y - 1, (x_end - x_start) as u32, 3)).ok();
}

fn draw_lane_arrows(canvas: &mut WindowCanvas) {
    canvas.set_draw_color(MARKING_COLOR);
    let lane_w = LANE_WIDTH;
    let it = INTERSECTION_TOP;
    let ib = INTERSECTION_BOTTOM;
    let il = INTERSECTION_LEFT;
    let ir = INTERSECTION_RIGHT;

    let arrow_y = (ib + 60.0) as i32;
    for i in 0..3 {
        let ax = (CENTER_X + lane_w / 2.0 + lane_w * i as f64) as i32;
        draw_arrow_up(canvas, ax, arrow_y);
    }
    let arrow_y = (it - 60.0) as i32;
    for i in 0..3 {
        let ax = (CENTER_X - lane_w / 2.0 - lane_w * i as f64) as i32;
        draw_arrow_down(canvas, ax, arrow_y);
    }
    let arrow_x = (il - 60.0) as i32;
    for i in 0..3 {
        let ay = (CENTER_Y - lane_w / 2.0 - lane_w * i as f64) as i32;
        draw_arrow_right(canvas, arrow_x, ay);
    }
    let arrow_x = (ir + 60.0) as i32;
    for i in 0..3 {
        let ay = (CENTER_Y + lane_w / 2.0 + lane_w * i as f64) as i32;
        draw_arrow_left(canvas, arrow_x, ay);
    }
}

// ─────────────────────────────────────────────────────────
//  Vehicle rendering
// ─────────────────────────────────────────────────────────

fn draw_vehicle(canvas: &mut WindowCanvas, v: &Vehicle, textures: &GameTextures) {
    match &textures.car {
        Some(car_tex) => draw_vehicle_textured(canvas, v, car_tex),
        None          => draw_vehicle_fallback(canvas, v),
    }
}

fn draw_vehicle_textured(canvas: &mut WindowCanvas, v: &Vehicle, tex: &Texture) {
    let w = v.width as u32;
    let h = v.height as u32;
    let dst = Rect::new(
        (v.x - v.width  / 2.0) as i32,
        (v.y - v.height / 2.0) as i32,
        w, h,
    );
    canvas.copy_ex(
        tex,
        None,
        Some(dst),
        v.angle,
        Some(Point::new(w as i32 / 2, h as i32 / 2)),
        false, false,
    ).ok();
}

const VEHICLE_COLORS: [Color; 6] = [
    Color::RGB(220,  50,  50),
    Color::RGB( 50, 100, 220),
    Color::RGB( 50, 180,  50),
    Color::RGB(220, 180,  30),
    Color::RGB(180,  50, 220),
    Color::RGB(220, 130,  30),
];

fn draw_vehicle_fallback(canvas: &mut WindowCanvas, v: &Vehicle) {
    let color = VEHICLE_COLORS[v.color_index % VEHICLE_COLORS.len()];
    let cx = v.x; let cy = v.y;
    let hw = v.width / 2.0; let hh = v.height / 2.0;
    let a = v.angle.to_radians();
    let (cos_a, sin_a) = (a.cos(), a.sin());

    let corners: [(i32, i32); 4] = [
        rot(-hw, -hh, cos_a, sin_a, cx, cy),
        rot( hw, -hh, cos_a, sin_a, cx, cy),
        rot( hw,  hh, cos_a, sin_a, cx, cy),
        rot(-hw,  hh, cos_a, sin_a, cx, cy),
    ];
    canvas.set_draw_color(color);
    fill_quad(canvas, &corners);
    canvas.set_draw_color(Color::RGB(30, 30, 30));
    for i in 0..4 {
        let j = (i + 1) % 4;
        canvas.draw_line(Point::new(corners[i].0, corners[i].1),
                         Point::new(corners[j].0, corners[j].1)).ok();
    }
    let wind: [(i32, i32); 4] = [
        rot(-hw+4.0, -hh+4.0,  cos_a, sin_a, cx, cy),
        rot( hw-4.0, -hh+4.0,  cos_a, sin_a, cx, cy),
        rot( hw-4.0, -hh+14.0, cos_a, sin_a, cx, cy),
        rot(-hw+4.0, -hh+14.0, cos_a, sin_a, cx, cy),
    ];
    canvas.set_draw_color(Color::RGB(180, 220, 255));
    fill_quad(canvas, &wind);
}

// ─────────────────────────────────────────────────────────
//  Geometry helpers
// ─────────────────────────────────────────────────────────

fn rot(dx: f64, dy: f64, cos_a: f64, sin_a: f64, cx: f64, cy: f64) -> (i32, i32) {
    ((dx * cos_a - dy * sin_a + cx) as i32,
     (dx * sin_a + dy * cos_a + cy) as i32)
}

fn fill_triangle(canvas: &mut WindowCanvas,
                 x0: i32, y0: i32, x1: i32, y1: i32, x2: i32, y2: i32) {
    let mut pts = [(x0, y0), (x1, y1), (x2, y2)];
    pts.sort_by_key(|p| p.1);
    let (ax, ay) = (pts[0].0 as f64, pts[0].1 as f64);
    let (bx, by) = (pts[1].0 as f64, pts[1].1 as f64);
    let (cx, cy) = (pts[2].0 as f64, pts[2].1 as f64);
    if (cy - ay).abs() < 1.0 { return; }
    for y in pts[0].1..=pts[2].1 {
        let yf = y as f64;
        let mut xs;
        let mut xe;
        if yf < by || (by - ay).abs() < 0.5 {
            xs = ax + (yf - ay) / (cy - ay) * (cx - ax);
            xe = ax + (yf - ay) / (by - ay + 0.001) * (bx - ax);
        } else {
            xs = ax + (yf - ay) / (cy - ay) * (cx - ax);
            xe = bx + (yf - by) / (cy - by + 0.001) * (cx - bx);
        }
        if xs > xe { std::mem::swap(&mut xs, &mut xe); }
        canvas.draw_line(Point::new(xs as i32, y), Point::new(xe as i32, y)).ok();
    }
}

fn fill_quad(canvas: &mut WindowCanvas, c: &[(i32, i32); 4]) {
    fill_triangle(canvas, c[0].0, c[0].1, c[1].0, c[1].1, c[2].0, c[2].1);
    fill_triangle(canvas, c[0].0, c[0].1, c[2].0, c[2].1, c[3].0, c[3].1);
}

fn draw_arrow_up(canvas: &mut WindowCanvas, x: i32, y: i32) {
    fill_triangle(canvas, x, y-8, x-6, y+4, x+6, y+4);
}
fn draw_arrow_down(canvas: &mut WindowCanvas, x: i32, y: i32) {
    fill_triangle(canvas, x, y+8, x-6, y-4, x+6, y-4);
}
fn draw_arrow_right(canvas: &mut WindowCanvas, x: i32, y: i32) {
    fill_triangle(canvas, x+8, y, x-4, y-6, x-4, y+6);
}
fn draw_arrow_left(canvas: &mut WindowCanvas, x: i32, y: i32) {
    fill_triangle(canvas, x-8, y, x+4, y-6, x+4, y+6);
}

// ─────────────────────────────────────────────────────────
//  Statistics screen
// ─────────────────────────────────────────────────────────

pub fn render_statistics(canvas: &mut WindowCanvas, stats: &Statistics) {
    canvas.set_draw_color(Color::RGB(30, 30, 50));
    canvas.clear();

    canvas.set_draw_color(Color::RGB(50, 80, 120));
    canvas.fill_rect(Rect::new(0, 0, WINDOW_WIDTH, 80)).ok();

    let box_x = 100i32;
    let box_y = 120i32;
    let box_w = (WINDOW_WIDTH - 200) as u32;
    let box_h = 500u32;

    canvas.set_draw_color(Color::RGB(45, 45, 65));
    canvas.fill_rect(Rect::new(box_x, box_y, box_w, box_h)).ok();
    canvas.set_draw_color(Color::RGB(80, 120, 180));
    canvas.draw_rect(Rect::new(box_x, box_y, box_w, box_h)).ok();
    canvas.draw_rect(Rect::new(box_x+1, box_y+1, box_w-2, box_h-2)).ok();

    let stat_items: Vec<(&str, f64, Color)> = vec![
        ("Vehicles Passed", stats.total_vehicles as f64, Color::RGB( 80, 200, 120)),
        ("Max Velocity",    stats.max_velocity,          Color::RGB(255, 100, 100)),
        ("Min Velocity",    stats.min_velocity,          Color::RGB(100, 180, 255)),
        ("Max Time",        stats.max_time,              Color::RGB(255, 200,  80)),
        ("Min Time",        stats.min_time,              Color::RGB(180, 130, 255)),
        ("Close Calls",     stats.close_calls as f64,   Color::RGB(255,  80,  80)),
    ];

    let item_h = 55i32;
    let padding = 20i32;
    for (i, (_, _, color)) in stat_items.iter().enumerate() {
        let y = box_y + padding + i as i32 * item_h;
        canvas.set_draw_color(Color::RGB(55, 55, 80));
        canvas.fill_rect(Rect::new(box_x+padding, y, box_w - padding as u32 * 2, item_h as u32 - 8)).ok();
        canvas.set_draw_color(*color);
        canvas.fill_rect(Rect::new(box_x+padding, y, 6, item_h as u32 - 8)).ok();
    }

    canvas.set_draw_color(Color::RGB(255, 255, 255));
    render_text_block(canvas, "SIMULATION STATISTICS", WINDOW_WIDTH as i32 / 2 - 150, 25, 3);

    for (i, (label, value, color)) in stat_items.iter().enumerate() {
        let y = box_y + padding + i as i32 * item_h;
        canvas.set_draw_color(Color::RGB(220, 220, 220));
        render_text_block(canvas, label, box_x + padding + 20, y + 10, 2);
        canvas.set_draw_color(*color);
        let val_str = if label.contains("Time") {
            format!("{:.2}s", value)
        } else if label.contains("Velocity") {
            format!("{:.1} px/s", value)
        } else {
            format!("{}", *value as u32)
        };
        render_text_block(canvas, &val_str, box_x + padding + 500, y + 10, 2);
    }

    canvas.set_draw_color(Color::RGB(150, 150, 180));
    render_text_block(canvas, "Press any key to exit",
        WINDOW_WIDTH as i32 / 2 - 100, box_y + box_h as i32 + 30, 1);
}

fn render_text_block(canvas: &mut WindowCanvas, text: &str, x: i32, y: i32, scale: u32) {
    let cw = (6 * scale) as i32;
    for (i, ch) in text.chars().enumerate() {
        if ch == ' ' { continue; }
        draw_char(canvas, ch, x + i as i32 * (cw + scale as i32), y, scale);
    }
}

fn draw_char(canvas: &mut WindowCanvas, ch: char, x: i32, y: i32, scale: u32) {
    let s = scale as i32;
    for (row, pattern) in get_char_pattern(ch).iter().enumerate() {
        for (col, &pixel) in pattern.iter().enumerate() {
            if pixel == 1 {
                canvas.fill_rect(Rect::new(x + col as i32 * s, y + row as i32 * s, scale, scale)).ok();
            }
        }
    }
}

fn get_char_pattern(ch: char) -> Vec<Vec<u8>> {
    match ch.to_ascii_uppercase() {
        'A' => vec![vec![0,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,1,1,1,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1]],
        'B' => vec![vec![1,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,1,1,1,0]],
        'C' => vec![vec![0,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,0,0,0,1],vec![0,1,1,1,0]],
        'D' => vec![vec![1,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,1,1,1,0]],
        'E' => vec![vec![1,1,1,1,1],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,1,1,1,0],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,1,1,1,1]],
        'F' => vec![vec![1,1,1,1,1],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,1,1,1,0],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,0,0,0,0]],
        'G' => vec![vec![0,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,0],vec![1,0,1,1,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![0,1,1,1,0]],
        'H' => vec![vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,1,1,1,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1]],
        'I' => vec![vec![1,1,1],vec![0,1,0],vec![0,1,0],vec![0,1,0],vec![0,1,0],vec![0,1,0],vec![1,1,1]],
        'J' => vec![vec![0,0,1,1,1],vec![0,0,0,1,0],vec![0,0,0,1,0],vec![0,0,0,1,0],vec![1,0,0,1,0],vec![1,0,0,1,0],vec![0,1,1,0,0]],
        'K' => vec![vec![1,0,0,0,1],vec![1,0,0,1,0],vec![1,0,1,0,0],vec![1,1,0,0,0],vec![1,0,1,0,0],vec![1,0,0,1,0],vec![1,0,0,0,1]],
        'L' => vec![vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,1,1,1,1]],
        'M' => vec![vec![1,0,0,0,1],vec![1,1,0,1,1],vec![1,0,1,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1]],
        'N' => vec![vec![1,0,0,0,1],vec![1,1,0,0,1],vec![1,0,1,0,1],vec![1,0,0,1,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1]],
        'O' => vec![vec![0,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![0,1,1,1,0]],
        'P' => vec![vec![1,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,1,1,1,0],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,0,0,0,0]],
        'Q' => vec![vec![0,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,1,0,1],vec![1,0,0,1,0],vec![0,1,1,0,1]],
        'R' => vec![vec![1,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,1,1,1,0],vec![1,0,1,0,0],vec![1,0,0,1,0],vec![1,0,0,0,1]],
        'S' => vec![vec![0,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,0],vec![0,1,1,1,0],vec![0,0,0,0,1],vec![1,0,0,0,1],vec![0,1,1,1,0]],
        'T' => vec![vec![1,1,1,1,1],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0]],
        'U' => vec![vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![0,1,1,1,0]],
        'V' => vec![vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![0,1,0,1,0],vec![0,1,0,1,0],vec![0,0,1,0,0]],
        'W' => vec![vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![1,0,1,0,1],vec![1,0,1,0,1],vec![1,1,0,1,1],vec![1,0,0,0,1]],
        'X' => vec![vec![1,0,0,0,1],vec![0,1,0,1,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,1,0,1,0],vec![1,0,0,0,1]],
        'Y' => vec![vec![1,0,0,0,1],vec![0,1,0,1,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0]],
        'Z' => vec![vec![1,1,1,1,1],vec![0,0,0,0,1],vec![0,0,0,1,0],vec![0,0,1,0,0],vec![0,1,0,0,0],vec![1,0,0,0,0],vec![1,1,1,1,1]],
        '0' => vec![vec![0,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,1,1],vec![1,0,1,0,1],vec![1,1,0,0,1],vec![1,0,0,0,1],vec![0,1,1,1,0]],
        '1' => vec![vec![0,0,1,0,0],vec![0,1,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,1,1,1,0]],
        '2' => vec![vec![0,1,1,1,0],vec![1,0,0,0,1],vec![0,0,0,0,1],vec![0,0,0,1,0],vec![0,0,1,0,0],vec![0,1,0,0,0],vec![1,1,1,1,1]],
        '3' => vec![vec![0,1,1,1,0],vec![1,0,0,0,1],vec![0,0,0,0,1],vec![0,0,1,1,0],vec![0,0,0,0,1],vec![1,0,0,0,1],vec![0,1,1,1,0]],
        '4' => vec![vec![0,0,0,1,0],vec![0,0,1,1,0],vec![0,1,0,1,0],vec![1,0,0,1,0],vec![1,1,1,1,1],vec![0,0,0,1,0],vec![0,0,0,1,0]],
        '5' => vec![vec![1,1,1,1,1],vec![1,0,0,0,0],vec![1,1,1,1,0],vec![0,0,0,0,1],vec![0,0,0,0,1],vec![1,0,0,0,1],vec![0,1,1,1,0]],
        '6' => vec![vec![0,1,1,1,0],vec![1,0,0,0,0],vec![1,0,0,0,0],vec![1,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![0,1,1,1,0]],
        '7' => vec![vec![1,1,1,1,1],vec![0,0,0,0,1],vec![0,0,0,1,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0],vec![0,0,1,0,0]],
        '8' => vec![vec![0,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![0,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![0,1,1,1,0]],
        '9' => vec![vec![0,1,1,1,0],vec![1,0,0,0,1],vec![1,0,0,0,1],vec![0,1,1,1,1],vec![0,0,0,0,1],vec![0,0,0,0,1],vec![0,1,1,1,0]],
        '.' => vec![vec![0,0,0],vec![0,0,0],vec![0,0,0],vec![0,0,0],vec![0,0,0],vec![0,0,0],vec![0,1,0]],
        ':' => vec![vec![0,0,0],vec![0,1,0],vec![0,0,0],vec![0,0,0],vec![0,0,0],vec![0,1,0],vec![0,0,0]],
        '/' => vec![vec![0,0,0,0,1],vec![0,0,0,1,0],vec![0,0,0,1,0],vec![0,0,1,0,0],vec![0,1,0,0,0],vec![0,1,0,0,0],vec![1,0,0,0,0]],
        _   => vec![vec![0,0,0],vec![0,0,0],vec![0,0,0],vec![0,0,0],vec![0,0,0],vec![0,0,0],vec![0,0,0]],
    }
}
