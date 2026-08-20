/// 3D Surface Renderer for Lyapunov function visualization
use macroquad::prelude::*;
use crate::physics::*;

/// Grid resolution for the surface mesh
const GRID_SIZE: usize = 60;
/// Spatial range for z₁ and z₂ axes
const Z1_RANGE: f32 = 2.5;
const Z2_RANGE: f32 = 3.5;
/// Vertical scale for V values
const V_SCALE: f32 = 3.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SurfaceTheme {
    Cyberpunk,   // Midnight Violet -> Neon Magenta -> Electric Cyan -> Gold
    Magma,       // Dark Charcoal -> Deep Crimson -> Fiery Orange -> Radiant Gold
    Oceanic,     // Abyssal Blue -> Turquoise -> Emerald Teal -> Spring Green
    Plasma,      // Deep Sapphire -> Electric Cyan -> Emerald -> Amber -> Crimson
}

impl SurfaceTheme {
    pub fn name(&self) -> &'static str {
        match self {
            SurfaceTheme::Cyberpunk => "Cyberpunk Neon",
            SurfaceTheme::Magma => "Magma & Fire",
            SurfaceTheme::Oceanic => "Deep Oceanic",
            SurfaceTheme::Plasma => "Plasma & Spectral",
        }
    }
}

/// Dynamic color palette selector with directional diffuse shading
fn surface_color(v: f32, v_max: f32, normal_shade: f32, theme: SurfaceTheme) -> Color {
    let t = (v / v_max).clamp(0.0, 1.0);

    let (r, g, b) = match theme {
        SurfaceTheme::Cyberpunk => {
            if t < 0.33 {
                let s = t / 0.33;
                (0.12 + s * 0.70, 0.05 + s * 0.05, 0.45 + s * 0.40)
            } else if t < 0.66 {
                let s = (t - 0.33) / 0.33;
                (0.82 - s * 0.82, 0.10 + s * 0.75, 0.85 + s * 0.10)
            } else {
                let s = (t - 0.66) / 0.34;
                (s * 1.0, 0.85 + s * 0.10, 0.95 - s * 0.85)
            }
        }
        SurfaceTheme::Magma => {
            if t < 0.33 {
                let s = t / 0.33;
                (0.08 + s * 0.67, 0.05 + s * 0.03, 0.18 - s * 0.08)
            } else if t < 0.66 {
                let s = (t - 0.33) / 0.33;
                (0.75 + s * 0.23, 0.08 + s * 0.37, 0.10 - s * 0.05)
            } else {
                let s = (t - 0.66) / 0.34;
                (0.98 + s * 0.02, 0.45 + s * 0.45, 0.05 + s * 0.15)
            }
        }
        SurfaceTheme::Oceanic => {
            if t < 0.33 {
                let s = t / 0.33;
                (0.02 + s * 0.02, 0.15 + s * 0.50, 0.45 + s * 0.40)
            } else if t < 0.66 {
                let s = (t - 0.33) / 0.33;
                (0.04 + s * 0.01, 0.65 + s * 0.20, 0.85 - s * 0.20)
            } else {
                let s = (t - 0.66) / 0.34;
                (0.05 + s * 0.40, 0.85 + s * 0.13, 0.65 - s * 0.20)
            }
        }
        SurfaceTheme::Plasma => {
            if t < 0.25 {
                let s = t / 0.25;
                (0.05 + s * 0.05, 0.25 + s * 0.45, 0.85 + s * 0.10)
            } else if t < 0.50 {
                let s = (t - 0.25) / 0.25;
                (0.10 + s * 0.10, 0.70 + s * 0.25, 0.95 - s * 0.45)
            } else if t < 0.75 {
                let s = (t - 0.50) / 0.25;
                (0.20 + s * 0.75, 0.95 - s * 0.10, 0.50 - s * 0.40)
            } else {
                let s = (t - 0.75) / 0.25;
                (0.95 + s * 0.05, 0.85 - s * 0.65, 0.10 + s * 0.15)
            }
        }
    };

    let shadow = normal_shade.clamp(0.35, 1.0);
    Color::new(r * shadow, g * shadow, b * shadow, 0.88)
}

/// Convert Color to [u8; 4] for Vertex
fn color_bytes(c: Color) -> [u8; 4] {
    [
        (c.r * 255.0) as u8,
        (c.g * 255.0) as u8,
        (c.b * 255.0) as u8,
        (c.a * 255.0) as u8,
    ]
}

/// Create a Vertex with all fields
fn vert(pos: Vec3, uv: Vec2, color: Color) -> Vertex {
    Vertex {
        position: pos,
        uv,
        color: color_bytes(color),
        normal: vec4(0.0, 1.0, 0.0, 0.0),
    }
}

/// Draw the 3D surface of V(z₁, z₂) with dynamic lighting and color theme
pub fn draw_lyapunov_surface(params: &PowerSystemParams, show_solid: bool, theme: SurfaceTheme) {
    let v_max = params.v_lim * 1.5;
    let light_dir = vec3(0.4, 1.0, 0.3).normalize();

    for i in 0..GRID_SIZE {
        for j in 0..GRID_SIZE {
            let z1_a = -Z1_RANGE + 2.0 * Z1_RANGE * (i as f32 / GRID_SIZE as f32);
            let z2_a = -Z2_RANGE + 2.0 * Z2_RANGE * (j as f32 / GRID_SIZE as f32);
            let z1_b = -Z1_RANGE + 2.0 * Z1_RANGE * ((i + 1) as f32 / GRID_SIZE as f32);
            let z2_b = -Z2_RANGE + 2.0 * Z2_RANGE * ((j + 1) as f32 / GRID_SIZE as f32);

            let v00 = params.lyapunov_value(z1_a, z2_a).max(0.0).min(v_max);
            let v10 = params.lyapunov_value(z1_b, z2_a).max(0.0).min(v_max);
            let v01 = params.lyapunov_value(z1_a, z2_b).max(0.0).min(v_max);
            let v11 = params.lyapunov_value(z1_b, z2_b).max(0.0).min(v_max);

            let p00 = vec3(z1_a, v00 * V_SCALE / v_max, z2_a);
            let p10 = vec3(z1_b, v10 * V_SCALE / v_max, z2_a);
            let p01 = vec3(z1_a, v01 * V_SCALE / v_max, z2_b);
            let p11 = vec3(z1_b, v11 * V_SCALE / v_max, z2_b);

            // Compute surface normal for directional diffuse shading
            let n0 = (p10 - p00).cross(p01 - p00).normalize();
            let shade = 0.5 + 0.5 * n0.dot(light_dir).max(0.0);

            if show_solid {
                let c00 = surface_color(v00, v_max, shade, theme);
                let c10 = surface_color(v10, v_max, shade, theme);
                let c01 = surface_color(v01, v_max, shade, theme);
                let c11 = surface_color(v11, v_max, shade, theme);

                let mesh = Mesh {
                    vertices: vec![
                        vert(p00, vec2(0.0, 0.0), c00),
                        vert(p10, vec2(1.0, 0.0), c10),
                        vert(p01, vec2(0.0, 1.0), c01),
                        vert(p11, vec2(1.0, 1.0), c11),
                    ],
                    indices: vec![0, 1, 2, 1, 3, 2],
                    texture: None,
                };
                draw_mesh(&mesh);
            }

            // Elegant, semi-transparent wireframe overlay
            let wire_col = match theme {
                SurfaceTheme::Cyberpunk => Color::new(0.9, 0.2, 0.9, 0.20),
                SurfaceTheme::Magma => Color::new(1.0, 0.5, 0.1, 0.20),
                SurfaceTheme::Oceanic => Color::new(0.0, 0.85, 0.7, 0.20),
                SurfaceTheme::Plasma => Color::new(0.0, 0.85, 1.0, 0.18),
            };
            draw_line_3d(p00, p10, wire_col);
            draw_line_3d(p00, p01, wire_col);
        }
    }
}

/// Draw horizontal invariant and limit planes with glassmorphic transparency and glowing borders
pub fn draw_level_planes(params: &PowerSystemParams) {
    let v_max = params.v_lim * 1.5;
    let y_invar = params.v_invar * V_SCALE / v_max;
    let y_lim = params.v_lim * V_SCALE / v_max;
    let extent = 3.5;

    // Invariant plane (Luminous Sapphire/Cyan)
    let ci = Color::new(0.0, 0.45, 0.95, 0.28);
    let mesh_invar = Mesh {
        vertices: vec![
            vert(vec3(-extent, y_invar, -extent), vec2(0.0, 0.0), ci),
            vert(vec3(extent, y_invar, -extent), vec2(1.0, 0.0), ci),
            vert(vec3(-extent, y_invar, extent), vec2(0.0, 1.0), ci),
            vert(vec3(extent, y_invar, extent), vec2(1.0, 1.0), ci),
        ],
        indices: vec![0, 1, 2, 1, 3, 2],
        texture: None,
    };
    draw_mesh(&mesh_invar);

    // Limit plane (Vibrant Spring Lime)
    let cl = Color::new(0.15, 0.90, 0.35, 0.22);
    let mesh_lim = Mesh {
        vertices: vec![
            vert(vec3(-extent, y_lim, -extent), vec2(0.0, 0.0), cl),
            vert(vec3(extent, y_lim, -extent), vec2(1.0, 0.0), cl),
            vert(vec3(-extent, y_lim, extent), vec2(0.0, 1.0), cl),
            vert(vec3(extent, y_lim, extent), vec2(1.0, 1.0), cl),
        ],
        indices: vec![0, 1, 2, 1, 3, 2],
        texture: None,
    };
    draw_mesh(&mesh_lim);

    // Glowing plane border lines
    let iw = Color::new(0.0, 0.75, 1.0, 0.95);
    let lw = Color::new(0.2, 1.0, 0.45, 0.95);
    for (y, c) in [(y_invar, iw), (y_lim, lw)] {
        draw_line_3d(vec3(-extent, y, -extent), vec3(extent, y, -extent), c);
        draw_line_3d(vec3(extent, y, -extent), vec3(extent, y, extent), c);
        draw_line_3d(vec3(extent, y, extent), vec3(-extent, y, extent), c);
        draw_line_3d(vec3(-extent, y, extent), vec3(-extent, y, -extent), c);
    }
}

/// Draw sublevel set contour S on the ground plane
pub fn draw_sublevel_contour(params: &PowerSystemParams, level: f32, color: Color) {
    let contour = compute_contour(params, level, 200);
    if contour.len() < 2 { return; }
    for i in 0..contour.len() {
        let (z1a, z2a) = contour[i];
        let (z1b, z2b) = contour[(i + 1) % contour.len()];
        draw_line_3d(vec3(z1a, 0.0, z2a), vec3(z1b, 0.0, z2b), color);
    }
}

/// Draw equilibrium point p₀ and related markers
pub fn draw_markers(params: &PowerSystemParams) {
    let v_max = params.v_lim * 1.5;

    // p₀ at origin on ground plane
    draw_sphere(vec3(0.0, 0.0, 0.0), 0.06, None, Color::new(0.0, 0.2, 1.0, 1.0));

    // V_invar and V_lim markers on the V axis
    let y_invar = params.v_invar * V_SCALE / v_max;
    let y_lim = params.v_lim * V_SCALE / v_max;
    draw_sphere(vec3(0.0, y_invar, 0.0), 0.05, None, Color::new(0.2, 0.6, 1.0, 1.0));
    draw_sphere(vec3(0.0, y_lim, 0.0), 0.05, None, Color::new(1.0, 0.2, 0.2, 1.0));

    // p_lim marker on ground
    let z1_uep = params.uep_angle_deviation();
    draw_sphere(vec3(z1_uep.min(Z1_RANGE), 0.0, 0.0), 0.05, None, Color::new(0.8, 0.1, 0.1, 1.0));

    // z₂_min and z₂_lim dashed markers
    let (z2_min, z2_lim) = find_z2_bounds(params);
    let dash_col = Color::new(0.3, 0.3, 1.0, 0.7);
    let num_dashes = 20;
    for d in 0..num_dashes {
        let t0 = d as f32 / num_dashes as f32;
        let t1 = (d as f32 + 0.5) / num_dashes as f32;
        // z₂_min dashed vertical
        draw_line_3d(vec3(0.0, t0 * y_invar, z2_min), vec3(0.0, t1 * y_invar, z2_min), dash_col);
        // z₂_lim dashed vertical
        draw_line_3d(vec3(0.0, t0 * y_lim, z2_lim), vec3(0.0, t1 * y_lim, z2_lim), dash_col);
    }
    // Ground-plane dashed lines to markers
    for d in 0..num_dashes {
        let t0 = d as f32 / num_dashes as f32;
        let t1 = (d as f32 + 0.5) / num_dashes as f32;
        let x0 = -Z1_RANGE + t0 * 0.5;
        let x1 = -Z1_RANGE + t1 * 0.5;
        draw_line_3d(vec3(x0, 0.0, z2_min), vec3(x1, 0.0, z2_min), dash_col);
        draw_line_3d(vec3(x0, 0.0, z2_lim), vec3(x1, 0.0, z2_lim), dash_col);
    }
}

/// Draw trajectory particles on the surface
pub fn draw_trajectories(particles: &[TrajectoryParticle], params: &PowerSystemParams) {
    let v_max = params.v_lim * 1.5;
    for particle in particles {
        if particle.history.len() < 2 { continue; }
        // Draw trail on the surface
        for i in 1..particle.history.len() {
            let (z1a, z2a) = particle.history[i - 1];
            let (z1b, z2b) = particle.history[i];
            let va = params.lyapunov_value(z1a, z2a).max(0.0).min(v_max);
            let vb = params.lyapunov_value(z1b, z2b).max(0.0).min(v_max);
            let ya = va * V_SCALE / v_max;
            let yb = vb * V_SCALE / v_max;
            let alpha = (i as f32 / particle.history.len() as f32) * 0.9 + 0.1;
            let c = Color::new(particle.color.r, particle.color.g, particle.color.b, alpha);
            draw_line_3d(vec3(z1a, ya, z2a), vec3(z1b, yb, z2b), c);
        }
        // Current position sphere
        if particle.active {
            let v = params.lyapunov_value(particle.z1, particle.z2).max(0.0).min(v_max);
            let y = v * V_SCALE / v_max;
            draw_sphere(vec3(particle.z1, y, particle.z2), 0.07, None, particle.color);
        }
        // Ground plane trail
        for i in 1..particle.history.len() {
            let (z1a, z2a) = particle.history[i - 1];
            let (z1b, z2b) = particle.history[i];
            let alpha = (i as f32 / particle.history.len() as f32) * 0.4;
            let c = Color::new(particle.color.r, particle.color.g, particle.color.b, alpha);
            draw_line_3d(vec3(z1a, 0.0, z2a), vec3(z1b, 0.0, z2b), c);
        }
    }
}

/// Draw axis tick marks
pub fn draw_axes() {
    let axis_col = Color::new(0.7, 0.7, 0.7, 0.9);
    let tick = 0.08;
    // z₁ axis (x)
    draw_line_3d(vec3(-Z1_RANGE, 0.0, 0.0), vec3(Z1_RANGE, 0.0, 0.0), axis_col);
    for i in -2..=2 {
        let x = i as f32;
        draw_line_3d(vec3(x, 0.0, -tick), vec3(x, 0.0, tick), axis_col);
    }
    // z₂ axis (z)
    draw_line_3d(vec3(0.0, 0.0, -Z2_RANGE), vec3(0.0, 0.0, Z2_RANGE), axis_col);
    for i in -3..=3 {
        let z = i as f32;
        draw_line_3d(vec3(-tick, 0.0, z), vec3(tick, 0.0, z), axis_col);
    }
    // V axis (y)
    draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(0.0, V_SCALE + 0.3, 0.0), axis_col);
    for i in 0..=6 {
        let y = i as f32 * 0.5;
        if y <= V_SCALE + 0.3 {
            draw_line_3d(vec3(-tick, y, 0.0), vec3(tick, y, 0.0), axis_col);
        }
    }
}

/// Draw the lateral (side) view — 2D cross-section at z₁=0
pub fn draw_lateral_view(params: &PowerSystemParams, particles: &[TrajectoryParticle]) {
    let sw = screen_width();
    let sh = screen_height();
    let panel_w = sw * 0.35;
    let panel_h = sh * 0.45;
    let px = sw - panel_w - 15.0;
    let py = sh - panel_h - 15.0;

    // Background
    draw_rectangle(px, py, panel_w, panel_h, Color::new(0.02, 0.02, 0.04, 0.95));
    draw_rectangle_lines(px, py, panel_w, panel_h, 2.0, Color::new(0.3, 0.6, 1.0, 0.6));

    draw_text("Lateral View  V(z1,z2)", px + 10.0, py + 18.0, 16.0, Color::new(0.7, 0.85, 1.0, 1.0));

    let margin = 30.0;
    let plot_x = px + margin;
    let plot_y = py + margin + 5.0;
    let plot_w = panel_w - 2.0 * margin;
    let plot_h = panel_h - 2.0 * margin - 10.0;
    let v_max = params.v_lim * 1.5;
    let z2_ext = Z2_RANGE;

    let to_screen = |z2: f32, v: f32| -> (f32, f32) {
        let sx = plot_x + ((z2 + z2_ext) / (2.0 * z2_ext)) * plot_w;
        let sy = plot_y + plot_h - (v / v_max) * plot_h;
        (sx, sy)
    };

    // Shaded region below V_lim
    let steps = 120;
    for i in 0..steps {
        let z2 = -z2_ext + 2.0 * z2_ext * (i as f32 / steps as f32);
        let v = params.lyapunov_value(0.0, z2).max(0.0).min(v_max);
        if v <= params.v_lim {
            let (sx, sy_top) = to_screen(z2, v);
            let (_, sy_lim) = to_screen(z2, params.v_lim.min(v_max));
            let h = sy_lim.min(plot_y + plot_h) - sy_top;
            if h > 0.0 {
                draw_rectangle(sx, sy_top, plot_w / steps as f32 + 1.0, h, Color::new(1.0, 0.4, 0.5, 0.15));
            }
        }
    }

    // V(0, z₂) curve
    let curve_col = Color::new(0.0, 0.7, 0.9, 1.0);
    let mut prev: Option<(f32, f32)> = None;
    for i in 0..=steps {
        let z2 = -z2_ext + 2.0 * z2_ext * (i as f32 / steps as f32);
        let v = params.lyapunov_value(0.0, z2).max(0.0).min(v_max);
        let (sx, sy) = to_screen(z2, v);
        if let Some((px_prev, py_prev)) = prev {
            draw_line(px_prev, py_prev, sx, sy, 2.0, curve_col);
        }
        prev = Some((sx, sy));
    }

    // Plane_invar and Plane_lim horizontal lines
    let (_, y_iv) = to_screen(0.0, params.v_invar.min(v_max));
    let (_, y_lm) = to_screen(0.0, params.v_lim.min(v_max));
    draw_line(plot_x, y_iv, plot_x + plot_w, y_iv, 1.5, Color::new(0.3, 0.5, 1.0, 0.8));
    draw_text("Plane_invar", plot_x + plot_w - 80.0, y_iv - 4.0, 12.0, Color::new(0.4, 0.6, 1.0, 1.0));
    draw_line(plot_x, y_lm, plot_x + plot_w, y_lm, 1.5, Color::new(0.3, 1.0, 0.5, 0.8));
    draw_text("Plane_lim", plot_x + plot_w - 70.0, y_lm - 4.0, 12.0, Color::new(0.3, 1.0, 0.5, 1.0));

    // Left labels
    draw_text("V_invar", plot_x - 5.0, y_iv + 4.0, 11.0, Color::new(0.5, 0.7, 1.0, 1.0));
    draw_text("V_lim", plot_x - 5.0, y_lm + 4.0, 11.0, Color::new(1.0, 0.3, 0.3, 1.0));

    // z₂_min and z₂_lim dashed markers
    let (z2_min, z2_lim) = find_z2_bounds(params);
    let (sx_min, _) = to_screen(z2_min, 0.0);
    let (sx_lim, _) = to_screen(z2_lim, 0.0);
    let dash_count = 15;
    for d in 0..dash_count {
        let t0 = d as f32 / dash_count as f32;
        let t1 = (d as f32 + 0.5) / dash_count as f32;
        let ya = plot_y + plot_h - t0 * plot_h;
        let yb = plot_y + plot_h - t1 * plot_h;
        draw_line(sx_min, ya, sx_min, yb, 1.0, Color::new(0.4, 0.4, 1.0, 0.5));
        draw_line(sx_lim, ya, sx_lim, yb, 1.0, Color::new(0.4, 0.4, 1.0, 0.5));
    }
    draw_text("z2min", sx_min - 12.0, plot_y + plot_h + 12.0, 11.0, Color::new(0.5, 0.5, 1.0, 1.0));
    draw_text("z2lim", sx_lim - 12.0, plot_y + plot_h + 12.0, 11.0, Color::new(0.5, 0.5, 1.0, 1.0));

    // p₀ and S labels
    let (sx_p0, _) = to_screen(0.0, 0.0);
    draw_text("p0", sx_p0 - 5.0, plot_y + plot_h + 12.0, 12.0, Color::new(0.2, 0.5, 1.0, 1.0));
    draw_text("S", plot_x + plot_w - 12.0, plot_y + plot_h + 12.0, 13.0, Color::new(0.2, 0.5, 1.0, 1.0));

    // Particle positions on lateral view
    for particle in particles {
        if particle.active {
            let v = params.lyapunov_value(0.0, particle.z2).max(0.0).min(v_max);
            let (sx, sy) = to_screen(particle.z2, v);
            draw_circle(sx, sy, 4.0, particle.color);
        }
    }

    // Axes border
    draw_line(plot_x, plot_y, plot_x, plot_y + plot_h, 1.0, Color::new(0.5, 0.5, 0.5, 0.8));
    draw_line(plot_x, plot_y + plot_h, plot_x + plot_w, plot_y + plot_h, 1.0, Color::new(0.5, 0.5, 0.5, 0.8));
    draw_text("V(z1,z2)", plot_x - 5.0, plot_y - 3.0, 11.0, Color::new(0.6, 0.6, 0.8, 1.0));
    draw_text("z2", plot_x + plot_w + 3.0, plot_y + plot_h + 4.0, 11.0, Color::new(0.6, 0.6, 0.8, 1.0));
}
