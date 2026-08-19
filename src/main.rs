/// Lyapunov Stability Simulation — Power System Frequency Control
///
/// Interactive 3D visualization of Lyapunov energy functions for
/// Single-Machine Infinite-Bus (SMIB) transient stability analysis.
///
/// Controls:
///   Mouse drag — orbit camera
///   Scroll    — zoom
///   egui panel — adjust all parameters

mod physics;
mod renderer;

use macroquad::prelude::*;
use physics::*;
use renderer::*;

/// Camera orbit state
struct CameraOrbit {
    yaw: f32,
    pitch: f32,
    distance: f32,
    target: Vec3,
    auto_rotate: bool,
}

impl Default for CameraOrbit {
    fn default() -> Self {
        Self {
            yaw: 0.8,
            pitch: 0.6,
            distance: 10.0,
            target: vec3(0.0, 1.0, 0.0),
            auto_rotate: false,
        }
    }
}

impl CameraOrbit {
    fn reset(&mut self) {
        self.yaw = 0.8;
        self.pitch = 0.6;
        self.distance = 10.0;
        self.target = vec3(0.0, 1.0, 0.0);
    }

    fn update(&mut self, egui_wants_input: bool, dt: f32) {
        if egui_wants_input {
            return;
        }

        // Auto-rotation when active and user is not manually orbiting
        if self.auto_rotate && !is_mouse_button_down(MouseButton::Left) {
            self.yaw += 0.3 * dt;
        }

        // Left-click drag: Rotate/Orbit
        if is_mouse_button_down(MouseButton::Left) {
            let delta = mouse_delta_position();
            self.yaw += delta.x * 0.005;
            self.pitch = (self.pitch - delta.y * 0.005).clamp(0.05, 1.52);
        }

        // Right-click or Middle-click drag: 3D Pan
        if is_mouse_button_down(MouseButton::Right) || is_mouse_button_down(MouseButton::Middle) {
            let delta = mouse_delta_position();
            let right = vec3(self.yaw.cos(), 0.0, -self.yaw.sin());
            let forward = vec3(-self.yaw.sin(), 0.0, -self.yaw.cos());
            self.target += right * (-delta.x * 0.008 * self.distance * 0.2)
                + forward * (delta.y * 0.008 * self.distance * 0.2);
        }

        // Mouse wheel: Zoom
        let wheel = mouse_wheel().1;
        if wheel != 0.0 {
            self.distance = (self.distance - wheel * 0.5).clamp(2.0, 35.0);
        }
    }

    fn camera(&self) -> Camera3D {
        let pos = self.target
            + vec3(
                self.distance * self.pitch.cos() * self.yaw.sin(),
                self.distance * self.pitch.sin(),
                self.distance * self.pitch.cos() * self.yaw.cos(),
            );
        Camera3D {
            position: pos,
            target: self.target,
            up: vec3(0.0, 1.0, 0.0),
            ..Default::default()
        }
    }
}

/// Simulation state
struct SimState {
    params: PowerSystemParams,
    particles: Vec<TrajectoryParticle>,
    orbit: CameraOrbit,
    show_solid: bool,
    show_planes: bool,
    show_contours: bool,
    show_lateral: bool,
    show_trajectories: bool,
    sim_speed: f32,
    sim_running: bool,
    time: f32,
    // Sliders for particle spawn
    spawn_z1: f32,
    spawn_z2: f32,
}

impl Default for SimState {
    fn default() -> Self {
        let params = PowerSystemParams::default();
        // Default particles at different initial conditions
        let particles = vec![
            TrajectoryParticle::new(0.8, 1.2, Color::new(1.0, 0.85, 0.0, 1.0)),
            TrajectoryParticle::new(-0.5, -1.5, Color::new(1.0, 0.3, 0.6, 1.0)),
            TrajectoryParticle::new(1.2, -0.8, Color::new(0.3, 1.0, 0.5, 1.0)),
        ];
        Self {
            params,
            particles,
            orbit: CameraOrbit::default(),
            show_solid: true,
            show_planes: true,
            show_contours: true,
            show_lateral: true,
            show_trajectories: true,
            sim_speed: 1.0,
            sim_running: true,
            time: 0.0,
            spawn_z1: 0.6,
            spawn_z2: 1.0,
        }
    }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Lyapunov Stability — Power System Frequency Control".to_string(),
        window_width: 1440,
        window_height: 900,
        window_resizable: true,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut state = SimState::default();

    loop {
        // ── egui UI ──
        let mut egui_wants_input = false;
        egui_macroquad::ui(|egui_ctx| {
            egui_wants_input = egui_ctx.wants_pointer_input() || egui_ctx.wants_keyboard_input();

            egui::Window::new("Lyapunov Stability Control")
                .default_pos([10.0, 10.0])
                .default_width(310.0)
                .resizable(true)
                .show(egui_ctx, |ui| {
                    ui.heading("Power System Parameters");
                    ui.separator();

                    ui.label("Inertia Constant M (s):");
                    ui.add(egui::Slider::new(&mut state.params.inertia, 0.02..=0.5).step_by(0.01));

                    ui.label("Damping Coefficient D (pu):");
                    ui.add(egui::Slider::new(&mut state.params.damping, 0.0..=0.3).step_by(0.005));

                    ui.label("Mechanical Power Pm (pu):");
                    let pm_changed = ui.add(egui::Slider::new(&mut state.params.mech_power, 0.1..=1.1).step_by(0.01)).changed();

                    ui.label("Max Electrical Power Pmax (pu):");
                    let pmax_changed = ui.add(egui::Slider::new(&mut state.params.max_elec_power, 0.5..=2.0).step_by(0.01)).changed();

                    if pm_changed || pmax_changed {
                        state.params.update_equilibrium();
                    }

                    ui.separator();
                    ui.heading("Level Set Controls");

                    let v_crit = state.params.critical_energy();
                    ui.label(format!("Critical Energy V_cr: {:.3}", v_crit));

                    ui.label("V_invar (invariant level):");
                    ui.add(egui::Slider::new(&mut state.params.v_invar, 0.01..=v_crit).step_by(0.01));

                    ui.label("V_lim (outer limit):");
                    ui.add(egui::Slider::new(&mut state.params.v_lim, 0.05..=v_crit * 1.2).step_by(0.01));

                    ui.separator();
                    ui.heading("Visualization & Camera");

                    ui.checkbox(&mut state.show_solid, "Solid Surface");
                    ui.checkbox(&mut state.show_planes, "Level Planes");
                    ui.checkbox(&mut state.show_contours, "Sublevel Contours");
                    ui.checkbox(&mut state.show_lateral, "Lateral View Panel");
                    ui.checkbox(&mut state.show_trajectories, "Show Trajectories");

                    ui.add_space(4.0);
                    ui.checkbox(&mut state.orbit.auto_rotate, "Auto-Rotate 3D Camera");
                    if ui.button("Reset Camera View").clicked() {
                        state.orbit.reset();
                    }
                    ui.label(egui::RichText::new("Controls: Left-drag rotate | Right-drag pan | Scroll zoom").weak().size(11.0));

                    ui.separator();
                    ui.heading("Trajectory Simulation");

                    ui.horizontal(|ui| {
                        if ui.button(if state.sim_running { "Pause" } else { "Run" }).clicked() {
                            state.sim_running = !state.sim_running;
                        }
                        if ui.button("Reset").clicked() {
                            state.particles.clear();
                            state.time = 0.0;
                        }
                    });

                    ui.label("Simulation Speed:");
                    ui.add(egui::Slider::new(&mut state.sim_speed, 0.1..=5.0).step_by(0.1));

                    ui.separator();
                    ui.label("Spawn Initial Condition:");
                    ui.add(egui::Slider::new(&mut state.spawn_z1, -2.0..=2.0).text("z1_0").step_by(0.05));
                    ui.add(egui::Slider::new(&mut state.spawn_z2, -3.0..=3.0).text("z2_0").step_by(0.05));

                    let v_spawn = state.params.lyapunov_value(state.spawn_z1, state.spawn_z2);
                    ui.label(format!("V(z1_0, z2_0) = {:.4}", v_spawn));

                    let colors = [
                        Color::new(1.0, 0.85, 0.0, 1.0),
                        Color::new(1.0, 0.3, 0.6, 1.0),
                        Color::new(0.3, 1.0, 0.5, 1.0),
                        Color::new(1.0, 0.5, 0.1, 1.0),
                        Color::new(0.6, 0.3, 1.0, 1.0),
                        Color::new(0.1, 0.9, 0.9, 1.0),
                    ];

                    if ui.button("Add Trajectory").clicked() {
                        let ci = state.particles.len() % colors.len();
                        state.particles.push(TrajectoryParticle::new(
                            state.spawn_z1,
                            state.spawn_z2,
                            colors[ci],
                        ));
                    }

                    ui.separator();
                    ui.heading("Theory");
                    ui.label(format!("δₛ = {:.3} rad ({:.1}°)", state.params.delta_s, state.params.delta_s.to_degrees()));
                    ui.label(format!("δ_uep = {:.3} rad", std::f32::consts::PI - state.params.delta_s));

                    let (z2_min, z2_lim) = find_z2_bounds(&state.params);
                    ui.label(format!("z₂_min = {:.3}", z2_min));
                    ui.label(format!("z₂_lim = {:.3}", z2_lim));

                    ui.label(format!("Active trajectories: {}", state.particles.iter().filter(|p| p.active).count()));
                });
        });

        // ── Physics update ──
        if state.sim_running {
            let dt = get_frame_time().min(0.033) * state.sim_speed;
            state.time += dt;
            let params_clone = state.params.clone();
            for particle in &mut state.particles {
                // Sub-step for stability
                let sub_steps = 4;
                let sub_dt = dt / sub_steps as f32;
                for _ in 0..sub_steps {
                    particle.step(&params_clone, sub_dt);
                }
            }
        }

        // ── Camera ──
        state.orbit.update(egui_wants_input, get_frame_time());

        // ── 3D Rendering ──
        clear_background(Color::new(0.04, 0.04, 0.08, 1.0));

        set_camera(&state.orbit.camera());

        // Ground grid
        draw_grid(20, 0.5, Color::new(0.15, 0.15, 0.2, 0.4), Color::new(0.1, 0.1, 0.15, 0.3));

        // Axes
        draw_axes();

        // Lyapunov surface
        draw_lyapunov_surface(&state.params, state.show_solid);

        // Level planes
        if state.show_planes {
            draw_level_planes(&state.params);
        }

        // Sublevel contours on ground
        if state.show_contours {
            draw_sublevel_contour(&state.params, state.params.v_invar, Color::new(0.3, 0.5, 1.0, 0.9));
            draw_sublevel_contour(&state.params, state.params.v_lim, Color::new(0.9, 0.2, 0.2, 0.9));
            // Extra contour for the "S" sublevel set (at a small value)
            draw_sublevel_contour(&state.params, state.params.v_invar * 0.4, Color::new(0.2, 0.4, 1.0, 0.6));
        }

        // Markers
        draw_markers(&state.params);

        // Trajectories
        if state.show_trajectories {
            draw_trajectories(&state.particles, &state.params);
        }

        // ── 2D Overlay ──
        set_default_camera();

        // Lateral view
        if state.show_lateral {
            draw_lateral_view(&state.params, &state.particles);
        }

        // Title overlay
        draw_text(
            "Lyapunov Stability — SMIB Frequency Control",
            15.0, screen_height() - 15.0, 18.0,
            Color::new(0.5, 0.7, 1.0, 0.7),
        );

        // Energy readout for active particles
        let mut y_off = 50.0;
        for (i, p) in state.particles.iter().enumerate() {
            if p.active {
                let v = state.params.lyapunov_value(p.z1, p.z2);
                let dv = state.params.lyapunov_derivative(p.z1, p.z2);
                draw_text(
                    &format!("T{}: V={:.4}  dV/dt={:.4}", i, v, dv),
                    15.0, screen_height() - y_off, 14.0,
                    p.color,
                );
                y_off += 18.0;
            }
        }

        // Render egui on top
        egui_macroquad::draw();

        next_frame().await
    }
}
