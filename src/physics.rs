/// Lyapunov Stability Physics Engine
///
/// Models the Single-Machine Infinite-Bus (SMIB) swing equation dynamics
/// and the associated Transient Energy Function (Lyapunov function).
///
/// Swing equation: M·δ̈ + D·δ̇ = Pm - Pmax·sin(δ)
/// State variables: z₁ = δ - δₛ (rotor angle deviation), z₂ = ω (speed deviation)
///
/// Lyapunov (Energy) function:
///   V(z₁, z₂) = ½·M·z₂² + Pmax·(cos(δₛ) - cos(δₛ + z₁)) - Pm·z₁
///
/// The surface V(z₁, z₂) is a bowl with minimum at the stable equilibrium p₀ = (0, 0).
/// Sublevel sets {(z₁, z₂) : V(z₁, z₂) ≤ c} define regions of attraction.

use macroquad::prelude::*;

/// Power system parameters for the SMIB model
#[derive(Clone)]
pub struct PowerSystemParams {
    /// Generator inertia constant M (seconds)
    pub inertia: f32,
    /// Damping coefficient D (pu)
    pub damping: f32,
    /// Mechanical power input Pm (pu)
    pub mech_power: f32,
    /// Maximum electrical power Pmax (pu)
    pub max_elec_power: f32,
    /// Stable equilibrium angle δₛ (radians)
    pub delta_s: f32,
    /// Level for the invariant plane V_invar
    pub v_invar: f32,
    /// Level for the outer limit plane V_lim
    pub v_lim: f32,
}

impl Default for PowerSystemParams {
    fn default() -> Self {
        let mech_power = 0.6_f32;
        let max_elec_power = 1.2_f32;
        // Stable equilibrium: Pm = Pmax * sin(δs) → δs = arcsin(Pm/Pmax)
        let delta_s = (mech_power / max_elec_power).asin();
        Self {
            inertia: 0.15,
            damping: 0.05,
            mech_power,
            max_elec_power,
            delta_s,
            v_invar: 0.25,
            v_lim: 0.55,
        }
    }
}

impl PowerSystemParams {
    /// Recompute δₛ from current Pm and Pmax
    pub fn update_equilibrium(&mut self) {
        let ratio = (self.mech_power / self.max_elec_power).clamp(-1.0, 1.0);
        self.delta_s = ratio.asin();
    }

    /// Evaluate the Lyapunov function V(z₁, z₂) at a point
    ///
    /// V = ½·M·z₂² + Pmax·(cos(δₛ) - cos(δₛ + z₁)) - Pm·z₁
    ///
    /// z₁ = δ - δₛ (angle deviation from equilibrium)
    /// z₂ = ω     (speed deviation)
    pub fn lyapunov_value(&self, z1: f32, z2: f32) -> f32 {
        let kinetic = 0.5 * self.inertia * z2 * z2;
        let potential = self.max_elec_power * (self.delta_s.cos() - (self.delta_s + z1).cos())
            - self.mech_power * z1;
        kinetic + potential
    }

    /// Compute the time derivative dV/dt along system trajectories
    /// For the damped system: dV/dt = -D·ω² ≤ 0
    pub fn lyapunov_derivative(&self, _z1: f32, z2: f32) -> f32 {
        -self.damping * z2 * z2
    }

    /// Compute the gradient of V for visualization
    /// ∂V/∂z₁ = Pmax·sin(δₛ + z₁) - Pm
    /// ∂V/∂z₂ = M·z₂
    #[allow(dead_code)]
    pub fn lyapunov_gradient(&self, z1: f32, z2: f32) -> (f32, f32) {
        let dv_dz1 = self.max_elec_power * (self.delta_s + z1).sin() - self.mech_power;
        let dv_dz2 = self.inertia * z2;
        (dv_dz1, dv_dz2)
    }

    /// State derivatives for the swing equation (for trajectory simulation)
    /// dz₁/dt = z₂
    /// dz₂/dt = (1/M)·(Pm - Pmax·sin(δₛ + z₁) - D·z₂)
    pub fn derivatives(&self, z1: f32, z2: f32) -> (f32, f32) {
        let dz1 = z2;
        let dz2 = (self.mech_power - self.max_elec_power * (self.delta_s + z1).sin()
            - self.damping * z2)
            / self.inertia;
        (dz1, dz2)
    }

    /// Compute the unstable equilibrium point (UEP) angle
    /// At UEP: Pm = Pmax·sin(δ_uep) and δ_uep = π - δₛ
    pub fn uep_angle_deviation(&self) -> f32 {
        std::f32::consts::PI - 2.0 * self.delta_s
    }

    /// Critical energy at the UEP (maximum V for the stability boundary)
    pub fn critical_energy(&self) -> f32 {
        let z1_uep = self.uep_angle_deviation();
        self.lyapunov_value(z1_uep, 0.0)
    }
}

/// A trajectory particle that follows the system dynamics
#[derive(Clone)]
pub struct TrajectoryParticle {
    pub z1: f32,
    pub z2: f32,
    pub history: Vec<(f32, f32)>,
    pub energy_history: Vec<f32>,
    pub color: Color,
    pub active: bool,
    pub age: f32,
}

impl TrajectoryParticle {
    pub fn new(z1: f32, z2: f32, color: Color) -> Self {
        Self {
            z1,
            z2,
            history: vec![(z1, z2)],
            energy_history: Vec::new(),
            color,
            active: true,
            age: 0.0,
        }
    }

    /// Advance the particle using RK4 integration
    pub fn step(&mut self, params: &PowerSystemParams, dt: f32) {
        if !self.active {
            return;
        }
        self.age += dt;

        // RK4 integration for accuracy
        let (k1_z1, k1_z2) = params.derivatives(self.z1, self.z2);
        let (k2_z1, k2_z2) = params.derivatives(
            self.z1 + 0.5 * dt * k1_z1,
            self.z2 + 0.5 * dt * k1_z2,
        );
        let (k3_z1, k3_z2) = params.derivatives(
            self.z1 + 0.5 * dt * k2_z1,
            self.z2 + 0.5 * dt * k2_z2,
        );
        let (k4_z1, k4_z2) = params.derivatives(
            self.z1 + dt * k3_z1,
            self.z2 + dt * k3_z2,
        );

        self.z1 += dt * (k1_z1 + 2.0 * k2_z1 + 2.0 * k3_z1 + k4_z1) / 6.0;
        self.z2 += dt * (k1_z2 + 2.0 * k2_z2 + 2.0 * k3_z2 + k4_z2) / 6.0;

        let v = params.lyapunov_value(self.z1, self.z2);
        self.energy_history.push(v);

        // Keep history bounded
        self.history.push((self.z1, self.z2));
        if self.history.len() > 800 {
            self.history.remove(0);
        }
        if self.energy_history.len() > 800 {
            self.energy_history.remove(0);
        }

        // Deactivate if out of bounds
        if self.z1.abs() > 4.0 || self.z2.abs() > 6.0 {
            self.active = false;
        }
    }
}

/// Compute contour points for V(z₁, z₂) = level on the z₁-z₂ plane
/// Uses marching approach: for each z₁, solve for z₂ analytically from
/// V = ½·M·z₂² + Vp(z₁) → z₂ = ±√(2·(level - Vp(z₁))/M)
pub fn compute_contour(params: &PowerSystemParams, level: f32, num_points: usize) -> Vec<(f32, f32)> {
    let mut points = Vec::new();
    let z1_range = 3.0_f32;

    for i in 0..num_points {
        let z1 = -z1_range + 2.0 * z1_range * (i as f32 / num_points as f32);
        let vp = params.max_elec_power * (params.delta_s.cos() - (params.delta_s + z1).cos())
            - params.mech_power * z1;
        let remainder = level - vp;
        if remainder >= 0.0 {
            let z2 = (2.0 * remainder / params.inertia).sqrt();
            points.push((z1, z2));
        }
    }

    // Add the negative z₂ branch in reverse
    let mut neg_points: Vec<(f32, f32)> = Vec::new();
    for i in (0..num_points).rev() {
        let z1 = -z1_range + 2.0 * z1_range * (i as f32 / num_points as f32);
        let vp = params.max_elec_power * (params.delta_s.cos() - (params.delta_s + z1).cos())
            - params.mech_power * z1;
        let remainder = level - vp;
        if remainder >= 0.0 {
            let z2 = -(2.0 * remainder / params.inertia).sqrt();
            neg_points.push((z1, z2));
        }
    }
    points.extend(neg_points);
    points
}

/// Find z₂_min and z₂_lim for the lateral view markers
pub fn find_z2_bounds(params: &PowerSystemParams) -> (f32, f32) {
    // z₂_min: the z₂ where V reaches V_invar at z₁=0
    let v_at_origin_z1 = 0.0_f32;
    let vp = params.max_elec_power * (params.delta_s.cos() - (params.delta_s + v_at_origin_z1).cos())
        - params.mech_power * v_at_origin_z1;
    let remainder_invar = params.v_invar - vp;
    let z2_min = if remainder_invar > 0.0 {
        (2.0 * remainder_invar / params.inertia).sqrt()
    } else {
        0.0
    };

    let remainder_lim = params.v_lim - vp;
    let z2_lim = if remainder_lim > 0.0 {
        (2.0 * remainder_lim / params.inertia).sqrt()
    } else {
        0.0
    };

    (z2_min, z2_lim)
}
