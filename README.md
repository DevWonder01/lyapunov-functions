# Transient Energy & Lyapunov Function Stability Engine for Power System Frequency Control

An interactive, high-fidelity 3D numerical simulation in Rust demonstrating direct stability certification of non-linear dynamical systems using **Lyapunov Energy Functions** and **Transient Energy Fu[...] 

---

## Table of Contents

1. [Executive Summary & Motivation](#-executive-summary--motivation)
2. [Theoretical & Mathematical Formulation](#-theoretical--mathematical-formulation)
   - [2.1 Synchronous Generator Swing Equation](#21-synchronous-generator-swing-equation)
   - [2.2 State-Space Deviation Coordinates](#22-state-space-deviation-coordinates)
   - [2.3 Lyapunov Candidate (Transient Energy Function)](#23-lyapunov-candidate-transient-energy-function)
   - [2.4 Stability Proof via LaSalle's Invariance Principle](#24-stability-proof-via-lasalles-invariance-principle)
   - [2.5 Unstable Equilibrium Points (UEP) & Critical Energy](#25-unstable-equilibrium-points-uep--critical-energy)
   - [2.6 Invariant Sublevel Sets & Boundary Markers](#26-invariant-sublevel-sets--boundary-markers)
3. [Numerical Methods & Algorithms](#-numerical-methods--algorithms)
   - [3.1 4th-Order Runge-Kutta (RK4) ODE Integration](#31-4th-order-runge-kutta-rk4-ode-integration)
   - [3.2 Dynamic 3D Mesh & Topology Generation](#32-dynamic-3d-mesh--topology-generation)
   - [3.3 Sublevel Set Contour Extraction](#33-sublevel-set-contour-extraction)
4. [Software Architecture & Module Overview](#-software-architecture--module-overview)
   - [4.1 `physics.rs`](#41-physicsrs)
   - [4.2 `renderer.rs`](#42-rendererrs)
   - [4.3 `main.rs`](#43-mainrs)
5. [User Interface & Simulation Control](#-user-interface--simulation-control)
   - [5.1 3D Orbit Camera Controls](#51-3d-orbit-camera-controls)
   - [5.2 Interactive `egui` Dashboard](#52-interactive-egui-dashboard)
   - [5.3 Lateral View Inset Panel](#53-lateral-view-inset-panel)
6. [Building & Execution Guide](#building--execution-guide)
7. [Academic & Technical References](#academic--technical-references)

---

## Executive Summary & Motivation

Traditional stability verification of large-scale non-linear power systems requires time-domain numerical integration of high-dimensional differential-algebraic equations (DAEs). While exact, time[...]

**Direct Methods via Lyapunov Functions** offer a mathematical framework to certify transient stability without solving trajectory equations explicitly. By defining a scalar Transient Energy Function [...]

$$S = \{ \mathbf{z} \in \mathbb{R}^2 \mid V(\mathbf{z}) \le c \}$$

If a power system perturbation (e.g., fault clearing or sudden load change) leaves the state $\mathbf{z}(t_{clear})$ inside an invariant sublevel set $S$, the system is mathematically guaranteed t[...]

This repository provides a high-performance, interactive 3D simulation of a **Single-Machine Infinite-Bus (SMIB)** system under frequency control, visualising the 3D energy surface $V(z_1, z_2)$, [...]

---

## Theoretical & Mathematical Formulation

### 2.1 Synchronous Generator Swing Equation

The mechanical and electrical dynamics of a single synchronous machine connected to an infinite bus through a transmission network are described by the non-linear swing equation:

```
  M · d²δ/dt² + D · dδ/dt = Pm - Pmax · sin(δ)
```

$$\mathbf{M \cdot \ddot{\delta} + D \cdot \dot{\delta} = P_m - P_{max} \cdot \sin(\delta)}$$

Where:
- **`M`**: Generator moment of inertia constant ($s$).
- **`D`**: Damping coefficient representing prime mover frequency control and damper winding losses ($pu$).
- **`δ` (delta)**: Generator rotor angle relative to the infinite bus reference frame ($rad$).
- **`Pm`**: Mechanical power supplied by the prime mover turbine ($pu$).
- **`Pmax`**: Maximum steady-state electrical power capability across the transmission line ($pu$).

### 2.2 State-Space Deviation Coordinates

At steady-state equilibrium, mechanical power equals electrical power output $P_m = P_{max} \sin(\delta_s)$. The **Stable Equilibrium Point (SEP)** angle $\delta_s$ is:

```
  δs = arcsin(Pm / Pmax)   for |Pm| <= Pmax
```

We define state deviation variables relative to the SEP $p_0 = (0, 0)$:

- **Rotor Angle Deviation ($z_1$)**:
  $$z_1 = \delta - \delta_s$$
- **Rotor Speed Deviation ($z_2$ / $\omega$)**:
  $$z_2 = \dot{\delta} = \omega$$

Substituting $z_1$ and $z_2$ into the swing dynamics yields the state-space model:

```
  dz₁/dt = z₂
  dz₂/dt = (1 / M) · [ Pm - Pmax · sin(δs + z₁) - D · z₂ ]
```

$$\begin{aligned}
\dot{z}_1 &= z_2 \\
\dot{z}_2 &= \frac{1}{M} \left( P_m - P_{max} \sin(\delta_s + z_1) - D z_2 \right)
\end{aligned}$$

---

### 2.3 Lyapunov Candidate (Transient Energy Function)

The Transient Energy Function $V(z_1, z_2)$ is constructed as the sum of **Kinetic Energy** $V_k$ and **Potential Energy** $V_p$:

```
  V(z₁, z₂) = V_kinetic(z₂) + V_potential(z₁)
```

1. **Kinetic Energy Component ($V_k$)**:
   $$V_k(z_2) = \frac{1}{2} M z_2^2$$

2. **Potential Energy Component ($V_p$)**:
   $$V_p(z_1) = \int_0^{z_1} \left( P_{max} \sin(\delta_s + \xi) - P_m \right) d\xi = P_{max} \left[ \cos(\delta_s) - \cos(\delta_s + z_1) \right] - P_m z_1$$

Summing both components yields the total Lyapunov energy function:

```
  V(z₁, z₂) = (1/2) · M · z₂²  +  Pmax · [ cos(δs) - cos(δs + z₁) ] - Pm · z₁
```

$$\mathbf{V(z_1, z_2) = \frac{1}{2} M z_2^2 + P_{max} \left[ \cos(\delta_s) - \cos(\delta_s + z_1) \right] - P_m z_1}$$

---

### 2.4 Stability Proof via LaSalle's Invariance Principle

To verify that $V(z_1, z_2)$ is a valid Lyapunov function, we examine its properties:

1. **Positive Definiteness**:
   - $V(0, 0) = 0$ at the SEP $p_0$.
   - $V(z_1, z_2) > 0$ for all non-zero $(z_1, z_2)$ in the neighborhood of $p_0$.

2. **Time Derivative along System Trajectories ($\dot{V}$)**:
   We evaluate the total time derivative $\dot{V}(z_1, z_2) = \frac{\partial V}{\partial z_1} \dot{z}_1 + \frac{\partial V}{\partial z_2} \dot{z}_2$:

   $$\frac{\partial V}{\partial z_1} = P_{max} \sin(\delta_s + z_1) - P_m$$
   $$\frac{\partial V}{\partial z_2} = M z_2$$

   Substituting state dynamics $\dot{z}_1$ and $\dot{z}_2$:

   $$\begin{aligned}
   \dot{V}(z_1, z_2) &= \left( P_{max} \sin(\delta_s + z_1) - P_m \right) z_2 + (M z_2) \frac{1}{M} \left( P_m - P_{max} \sin(\delta_s + z_1) - D z_2 \right) \\
   &= \left( P_{max} \sin(\delta_s + z_1) - P_m \right) z_2 + z_2 \left( P_m - P_{max} \sin(\delta_s + z_1) \right) - D z_2^2 \\
   &= -D z_2^2 \le 0
   \end{aligned}$$

$$\mathbf{\dot{V}(z_1, z_2) = -D z_2^2 \le 0}$$

Since $D > 0$, the energy derivative $\dot{V} \le 0$ is negative semi-definite everywhere. By **LaSalle's Invariance Principle**, the set of states where $\dot{V} = 0$ corresponds to $z_2 = 0$. Pluggi[...] 

---

### 2.5 Unstable Equilibrium Points (UEP) & Critical Energy

The boundary of the Region of Attraction (ROA) is determined by the **controlling Unstable Equilibrium Point (UEP)** $p_{lim}$.

At the UEP, mechanical power equals electrical power on the unstable slope of the power-angle curve:

```
  δ_uep = π - δs
```

In error coordinates $z_1 = \delta - \delta_s$:

```
  z₁_uep = π - 2 · δs
  p_lim  = (z₁_uep, 0)
```

The **Critical Energy ($V_{cr}$)** defining the theoretical stability boundary is:

$$V_{cr} = V(z_{1, \text{uep}}, 0) = P_{max} \left[ \cos(\delta_s) - \cos(\pi - \delta_s) \right] - P_m (\pi - 2\delta_s) = 2 P_{max} \cos(\delta_s) - P_m (\pi - 2\delta_s)$$

Any fault that leaves the system with energy $V > V_{cr}$ will cause loss of synchronism (pole slipping).

---

### 2.6 Invariant Sublevel Sets & Boundary Markers

```
                           Lateral View Energy Section V(0, z₂)
         V ^
           |                 /  Plane_lim  (V_lim)  \
   V_lim  -+----------------/------------------------\-----------------
           |               /   Plane_invar (V_invar)  \
  V_invar -+--------------/----------------------------\--------------
           |             (        Sublevel Set S        )
           |              \                            /
         0 -+--------------\------------p₀------------/------------------> z₂
                           z₂lim       z₂min         z₂lim
```

- **Invariant Sublevel Set ($S$)**: $\Omega_c = \{ (z_1, z_2) \mid V(z_1, z_2) \le c \}$.
- **Invariant Level ($V_{invar}$)**: Represents a conservative inner security threshold ($V_{invar} < V_{cr}$).
- **Outer Limit Level ($V_{lim}$)**: Represents an upper operating bound ($V_{lim} \le V_{cr}$).
- **Speed Deviation Markers ($z_{2\min}, z_{2\lim}$)**: The speed bounds at $z_1 = 0$ for a given energy level $V_{\text{level}}$ are found by setting $z_1 = 0$ ($V_p(0) = 0$):

  $$V_{\text{level}} = \frac{1}{2} M z_2^2 + 0 \implies z_{2, \text{bound}} = \sqrt{\frac{2 \cdot V_{\text{level}}}{M}}$$

  - $z_{2\min} = \sqrt{2 \cdot V_{invar} / M}$
  - $z_{2\lim} = \sqrt{2 \cdot V_{lim} / M}$

---

## Algorithm Architecture & Design

### 3.1 4th-Order Runge-Kutta (RK4) ODE Integration

State trajectory particles are integrated using explicit 4th-order Runge-Kutta steps for high energy conservation accuracy:

$$\begin{aligned}
k_1 &= f(\mathbf{z}_n) \\
k_2 &= f(\mathbf{z}_n + \frac{\Delta t}{2} k_1) \\
k_3 &= f(\mathbf{z}_n + \frac{\Delta t}{2} k_2) \\
k_4 &= f(\mathbf{z}_n + \Delta t \, k_3) \\
\mathbf{z}_{n+1} &= \mathbf{z}_n + \frac{\Delta t}{6} (k_1 + 2 k_2 + 2 k_3 + k_4)
\end{aligned}$$

### 3.2 Dynamic 3D Mesh & Topology Generation

The 3D surface $V(z_1, z_2)$ is constructed per-frame on a regular grid $(N \times N = 60 \times 60)$:

1. Evaluate $V(z_{1,i}, z_{2,j})$ for each grid point.
2. Build quad primitives out of 2 triangles using custom `Vertex` structures containing:
   - `position`: `Vec3` coordinate $(z_1, \text{scale} \cdot V, z_2)$.
   - `color`: `[u8; 4]` converted from HSL palette based on normalized energy level $V / V_{max}$.
   - `normal`: `Vec4` surface normal.
3. Draw custom wireframe lines across grid edges for clear 3D depth perception.

### 3.3 Sublevel Set Contour Extraction

Ground-plane sublevel contours for $V(z_1, z_2) = c$ are computed analytically by sweeping $z_1 \in [-3, 3]$:

$$z_2(z_1) = \pm \sqrt{\frac{2 \cdot \max(0, c - V_p(z_1))}{M}}$$

The positive and negative roots form a continuous closed contour loop rendered on the ground plane.

---

## Software Architecture & Module Overview

The project is structured into 3 modular Rust source files:

```
lyapunov-functions/
├── Cargo.toml          # Package configuration & dependencies
├── README.md           # Theoretical documentation
└── src/
    ├── main.rs         # Entry point, 3D orbit camera, main loop & egui interface
    ├── physics.rs      # Swing equation dynamics, Lyapunov TEF engine & RK4 solver
    └── renderer.rs     # 3D surface mesh renderer, level planes & lateral 2D view
```

### 4.1 `physics.rs`

Contains system parameters and ODE integration logic:

- `PowerSystemParams`: Stores $M, D, P_m, P_{max}, \delta_s, V_{invar}, V_{lim}$.
- `lyapunov_value(z1, z2)`: Computes $V(z_1, z_2)$.
- `lyapunov_derivative(z1, z2)`: Computes $\dot{V}(z_1, z_2) = -D z_2^2$.
- `derivatives(z1, z2)`: Evaluates state derivatives $(\dot{z}_1, \dot{z}_2)$.
- `critical_energy()`: Calculates $V_{cr}$ at the UEP.
- `TrajectoryParticle`: State particle that advances using RK4 and stores trajectory history.

### 4.2 `renderer.rs`

Handles all graphics rendering pipeline operations:

- `draw_lyapunov_surface(...)`: Renders the 3D surface mesh and wireframe overlay.
- `draw_level_planes(...)`: Renders semi-transparent horizontal planes at $V_{invar}$ and $V_{lim}$.
- `draw_sublevel_contour(...)`: Draws closed 2D contour paths on the ground plane.
- `draw_markers(...)`: Displays equilibrium $p_0$, UEP $p_{lim}$, and dashed marker lines ($z_{2\min}, z_{2\lim}$).
- `draw_lateral_view(...)`: Renders the 2D side cross-section panel at $z_1 = 0$.

### 4.3 `main.rs`

Manages the application lifecycle:

- `CameraOrbit`: Orbital camera with mouse dragging and scroll zoom.
- `SimState`: Central application state holding system parameters, particles, and toggles.
- `egui_macroquad`: Integration loop drawing floating controls over 3D scenes.

---

## User Interface & Simulation Control

### 5.1 3D Orbit & Pan Camera Controls

- **Orbit Rotation**: Hold **Left Mouse Button** and drag to rotate the camera around the target point.
- **3D Panning**: Hold **Right Mouse Button** or **Middle Mouse Button** and drag to translate the camera target anywhere in 3D space.
- **Zoom**: Scroll **Mouse Wheel** to zoom in or out.
- **Auto-Rotate**: Check **Auto-Rotate 3D Camera** in the `egui` panel to smoothly spin the camera automatically around the energy surface.
- **Reset Camera**: Click **Reset Camera View** to restore default zoom, target, and viewing angles.

### 5.2 Interactive `egui` Dashboard

The floating UI window allows live manipulation:

- **Power System Parameters**:
  - `Inertia M`: Slider from $0.02s$ to $0.50s$.
  - `Damping D`: Slider from $0.00$ to $0.30\\,pu$.
  - `Mechanical Power Pm`: Slider from $0.10$ to $1.10\\,pu$.
  - `Max Power Pmax`: Slider from $0.50$ to $2.00\\,pu$.
- **Level Set Controls**:
  - `V_invar`: Adjust invariant inner level set threshold.
  - `V_lim`: Adjust limit level set threshold.
  - `Critical Energy V_cr`: Displays the exact theoretical UEP energy limit.
- **Trajectory Simulation**:
  - `Run / Pause`: Toggle continuous RK4 time integration.
  - `Reset`: Clear particle histories and reset time.
  - `Spawn Initial Condition`: Choose initial $(z_{1,0}, z_{2,0})$ and click **Add Trajectory**.
  - `Theory Readouts`: Displays live values of $\delta_s$, $\delta_{uep}$, $z_{2\min}$, and $z_{2\lim}$.

### 5.3 Lateral View Inset Panel

Located in the bottom-right corner, this panel renders:
- The 2D energy profile $V(0, z_2)$.
- Shaded region below $V_{lim}$.
- Horizontal lines for `Plane_invar` and `Plane_lim`.
- Vertical dashed markers for `z2min` and `z2lim`.
- Active projected positions of state trajectory particles.

---

## Building & Execution Guide

### Prerequisites

- **Rust Toolchain**: Install Rust via [rustup](https://rustup.rs/):
  ```bash
  rustc --version
  cargo --version
  ```

- **Linux System Libraries**: (Ubuntu/Debian)
  ```bash
  sudo apt update
  sudo apt install libx11-dev libxi-dev libgl1-mesa-dev pkg-config
  ```

### Compiling and Running

1. **Clone the repository**:
   ```bash
   git clone https://github.com/Simulations/lyapunov-functions.git
   cd lyapunov-functions
   ```

2. **Run in Release Mode**:
   ```bash
   cargo run --release
   ```

### WebAssembly (WASM) Build

To compile and serve the simulation in a web browser:

1. **Add the WASM target**:
   ```bash
   rustup target add wasm32-unknown-unknown
   ```

2. **Compile to WASM**:
   ```bash
   cargo build --target wasm32-unknown-unknown --release
   ```

3. **Copy the compiled WASM binary**:
   ```bash
   cp target/wasm32-unknown-unknown/release/lyapunov-functions.wasm ./
   ```

4. **Serve locally using basic HTTP server**:
   ```bash
   # Using Python 3
   python3 -m http.server 8080

   # Or using basic-http-server
   basic-http-server .
   ```

5. Open `http://localhost:8080` in your web browser to run the 3D WebGL simulation!

3. **Run in Debug Mode**:
   ```bash
   cargo run
   ```

---

## Academic & Technical References

1. **Pai, M. A.** (1989). *Energy Function Analysis of Power System Stability*. Springer US.
2. **Kundur, P.** (1994). *Power System Stability and Control*. McGraw-Hill.
3. **Chiang, H. D.** (2011). *Direct Methods for Power System Stability Assessment: Theoretical Foundations, BCU Methodologies, and Applications*. John Wiley & Sons.
4. **Khalil, H. K.** (2002). *Nonlinear Systems* (3rd ed.). Prentice Hall.
5. **LaSalle, J. P.** (1976). *The Stability of Dynamical Systems*. SIAM Regional Conference Series in Applied Mathematics.

---

## License

Distributed under the MIT License. See `LICENSE` for details.
