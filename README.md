# 4-DOF Half-Car Vehicle Suspension System Simulator (MATLAB)

A modular, research-grade MATLAB simulation framework for a **4-Degree-of-Freedom (4-DOF) Half-Car Passive Suspension System** solved using `ode45`.

---

## 1. Mathematical Foundation

### 1.1 System Architecture & Degrees of Freedom
The half-car model captures longitudinal pitch and vertical heave dynamics, representing one longitudinal track of a passenger car with front and rear suspensions:

```
                  +-----------------------------------+
                  |         Sprung Mass (m_s)         |  <--- Heave z_s, Pitch theta
                  |         Pitch Inertia (I_y)       |
                  +-----------------+-----------------+
                                    | CG
                     <------ a ---->|<------- b ----->
                    /                                 \
            +------+------+                     +------+------+
            | k_sf | c_sf | (Front Susp)        | k_sr | c_sr | (Rear Susp)
            +------+------+                     +------+------+
                   |                                   |
            [ m_uf (Wheel) ]                    [ m_ur (Wheel) ] <--- z_uf, z_ur
                   |                                   |
            +------+------+                     +------+------+
            | k_tf | c_tf | (Front Tyre)        | k_tr | c_tr | (Rear Tyre)
            +------+------+                     +------+------+
                 / \                                 / \
             ~ ~ ~ ~ ~ ~                         ~ ~ ~ ~ ~ ~
            z_0f(t) (Road)                      z_0r(t) = z_0f(t - tau)
```

The system possesses **4 degrees of freedom (4-DOF)**:
1. $z_s(t)$: Sprung mass vertical heave displacement at CG ($\text{m}$, positive upwards).
2. $\theta(t)$: Sprung mass pitch angular displacement ($\text{rad}$, positive nose-up / counter-clockwise).
3. $z_{uf}(t)$: Front unsprung mass vertical displacement ($\text{m}$, positive upwards).
4. $z_{ur}(t)$: Rear unsprung mass vertical displacement ($\text{m}$, positive upwards).

---

### 1.2 Kinematics & Suspension Attachment Points
Under the small-angle assumption ($\sin\theta \approx \theta$, $\cos\theta \approx 1$):
- **Front sprung connection point:**
  $$z_{sf} = z_s + a \theta, \quad \dot{z}_{sf} = \dot{z}_s + a \dot{\theta}$$
- **Rear sprung connection point:**
  $$z_{sr} = z_s - b \theta, \quad \dot{z}_{sr} = \dot{z}_s - b \dot{\theta}$$

Where:
- $a$: Longitudinal distance from vehicle Center of Gravity (CG) to front axle ($\text{m}$).
- $b$: Longitudinal distance from vehicle CG to rear axle ($\text{m}$).
- $L = a + b$: Total wheelbase ($\text{m}$).

---

### 1.3 Suspension Deflections & Forces
Measuring motions from static equilibrium (gravity forces are counterbalanced by initial static spring deflections):

- **Front Suspension Deflection & Relative Velocity:**
  $$\delta_{sf} = z_{uf} - z_{sf} = z_{uf} - (z_s + a \theta)$$
  $$\dot{\delta}_{sf} = \dot{z}_{uf} - \dot{z}_{sf} = \dot{z}_{uf} - (\dot{z}_s + a \dot{\theta})$$
- **Rear Suspension Deflection & Relative Velocity:**
  $$\delta_{sr} = z_{ur} - z_{sr} = z_{ur} - (z_s - b \theta)$$
  $$\dot{\delta}_{sr} = \dot{z}_{ur} - \dot{z}_{sr} = \dot{z}_{ur} - (\dot{z}_s - b \dot{\theta})$$

- **Dynamic Suspension Forces (positive pushing upwards on sprung mass):**
  $$F_{sf} = k_{sf} \delta_{sf} + c_{sf} \dot{\delta}_{sf}$$
  $$F_{sr} = k_{sr} \delta_{sr} + c_{sr} \dot{\delta}_{sr}$$

---

### 1.4 Tyre Kinematics & Dynamic Ground Forces
- **Front and Rear Tyre Deflections:**
  $$\delta_{tf} = z_{0f}(t) - z_{uf}, \quad \delta_{tr} = z_{0r}(t) - z_{ur}$$
- **Tyre Forces (positive pushing upwards on unsprung mass):**
  $$F_{tf} = k_{tf} (z_{0f} - z_{uf}) + c_{tf} (\dot{z}_{0f} - \dot{z}_{uf})$$
  $$F_{tr} = k_{tr} (z_{0r} - z_{ur}) + c_{tr} (\dot{z}_{0r} - \dot{z}_{ur})$$

- **Wheelbase Time Delay:**
  For forward speed $v$, the rear wheel hits the same road profile delayed by:
  $$\tau = \frac{L}{v} = \frac{a + b}{v}$$
  $$z_{0r}(t) = z_{0f}(t - \tau), \quad \dot{z}_{0r}(t) = \dot{z}_{0f}(t - \tau)$$

### 1.5 Multi-Bump Continuous-Time Road Profiles
The simulator provides three continuous-time multi-bump road profiles to model speed tables, rumble strips, and consecutive road undulations:

1. **Rectangular Multi-Bump:**
   - Flat-top plateau at amplitude $H$.
   - Smooth $C^1$ cosine transitions at edges over $\delta = r_{\text{smooth}} \cdot W$ to prevent step velocity singularities:
     $$z(u) = \begin{cases}
     \frac{H}{2}\left[1 - \cos\left(\pi \frac{u}{\delta}\right)\right], & 0 \le u < \delta \\
     H, & \delta \le u \le W - \delta \\
     \frac{H}{2}\left[1 + \cos\left(\pi \frac{u - (W - \delta)}{\delta}\right)\right], & W - \delta < u \le W
     \end{cases}$$

2. **Triangular Multi-Bump:**
   - Linear flank slope $m = H / (W/2)$.
   - Parabolic filleted base $[0, \delta]$ and rounded apex $[W/2 - \delta, W/2 + \delta]$ guaranteeing continuous slope $dz/dx$ without sharp apex corners:
     $$\text{Apex Rounding: } z(u) = H - \frac{m}{2\delta}\left(\frac{W}{2} - u\right)^2, \quad z'(u) = \frac{m}{\delta}\left(\frac{W}{2} - u\right)$$

3. **Half-Sinusoidal Multi-Bump:**
   - Sinusoidal pulse $z(u) = H \sin\left(\pi \frac{u}{W}\right)$ for $u \in [0, W]$.
   - Equipped with smooth $C^1$ cosine boundary tapers at entry and exit so $z(0) = 0$, $z'(0) = 0$, $z(W) = 0$, $z'(W) = 0$, enabling stable `ode45` convergence without unphysical damping force shocks.

#### 1.6 Wheelbase Time Delay & Encounter Timetable
For vehicle speed $v$ and bump pitch $P = W + S$:
- **Bump $k$ Front Wheel Encounter:**
  $$t_{f,\text{enter}}(k) = t_{\text{start}} + (k - 1)\frac{P}{v}, \quad t_{f,\text{exit}}(k) = t_{f,\text{enter}}(k) + \frac{W}{v}$$
- **Bump $k$ Rear Wheel Encounter:**
  $$t_{r,\text{enter}}(k) = t_{f,\text{enter}}(k) + \tau, \quad t_{r,\text{exit}}(k) = t_{f,\text{exit}}(k) + \tau, \quad \tau = \frac{L}{v}$$

---

### 1.7 Equations of Motion (Newton-Euler)
Around the static equilibrium configuration:

1. **Sprung Mass Heave:**
   $$m_s \ddot{z}_s = F_{sf} + F_{sr}$$
   $$\ddot{z}_s = \frac{F_{sf} + F_{sr}}{m_s}$$

2. **Sprung Mass Pitch (Moments about CG):**
   $$I_y \ddot{\theta} = a F_{sf} - b F_{sr}$$
   $$\ddot{\theta} = \frac{a F_{sf} - b F_{sr}}{I_y}$$

3. **Front Unsprung Mass (Wheel):**
   $$m_{uf} \ddot{z}_{uf} = -F_{sf} + F_{tf}$$
   $$\ddot{z}_{uf} = \frac{-F_{sf} + F_{tf}}{m_{uf}}$$

4. **Rear Unsprung Mass (Wheel):**
   $$m_{ur} \ddot{z}_{ur} = -F_{sr} + F_{tr}$$
   $$\ddot{z}_{ur} = \frac{-F_{sr} + F_{tr}}{m_{ur}}$$

---

### 1.6 State-Space Formulation for ODE45
We convert the 4 second-order ODEs into a system of 8 first-order ODEs:
$$\mathbf{x} = \begin{bmatrix} x_1 \\ x_2 \\ x_3 \\ x_4 \\ x_5 \\ x_6 \\ x_7 \\ x_8 \end{bmatrix} = \begin{bmatrix} z_s \\ \theta \\ z_{uf} \\ z_{ur} \\ \dot{z}_s \\ \dot{\theta} \\ \dot{z}_{uf} \\ \dot{z}_{ur} \end{bmatrix}$$

The state derivative vector is:
$$\dot{\mathbf{x}} = \begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \\ \dot{x}_3 \\ \dot{x}_4 \\ \dot{x}_5 \\ \dot{x}_6 \\ \dot{x}_7 \\ \dot{x}_8 \end{bmatrix} = \begin{bmatrix} x_5 \\ x_6 \\ x_7 \\ x_8 \\ \ddot{z}_s \\ \ddot{\theta} \\ \ddot{z}_{uf} \\ \ddot{z}_{ur} \end{bmatrix}$$

---

### 1.7 Analytical Modal Analysis & Validation
In matrix form without damping and external forces:
$$\mathbf{M} \ddot{\mathbf{q}} + \mathbf{K} \mathbf{q} = \mathbf{0}, \quad \mathbf{q} = [z_s, \theta, z_{uf}, z_{ur}]^T$$

$$\mathbf{M} = \text{diag}([m_s, I_y, m_{uf}, m_{ur}])$$

$$\mathbf{K} = \begin{bmatrix}
k_{sf} + k_{sr} & k_{sf} a - k_{sr} b & -k_{sf} & -k_{sr} \\
k_{sf} a - k_{sr} b & k_{sf} a^2 + k_{sr} b^2 & -k_{sf} a & k_{sr} b \\
-k_{sf} & -k_{sf} a & k_{sf} + k_{tf} & 0 \\
-k_{sr} & k_{sr} b & 0 & k_{sr} + k_{tr}
\end{bmatrix}$$

Solving generalized eigenvalue problem $\det(\mathbf{K} - \omega_n^2 \mathbf{M}) = 0$ yields 4 natural modes:
- **Mode 1 (Body Heave / Bounce):** $\approx 1.0 - 1.3\text{ Hz}$
- **Mode 2 (Body Pitch):** $\approx 1.2 - 1.8\text{ Hz}$
- **Mode 3 (Front Wheel Hop):** $\approx 10 - 12\text{ Hz}$
- **Mode 4 (Rear Wheel Hop):** $\approx 10 - 12\text{ Hz}$

---

## 2. Project Architecture & File Organization

```
VehicleSuspension/
├── README.md                           # Mathematical derivation & usage docs
├── main.m                              # Primary executable script
├── run_dashboard.m                     # Interactive UI Dashboard & Bump Analyzer launcher
├── run_animation.m                     # Top-level physics-driven 2D animation runner
├── test_multi_bumps.m                  # Comprehensive 3-profile comparison test script
├── run_parameter_study.m               # Parameter sensitivity & dynamic graph update study
├── config/
│   └── get_vehicle_params.m            # Sedan, SUV, and Sport parameter configurations
├── models/
│   ├── half_car_4dof_ode.m             # Core 8-state ODE function for ode45
│   └── calculate_outputs.m             # Post-processing: forces, accelerations, ride metrics
├── road_profiles/
│   ├── get_road_profile.m              # Unified road dispatcher
│   ├── road_multi_bump.m               # Multi-bump engine (Rectangular, Triangular, Half-Sine)
│   ├── road_rectangular.m              # Dedicated rectangular multi-bump wrapper
│   ├── road_triangular.m               # Dedicated triangular multi-bump wrapper
│   ├── road_halfsine.m                 # Dedicated half-sinusoidal multi-bump wrapper
│   ├── road_bump.m                     # Cosine speed bump with continuous derivative
│   ├── road_step.m                     # Smooth sigmoid step profile
│   ├── road_sine.m                     # Undulating harmonic road profile
│   └── road_iso8608.m                  # ISO 8608 random road roughness generator
├── simulation/
│   └── run_simulation.m                # Encapsulates ODE45 call, tolerances, and outputs
├── dashboard/
│   └── suspension_dashboard.m          # Professional interactive App Designer UI dashboard
├── plotting/
│   ├── plot_suspension_analysis.m      # Master 7-graph dashboard with numerical KPI card
│   ├── plot_road_profile.m             # Graph 1: Road displacement profile (front vs rear)
│   ├── plot_body_displacement.m        # Graph 2: Vehicle body vertical displacement (heave)
│   ├── plot_pitch_angle.m              # Graph 3: Vehicle body pitch angle
│   ├── plot_front_suspension_deflection.m # Graph 4: Front suspension deflection with limits
│   ├── plot_rear_suspension_deflection.m  # Graph 5: Rear suspension deflection with limits
│   ├── plot_body_acceleration.m        # Graph 6: Vehicle body vertical acceleration (CG & driver)
│   └── plot_wheel_displacement.m       # Graph 7: Front & rear wheel displacements vs road
├── animation/
│   ├── animate_vehicle_suspension.m    # Master animation coordinator & GIF/PNG exporter
│   ├── draw_sedan_body.m               # Sedan body, windows, B-pillar, lights, CG marker
│   ├── draw_wheel.m                    # Wheels with rotating spokes & alloy rims
│   ├── draw_suspension_spring_damper.m # Parametric coil spring & telescoping damper dashpot
│   └── draw_outdoor_scenery.m          # Sky, clouds, buildings, trees, road & bumps
├── analysis/
│   ├── calculate_kpis.m                # Quantitative KPI extraction (max heave, accel, travel, etc.)
│   └── print_performance_report.m     # Formatted console performance indicator report
├── utils/
│   ├── validate_model.m                # Eigenvalue, modal frequencies & damping analysis
│   └── plot_simulation_results.m       # 8-panel publication-quality results plot
└── outputs/
    ├── dashboard_screenshot.png        # High-resolution interactive dashboard snapshot
    ├── suspension_animation.gif        # Synchronized physics-driven animated GIF
    ├── suspension_animation_frame.png  # High-resolution animation snapshot
    ├── suspension_analysis.png         # Master 7-graph analysis with numerical KPI card
    ├── parameter_variation_damping.png # Parameter sensitivity study (soft vs nominal vs firm)
    ├── road_profiles_comparison.png    # 3-profile road input front vs rear comparison
    └── multi_bump_simulation_comparison.png # 3-profile 4-DOF dynamic comparison
```

---

## 3. How to Run

1. Open MATLAB.
2. Navigate to the project root:
   ```matlab
   cd('e:\Semester 3\Projects\VehicleSuspension');
   ```
3. **Launch the Interactive Bump Analyzer Dashboard (UIFigure):**
   ```matlab
   run_dashboard
   ```
4. Run the standalone physics-driven 2D animation:
   ```matlab
   run_animation
   ```
5. Run the primary simulation script (executes ODE45, prints KPIs, and renders the 7 graphs):
   ```matlab
   main
   ```
6. Run the parameter variation & sensitivity study (proves all graphs update when parameters change):
   ```matlab
   run_parameter_study
   ```
7. Run the multi-profile comparative analysis (tests Rectangular, Triangular, and Half-Sinusoidal consecutive bumps):
   ```matlab
   test_multi_bumps
   ```

### 3.1 Customizing Scenarios in `main.m` & `run_animation.m`
- **Vehicle presets:**
  ```matlab
  params = get_vehicle_params('sedan'); % Options: 'sedan', 'suv', 'sport'
  ```
- **Road profiles:**
  ```matlab
  road_cfg.type          = 'half_sine'; % Options: 'half_sine', 'triangular', 'rectangular', 'bump'
  road_cfg.num_bumps     = 6;           % Consecutive bump count
  road_cfg.H             = 0.055;       % Bump height in meters
  road_cfg.v             = 10.0;        % Forward speed in m/s (10 m/s = 36 km/h)
  ```

---

## 4. Physics-Driven Animated Visualization

The animation visualizes the exact, calculated states from the 4-DOF half-car model with zero hardcoded or random motion:

1. **Sedan Geometry & Aesthetics:**
   - Modeled after the reference visual guide with a sky-blue exterior finish (`[0.55, 0.77, 0.92]`).
   - Includes front and rear tinted windows, structural B-pillar, door handle recesses, front aerodynamic headlights, and red taillights.
   - Features the Center of Gravity (CG) quartered circle marker with dynamic dimensions $L_f$, $L_r$, and pitch angle indicator $\theta$.

2. **Dynamic Suspension Springs & Dampers:**
   - Front and rear suspensions render realistic mechanical coil springs (zigzag turns) and dashpot dampers (telescoping piston inside fluid cylinder).
   - The top mounts are affixed to the chassis and rotate with pitch $\theta$ and translate with heave $z_s$.
   - The bottom mounts connect directly to wheel hubs moving with independent vertical wheel displacements $z_{uf}$ and $z_{ur}$.
   - Springs and dampers dynamically stretch and compress according to calculated deflections $\delta_{sf}(t)$ and $\delta_{sr}(t)$.

3. **Wheel Kinematics & Rotation:**
   - Wheels translate vertically with unsprung masses ($z_{uf}, z_{ur}$).
   - Spoke rotation is synchronized with forward vehicle speed: $\theta_{\text{spoke}}(t) = -v \cdot t / R_w$.

4. **Multi-Bump Outdoor Environment:**
   - Outdoor background featuring gradient sky, procedural cumulus clouds, distant city skyline buildings, verge greenery, and asphalt road.
   - The multiple road bumps are drawn directly on the terrain using the mathematical bump formulation ($H, W, S$).
   - Diagonally hatched ground foundation below bumps matches the reference engineering diagram.

5. **Scale & Camera Tracking:**
   - Vehicle body length is $4.60\text{ m}$.
   - The viewing window horizontal span is set to $18.4\text{ m}$, keeping the car at exactly $25\%$ of the simulation window width.
   - The camera moves horizontally with the vehicle ($X_{\text{cam}} = X_{CG} - 8.4\text{ m}$ to $X_{CG} + 10.0\text{ m}$), smoothly tracking the sedan across consecutive bumps.
   - Real-time engineering HUD displays elapsed time, forward speed, body heave, pitch angle, suspension travel, and wheel hop.

---

## 5. Interactive MATLAB Dashboard & Bump Analyzer

The dashboard (`run_dashboard.m` / `dashboard/suspension_dashboard.m`) is built using MATLAB UI components (`uifigure`, `uiaxes`, `uidropdown`, `uislider`, `uibutton`, `uipanel`, `uilabel`, `uieditfield`) designed after the reference engineering UI guide:

1. **Top Header Card:**
   - Deep navy boxed frame: `"BUMP ANALYZER"`.

2. **Interactive Simulation Controls:**
   - **Road Profile Dropdown (`uidropdown`):** Select between `'Half-sinusoidal'`, `'Triangular'`, and `'Rectangular'`.
   - **Bump Amplitude Slider (`uislider`):** Range $15 - 110\text{ mm}$ (live value display, default $55.0\text{ mm}$).
   - **Vehicle Speed Slider (`uislider`):** Range $10 - 90\text{ km/h}$ (live $\text{km/h}$ and $\text{m/s}$ display, default $36.0\text{ km/h}$).
   - **Damping Coefficient Slider (`uislider`):** Range $600 - 3600\text{ N}\cdot\text{s/m}$ (live display, default $1600\text{ N}\cdot\text{s/m}$).
   - **Natural Frequency Slider (`uislider`):** Range $0.8 - 2.6\text{ Hz}$ (live Hz and calculated stiffness display, default $1.30\text{ Hz}$).
   - **Action Buttons (`uibutton`):**
     - `[Start]`: Solves ODE45 with current slider settings, updates all graphs and KPI readouts, and launches real-time 25 FPS animation.
     - `[Pause]`: Freezes timer playback and cursor indicators.
     - `[Resume]`: Resumes playback timer from current frame.
     - `[Reset]`: Restores default vehicle & bump parameters, re-simulates, and returns to $t = 0$.

3. **Vehicle Physical Specifications Grid:**
   - 2-column parameter table matching the reference UI layout:
     - Percentage of Front Weight ($\%$)
     - Front & Rear Suspension Stiffness ($k_{sf}, k_{sr}$) dynamically calculated from $f_n$
     - Front & Rear Damping Coefficients ($c_{sf}, c_{sr}$)
     - Sprung Mass ($m_s$) and Unsprung Masses ($m_{uf}, m_{ur}$)
     - Tyre Stiffnesses ($k_{tf}, k_{tr}$), Pitch Inertia ($I_y$), and Wheelbase ($L$)

4. **Numerical Performance Indicators (Live KPIs):**
   - Maximum Body Heave Displacement ($\text{mm}$)
   - Maximum Vertical Acceleration at CG ($\text{m/s}^2$ and $g$)
   - Driver Seat Acceleration (RMS, $\text{m/s}^2$)
   - ISO 2631 Ride Comfort Level classification
   - Maximum Pitch Angle ($\deg$)
   - Front & Rear Suspension Travel Utilization (vs $80\text{ mm}$ bumpstop limit)
   - Dynamic Road Grip Status (Safe vs Transient Tyre Lift-off Warning)

5. **Live Response Graphs & Bump Geometry:**
   - **Road Bump Geometry:** Front vs rear wheel road height profiles $z_{0f}, z_{0r}$ (faithful to reference UI).
   - **Live Body Displacement:** Heave $z_s$, front connection $z_{sf}$, rear connection $z_{sr}$.
   - **Live Acceleration:** CG vertical acceleration and driver seat acceleration with ISO comfort thresholds.
   - **Suspension Deflection:** Front and rear travels with $\pm 80\text{ mm}$ bumpstop limits.
   - **Pitch Response:** Pitch angle $\theta$ and pitch angular velocity $\dot{\theta}$.
   - **Synchronized Time Cursors:** Vertical red dashed lines move across all 5 graphs in real-time as the vehicle traverses the bumps.
