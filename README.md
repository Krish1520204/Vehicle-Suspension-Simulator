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

---

### 1.5 Equations of Motion (Newton-Euler)
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
├── config/
│   └── get_vehicle_params.m            # Sedan, SUV, and Sport parameter configurations
├── models/
│   ├── half_car_4dof_ode.m             # Core 8-state ODE function for ode45
│   └── calculate_outputs.m             # Post-processing: forces, accelerations, ride metrics
├── road_profiles/
│   ├── get_road_profile.m              # Road dispatcher
│   ├── road_bump.m                     # Cosine speed bump with continuous derivative
│   ├── road_step.m                     # Smooth sigmoid step profile
│   ├── road_sine.m                     # Undulating harmonic road profile
│   └── road_iso8608.m                  # ISO 8608 random road roughness generator
├── simulation/
│   └── run_simulation.m                # Encapsulates ODE45 call, tolerances, and outputs
├── utils/
│   ├── validate_model.m                # Eigenvalue, modal frequencies & damping analysis
│   └── plot_simulation_results.m       # 8-panel publication-quality results plot
└── outputs/
    └── simulation_results.png          # Generated simulation plots
```

---

## 3. How to Run

1. Open MATLAB.
2. Navigate to the project root:
   ```matlab
   cd('e:\Semester 3\Projects\VehicleSuspension');
   ```
3. Run the main driver:
   ```matlab
   main
   ```

### 3.1 Customizing Scenarios in `main.m`
- **Vehicle presets:**
  ```matlab
  params = get_vehicle_params('sedan'); % Options: 'sedan', 'suv', 'sport'
  ```
- **Road profiles:**
  ```matlab
  road_cfg.type = 'bump';    % Options: 'bump', 'step', 'sine', 'iso8608'
  road_cfg.v    = 12.5;      % Forward speed in m/s (12.5 m/s = 45 km/h)
  ```
