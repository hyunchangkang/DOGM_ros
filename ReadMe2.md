# dogm_ros (Dual Radar Version)

`dogm_ros` is a particle filter-based dynamic occupancy grid mapping (DOGM) package optimized for **Dual Radar** and LiDAR sensor fusion. It estimates the dynamic/static state and velocity of grid cells by leveraging overlapping radar fields of view to reconstruct true 2D velocity vectors.

---

## Key Features
- **Dual Radar Fusion**: Utilizes two radar sensors to reconstruct full 2D velocity vectors ($v_x, v_y$) in overlapping regions using a Least Squares Solver.
- **Hybrid Likelihood Model**: Switches between 2D vector comparison (precision mode) and 1D Doppler projection (survival mode) based on sensor coverage.
- **Adaptive Birth Process**: Dynamically adjusts the ratio of dynamic/static particles to accelerate static classification in radar-blind spots.
- **Robust Tracking**: Maintains tracking of laterally moving objects even when they leave the overlapping radar zone.

---

## Algorithm Details

### Overview
The core algorithm has been upgraded to a **Hybrid Dual-Radar Particle Filter**. It distinguishes between **Overlap Zones** (where two radars cross) and **Single Zones** (only one radar covers) to maximize both accuracy and robustness.

The updated pipeline follows these steps:

1.  **Measurement Grid & Solver**: Aggregates radar points from neighbors and solves for true velocity.
2.  **Particle Prediction**: Propagates particles with ego-motion and acceleration compensation.
3.  **Hybrid Weight Update**: Updates particle weights using the best available information (2D Vector vs. 1D Doppler).
4.  **Adaptive Birth**: Generates new particles with a bias towards static or dynamic based on radar presence.
5.  **Velocity Estimation & Classification**: Final decision based on particle consensus and Solver results.

### Detailed Function Analysis

#### 1. `generateMeasurementGrid` (Pre-processing & Solver)
This stage converts raw sensor data into grid features.
-   **Spatial Aggregation**: For each cell, radar points are gathered from a search radius (`radar_hint_search_radius`).
-   **Velocity Solver ($Ax=b$)**:
    -   If points from **both sensors** are present (Overlap Zone), a **Least Squares Solver** reconstructs the true 2D velocity vector ($v_x, v_y$).
    -   If only one sensor is present (Single Zone), raw Doppler data is stored for projection-based comparison.
    -   If no radar data exists, the cell is flagged for static prior application.

#### 2. `predict` (Prediction)
Propagates particles using a constant velocity model.
-   **Ego-Motion Compensation**: Compensates for robot velocity ($v_{ego}$) and acceleration ($a_{ego}$) to keep particles consistent in the world frame.
-   **Velocity Damping**: Applies a damping factor (`velocity_damping_factor`) to slow particles if they have not been updated for a long time, preventing ghost noise.

#### 3. `updateWeights` (Hybrid Likelihood)
This is the core logic that adapts to the quality of sensor data. The radar likelihood ($L_{Radar}$) is calculated in three ways:

* **Case A: Overlap Zone (Solver Success)**
    * **Method**: **2D Direct Comparison**
    * **Logic**: Compares the particle's velocity vector directly with the Solver's reconstructed vector ($v_{solver}$).
    * **Effect**: Strongly constrains particle direction, eliminating "arrow jitter" and ensuring precise tracking.
    * $L_{Radar} \propto \exp\left( - \frac{\| \mathbf{v}_p - \mathbf{v}_{solver} \|^2}{2\sigma_{tight}^2} \right)$

* **Case B: Single Zone (Solver Failed)**
    * **Method**: **1D Projection (Survival Mode)**
    * **Logic**: Projects particle velocity onto the radar azimuth and compares it with the measured Doppler velocity ($v_r$). Uses a looser $\sigma$ to allow lateral particles to survive.
    * **Effect**: Maintains tracking of objects (including lateral movement) even without full 2D information.

* **Case C: No Radar Data**
    * **Method**: **Static Prior**
    * **Logic**: Applies a penalty to moving particles, assuming zero velocity for the cell.
    * **Effect**: Accelerates the suppression of noise and static objects.

#### 4. `generateNewParticles` (Adaptive Birth)
When a new object appears, particles are generated with a ratio determined by radar evidence:
-   **With Radar/Solver**: High dynamic ratio (~90%) to quickly capture movement.
-   **Without Radar**: High static ratio (~70%) to force the cell to be static by default (noise suppression).

#### 5. `calculateVelocityStatistics` (Classification)
Final classification of a cell as **Dynamic** or **Static**.
-   **Criteria**: A cell is a "Dynamic Candidate" if:
    1.  The weighted average velocity of particles is high ($> 0.2$ m/s).
    2.  OR The Solver's reconstructed velocity is high ($> 0.2$ m/s).
-   **Hysteresis**: Requires 2 consecutive frames to confirm Dynamic state, and 4 frames to revert to Static.

## Parameter Configuration
Key parameters in `config/params.yaml`:

- **`radar_hint_search_radius`**: Radius (in cells) to aggregate radar points for the Solver (default: 2).
- **`radar_static_velocity_threshold`**: Velocity threshold for the Solver to classify an object as dynamic (default: 0.5 m/s).
- **`particle_static_velocity_threshold`**: Velocity threshold for particle consensus (default: 0.3 m/s).
- **`min_dynamic_birth_ratio`**: Ratio of dynamic particles when **NO radar** is present (default: 0.3 -> 70% Static).
- **`max_dynamic_birth_ratio`**: Ratio of dynamic particles when **Radar/Solver** is present (default: 0.9).