# dogm_ros

`dogm_ros` is a particle filter-based dynamic occupancy grid mapping (DOGM) package that fuses LiDAR and Radar data to estimate the dynamic/static state and velocity of grid cells. This README provides an overview of the package, its features, and the algorithmic details.

---

## Features
- **Sensor Fusion**: Combines LiDAR and Radar data for robust environmental modeling.
- **Dynamic/Static Classification**: Classifies grid cells as dynamic or static based on sensor data.
- **Velocity Estimation**: Estimates the velocity and direction of dynamic cells.
- **Particle Filter**: Implements a survival-birth particle filter for state estimation.

---

## Installation
1. Clone the repository into your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone <repository_url>
   ```
2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
4. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

---

## Usage
1. Launch the DOGM node:
   ```bash
   roslaunch dogm_ros dogm.launch
   ```
2. Visualize the results in RViz:
   ```bash
   rosrun rviz rviz -d $(rospack find dogm_ros)/rviz/dogm.rviz
   ```
3. Adjust parameters in `config/params.yaml` to fine-tune the algorithm.

---

## Algorithm Details

### Overview
The algorithm employs a **survival-birth particle filter** to estimate the state of grid cells. It combines a **survival process**, which updates existing particles based on new sensor data, and a **birth process**, which introduces new particles in newly occupied areas. This hybrid approach allows the filter to track existing objects while quickly adapting to new ones.

The core logic resides in the `updateLoop` function of `dogm_node.cpp`, which orchestrates the following sequence of operations at each timestep:

1.  **Measurement Grid Generation**: Processes raw LiDAR and Radar data to build a statistical model of the environment for the current timestep.
2.  **Particle Prediction**: Propagates surviving particles from the previous timestep to the current time using a motion model.
3.  **Weight Update**: Updates the weight (belief) of each particle by comparing its predicted state to the measurement grid.
4.  **Occupancy Update**: Updates the occupancy probability of each grid cell and identifies candidate cells for the "birth" process.
5.  **Velocity Estimation & Classification**: Estimates the velocity of each cell and makes a final classification of its state (dynamic or static).
6.  **New Particle Generation**: Creates new "birth" particles in newly detected, uncertain areas.
7.  **Particle Resampling**: Combines the surviving and newly born particles and resamples them to form the particle set for the next timestep.

### Code Structure
The package is organized as follows:
```
dogm_ros/
├── include/dogm_ros/
│   ├── dynamic_grid_map.h   # DOGM logic header
│   ├── particle_filter.h    # Particle filter header
│   └── structures.h         # Core data structures
├── src/
│   ├── dogm_node.cpp        # ROS node, main update loop
│   ├── dynamic_grid_map.cpp # DOGM logic implementation
│   └── particle_filter.cpp  # Particle filter implementation
└── config/
    └── params.yaml          # Algorithm parameters
```

### Detailed Function Analysis

#### 1. `generateMeasurementGrid` (in `dynamic_grid_map.cpp`)
This function acts as the sensor pre-processing stage.
-   **LiDAR Processing**: It performs ray-casting to determine free space. For cells with sufficient LiDAR hits (`lidar_hit_point`), it computes a statistical model consisting of the mean position ($\boldsymbol{\mu}_c$) and inverse covariance matrix ($\boldsymbol{\Sigma}_c^{-1}$) of the hit points. This model represents the geometric shape of the object in the cell.
-   **Radar Processing**: It maintains a time-averaged buffer of radar detections for each cell. If a cell has enough recent radar points (`min_radar_points`), it calculates a time-averaged radial velocity hint ($v_{r, \text{hint}}$) and the corresponding angle ($\theta_c$). This hint provides crucial kinematic information.

#### 2. `predict` (in `particle_filter.cpp`)
This is the prediction step of the particle filter. Each particle's state (position and velocity) is propagated forward in time based on a constant velocity model, with random noise added to account for process uncertainty. The new position is calculated from the old position and velocity, and the new velocity is based on the old velocity, both with added noise.

#### 3. `updateWeights` (in `particle_filter.cpp`)
This is the core of the "survival" process, where each particle's weight is updated based on how well its state matches the sensor measurements. The new weight is the product of the previous weight and two likelihoods: `Weight_new = Weight_old * L_LiDAR * L_Radar`.

-   **LiDAR Likelihood ($L_{\text{LiDAR}}$)**: This term evaluates the geometric consistency. It measures the probability of a particle's position given the LiDAR measurement model of the cell it occupies, using a multivariate Gaussian probability density function. In essence, it checks if the particle is located where the LiDAR sensor detected an object.

-   **Radar Likelihood ($L_{\text{Radar}}$)**: This term evaluates the kinematic consistency. It compares the particle's predicted radial velocity to the smoothed radar velocity hint for that cell. This checks if the particle is moving at a velocity consistent with what the radar sensor detected.

If a cell has no radar hint, `L_Radar` is treated as 1.0, meaning the weight update relies solely on LiDAR data. This prevents penalizing particles in areas without radar coverage.

#### 4. `calculateVelocityStatistics` (in `dynamic_grid_map.cpp`)
This function performs the final state estimation and classification for each cell based on the weighted "surviving" particles within it.

-   **Velocity Estimation**: It uses a "Winner-Clustering" approach. First, it finds the particle with the highest weight (the "winner"). Then, it forms a cluster of particles whose velocities are similar to the winner's. The cell's final velocity ($\mathbf{v}_c$) is the weighted average velocity of the particles in this cluster.
-   **Dynamic/Static Classification**: The classification is made using a **Dual Threshold** logic that cross-validates the particle-based velocity with the raw radar data. A cell is considered a dynamic candidate if it is occupied AND its particle-based speed is above a low threshold OR its radar-based speed is above a high threshold. This decision is further refined by heuristics to handle challenging scenarios like crossing objects (`use_mc_`) and slow-starting objects (`use_fsd_`).

#### 5. `generateNewParticles` (in `dynamic_grid_map.cpp`)
This function implements the "birth" process, creating new particles in cells that were previously unknown but are now occupied (`rho_b > 0.5`).
-   It uses the smoothed radar hint to decide the ratio of dynamic to static particles to generate. If a strong radar velocity is present, it creates a higher percentage of dynamic particles (up to `max_dynamic_birth_ratio`). If no radar velocity is detected, it defaults to a lower ratio (`min_dynamic_birth_ratio`).
-   This "bootstrapping" mechanism is crucial for initializing the filter in new areas and allows the system to quickly react to newly appearing dynamic objects.

#### 6. `resample` (in `particle_filter.cpp`)
In the final step, the "surviving" particles (with their updated weights) and the "newborn" particles are combined. A new set of particles is then drawn from this combined pool using **Low Variance Resampling**. This process ensures that particles with higher weights are more likely to be selected for the next generation, effectively focusing computational resources on more probable hypotheses. After resampling, all particles are given a uniform weight.

---

## Parameter Configuration
The algorithm is highly configurable via `config/params.yaml`. Key parameters include:
- `particle_static_vel_thresh`: Velocity threshold for classifying a particle as static.
- `radar_static_vel_thresh`: Velocity threshold for classifying a radar detection as static.
- `max_dynamic_birth_ratio`: Maximum ratio of dynamic particles to generate in a new cell.
- `min_dynamic_birth_ratio`: Minimum ratio of dynamic particles to generate in a new cell.

---

## License
This package is licensed under the [MIT License](LICENSE).

---

## Acknowledgments
This package is inspired by state-of-the-art research in dynamic occupancy grid mapping and sensor fusion.