#include "dogm_ros/particle_filter.h"
#include "dogm_ros/dynamic_grid_map.h" // Needed for getSmoothedRadarVrHint
#include <algorithm>
#include <numeric>
#include <cmath>
#include <omp.h> // [NEW] Include OpenMP header

ParticleFilter::ParticleFilter(int num_particles, double process_noise_pos, double process_noise_vel)
    : num_particles_(num_particles),
      process_noise_pos_(process_noise_pos),
      process_noise_vel_(process_noise_vel),
      // [FIXED] Corrected typo from mt19373 to mt19937
      random_generator_(std::mt19937(std::random_device()())),
      pos_noise_dist_(0.0, process_noise_pos_),
      vel_noise_dist_(0.0, process_noise_vel_)
{
    particles_.resize(num_particles_);
    double initial_weight = 1.0 / num_particles_;
    for (auto& p : particles_) {
        p.weight = initial_weight;
    }
}

// [predict] (Parallelized)
void ParticleFilter::predict(double dt, double survival_prob,
                             double damping_thresh, double damping_factor,
                             double max_vel)
{
    const int GRACE_PERIOD = 3;

    // [NEW] Parallelize the prediction step
    #pragma omp parallel for
    for (int i = 0; i < particles_.size(); ++i)
    {
        auto& p = particles_[i];
        
        // Note: Using a single random_generator_ in a parallel loop
        // is not strictly thread-safe. For robust parallel random numbers,
        // each thread should have its own generator instance.
        double pos_noise_x, pos_noise_y, vel_noise_x, vel_noise_y;
        #pragma omp critical (random_gen)
        {
            pos_noise_x = pos_noise_dist_(random_generator_);
            pos_noise_y = pos_noise_dist_(random_generator_);
            vel_noise_x = vel_noise_dist_(random_generator_);
            vel_noise_y = vel_noise_dist_(random_generator_);
        }
        
        p.x += p.vx * dt + pos_noise_x;
        p.y += p.vy * dt + pos_noise_y;

        p.vx += vel_noise_x;
        p.vy += vel_noise_y;

        double speed = std::sqrt(p.vx * p.vx + p.vy * p.vy);
        if (speed > max_vel) {
            p.vx = (p.vx / speed) * max_vel;
            p.vy = (p.vy / speed) * max_vel;
        }

        if (p.age > GRACE_PERIOD) {
            if (speed < damping_thresh) {
                p.vx *= damping_factor;
                p.vy *= damping_factor;
            }
        }

        p.weight *= survival_prob;
        p.age++;
    }
}

/**
 * @brief [MODIFIED] Updates weights using:
 * 1. Continuous 2D Gaussian PDF for L_LiDAR.
 * 2. Spatially-Smoothed (radar_search_radius) hint for L_Radar.
 * 3. OpenMP parallelization.
 */
void ParticleFilter::updateWeights(const std::vector<MeasurementCell>& measurement_grid,
                                   const std::vector<GridCell>& grid,
                                   const DynamicGridMap& grid_map, // Used for getSmoothedRadarVrHint
                                   double radar_noise_stddev,
                                   double radar_static_penalty_strength)
{
    double total_weight = 0.0;
    const double radar_variance = radar_noise_stddev * radar_noise_stddev;
    const double radar_norm_factor = -0.5 / std::max(1e-9, radar_variance);
    const double C_static = -radar_static_penalty_strength;

    // [NEW] Parallelize the weight update loop (User Request 1)
    // We use a 'reduction' clause to safely sum total_weight across threads
    #pragma omp parallel for reduction(+:total_weight)
    for (int i = 0; i < particles_.size(); ++i)
    {
        auto& p = particles_[i];
        
        if (p.weight > 1e-9)
        {
            if (p.grid_cell_idx >= 0 && p.grid_cell_idx < measurement_grid.size())
            {
                // 1. LiDAR Likelihood (Geometric verification)
                // [MODIFIED] Use continuous 2D Gaussian PDF (Mahalanobis distance)
                const auto& meas_cell = measurement_grid[p.grid_cell_idx];
                double lidar_likelihood;

                if (meas_cell.has_lidar_model) {
                    // This cell has a valid (μ, Σ⁻¹) model calculated from hits
                    double dx = p.x - meas_cell.mean_x;
                    double dy = p.y - meas_cell.mean_y;

                    // (x-μ)' * Σ⁻¹ * (x-μ)
                    double zx = dx * meas_cell.inv_cov_xx + dy * meas_cell.inv_cov_xy;
                    double zy = dx * meas_cell.inv_cov_xy + dy * meas_cell.inv_cov_yy;
                    double mahal_dist_sq = zx * dx + zy * dy;

                    // L ∝ exp(-0.5 * mahal_dist_sq)
                    lidar_likelihood = std::exp(-0.5 * mahal_dist_sq);
                } else {
                    // No valid model (empty space, etc.)
                    lidar_likelihood = 1e-9; 
                }
                // --- [END L_LiDAR MODIFICATION] ---


                // 2. Radar Likelihood (Kinematic verification)
                // [MODIFIED] Use Spatially-Smoothed hint (User Request 2)
                double radar_likelihood = 1.0; 
                const auto& grid_cell = grid[p.grid_cell_idx]; // Still need this for cos/sin

                int gx, gy;
                grid_map.indexToGrid(p.grid_cell_idx, gx, gy);
                double smoothed_vr_hint = 0.0;

                if (grid_map.getSmoothedRadarVrHint(gx, gy, smoothed_vr_hint))
                {
                    // --- Case 1: Spatially-Smoothed hint exists ---
                    // Use this smoothed hint as the PDF mean (μ_R)
                    const double vr_guess = p.vx * grid_cell.radar_cos_theta + 
                                           p.vy * grid_cell.radar_sin_theta;
                    const double error = vr_guess - smoothed_vr_hint;
                    radar_likelihood = std::exp(error * error * radar_norm_factor);
                }
                else 
                {
                    // --- Case 2: No hint in neighborhood ---
                    // Apply static assumption penalty (User Request 3)
                    const double speed_sq = p.vx * p.vx + p.vy * p.vy; 
                    radar_likelihood = std::exp(speed_sq * C_static);
                }
                // --- [END L_Radar MODIFICATION] ---


                // 3. Combine Likelihoods
                p.weight *= (lidar_likelihood * radar_likelihood);
            }
            else
            {
                p.weight *= 0.01; // Particle is outside the grid
            }
        }
        
        total_weight += p.weight;
    } // --- End of parallel loop ---

    // Normalize total weights (done in serial after the parallel loop)
    if (total_weight > 1e-9)
    {
        // [FIXED] Corrected bracket syntax error
        for (auto& p : particles_)
        {
            p.weight /= total_weight;
        }
    }
} // [FIXED] Corrected bracket syntax error


// [sortParticlesByGridCell] (Parallelized)
void ParticleFilter::sortParticlesByGridCell(const DynamicGridMap& grid_map)
{
    // [NEW] Parallelize the grid index calculation
    #pragma omp parallel for
    for (int i = 0; i < particles_.size(); ++i)
    {
        auto& p = particles_[i];
        int grid_x, grid_y;
        if (grid_map.worldToGrid(p.x, p.y, grid_x, grid_y))
        {
            p.grid_cell_idx = grid_map.gridToIndex(grid_x, grid_y);
        }
        else
        {
            p.grid_cell_idx = -1;
        }
    }

    // Serial sort is kept for simplicity.
    std::sort(particles_.begin(), particles_.end(),
              [](const Particle& a, const Particle& b) {
                  return a.grid_cell_idx < b.grid_cell_idx;
              });
}

// [resample] (no change)
void ParticleFilter::resample(const std::vector<Particle>& new_born_particles)
{
    std::vector<Particle> combined_pool;
    combined_pool.reserve(particles_.size() + new_born_particles.size());
    combined_pool.insert(combined_pool.end(), particles_.begin(), particles_.end());
    combined_pool.insert(combined_pool.end(), new_born_particles.begin(), new_born_particles.end());

    if (combined_pool.empty()) {
        particles_.clear();
        return;
    }

    double total_weight = 0.0;
    for (const auto& p : combined_pool)
    {
        total_weight += p.weight;
    }

    if (total_weight < 1e-9)
    {
        for (auto& p : combined_pool) {
            p.weight = 1.0 / combined_pool.size();
        }
        total_weight = 1.0;
    }
    else
    {
        for (auto& p : combined_pool) {
            p.weight /= total_weight;
        }
    }

    std::vector<Particle> new_particle_set;
    new_particle_set.reserve(num_particles_);

    // Low Variance Resampling (LVR) - This is an O(N) serial algorithm.
    std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
    double r = dist(random_generator_); 
    double c = combined_pool[0].weight;
    int i = 0;

    for (int m = 0; m < num_particles_; ++m)
    {
        double u = r + m * (1.0 / num_particles_);
        while (u > c)
        {
            i++;
            if (i >= combined_pool.size()) i = combined_pool.size() - 1;
            c += combined_pool[i].weight;
        }
        new_particle_set.push_back(combined_pool[i]);
    }

    particles_ = new_particle_set;

    if (!particles_.empty()) {
        double uniform_weight = 1.0 / particles_.size();
        for (auto& p : particles_)
        {
            p.weight = uniform_weight;
        }
    }
}