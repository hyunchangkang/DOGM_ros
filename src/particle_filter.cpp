#include "dogm_ros/particle_filter.h"
#include "dogm_ros/dynamic_grid_map.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <omp.h>

ParticleFilter::ParticleFilter(int num_particles, double process_noise_pos, double process_noise_vel)
    : num_particles_(num_particles),
      process_noise_pos_(process_noise_pos),
      process_noise_vel_(process_noise_vel),
      random_generator_(std::mt19937(std::random_device()())),
      pos_noise_dist_(0.0, process_noise_pos_),
      vel_noise_dist_(0.0, process_noise_vel_)
{
    particles_.resize(num_particles_);
    double initial_weight = 1.0 / num_particles_;
    for (auto& p : particles_) { p.weight = initial_weight; }
}

void ParticleFilter::predict(double dt, double survival_prob,
                             double damping_thresh, double damping_factor,
                             double max_vel,
                             double d_ego_vx, double d_ego_vy) 
{
    const int GRACE_PERIOD = 3;
    #pragma omp parallel for
    for (int i = 0; i < particles_.size(); ++i) {
        auto& p = particles_[i];
        p.vx -= d_ego_vx; p.vy -= d_ego_vy;
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
        p.vx += vel_noise_x; p.vy += vel_noise_y;
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

// [수정] Radar Hints Loop & Intersection
void ParticleFilter::updateWeights(const std::vector<MeasurementCell>& measurement_grid,
                                   const std::vector<GridCell>& grid,
                                   const DynamicGridMap& grid_map,
                                   double radar_noise_stddev)
{
    double total_weight = 0.0;
    const double radar_variance = radar_noise_stddev * radar_noise_stddev;
    // Normalization factor not strictly needed as weights are normalized later, but kept for scale
    const double radar_norm_factor = -0.5 / std::max(1e-9, radar_variance);

    #pragma omp parallel for reduction(+:total_weight)
    for (int i = 0; i < particles_.size(); ++i)
    {
        auto& p = particles_[i];
        
        if (p.weight > 1e-9)
        {
            if (p.grid_cell_idx >= 0 && p.grid_cell_idx < measurement_grid.size())
            {
                const auto& meas_cell = measurement_grid[p.grid_cell_idx];
                double lidar_likelihood = 1e-9;
                if (meas_cell.has_lidar_model) {
                    double dx = p.x - meas_cell.mean_x;
                    double dy = p.y - meas_cell.mean_y;
                    double zx = dx * meas_cell.inv_cov_xx + dy * meas_cell.inv_cov_xy;
                    double zy = dx * meas_cell.inv_cov_xy + dy * meas_cell.inv_cov_yy;
                    double mahal_dist_sq = zx * dx + zy * dy;
                    lidar_likelihood = std::exp(-0.5 * mahal_dist_sq);
                }

                // [핵심] Dual Radar Likelihood Intersection
                double radar_likelihood = 1.0;
                const auto& grid_cell = grid[p.grid_cell_idx];

                if (!grid_cell.radar_hints.empty()) {
                    for (const auto& hint : grid_cell.radar_hints) {
                        // 1. Particle -> Sensor Azimuth
                        double dx = p.x - hint.sensor_x;
                        double dy = p.y - hint.sensor_y;
                        double azimuth = std::atan2(dy, dx);
                        
                        // 2. Project Particle Velocity
                        double vr_expected = p.vx * std::cos(azimuth) + p.vy * std::sin(azimuth);
                        
                        // 3. Error & Likelihood
                        double error = vr_expected - hint.vr;
                        double prob = std::exp(error * error * radar_norm_factor); // exp(-error^2 / 2var)
                        
                        // 4. Multiply (Intersection)
                        radar_likelihood *= prob;
                    }
                }

                p.weight *= (lidar_likelihood * radar_likelihood);
            }
            else { p.weight *= 0.01; }
        }
        total_weight += p.weight;
    }

    if (total_weight > 1e-9) {
        for (auto& p : particles_) p.weight /= total_weight;
    }
}

void ParticleFilter::sortParticlesByGridCell(const DynamicGridMap& grid_map)
{
    // (이전과 동일)
    #pragma omp parallel for
    for (int i = 0; i < particles_.size(); ++i) {
        auto& p = particles_[i];
        int grid_x, grid_y;
        if (grid_map.worldToGrid(p.x, p.y, grid_x, grid_y)) {
            p.grid_cell_idx = grid_map.gridToIndex(grid_x, grid_y);
        } else {
            p.grid_cell_idx = -1;
        }
    }
    std::sort(particles_.begin(), particles_.end(),
              [](const Particle& a, const Particle& b) {
                  return a.grid_cell_idx < b.grid_cell_idx;
              });
}

void ParticleFilter::resample(const std::vector<Particle>& new_born_particles)
{
    // (이전과 동일)
    std::vector<Particle> combined_pool;
    combined_pool.reserve(particles_.size() + new_born_particles.size());
    combined_pool.insert(combined_pool.end(), particles_.begin(), particles_.end());
    combined_pool.insert(combined_pool.end(), new_born_particles.begin(), new_born_particles.end());

    if (combined_pool.empty()) { particles_.clear(); return; }

    double total_weight = 0.0;
    for (const auto& p : combined_pool) total_weight += p.weight;

    if (total_weight < 1e-9) {
        for (auto& p : combined_pool) p.weight = 1.0 / combined_pool.size();
        total_weight = 1.0;
    } else {
        for (auto& p : combined_pool) p.weight /= total_weight;
    }

    std::vector<Particle> new_particle_set;
    new_particle_set.reserve(num_particles_);
    std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
    double r = dist(random_generator_); 
    double c = combined_pool[0].weight;
    int i = 0;

    for (int m = 0; m < num_particles_; ++m) {
        double u = r + m * (1.0 / num_particles_);
        while (u > c) {
            i++;
            if (i >= combined_pool.size()) i = combined_pool.size() - 1;
            c += combined_pool[i].weight;
        }
        new_particle_set.push_back(combined_pool[i]);
    }
    particles_ = new_particle_set;
    if (!particles_.empty()) {
        double uniform_weight = 1.0 / particles_.size();
        for (auto& p : particles_) p.weight = uniform_weight;
    }
}