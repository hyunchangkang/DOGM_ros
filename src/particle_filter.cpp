#include "dogm_ros/particle_filter.h"
#include "dogm_ros/dynamic_grid_map.h"
#include <algorithm>
#include <numeric>
#include <cmath>

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
    for (auto& p : particles_) {
        p.weight = initial_weight;
    }
}

// [종합 수정] 최대 속도 제한 및 age 기반 감쇠 로직이 적용된 최종 predict 함수
void ParticleFilter::predict(double dt, double survival_prob,
                             double damping_thresh, double damping_factor,
                             double max_vel)
{
    const int GRACE_PERIOD = 3;

    for (auto& p : particles_)
    {
        p.x += p.vx * dt + pos_noise_dist_(random_generator_);
        p.y += p.vy * dt + pos_noise_dist_(random_generator_);

        p.vx += vel_noise_dist_(random_generator_);
        p.vy += vel_noise_dist_(random_generator_);

        double speed = std::hypot(p.vx, p.vy);
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

void ParticleFilter::updateWeights(const std::vector<MeasurementCell>& measurement_grid)
{
    double total_weight = 0.0;
    for (auto& p : particles_)
    {
        if (p.grid_cell_idx >= 0 && p.grid_cell_idx < measurement_grid.size())
        {
            const auto& meas_cell = measurement_grid[p.grid_cell_idx];
            double likelihood = 0.05 + 0.95 * (meas_cell.m_occ_z * meas_cell.m_occ_z);
            p.weight *= likelihood;
        }
        else
        {
            p.weight *= 0.01;
        }
        total_weight += p.weight;
    }

    if (total_weight > 1e-9)
    {
        for (auto& p : particles_)
        {
            p.weight /= total_weight;
        }
    }
}


void ParticleFilter::sortParticlesByGridCell(const DynamicGridMap& grid_map)
{
    for (auto& p : particles_)
    {
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

    std::sort(particles_.begin(), particles_.end(),
              [](const Particle& a, const Particle& b) {
                  return a.grid_cell_idx < b.grid_cell_idx;
              });
}

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