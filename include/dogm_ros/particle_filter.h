#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <vector>
#include <random>
#include "structures.h"

class DynamicGridMap; // Forward declaration

class ParticleFilter {
public:
    ParticleFilter(int num_particles, double process_noise_pos, double process_noise_vel);
    ~ParticleFilter() = default;

    // [수정] 가속도 보정을 위한 d_ego_vx, d_ego_vy 인자 추가
    void predict(double dt, double survival_prob,
                 double damping_thresh, double damping_factor,
                 double max_vel,
                 double d_ego_vx = 0.0, double d_ego_vy = 0.0);

    void updateWeights(const std::vector<MeasurementCell>& measurement_grid,
                       const std::vector<GridCell>& grid,
                       const DynamicGridMap& grid_map,
                       double radar_noise_stddev);

    void sortParticlesByGridCell(const DynamicGridMap& grid_map);
    
    void resample(const std::vector<Particle>& new_born_particles);

    std::vector<Particle>& getParticles() { return particles_; }
    const std::vector<Particle>& getParticles() const { return particles_; }

private:
    int num_particles_;
    double process_noise_pos_;
    double process_noise_vel_;
    std::vector<Particle> particles_;
    
    std::mt19937 random_generator_; 
    
    std::normal_distribution<double> pos_noise_dist_;
    std::normal_distribution<double> vel_noise_dist_;
};

#endif // PARTICLE_FILTER_H