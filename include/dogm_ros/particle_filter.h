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

    void predict(double dt, double survival_prob,
                 double damping_thresh, double damping_factor,
                 double max_vel);

    /**
     * @brief Updates particle weights based on BOTH LiDAR and Radar.
     * @param measurement_grid LiDAR (μ, Σ⁻¹) model
     * @param grid The full GridCell vector (for radar cos/sin)
     * @param grid_map The map object (needed for getSmoothedRadarVrHint)
     * @param radar_noise_stddev Standard deviation of radar velocity noise
     */
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