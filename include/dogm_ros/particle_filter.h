#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <vector>
#include <random>
#include "structures.h"

class DynamicGridMap; // Forward declaration

class ParticleFilter {
public:
    // [MODIFIED] Constructor simplified
    ParticleFilter(int num_particles, double process_noise_pos, double process_noise_vel);
    ~ParticleFilter() = default;

    void predict(double dt, double survival_prob,
                 double damping_thresh, double damping_factor,
                 double max_vel);

    /**
     * @brief [MODIFIED] Updates particle weights based on BOTH LiDAR and Radar.
     * @param measurement_grid LiDAR (μ, Σ⁻¹) model
     * @param grid The full GridCell vector (for radar cos/sin)
     * @param grid_map The map object (needed for getSmoothedRadarVrHint)
     * @param radar_noise_stddev Standard deviation of radar velocity noise
     * @param radar_static_penalty_strength Strength for static assumption
     */
    void updateWeights(const std::vector<MeasurementCell>& measurement_grid,
                       const std::vector<GridCell>& grid,
                       const DynamicGridMap& grid_map, // [NEW] For neighbor search
                       double radar_noise_stddev,
                       double radar_static_penalty_strength); // [MODIFIED] Renamed

    void sortParticlesByGridCell(const DynamicGridMap& grid_map);
    
    // [MODIFIED] Resample now takes new_born_particles as an argument
    void resample(const std::vector<Particle>& new_born_particles);

    // [REMOVED] generateNewParticles is now handled by DynamicGridMap
    // void generateNewParticles(...); 

    std::vector<Particle>& getParticles() { return particles_; }
    const std::vector<Particle>& getParticles() const { return particles_; }

private:
    int num_particles_;
    double process_noise_pos_;
    double process_noise_vel_;
    std::vector<Particle> particles_;
    
    // [MODIFIED] Corrected typo
    std::mt19937 random_generator_; 
    
    std::normal_distribution<double> pos_noise_dist_;
    std::normal_distribution<double> vel_noise_dist_;
};

#endif // PARTICLE_FILTER_H