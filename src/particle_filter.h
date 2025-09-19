#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <vector>
#include <random>
#include <memory>
#include "structures.h"

class ParticleFilter {
public:
    ParticleFilter(int num_particles, double process_noise_pos, double process_noise_vel, 
                   double newborn_vel_stddev);
    ~ParticleFilter();
    
    // Main filter operations
    void predict(double dt, double survival_prob);
    void sortParticlesByGridCell(const class DynamicGridMap& grid_map);
    void updateWeights(const std::vector<GridCell>& grid);
    void generateNewParticles(const std::vector<GridCell>& grid, const class DynamicGridMap& grid_map);
    void resample();
    
    // Getters
    const std::vector<Particle>& getParticles() const { return particles_; }
    std::vector<Particle>& getParticles() { return particles_; }
    int getNumParticles() const { return num_particles_; }
    double getEffectiveParticleCount() const;
    
    // Statistics
    void printStatistics() const;
    
private:
    // Parameters
    int num_particles_;
    double process_noise_pos_;
    double process_noise_vel_;
    double newborn_vel_stddev_;
    
    // Particle storage
    std::vector<Particle> particles_;
    std::vector<Particle> temp_particles_; // For resampling
    
    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> normal_dist_;
    std::uniform_real_distribution<double> uniform_dist_;
    
    // Helper functions
    void initializeParticles();
    double sampleGaussian(double mean, double stddev);
    void normalizeWeights();
    std::vector<double> calculateCumulativeWeights() const;
    int binarySearchCumulative(const std::vector<double>& cumulative_weights, double value) const;
    
    // Resampling methods
    void systematicResample();
    bool needsResampling() const;
};

#endif // PARTICLE_FILTER_H
