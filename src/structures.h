#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <vector>
#include <cmath>

// Particle structure for particle filter
struct Particle {
    double x, y;      // Position (m)
    double vx, vy;    // Velocity (m/s)
    double weight;    // Particle weight
    int grid_cell_idx; // Grid cell index this particle belongs to
    
    Particle() : x(0), y(0), vx(0), vy(0), weight(0), grid_cell_idx(-1) {}
    
    Particle(double x_, double y_, double vx_, double vy_, double weight_, int idx = -1)
        : x(x_), y(y_), vx(vx_), vy(vy_), weight(weight_), grid_cell_idx(idx) {}
};

// Grid cell structure for dynamic grid map
struct GridCell {
    double m_occ;      // Mass for occupied (Dempster-Shafer)
    double m_free;     // Mass for free (Dempster-Shafer)
    double rho_p;      // Persistent mass (for existing particles)
    double rho_b;      // Birth mass (for new particles)
    
    // Velocity statistics
    double mean_vx, mean_vy;    // Mean velocity
    double var_vx, var_vy;      // Velocity variance
    double covar_vxy;           // Cross-covariance
    
    bool is_dynamic;            // Dynamic/static classification result
    double mahalanobis_dist;    // Mahalanobis distance for classification
    
    GridCell() : m_occ(0), m_free(0), rho_p(0), rho_b(0),
                 mean_vx(0), mean_vy(0), var_vx(0), var_vy(0), covar_vxy(0),
                 is_dynamic(false), mahalanobis_dist(0) {}
    
    // Get occupancy probability using Dempster-Shafer
    double getOccupancyProb() const {
        double total_belief = m_occ + m_free;
        if (total_belief < 1e-6) return 0.5; // Unknown
        return m_occ / total_belief;
    }
    
    // Get uncertainty (unknown mass)
    double getUncertainty() const {
        return std::max(0.0, 1.0 - m_occ - m_free);
    }
    
    // Reset cell to initial state
    void reset() {
        m_occ = m_free = 0;
        rho_p = rho_b = 0;
        mean_vx = mean_vy = 0;
        var_vx = var_vy = covar_vxy = 0;
        is_dynamic = false;
        mahalanobis_dist = 0;
    }
};

// Measurement from LiDAR for a grid cell
struct MeasurementCell {
    double m_occ_z;    // Measured occupancy mass
    double m_free_z;   // Measured free mass
    
    MeasurementCell() : m_occ_z(0), m_free_z(0) {}
    MeasurementCell(double occ, double free) : m_occ_z(occ), m_free_z(free) {}
};

#endif // STRUCTURES_H
