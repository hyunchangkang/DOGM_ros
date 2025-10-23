#ifndef DYNAMIC_GRID_MAP_H
#define DYNAMIC_GRID_MAP_H

#include <vector>
#include <memory>
#include <string>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>

#include "dogm_ros/structures.h"
#include "dogm_ros/particle_filter.h"

class DynamicGridMap {
public:
    // [MODIFIED] Added bool use_radar at the end
    DynamicGridMap(double grid_size, double resolution, int num_particles,
                   double process_noise_pos, double process_noise_vel,
                   int radar_buffer_size, int min_radar_points,
                   int radar_hint_search_radius,
                   bool use_fsd, int fsd_T_static, int fsd_T_free,
                   bool use_mc,
                   bool use_radar); // Added this argument
    ~DynamicGridMap() = default;

    void generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan,
                                 const pcl::PointCloud<mmWaveCloudType>::ConstPtr& radar_cloud);

    void updateOccupancy(double birth_prob);

    std::vector<Particle> generateNewParticles(double newborn_vel_stddev,
                                               double min_dynamic_birth_ratio,
                                               double max_dynamic_birth_ratio,
                                               double max_radar_speed_for_scaling,
                                               double dynamic_newborn_vel_stddev);

    // [MODIFIED] Signature changed to just use member variables for safety nets
    void calculateVelocityStatistics(double static_vel_thresh,
                                     double max_vel_for_scaling,
                                     bool   use_ego_comp,
                                     double ego_vx, double ego_vy);

    void toOccupancyGridMsg(nav_msgs::OccupancyGrid& msg, const std::string& frame_id) const;
    void toMarkerArrayMsg(visualization_msgs::MarkerArray& arr,
                          const std::string& frame_id,
                          bool show_velocity_arrows) const;

    ParticleFilter& getParticleFilter() { return *particle_filter_; }
    const std::vector<MeasurementCell>& getMeasurementGrid() const { return measurement_grid_; }
    // [NEW] Getter for the full grid, needed by updateWeights
    const std::vector<GridCell>& getGrid() const { return grid_; }

    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;
    int gridToIndex(int gx, int gy) const;
    bool isInside(int gx, int gy) const;

private:
    // [MODIFIED] Returns 1D Vr hint, not 2D vector
    bool getSmoothedRadarVrHint(int center_gx, int center_gy, double& smoothed_vr_hint) const;

    double grid_size_;
    double resolution_;
    int grid_width_;
    int grid_height_;
    double origin_x_;
    double origin_y_;
    std::vector<GridCell> grid_;
    std::vector<MeasurementCell> measurement_grid_;
    std::unique_ptr<ParticleFilter> particle_filter_;
    std::mt19937 random_generator_;

    // Radar buffer parameters
    int radar_buffer_size_;
    int min_radar_points_;
    int radar_hint_search_radius_;

    // [NEW] Safety Net Parameters
    bool use_fsd_;  // Use False Static Detection
    int fsd_T_static_;
    int fsd_T_free_;
    bool use_mc_;   // Use Measurement Correction

    // [NEW] Store the radar usage flag
    bool use_radar_;
};

#endif // DYNAMIC_GRID_MAP_H