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
#include "dogm_ros/ego_calibration.h"

// [Dual Radar] Radar Data Structure
struct RadarDataPacket {
    pcl::PointCloud<mmWaveCloudType>::ConstPtr cloud;
    double sensor_x;
    double sensor_y;
    double sensor_yaw;  // Sensor orientation (rotation around Z-axis)
};

class DynamicGridMap {
public:
    DynamicGridMap(double grid_size, double resolution, int num_particles,
                   double process_noise_pos, double process_noise_vel,
                   int radar_buffer_size, int min_radar_points,
                   int radar_hint_search_radius,
                   bool use_fsd, int fsd_T_static, int fsd_T_free,
                   bool use_mc,
                   bool use_radar,
                   int lidar_hit_point,
                   double lidar_noise_stddev,
                   // [Changed] Separated Thresholds
                   double particle_vector_vel_thresh, 
                   double particle_vector_ang_thresh,
                   double particle_static_vel_thresh,
                   double radar_static_vel_thresh,
                   bool cluster_mode);
    ~DynamicGridMap() = default;

    void generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan,
                                 const std::vector<RadarDataPacket>& radar_packets);

    void updateOccupancy(double birth_prob);
    void shiftGrid(double dx, double dy);

    std::vector<Particle> generateNewParticles(double newborn_vel_stddev,
                                               double min_dynamic_birth_ratio,
                                               double max_dynamic_birth_ratio,
                                               double max_radar_speed_for_scaling,
                                               double dynamic_newborn_vel_stddev,
                                               const EgoCalibration& ego_calib);

    void calculateVelocityStatistics(double max_vel_for_scaling,
                                     bool   use_ego_comp,
                                     const EgoCalibration& ego_calib);

    void toOccupancyGridMsg(nav_msgs::OccupancyGrid& msg, const std::string& frame_id) const;
    void toMarkerArrayMsg(visualization_msgs::MarkerArray& arr,
                          const std::string& frame_id,
                          bool show_velocity_arrows,
                          const EgoCalibration& ego_calib) const;

    ParticleFilter& getParticleFilter() { return *particle_filter_; }
    const std::vector<MeasurementCell>& getMeasurementGrid() const { return measurement_grid_; }
    const std::vector<GridCell>& getGrid() const { return grid_; }

    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;
    int gridToIndex(int gx, int gy) const;
    void indexToGrid(int idx, int& gx, int& gy) const;
    bool isInside(int gx, int gy) const;

    int getRadarHintSearchRadius() const { return radar_hint_search_radius_; }

private:
    void processDynamicClusters(const EgoCalibration& ego_calib);

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

    int radar_buffer_size_;
    int min_radar_points_;
    int radar_hint_search_radius_;
    bool use_fsd_;
    int fsd_T_static_;
    int fsd_T_free_;
    bool use_mc_;
    bool use_radar_;
    int lidar_hit_point_;
    double lidar_noise_stddev_;
    
    // [Changed] New Threshold Variables
    double particle_vector_vel_thresh_; 
    double particle_vector_ang_thresh_; 

    double particle_static_vel_thresh_;
    double radar_static_vel_thresh_;
    
    bool cluster_mode_;
};

#endif // DYNAMIC_GRID_MAP_H