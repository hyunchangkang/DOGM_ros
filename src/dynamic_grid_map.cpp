// dogm_ros/src/dynamic_grid_map.cpp
// Contains all modifications for:
// 1. Reduced 'm_free_z' accumulation (for flickering)
// 2. Branched 'generateNewParticles' logic (for LiDAR-Only)
// 3. [NEW] Branched 'calculateVelocityStatistics' logic (for LiDAR-Only)
// 4. [NEW] Using 'getSmoothedRadarVrHint' for 'has_speed_from_radar'
// 5. [NEW] Changed final check from AND to OR

#include "dogm_ros/dynamic_grid_map.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <random>

// Constructor (no changes)
DynamicGridMap::DynamicGridMap(double grid_size, double resolution, int num_particles,
                               double process_noise_pos, double process_noise_vel,
                               int radar_buffer_size, int min_radar_points,
                               int radar_hint_search_radius,
                               bool use_fsd, int fsd_T_static, int fsd_T_free,
                               bool use_mc,
                               bool use_radar,
                               int lidar_point_counts)
    : grid_size_(grid_size),
      resolution_(resolution),
      radar_buffer_size_(radar_buffer_size),
      min_radar_points_(min_radar_points),
      radar_hint_search_radius_(radar_hint_search_radius),
      use_fsd_(use_fsd),
      fsd_T_static_(fsd_T_static),
      fsd_T_free_(fsd_T_free),
      use_mc_(use_mc),
      use_radar_(use_radar),
      lidar_point_counts_(lidar_point_counts),
      random_generator_(std::mt19937(std::random_device()()))
{
    grid_width_  = static_cast<int>(std::round(grid_size_ / resolution_));
    grid_height_ = grid_width_;
    origin_x_ = 0.0;
    origin_y_ = -grid_size_ / 2.0;
    grid_.assign(grid_width_ * grid_height_, GridCell{});
    measurement_grid_.assign(grid_width_ * grid_height_, MeasurementCell{});
    particle_filter_ = std::make_unique<ParticleFilter>(
        num_particles, process_noise_pos, process_noise_vel
    );
}

// Basic grid conversion functions (no changes)
bool DynamicGridMap::isInside(int gx, int gy) const
{
    return (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_);
}

int DynamicGridMap::gridToIndex(int gx, int gy) const
{
    return gy * grid_width_ + gx;
}

bool DynamicGridMap::worldToGrid(double wx, double wy, int& gx, int& gy) const
{
    gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    return isInside(gx, gy);
}

void DynamicGridMap::gridToWorld(int gx, int gy, double& wx, double& wy) const
{
    wx = origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_;
    wy = origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_;
}

// generateMeasurementGrid (flickering fix included)
void DynamicGridMap::generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan, const pcl::PointCloud<mmWaveCloudType>::ConstPtr& radar_cloud)
{
    // 1. Initialize measurement grid and age/filter Radar buffer
    for (auto& m : measurement_grid_) { m.m_occ_z = 0.0; m.m_free_z = 0.0; }
    for (auto& c : grid_) {
        for (auto& rp : c.radar_points_buffer) { rp.age++; }
        c.radar_points_buffer.erase(
            std::remove_if(c.radar_points_buffer.begin(), c.radar_points_buffer.end(),
                           [this](const RadarPoint& rp) { return rp.age > this->radar_buffer_size_; }),
            c.radar_points_buffer.end());

        // Calculate time-weighted average '1D velocity hint (vr)'
        // This is the 1-cell hint, which we will NOT use in the final check anymore,
        // but it is still used by the smoothing function, so we keep it.
        if (c.radar_points_buffer.size() >= min_radar_points_) {
            double sum_weighted_vr = 0.0, sum_weighted_x = 0.0, sum_weighted_y = 0.0, sum_weights = 0.0;
            for (const auto& rp : c.radar_points_buffer) {
                double weight = 1.0 - (static_cast<double>(rp.age) / (radar_buffer_size_ + 1.0));
                sum_weighted_vr += rp.radial_velocity * weight;
                sum_weighted_x += rp.x * weight;
                sum_weighted_y += rp.y * weight;
                sum_weights += weight;
            }
            if (sum_weights > 1e-9) {
                c.radar_vr_hint = sum_weighted_vr / sum_weights;
                c.radar_theta_hint = std::atan2(sum_weighted_y / sum_weights, sum_weighted_x / sum_weights);
                c.has_reliable_radar = true;
            } else {
                c.has_reliable_radar = false;
                c.radar_vr_hint = 0.0;
            }
        } else {
            c.has_reliable_radar = false;
            c.radar_vr_hint = 0.0;
        }
    }

    // 2. Add new Radar data to buffer (only if radar is enabled)
    if (use_radar_ && radar_cloud) {
        for (const auto& pt : radar_cloud->points) {
            int gx, gy;
            if (worldToGrid(pt.x, pt.y, gx, gy)) {
                int idx = gridToIndex(gx, gy);
                RadarPoint new_rp;
                new_rp.radial_velocity = pt.velocity;
                new_rp.x = pt.x;
                new_rp.y = pt.y;
                new_rp.age = 0;
                grid_[idx].radar_points_buffer.push_back(new_rp);
            }
        }
    }

    // 3. Process LiDAR data (ray casting)
    std::vector<int> hit_counts(grid_width_ * grid_height_, 0);
    if (!scan) return;
    const double angle_min = static_cast<double>(scan->angle_min);
    const double angle_inc = static_cast<double>(scan->angle_increment);
    const double range_max = static_cast<double>(scan->range_max);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        const double r  = static_cast<double>(scan->ranges[i]);
        if (!std::isfinite(r)) continue;
        const double th = angle_min + angle_inc * static_cast<double>(i);

        // --- Ray Casting for Free Space ---
        const double step  = resolution_ * 0.9;
        const double limit = std::min(r, range_max);
        for (double rr = 0.0; rr < limit; rr += step) {
            const double wx = rr * std::cos(th);
            const double wy = rr * std::sin(th);
            int gx, gy;
            if (!worldToGrid(wx, wy, gx, gy)) break;
            auto& cell = measurement_grid_[gridToIndex(gx, gy)];
            
            // [MODIFICATION 1: Flickering fix]
            cell.m_free_z = std::min(1.0, cell.m_free_z + 0.03); // Reduced increment
        }
        // --- End Ray Casting ---

        // --- Hit Counting for Occupied Space ---
        if (r < range_max) {
            const double wx = r * std::cos(th);
            const double wy = r * std::sin(th);
            int gx, gy;
            if (worldToGrid(wx, wy, gx, gy)) {
                hit_counts[gridToIndex(gx, gy)]++;
            }
        }
        // --- End Hit Counting ---
    }

    // --- Calculate Occupied Mass based on Hit Counts ---
    for (size_t i = 0; i < hit_counts.size(); ++i) {
        if (hit_counts[i] > lidar_point_counts_) {
            double confidence = 1.0 - std::pow(0.4, hit_counts[i]);
            measurement_grid_[i].m_occ_z = std::min(1.0, confidence);
        }
    }
    // --- End Occupied Mass Calculation ---
}

// updateOccupancy (no changes)
void DynamicGridMap::updateOccupancy(double birth_prob)
{
    for (auto& c : grid_) {
        c.m_occ  = std::max(0.0, c.m_occ  * 0.98);
        c.m_free = std::max(0.0, c.m_free * 0.98);
        c.rho_b  = std::max(0.0, c.rho_b  * 0.98);
        c.rho_p  = std::max(0.0, c.rho_p  * 0.98);
    }

    for (int idx = 0; idx < static_cast<int>(grid_.size()); ++idx) {
        auto& cell = grid_[idx];
        const auto& meas = measurement_grid_[idx];
        double m_occ_pred = std::min(1.0, std::max(0.0, cell.m_occ));
        double m_free_pred= std::min(1.0, std::max(0.0, cell.m_free));
        double m_unk_pred = std::max(0.0, 1.0 - m_occ_pred - m_free_pred);
        double K = m_occ_pred * meas.m_free_z + m_free_pred * meas.m_occ_z;
        double norm = 1.0 / std::max(1e-9, (1.0 - K));
        double m_occ_upd = norm * (m_occ_pred * (1.0 - meas.m_free_z) + m_unk_pred * meas.m_occ_z);
        double m_free_upd= norm * (m_free_pred * (1.0 - meas.m_occ_z) + m_unk_pred * meas.m_free_z);
        m_occ_upd = std::min(1.0, std::max(0.0, m_occ_upd));
        m_free_upd= std::min(1.0, std::max(0.0, m_free_upd));
        double term = m_occ_pred + birth_prob * m_unk_pred;
        cell.rho_b = (term > 1e-9) ? (m_occ_upd * birth_prob * m_unk_pred) / term : 0.0;
        cell.rho_p = std::max(0.0, m_occ_upd - cell.rho_b);
        cell.m_occ  = m_occ_upd;
        cell.m_free = m_free_upd;
    }
}

// getSmoothedRadarVrHint (no changes)
bool DynamicGridMap::getSmoothedRadarVrHint(int center_gx, int center_gy, double& smoothed_vr_hint) const
{
    double sum_weighted_vr = 0.0, sum_weights = 0.0;
    for (int dy = -radar_hint_search_radius_; dy <= radar_hint_search_radius_; ++dy) {
        for (int dx = -radar_hint_search_radius_; dx <= radar_hint_search_radius_; ++dx) {
            int gx = center_gx + dx;
            int gy = center_gy + dy;
            if (isInside(gx, gy)) {
                const auto& cell = grid_[gridToIndex(gx, gy)];
                if (cell.has_reliable_radar) {
                    int layer = std::max(std::abs(dx), std::abs(dy));
                    double weight = (layer == 0) ? 1.0 : ( (layer <= radar_hint_search_radius_) ? 0.5 : 0.0 );
                    sum_weighted_vr += cell.radar_vr_hint * weight;
                    sum_weights += weight;
                }
            }
        }
    }
    if (sum_weights > 1e-9) {
        smoothed_vr_hint = sum_weighted_vr / sum_weights;
        return true;
    }
    return false;
}

// generateNewParticles (LiDAR-Only birth logic included)
std::vector<Particle> DynamicGridMap::generateNewParticles(double newborn_vel_stddev, double min_dynamic_birth_ratio, double max_dynamic_birth_ratio, double max_radar_speed_for_scaling, double dynamic_newborn_vel_stddev)
{
    std::vector<Particle> new_particles;
    std::normal_distribution<double> static_vel_dist(0.0, newborn_vel_stddev);
    std::normal_distribution<double> dynamic_vel_dist(0.0, dynamic_newborn_vel_stddev);

    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            int idx = gridToIndex(x, y);
            const auto& cell = grid_[idx];

            if (cell.rho_b > 0.5 && cell.m_occ > 0.6) {
                int num_to_birth = static_cast<int>(std::ceil(cell.rho_b * 4.0));
                double smoothed_vr = 0.0;
                
                // [MODIFICATION 2: LiDAR-Only Particle Birth]
                double current_dynamic_ratio = min_dynamic_birth_ratio;
                if (use_radar_)
                {
                    if (getSmoothedRadarVrHint(x, y, smoothed_vr)) {
                        double smoothed_speed = std::abs(smoothed_vr);
                        double scale = std::min(1.0, smoothed_speed / std::max(1e-6, max_radar_speed_for_scaling));
                        current_dynamic_ratio = min_dynamic_birth_ratio + (max_dynamic_birth_ratio - min_dynamic_birth_ratio) * scale;
                    }
                }
                else
                {
                    current_dynamic_ratio = max_dynamic_birth_ratio;
                }
                // [END MODIFICATION 2]

                int num_dynamic = static_cast<int>(num_to_birth * current_dynamic_ratio);
                int num_static = num_to_birth - num_dynamic;

                for (int i = 0; i < num_static; ++i) {
                    Particle p;
                    gridToWorld(x, y, p.x, p.y);
                    p.vx = static_vel_dist(random_generator_);
                    p.vy = static_vel_dist(random_generator_);
                    p.weight = cell.rho_b / static_cast<double>(num_to_birth);
                    p.grid_cell_idx = idx;
                    p.age = 0;
                    new_particles.push_back(p);
                }
                for (int i = 0; i < num_dynamic; ++i) {
                    Particle p;
                    gridToWorld(x, y, p.x, p.y);
                    p.vx = dynamic_vel_dist(random_generator_);
                    p.vy = dynamic_vel_dist(random_generator_);
                    p.weight = cell.rho_b / static_cast<double>(num_to_birth);
                    p.grid_cell_idx = idx;
                    p.age = 0;
                    new_particles.push_back(p);
                }
            }
        }
    }
    return new_particles;
}


// calculateVelocityStatistics (ALL user requests included)
void DynamicGridMap::calculateVelocityStatistics(double static_vel_thresh,
                                                 double max_vel_for_scaling,
                                                 bool   use_ego_comp,
                                                 double ego_vx, double ego_vy)
{
    // 1. Pre-process (no changes)
    for (auto& cell : grid_) {
        cell.dynamic_score *= 0.90;
        if (cell.dynamic_score < 0.01) cell.dynamic_score = 0.0;
        if (cell.m_free > 0.8) {
            cell.free_streak = std::min<std::uint8_t>(255, cell.free_streak + 1);
        } else if (cell.m_occ > 0.6) {
            cell.free_streak = 0;
        }
    }

    auto& parts = particle_filter_->getParticles();
    const int need_on_frames  = 2;
    const int need_off_frames = 4;

    // 2. Define lambda for processing particles in a cell
    auto flush_cell = [&](int cell_idx, int start, int end) {
        if (cell_idx < 0 || cell_idx >= static_cast<int>(grid_.size())) return;
        auto& c = grid_[cell_idx];

        // --- Calculate Particle Statistics (no changes) ---
        if (end - start <= 2) {
            c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            if (c.stat_streak >= need_off_frames) c.is_dynamic = false;
            c.dyn_streak = 0; return;
        }
        double w_sum=0, vx_sum=0, vy_sum=0;
        for (int j = start; j < end; ++j) {
            w_sum += parts[j].weight; vx_sum += parts[j].weight * parts[j].vx; vy_sum += parts[j].weight * parts[j].vy;
        }
        if (w_sum <= 1e-9) {
             c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            if (c.stat_streak >= need_off_frames) c.is_dynamic = false;
            c.dyn_streak = 0; return;
        }
        c.mean_vx = vx_sum / w_sum; c.mean_vy = vy_sum / w_sum;
        double speed = std::hypot(c.mean_vx, c.mean_vy);
        if (use_ego_comp) { speed = std::hypot(c.mean_vx - ego_vx, c.mean_vy - ego_vy); }
        // --- End Particle Statistics ---


        // ================== [MODIFICATION 3: User Request Logic] ==================

        // --- Determine Dynamic Candidate Status (Inputs) ---
        const bool is_occupied = (c.m_occ > 0.60);
        
        // Input 1: Particle-based check (already includes fusion)
        const bool has_speed_from_particles = (speed > static_vel_thresh);

        // Input 2: Radar-based check (NOW uses spatial smoothing as requested)
        double smoothed_vr_hint = 0.0;
        // Get coordinates from index for the smoothing function
        int gx = cell_idx % grid_width_;
        int gy = cell_idx / grid_width_;
        bool has_reliable_smoothed_radar = use_radar_ && getSmoothedRadarVrHint(gx, gy, smoothed_vr_hint);
        
        double smoothed_radar_speed = std::abs(smoothed_vr_hint);
        
        // This variable now uses the smoothed (49-cell) hint
        const bool has_speed_from_radar = has_reliable_smoothed_radar && (smoothed_radar_speed > static_vel_thresh);
        // This variable also uses the smoothed (49-cell) hint
        const bool is_reliably_static_by_radar = has_reliable_smoothed_radar && (smoothed_radar_speed < static_vel_thresh);


        // --- Logic Branching (use_radar flag) ---
        bool dyn_candidate = false; // Initialize

        if (use_radar_) 
        {
            // --- 1. Radar-Fusion Mode (Using OR as requested) ---
            // (Particle fusion check) OR (Smoothed radar check)
            dyn_candidate = is_occupied 
                            && !is_reliably_static_by_radar // Veto if reliably static
                            && (has_speed_from_particles && has_speed_from_radar); // Changed to OR

            // Safety Nets (remain the same)
            if (use_mc_ && is_occupied && !dyn_candidate && has_speed_from_particles && is_reliably_static_by_radar) { 
                dyn_candidate = true;
            }
            bool is_currently_static = is_occupied && !dyn_candidate;
            if (use_fsd_ && is_currently_static && c.stat_streak >= fsd_T_static_ && c.free_streak >= fsd_T_free_) { 
                dyn_candidate = true;
                c.free_streak = 0; 
            }
        }
        else 
        {
            // --- 2. LiDAR-Only Mode ---
            // (Particle fusion check) only
            dyn_candidate = is_occupied && has_speed_from_particles;
        }
        // =====================================================================


        // --- Hysteresis update logic (no changes) ---
        if (dyn_candidate) {
            // 'has_speed_from_radar' now uses the smoothed hint, which is fine
            int streak_increase = (has_speed_from_radar) ? 2 : 1;
            c.dyn_streak  = std::min<std::uint8_t>(255, c.dyn_streak + streak_increase);
            c.stat_streak = 0;
        } else {
            c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            c.dyn_streak  = 0;
        }

        // --- Determine Final Cell State based on Hysteresis (no changes) ---
        if (!c.is_dynamic && c.dyn_streak >= need_on_frames) {
            c.is_dynamic = true;
        }
        if ( c.is_dynamic && c.stat_streak >= need_off_frames) {
            c.is_dynamic = false;
        }

        // --- Update Dynamic Score (for visualization) (no changes) ---
        const double target = c.is_dynamic ? std::min(1.0, speed / std::max(1e-6, max_vel_for_scaling)) : 0.0;
        const double alpha  = 0.6;
        c.dynamic_score = alpha * target + (1.0 - alpha) * c.dynamic_score;
    }; // End of flush_cell lambda

    // 3. Iterate through sorted particles and call flush_cell (no changes)
    int current_idx = -1; int first_i = 0;
    for (int i = 0; i <= static_cast<int>(parts.size()); ++i) {
        bool last = (i == static_cast<int>(parts.size()));
        int idx   = last ? -1 : parts[i].grid_cell_idx;
        if (last || idx != current_idx) {
            flush_cell(current_idx, first_i, i);
            if (last) break;
            current_idx = idx;
            first_i = i;
        }
    }
} // End of calculateVelocityStatistics

// toOccupancyGridMsg (no changes)
void DynamicGridMap::toOccupancyGridMsg(nav_msgs::OccupancyGrid& msg, const std::string& frame_id) const
{
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.info.resolution = resolution_;
    msg.info.width  = grid_width_;
    msg.info.height = grid_height_;
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(grid_width_ * grid_height_, -1);
    for (size_t i = 0; i < grid_.size(); ++i) {
        const auto& c = grid_[i];
        double p_occ = c.m_occ + 0.5 * (1.0 - c.m_occ - c.m_free);
        if (std::abs(p_occ - 0.5) < 0.1) {
            msg.data[i] = -1;
        } else {
            msg.data[i] = static_cast<int8_t>(std::round(std::min(1.0, std::max(0.0, p_occ)) * 100.0));
        }
    }
}

// toMarkerArrayMsg (no changes)
void DynamicGridMap::toMarkerArrayMsg(visualization_msgs::MarkerArray& arr,
                                      const std::string& frame_id,
                                      bool show_velocity_arrows) const
{
    arr.markers.clear();
    visualization_msgs::Marker cubes;
    cubes.header.stamp = ros::Time::now();
    cubes.header.frame_id = frame_id;
    cubes.ns = "dogm_cells";
    cubes.id = 0;
    cubes.type = visualization_msgs::Marker::CUBE_LIST;
    cubes.action = visualization_msgs::Marker::ADD;
    cubes.pose.orientation.w = 1.0;
    cubes.scale.x = resolution_;
    cubes.scale.y = resolution_;
    cubes.scale.z = 0.02;
    cubes.lifetime = ros::Duration(1);
    cubes.points.reserve(grid_.size());
    cubes.colors.reserve(grid_.size());
    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            const auto& c = grid_[gridToIndex(x, y)];
            geometry_msgs::Point p;
            p.x = origin_x_ + (x + 0.5) * resolution_;
            p.y = origin_y_ + (y + 0.5) * resolution_;
            p.z = -0.02;
            std_msgs::ColorRGBA col;
            col.a = 0.2;
            if (c.m_occ > 0.6) {
                if (c.is_dynamic) { col.r = 1.0f; col.g = 0.0f; col.b = 0.0f; } // Red
                else { col.r = 0.0f; col.g = 0.0f; col.b = 1.0f; } // Blue
                col.a = 0.6 + 0.4 * std::min(1.0, c.m_occ);
            } else if (c.m_free > 0.6) {
                col.r = col.g = col.b = 1.0f; col.a = 1.0f; // White
            } else {
                col.r = col.g = col.b = 0.5f; col.a = 1.0f; // Gray
            }
            cubes.points.push_back(p);
            cubes.colors.push_back(col);
        }
    }
    arr.markers.push_back(cubes);

    if (show_velocity_arrows) {
        visualization_msgs::Marker arrows;
        arrows.header.stamp = ros::Time::now();
        arrows.header.frame_id = frame_id;
        arrows.ns = "dogm_vel";
        arrows.id = 1;
        arrows.type = visualization_msgs::Marker::ARROW;
        arrows.action = visualization_msgs::Marker::ADD;
        arrows.scale.x = 0.02; arrows.scale.y = 0.04; arrows.scale.z = 0.04;
        arrows.color.r = 1.0; arrows.color.g = 0.0; arrows.color.b = 0.0; arrows.color.a = 1.0;
        arrows.lifetime = ros::Duration(0.2);
        int arrow_id = 10;
        for (int y = 0; y < grid_height_; ++y) {
            for (int x = 0; x < grid_width_; ++x) {
                const auto& c = grid_[gridToIndex(x, y)];
                if (!c.is_dynamic || c.m_occ < 0.6) continue;
                geometry_msgs::Point p0, p1;
                p0.x = origin_x_ + (x + 0.5) * resolution_;
                p0.y = origin_y_ + (y + 0.5) * resolution_;
                p0.z = 0.00;
                const double scale = 0.25;
                p1.x = p0.x + scale * c.mean_vx;
                p1.y = p0.y + scale * c.mean_vy;
                p1.z = 0.00;
                visualization_msgs::Marker a = arrows;
                a.id = arrow_id++;
                a.points.clear();
                a.points.push_back(p0);
                a.points.push_back(p1);
                arr.markers.push_back(a);
            }
        }
    }
}