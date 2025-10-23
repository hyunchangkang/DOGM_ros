#include "dogm_ros/dynamic_grid_map.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <random>

// Constructor (accepts use_radar flag, remains the same as previous)
DynamicGridMap::DynamicGridMap(double grid_size, double resolution, int num_particles,
                               double process_noise_pos, double process_noise_vel,
                               int radar_buffer_size, int min_radar_points,
                               int radar_hint_search_radius,
                               bool use_fsd, int fsd_T_static, int fsd_T_free,
                               bool use_mc,
                               bool use_radar) // Argument still needed
    : grid_size_(grid_size),
      resolution_(resolution),
      radar_buffer_size_(radar_buffer_size),
      min_radar_points_(min_radar_points),
      radar_hint_search_radius_(radar_hint_search_radius),
      use_fsd_(use_fsd),
      fsd_T_static_(fsd_T_static),
      fsd_T_free_(fsd_T_free),
      use_mc_(use_mc),
      use_radar_(use_radar), // Store the flag for other logic
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

// ... (isInside, gridToIndex, worldToGrid, gridToWorld remain the same) ...
bool DynamicGridMap::isInside(int gx, int gy) const { /* ... */ return (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_); }
int DynamicGridMap::gridToIndex(int gx, int gy) const { /* ... */ return gy * grid_width_ + gx; }
bool DynamicGridMap::worldToGrid(double wx, double wy, int& gx, int& gy) const { /* ... */ gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_)); gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_)); return isInside(gx, gy); }
void DynamicGridMap::gridToWorld(int gx, int gy, double& wx, double& wy) const { /* ... */ wx = origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_; wy = origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_; }

// ... (generateMeasurementGrid, updateOccupancy, getSmoothedRadarVrHint, generateNewParticles remain the same as the previous correct version) ...
void DynamicGridMap::generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan, const pcl::PointCloud<mmWaveCloudType>::ConstPtr& radar_cloud) { /* ... same correct logic ... */
    // 1. Initialize measurement grid and age/filter Radar buffer
    for (auto& m : measurement_grid_) { m.m_occ_z = 0.0; m.m_free_z = 0.0; }
    for (auto& c : grid_) {
        // Age buffer and remove old points
        for (auto& rp : c.radar_points_buffer) { rp.age++; }
        c.radar_points_buffer.erase(
            std::remove_if(c.radar_points_buffer.begin(), c.radar_points_buffer.end(),
                           [this](const RadarPoint& rp) { return rp.age > this->radar_buffer_size_; }),
            c.radar_points_buffer.end());
        // Calculate '1D velocity hint (vr)' and 'direction (theta)'
        if (c.radar_points_buffer.size() >= min_radar_points_) {
            double sum_weighted_vr = 0.0, sum_weighted_x = 0.0, sum_weighted_y = 0.0, sum_weights = 0.0;
            for (const auto& rp : c.radar_points_buffer) {
                double weight = 1.0 - (static_cast<double>(rp.age) / (radar_buffer_size_ + 1.0));
                sum_weighted_vr += rp.radial_velocity * weight;
                sum_weighted_x += rp.x * weight; sum_weighted_y += rp.y * weight; sum_weights += weight;
            }
            if (sum_weights > 1e-9) {
                c.radar_vr_hint = sum_weighted_vr / sum_weights;
                c.radar_theta_hint = std::atan2(sum_weighted_y / sum_weights, sum_weighted_x / sum_weights);
                c.has_reliable_radar = true;
            } else { c.has_reliable_radar = false; c.radar_vr_hint = 0.0; }
        } else { c.has_reliable_radar = false; c.radar_vr_hint = 0.0; }
    }
    // 2. Add new Radar data to buffer
    if (use_radar_ && radar_cloud) { // Only add if use_radar is true
        for (const auto& pt : radar_cloud->points) {
            int gx, gy;
            if (worldToGrid(pt.x, pt.y, gx, gy)) {
                int idx = gridToIndex(gx, gy); RadarPoint new_rp;
                new_rp.radial_velocity = pt.velocity; new_rp.x = pt.x; new_rp.y = pt.y; new_rp.age = 0;
                grid_[idx].radar_points_buffer.push_back(new_rp);
            }
        }
    }
    // 3. LiDAR 데이터 처리 (기존 로직과 동일)
    std::vector<int> hit_counts(grid_width_ * grid_height_, 0);
    if (!scan) return;
    const double angle_min = static_cast<double>(scan->angle_min); const double angle_inc = static_cast<double>(scan->angle_increment); const double range_max = static_cast<double>(scan->range_max);
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        const double r  = static_cast<double>(scan->ranges[i]); if (!std::isfinite(r)) continue;
        const double th = angle_min + angle_inc * static_cast<double>(i); const double step  = resolution_ * 0.9; const double limit = std::min(r, range_max);
        for (double rr = 0.0; rr < limit; rr += step) {
            const double wx = rr * std::cos(th); const double wy = rr * std::sin(th); int gx, gy;
            if (!worldToGrid(wx, wy, gx, gy)) break; auto& cell = measurement_grid_[gridToIndex(gx, gy)]; cell.m_free_z = std::min(1.0, cell.m_free_z + 0.10);
        }
        if (r < range_max) {
            const double wx = r * std::cos(th); const double wy = r * std::sin(th); int gx, gy;
            if (worldToGrid(wx, wy, gx, gy)) { hit_counts[gridToIndex(gx, gy)]++; }
        }
    }
    for (size_t i = 0; i < hit_counts.size(); ++i) { if (hit_counts[i] > 3) { double confidence = 1.0 - std::pow(0.4, hit_counts[i]); measurement_grid_[i].m_occ_z = std::min(1.0, confidence); } }
}
void DynamicGridMap::updateOccupancy(double birth_prob) { /* ... same correct logic ... */
    for (auto& c : grid_) { c.m_occ = std::max(0.0, c.m_occ * 0.98); c.m_free = std::max(0.0, c.m_free * 0.98); c.rho_b = std::max(0.0, c.rho_b * 0.98); c.rho_p = std::max(0.0, c.rho_p * 0.98); }
    for (int idx = 0; idx < static_cast<int>(grid_.size()); ++idx) {
        auto& cell = grid_[idx]; const auto& meas = measurement_grid_[idx]; double m_occ_pred = std::min(1.0, std::max(0.0, cell.m_occ));
        double K = m_occ_pred * meas.m_free_z + (1.0 - m_occ_pred) * meas.m_occ_z; double norm = 1.0 / std::max(1e-9, (1.0 - K));
        double m_occ_upd = norm * (m_occ_pred * (1.0 - meas.m_free_z) + (1.0 - m_occ_pred) * meas.m_occ_z * birth_prob);
        double m_free_upd = norm * ((1.0 - m_occ_pred) * (1.0 - meas.m_occ_z) + m_occ_pred * meas.m_free_z);
        m_occ_upd = std::min(1.0, std::max(0.0, m_occ_upd)); m_free_upd = std::min(1.0, std::max(0.0, m_free_upd));
        double term = m_occ_pred + birth_prob * (1.0 - m_occ_pred); cell.rho_b = (term > 1e-9) ? (m_occ_upd * birth_prob * (1.0 - m_occ_pred)) / term : 0.0;
        cell.rho_p = std::max(0.0, m_occ_upd - cell.rho_b); cell.m_occ = m_occ_upd; cell.m_free = m_free_upd;
    }
}
bool DynamicGridMap::getSmoothedRadarVrHint(int center_gx, int center_gy, double& smoothed_vr_hint) const { /* ... same correct logic ... */
    double sum_weighted_vr = 0.0, sum_weights = 0.0;
    for (int dy = -radar_hint_search_radius_; dy <= radar_hint_search_radius_; ++dy) {
        for (int dx = -radar_hint_search_radius_; dx <= radar_hint_search_radius_; ++dx) {
            int gx = center_gx + dx; int gy = center_gy + dy;
            if (isInside(gx, gy)) {
                const auto& cell = grid_[gridToIndex(gx, gy)];
                if (cell.has_reliable_radar) {
                    int layer = std::max(std::abs(dx), std::abs(dy)); double weight = (layer == 0) ? 1.0 : ( (layer <= radar_hint_search_radius_) ? 0.5 : 0.0 );
                    sum_weighted_vr += cell.radar_vr_hint * weight; sum_weights += weight;
                }
            }
        }
    }
    if (sum_weights > 1e-9) { smoothed_vr_hint = sum_weighted_vr / sum_weights; return true; } return false;
}
std::vector<Particle> DynamicGridMap::generateNewParticles(double newborn_vel_stddev, double min_dynamic_birth_ratio, double max_dynamic_birth_ratio, double max_radar_speed_for_scaling, double dynamic_newborn_vel_stddev) { /* ... same correct logic ... */
    std::vector<Particle> new_particles; std::normal_distribution<double> static_vel_dist(0.0, newborn_vel_stddev); std::normal_distribution<double> dynamic_vel_dist(0.0, dynamic_newborn_vel_stddev);
    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            int idx = gridToIndex(x, y); const auto& cell = grid_[idx];
            if (cell.rho_b > 0.5 && cell.m_occ > 0.6) {
                int num_to_birth = static_cast<int>(std::ceil(cell.rho_b * 4.0)); double smoothed_vr = 0.0; double current_dynamic_ratio = min_dynamic_birth_ratio;
                if (use_radar_ && getSmoothedRadarVrHint(x, y, smoothed_vr)) {
                    double smoothed_speed = std::abs(smoothed_vr); double scale = std::min(1.0, smoothed_speed / std::max(1e-6, max_radar_speed_for_scaling));
                    current_dynamic_ratio = min_dynamic_birth_ratio + (max_dynamic_birth_ratio - min_dynamic_birth_ratio) * scale;
                }
                int num_dynamic = static_cast<int>(num_to_birth * current_dynamic_ratio); int num_static = num_to_birth - num_dynamic;
                for (int i = 0; i < num_static; ++i) { Particle p; gridToWorld(x, y, p.x, p.y); p.vx = static_vel_dist(random_generator_); p.vy = static_vel_dist(random_generator_); p.weight = cell.rho_b / static_cast<double>(num_to_birth); p.grid_cell_idx = idx; p.age = 0; new_particles.push_back(p); }
                for (int i = 0; i < num_dynamic; ++i) { Particle p; gridToWorld(x, y, p.x, p.y); p.vx = dynamic_vel_dist(random_generator_); p.vy = dynamic_vel_dist(random_generator_); p.weight = cell.rho_b / static_cast<double>(num_to_birth); p.grid_cell_idx = idx; p.age = 0; new_particles.push_back(p); }
            }
        }
    } return new_particles;
}

// [MODIFIED] Statistics calculation with stricter dyn_streak update
void DynamicGridMap::calculateVelocityStatistics(double static_vel_thresh,
                                                 double max_vel_for_scaling,
                                                 bool   use_ego_comp,
                                                 double ego_vx, double ego_vy)
{
    // Attenuate dynamic score & update free streak
    for (auto& cell : grid_) {
        cell.dynamic_score *= 0.90;
        if (cell.dynamic_score < 0.01) cell.dynamic_score = 0.0;
        if (cell.m_free > 0.8) { cell.free_streak = std::min<std::uint8_t>(255, cell.free_streak + 1); }
        else if (cell.m_occ > 0.6) { cell.free_streak = 0; }
    }

    auto& parts = particle_filter_->getParticles();
    const int need_on_frames  = 2;
    const int need_off_frames = 4;

    auto flush_cell = [&](int cell_idx, int start, int end) {
        if (cell_idx < 0 || cell_idx >= static_cast<int>(grid_.size())) return;
        auto& c = grid_[cell_idx];

        // --- Calculate Particle Statistics (same as before) ---
        if (end - start <= 2) { /* ... */ c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1); if (c.stat_streak >= need_off_frames) c.is_dynamic = false; c.dyn_streak = 0; return; }
        double w_sum=0, vx_sum=0, vy_sum=0; for (int j = start; j < end; ++j) { /* ... */ w_sum += parts[j].weight; vx_sum += parts[j].weight * parts[j].vx; vy_sum += parts[j].weight * parts[j].vy; }
        if (w_sum <= 1e-9) { /* ... */ c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1); if (c.stat_streak >= need_off_frames) c.is_dynamic = false; c.dyn_streak = 0; return; }
        c.mean_vx = vx_sum / w_sum; c.mean_vy = vy_sum / w_sum; double speed = std::hypot(c.mean_vx, c.mean_vy); if (use_ego_comp) { speed = std::hypot(c.mean_vx - ego_vx, c.mean_vy - ego_vy); }
        // --- End Particle Statistics ---

        const bool is_occupied = (c.m_occ > 0.60);
        const bool has_reliable_radar = use_radar_ && c.has_reliable_radar;
        const double radar_speed = std::abs(c.radar_vr_hint);

        const bool is_reliably_static_by_radar = has_reliable_radar && (radar_speed < static_vel_thresh);
        const bool has_speed_from_particles = (speed > static_vel_thresh);
        const bool has_speed_from_radar = has_reliable_radar && (radar_speed > static_vel_thresh);

        // Base dynamic candidate condition
        bool dyn_candidate = is_occupied && !is_reliably_static_by_radar && (has_speed_from_particles && has_speed_from_radar);

        // Apply Safety Nets (only if use_radar_ is true)
        if (use_radar_) {
            if (use_mc_ && is_occupied && !dyn_candidate && has_speed_from_particles && is_reliably_static_by_radar) { dyn_candidate = true; }
            bool is_currently_static = is_occupied && !dyn_candidate;
            if (use_fsd_ && is_currently_static && c.stat_streak >= fsd_T_static_ && c.free_streak >= fsd_T_free_) { dyn_candidate = true; c.free_streak = 0; }
        }

        // --- [MODIFIED] Hysteresis update logic ---
        if (dyn_candidate) {
            int streak_increase = 0;
            if (has_speed_from_radar) {
                // If radar confirms speed, increase quickly
                streak_increase = 2;
            } else if (has_speed_from_particles && !has_reliable_radar) {
                // If only particles show speed AND there's no radar info at all, increase slowly
                streak_increase = 1;
            }
            // Note: If particles are fast but reliable radar says static, increase is 0

            c.dyn_streak  = std::min<std::uint8_t>(255, c.dyn_streak + streak_increase);
            c.stat_streak = 0;
        } else {
            c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            c.dyn_streak  = 0;
        }
        // --- [END MODIFICATION] ---

        // Final state determination (remains the same)
        if (!c.is_dynamic && c.dyn_streak  >= need_on_frames) c.is_dynamic = true;
        if ( c.is_dynamic && c.stat_streak >= need_off_frames) c.is_dynamic = false;

        // Scoring (remains the same)
        const double target = c.is_dynamic ? std::min(1.0, speed / std::max(1e-6, max_vel_for_scaling)) : 0.0;
        const double alpha  = 0.6;
        c.dynamic_score = alpha * target + (1.0 - alpha) * c.dynamic_score;
    };

    // Iterate through sorted particles (remains the same)
    int current_idx = -1; int first_i = 0;
    for (int i = 0; i <= static_cast<int>(parts.size()); ++i) { /* ... */ bool last = (i == static_cast<int>(parts.size())); int idx = last ? -1 : parts[i].grid_cell_idx; if (last || idx != current_idx) { flush_cell(current_idx, first_i, i); if (last) break; current_idx = idx; first_i = i; } }
}

// toOccupancyGridMsg (remains the same)
void DynamicGridMap::toOccupancyGridMsg(nav_msgs::OccupancyGrid& msg, const std::string& frame_id) const { /* ... */ msg.header.stamp = ros::Time::now(); msg.header.frame_id = frame_id; msg.info.resolution = resolution_; msg.info.width = grid_width_; msg.info.height = grid_height_; msg.info.origin.position.x = origin_x_; msg.info.origin.position.y = origin_y_; msg.info.origin.orientation.w = 1.0; msg.data.assign(grid_width_ * grid_height_, -1); for (size_t i = 0; i < grid_.size(); ++i) { const auto& c = grid_[i]; double p_occ = c.m_occ + 0.5 * (1.0 - c.m_occ - c.m_free); if (std::abs(p_occ - 0.5) < 0.1) msg.data[i] = -1; else msg.data[i] = static_cast<int8_t>(std::round(std::min(1.0, std::max(0.0, p_occ)) * 100.0)); } }

// toMarkerArrayMsg (remains the same, using original Red/Blue/White/Gray)
void DynamicGridMap::toMarkerArrayMsg(visualization_msgs::MarkerArray& arr,
                                      const std::string& frame_id,
                                      bool show_velocity_arrows) const
{
    arr.markers.clear();

    // --- 1. Grid Cell Visualization (Cubes) ---
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
    cubes.scale.z = 0.02; // Make cubes flat
    cubes.lifetime = ros::Duration(0.2); // Automatically disappear if not updated

    cubes.points.reserve(grid_.size());
    cubes.colors.reserve(grid_.size());

    // Iterate through each grid cell
    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            const auto& c = grid_[gridToIndex(x, y)];

            // Calculate cell center position
            geometry_msgs::Point p;
            p.x = origin_x_ + (x + 0.5) * resolution_;
            p.y = origin_y_ + (y + 0.5) * resolution_;
            p.z = -0.02; // Slightly below z=0 plane

            std_msgs::ColorRGBA col;
            col.a = 0.2; // Default transparency

            // --- Determine Color based on cell state ---
            // Always use the original Red/Blue/White/Gray scheme
            if (c.m_occ > 0.6) { // Occupied
                if (c.is_dynamic) {
                    // Dynamic = Red 🔴
                    col.r =  1.0f; col.g = 0.0f; col.b = 0.0f;
                } else {
                    // Static = Blue 🔵
                    col.r = 0.0f; col.g = 0.0f; col.b = 1.0f;
                }
                // Make more occupied cells less transparent
                col.a = 0.6 + 0.4 * std::min(1.0, c.m_occ);
            } else if (c.m_free > 0.6) { // Free
                // Free = White ⚪
                col.r = col.g = col.b = 1.0f; col.a = 1.0f; // Fully opaque white
            } else { // Unknown
                // Unknown = Gray (0.5) ⚫️
                col.r = col.g = col.b = 0.5f; col.a = 1.0f; // Fully opaque gray
            }
            // --- End Color Determination ---

            cubes.points.push_back(p);
            cubes.colors.push_back(col);
        }
    }
    arr.markers.push_back(cubes); // Add the cube list marker to the array

    // --- 2. Velocity Arrow Visualization ---
    if (show_velocity_arrows) {
        visualization_msgs::Marker arrows; // Base marker for arrows
        arrows.header.stamp = ros::Time::now();
        arrows.header.frame_id = frame_id;
        arrows.ns = "dogm_vel";
        arrows.id = 1; // Use a different ID from cubes
        arrows.type = visualization_msgs::Marker::ARROW;
        arrows.action = visualization_msgs::Marker::ADD;
        // Arrow dimensions
        arrows.scale.x = 0.02; // Shaft diameter
        arrows.scale.y = 0.04; // Head diameter
        arrows.scale.z = 0.04; // Head length (less relevant for 2D)
        // Arrow color (always red for dynamic)
        arrows.color.r = 1.0; arrows.color.g = 0.0; arrows.color.b = 0.0; arrows.color.a = 1.0;
        arrows.lifetime = ros::Duration(0.2);

        int arrow_id = 10; // Start arrow IDs from 10 to avoid conflict
        // Iterate through cells again to draw arrows for dynamic ones
        for (int y = 0; y < grid_height_; ++y) {
            for (int x = 0; x < grid_width_; ++x) {
                const auto& c = grid_[gridToIndex(x, y)];

                // Draw arrow only if cell is dynamic and sufficiently occupied
                if (!c.is_dynamic || c.m_occ < 0.6) continue;

                // Arrow start point (cell center at z=0)
                geometry_msgs::Point p0;
                p0.x = origin_x_ + (x + 0.5) * resolution_;
                p0.y = origin_y_ + (y + 0.5) * resolution_;
                p0.z = 0.00;

                // Arrow end point (scaled by velocity)
                geometry_msgs::Point p1;
                const double scale = 0.25; // Scale factor for arrow length
                p1.x = p0.x + scale * c.mean_vx;
                p1.y = p0.y + scale * c.mean_vy;
                p1.z = 0.00;

                // Create a copy of the base arrow marker
                visualization_msgs::Marker a = arrows;
                a.id = arrow_id++; // Assign unique ID to each arrow
                a.points.clear();
                a.points.push_back(p0); // Start point
                a.points.push_back(p1); // End point
                arr.markers.push_back(a); // Add individual arrow to the array
            }
        }
    }
}