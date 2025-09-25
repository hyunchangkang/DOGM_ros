#include "dogm_ros/dynamic_grid_map.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <random>

DynamicGridMap::DynamicGridMap(double grid_size, double resolution, int num_particles,
                               double process_noise_pos, double process_noise_vel,
                               int radar_buffer_size, int min_radar_points)
    : grid_size_(grid_size),
      resolution_(resolution),
      radar_buffer_size_(radar_buffer_size),
      min_radar_points_(min_radar_points),
      random_generator_(std::mt19937(std::random_device()()))
{
    grid_width_  = static_cast<int>(std::round(grid_size_ / resolution_));
    grid_height_ = grid_width_;
    origin_x_ = 0.0;
    origin_y_ = -grid_size_ / 2.0;

    grid_.assign(grid_width_ * grid_height_, GridCell{});
    measurement_grid_.assign(grid_width_ * grid_height_, MeasurementCell{});

    particle_filter_ = std::make_unique<ParticleFilter>(
        num_particles,
        process_noise_pos,
        process_noise_vel
    );
}
bool DynamicGridMap::isInside(int gx, int gy) const {
    return (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_);
}
int DynamicGridMap::gridToIndex(int gx, int gy) const { return gy * grid_width_ + gx; }

bool DynamicGridMap::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    return isInside(gx, gy);
}

void DynamicGridMap::gridToWorld(int gx, int gy, double& wx, double& wy) const {
    wx = origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_;
    wy = origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_;
}

// LiDAR와 Radar 데이터를 함께 처리하는 함수
void DynamicGridMap::generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan,
                                             const pcl::PointCloud<mmWaveCloudType>::ConstPtr& radar_cloud)
{
    // 1. 측정 그리드 초기화 및 Radar 버퍼 노화(Aging) 및 평균 재계산
    for (auto& m : measurement_grid_) { m.m_occ_z = 0.0; m.m_free_z = 0.0; }
    for (auto& c : grid_) {
        // ... (버퍼 노화 및 오래된 포인트 제거 로직은 기존과 동일) ...
        for (auto& rp : c.radar_points_buffer) {
            rp.age++;
        }
        c.radar_points_buffer.erase(
            std::remove_if(c.radar_points_buffer.begin(), c.radar_points_buffer.end(),
                           [this](const RadarPoint& rp) { return rp.age > this->radar_buffer_size_; }),
            c.radar_points_buffer.end());

        // ====================== [핵심 수정: 시간 가중 평균] ======================
        // 버퍼에 남은 포인트들로 '시간 가중 평균' 속도 계산
        if (c.radar_points_buffer.size() >= min_radar_points_) {
            double sum_weighted_vx = 0.0, sum_weighted_vy = 0.0, sum_weights = 0.0;
            for (const auto& rp : c.radar_points_buffer) {
                // 최신일수록(age가 작을수록) 높은 가중치 부여 (e.g., age=0 -> w=1.0, age=4 -> w=0.2)
                double weight = 1.0 - (static_cast<double>(rp.age) / (radar_buffer_size_ + 1.0));
                
                sum_weighted_vx += rp.vx * weight;
                sum_weighted_vy += rp.vy * weight;
                sum_weights += weight;
            }

            if (sum_weights > 1e-9) {
                c.radar_vx = sum_weighted_vx / sum_weights;
                c.radar_vy = sum_weighted_vy / sum_weights;
                c.has_reliable_radar = true;
            } else { // 모든 포인트의 가중치가 0에 가까운 예외적인 경우
                c.has_reliable_radar = false;
                c.radar_vx = 0.0;
                c.radar_vy = 0.0;
            }
        } else {
            c.has_reliable_radar = false;
            c.radar_vx = 0.0;
            c.radar_vy = 0.0;
        }
    }
    // 2. 새로운 Radar 데이터 버퍼에 추가 (FIFO의 First-In)
    if (radar_cloud) {
        for (const auto& pt : radar_cloud->points) {
            int gx, gy;
            if (worldToGrid(pt.x, pt.y, gx, gy)) {
                int idx = gridToIndex(gx, gy);
                double angle = std::atan2(pt.y, pt.x);
                
                RadarPoint new_rp;
                new_rp.vx = pt.velocity * std::cos(angle);
                new_rp.vy = pt.velocity * std::sin(angle);
                new_rp.age = 0; // 새로 추가된 포인트는 나이가 0

                grid_[idx].radar_points_buffer.push_back(new_rp);
            }
        }
    }


    // 3. LiDAR 데이터 처리 (기존 로직과 거의 동일)
    std::vector<int> hit_counts(grid_width_ * grid_height_, 0);
    if (!scan) return;
    // ... (LiDAR free-space, hit-count 계산 로직은 기존과 동일) ...
    const double angle_min = static_cast<double>(scan->angle_min);
    const double angle_inc = static_cast<double>(scan->angle_increment);
    const double range_max = static_cast<double>(scan->range_max);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        const double r  = static_cast<double>(scan->ranges[i]);
        if (!std::isfinite(r)) continue;
        const double th = angle_min + angle_inc * static_cast<double>(i);

        const double step  = resolution_ * 0.9;
        const double limit = std::min(r, range_max);
        for (double rr = 0.0; rr < limit; rr += step) {
            const double wx = rr * std::cos(th);
            const double wy = rr * std::sin(th);
            int gx, gy;
            if (!worldToGrid(wx, wy, gx, gy)) break;
            auto& cell = measurement_grid_[gridToIndex(gx, gy)];
            cell.m_free_z = std::min(1.0, cell.m_free_z + 0.10);
        }

        if (r < range_max) {
            const double wx = r * std::cos(th);
            const double wy = r * std::sin(th);
            int gx, gy;
            if (worldToGrid(wx, wy, gx, gy)) {
                hit_counts[gridToIndex(gx, gy)]++;
            }
        }
    }

    for (size_t i = 0; i < hit_counts.size(); ++i) {
        if (hit_counts[i] > 0) {
            double confidence = 1.0 - std::pow(0.4, hit_counts[i]);
            measurement_grid_[i].m_occ_z = std::min(1.0, confidence);
        }
    }
}


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
        double K = m_occ_pred * meas.m_free_z + (1.0 - m_occ_pred) * meas.m_occ_z;
        double norm = 1.0 / std::max(1e-9, (1.0 - K));

        double m_occ_upd = norm * (m_occ_pred * (1.0 - meas.m_free_z) + (1.0 - m_occ_pred) * meas.m_occ_z * birth_prob);
        double m_free_upd = norm * ((1.0 - m_occ_pred) * (1.0 - meas.m_occ_z) + m_occ_pred * meas.m_free_z);

        m_occ_upd = std::min(1.0, std::max(0.0, m_occ_upd));
        m_free_upd = std::min(1.0, std::max(0.0, m_free_upd));

        double term = m_occ_pred + birth_prob * (1.0 - m_occ_pred);
        cell.rho_b = (term > 1e-9) ? (m_occ_upd * birth_prob * (1.0 - m_occ_pred)) / term : 0.0;
        cell.rho_p = std::max(0.0, m_occ_upd - cell.rho_b);

        cell.m_occ  = m_occ_upd;
        cell.m_free = m_free_upd;
    }
}
bool DynamicGridMap::getSmoothedRadarHint(int center_gx, int center_gy, double& hint_vx, double& hint_vy) const
{
    double sum_weighted_vx = 0.0, sum_weighted_vy = 0.0, sum_weights = 0.0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int gx = center_gx + dx;
            int gy = center_gy + dy;
            if (isInside(gx, gy)) {
                const auto& cell = grid_[gridToIndex(gx, gy)];
                // [수정] 'has_reliable_radar' 플래그로 신뢰도 판단
                if (cell.has_reliable_radar) {
                    double weight = 0.0;
                    if (dx == 0 && dy == 0) weight = 1.0;
                    else if (dx == 0 || dy == 0) weight = 0.5;
                    else weight = 0.25;
                    
                    sum_weighted_vx += cell.radar_vx * weight;
                    sum_weighted_vy += cell.radar_vy * weight;
                    sum_weights += weight;
                }
            }
        }
    }
    if (sum_weights > 1e-9) {
        hint_vx = sum_weighted_vx / sum_weights;
        hint_vy = sum_weighted_vy / sum_weights;
        return true;
    }
    return false;
}

std::vector<Particle> DynamicGridMap::generateNewParticles(double newborn_vel_stddev,
                                                           double dynamic_birth_ratio,
                                                           double dynamic_newborn_vel_stddev,
                                                           double radar_newborn_vel_stddev)
{
    std::vector<Particle> new_particles;
    std::normal_distribution<double> static_vel_dist(0.0, newborn_vel_stddev);
    std::normal_distribution<double> dynamic_vel_dist(0.0, dynamic_newborn_vel_stddev);
    std::normal_distribution<double> radar_vel_dist(0.0, radar_newborn_vel_stddev);

    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            int idx = gridToIndex(x, y);
            const auto& cell = grid_[idx];

            if (cell.rho_b > 0.5 && cell.m_occ > 0.6) {
                int num_to_birth = static_cast<int>(std::ceil(cell.rho_b * 4.0));
                
                double hint_vx, hint_vy;
                // [수정] 가중 평균된 부드러운 Radar 힌트를 가져옴
                bool has_smoothed_hint = getSmoothedRadarHint(x, y, hint_vx, hint_vy);

                if (has_smoothed_hint) { //  smoothed Radar 힌트가 있으면
                    for (int i = 0; i < num_to_birth; ++i) {
                        Particle p;
                        gridToWorld(x, y, p.x, p.y);
                        // 힌트 속도를 중심으로 파티클 생성
                        p.vx = hint_vx + radar_vel_dist(random_generator_);
                        p.vy = hint_vy + radar_vel_dist(random_generator_);
                        p.weight = cell.rho_b / static_cast<double>(num_to_birth);
                        p.grid_cell_idx = idx;
                        p.age = 0;
                        new_particles.push_back(p);
                    }
                } else { // 주변에 Radar 힌트가 없으면 기존 방식대로
                    int num_dynamic = static_cast<int>(num_to_birth * dynamic_birth_ratio);
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
    }
    return new_particles;
}



void DynamicGridMap::calculateVelocityStatistics(double static_vel_thresh,
                                                 double max_vel_for_scaling,
                                                 bool   use_ego_comp,
                                                 double ego_vx, double ego_vy)
{
    // 각 셀의 동적 점수를 시간이 지남에 따라 약간씩 감소시킴
    for (auto& cell : grid_) {
        cell.dynamic_score *= 0.90;
        if (cell.dynamic_score < 0.01) cell.dynamic_score = 0.0;
    }

    auto& parts = particle_filter_->getParticles();
    // 동적/정적 상태 변경을 위한 연속 프레임 카운트 임계값
    const int need_on_frames  = 2; // 동적으로 바뀌기 위해 필요한 연속 프레임 수
    const int need_off_frames = 4; // 정적으로 바뀌기 위해 필요한 연속 프레임 수

    // 파티클이 셀별로 정렬되어 있으므로, 각 셀에 대한 통계를 한 번에 계산하는 람다 함수
    auto flush_cell = [&](int cell_idx, int start, int end) {
        if (cell_idx < 0 || cell_idx >= static_cast<int>(grid_.size())) return;
        auto& c = grid_[cell_idx];

        // 셀에 파티클이 너무 적으면(2개 이하) 통계 계산이 무의미하므로 정적 카운트만 올림
        if (end - start <= 2) {
            c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            if (c.stat_streak >= need_off_frames) c.is_dynamic = false;
            c.dyn_streak = 0;
            return;
        }

        // 파티클 가중치를 고려한 평균 속도 계산
        double w_sum=0, vx_sum=0, vy_sum=0;
        for (int j = start; j < end; ++j) {
            const auto& p = parts[j];
            const double w = p.weight;
            w_sum  += w;
            vx_sum += w * p.vx;
            vy_sum += w * p.vy;
        }

        // 가중치 합이 너무 작으면 계산 오류를 방지하고 정적으로 처리
        if (w_sum <= 1e-9) {
             c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            if (c.stat_streak >= need_off_frames) c.is_dynamic = false;
            c.dyn_streak = 0;
            return;
        }
        c.mean_vx = vx_sum / w_sum;
        c.mean_vy = vy_sum / w_sum;

        // 로봇 자신의 움직임을 보상한 상대 속도 계산
        double speed = std::hypot(c.mean_vx, c.mean_vy);
        if (use_ego_comp) {
            speed = std::hypot(c.mean_vx - ego_vx, c.mean_vy - ego_vy);
        }

        // ====================== [핵심 판단 로직] ======================
        const bool is_occupied = (c.m_occ > 0.60);

        // 1. FIFO 버퍼에 기반한 신뢰할 수 있는 Radar 정보가 있는지 확인
        const bool has_reliable_radar = c.has_reliable_radar;
        const double radar_speed = std::hypot(c.radar_vx, c.radar_vy);

        // 2. 신뢰할 수 있는 Radar가 '정적'이라고 강하게 주장하는지 판단
        const bool is_reliably_static_by_radar = has_reliable_radar && (radar_speed < static_vel_thresh);
        
        // 3. 파티클 필터 추정치가 '동적'이라고 주장하는지 판단
        const bool has_speed_from_particles = (speed > static_vel_thresh);

        // 4. 신뢰할 수 있는 Radar가 '동적'이라고 주장하는지 판단
        const bool has_speed_from_radar = has_reliable_radar && (radar_speed > static_vel_thresh);

        // 5. 최종 동적 후보 결정
        // 조건: 점유되어 있고, Radar가 '정적'이라고 주장하지 않으며(!),
        //       파티클이나 Radar 둘 중 하나가 '동적'이라고 주장해야 함.
        const bool dyn_candidate = is_occupied && !is_reliably_static_by_radar && (has_speed_from_particles || has_speed_from_radar);
        // ===============================================================

        // 히스테리시스(Hysteresis) 적용: 상태가 급격히 바뀌는 것을 방지
        if (dyn_candidate) {
            // Radar 증거가 있으면 동적 카운트를 2배로 빠르게 올려 더 확신을 줌
            c.dyn_streak  = std::min<std::uint8_t>(255, c.dyn_streak + (has_speed_from_radar ? 2 : 1));
            c.stat_streak = 0; // 동적 후보이므로 정적 카운트는 리셋
        } else {
            c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            c.dyn_streak  = 0; // 정적 후보이므로 동적 카운트는 리셋
        }

        // 최종 상태 결정
        if (!c.is_dynamic && c.dyn_streak  >= need_on_frames) c.is_dynamic = true;
        if ( c.is_dynamic && c.stat_streak >= need_off_frames) c.is_dynamic = false;

        // RViz 시각화를 위한 동적 점수(0~1) 계산 (부드러운 색상 변화용)
        const double target = c.is_dynamic ? std::min(1.0, speed / std::max(1e-6, max_vel_for_scaling)) : 0.0;
        const double alpha  = 0.6; // 이전 점수를 얼마나 유지할지 결정
        c.dynamic_score = alpha * target + (1.0 - alpha) * c.dynamic_score;
    };

    // 정렬된 파티클 리스트를 순회하며 셀 단위로 flush_cell 함수 호출
    int current_idx = -1;
    int first_i = 0;
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
}

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
        if (std::abs(p_occ - 0.5) < 0.1) msg.data[i] = -1;
        else msg.data[i] = static_cast<int8_t>(std::round(std::min(1.0, std::max(0.0, p_occ)) * 100.0));
    }
}

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
    cubes.lifetime = ros::Duration(0.2);

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
                if (c.is_dynamic) { col.r =  1.0; col.g = 0.0; col.b = 0.0; }
                else              { col.r = 0.529f; col.g = 0.808f; col.b = 0.980f; }
                col.a = 0.6 + 0.4 * std::min(1.0, c.m_occ);
            } else if (c.m_free > 0.6) {
                col.r = col.g = col.b = 1.0; col.a = 1;
            } else {
                col.r = col.g = col.b = 0.5; col.a = 1;
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
        arrows.scale.x = 0.02;
        arrows.scale.y = 0.04;
        arrows.scale.z = 0.04;
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