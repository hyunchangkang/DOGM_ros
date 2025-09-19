#include "dogm_ros/dynamic_grid_map.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <random>

DynamicGridMap::DynamicGridMap(double grid_size, double resolution, int num_particles,
                               double process_noise_pos, double process_noise_vel)
    : grid_size_(grid_size),
      resolution_(resolution),
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

void DynamicGridMap::generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // 측정 그리드를 초기화하고, 각 셀의 Lidar hit 횟수를 저장할 임시 카운터를 만듭니다.
    for (auto& m : measurement_grid_) { m.m_occ_z = 0.0; m.m_free_z = 0.0; }
    std::vector<int> hit_counts(grid_width_ * grid_height_, 0);

    if (!scan) return;

    const double angle_min = static_cast<double>(scan->angle_min);
    const double angle_inc = static_cast<double>(scan->angle_increment);
    const double range_max = static_cast<double>(scan->range_max);

    // 1단계: 모든 Lidar 포인트에 대해 순회하며 Free-space와 Hit-count를 기록합니다.
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        const double r  = static_cast<double>(scan->ranges[i]);
        if (!std::isfinite(r)) continue;
        const double th = angle_min + angle_inc * static_cast<double>(i);

        // Free-space 업데이트 (기존과 동일)
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

        // [핵심 수정] Lidar 포인트가 닿은 셀의 'hit 횟수'를 1 증가시킵니다.
        if (r < range_max) {
            const double wx = r * std::cos(th);
            const double wy = r * std::sin(th);
            int gx, gy;
            if (worldToGrid(wx, wy, gx, gy)) {
                hit_counts[gridToIndex(gx, gy)]++;
            }
        }
    }

    // 2단계: 기록된 hit 횟수를 기반으로 점유 질량(m_occ_z)을 계산합니다.
    for (size_t i = 0; i < hit_counts.size(); ++i) {
        if (hit_counts[i] > 0) {
            // [핵심 수정] hit 횟수에 비례하여 점유 질량을 부여합니다.
            // 1번 맞으면 0.6, 2번 맞으면 0.72, 많이 맞을수록 1.0에 수렴.
            // 이는 더 많은 Lidar 포인트가 관측된 셀을 더 '확실한' 장애물로 판단하게 합니다.
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

std::vector<Particle> DynamicGridMap::generateNewParticles(double newborn_vel_stddev,
                                                           double dynamic_birth_ratio,
                                                           double dynamic_newborn_vel_stddev)
{
    std::vector<Particle> new_particles;
    std::normal_distribution<double> static_vel_dist(0.0, newborn_vel_stddev);
    std::normal_distribution<double> dynamic_vel_dist(0.0, dynamic_newborn_vel_stddev);

    for (int y = 0; y < grid_height_; ++y)
    {
        for (int x = 0; x < grid_width_; ++x)
        {
            int idx = gridToIndex(x, y);
            const auto& cell = grid_[idx];

            if (cell.rho_b > 0.5 && cell.m_occ > 0.6)
            {
                int num_to_birth = static_cast<int>(std::ceil(cell.rho_b * 4.0));
                int num_dynamic = static_cast<int>(num_to_birth * dynamic_birth_ratio);
                int num_static = num_to_birth - num_dynamic;

                for (int i = 0; i < num_static; ++i)
                {
                    Particle p;
                    gridToWorld(x, y, p.x, p.y);
                    p.vx = static_vel_dist(random_generator_);
                    p.vy = static_vel_dist(random_generator_);
                    p.weight = cell.rho_b / static_cast<double>(num_to_birth);
                    p.grid_cell_idx = idx;
                    p.age = 0;
                    new_particles.push_back(p);
                }

                for (int i = 0; i < num_dynamic; ++i)
                {
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


// [종합 수정] 모든 개선 사항이 적용된 최종 버전의 함수
void DynamicGridMap::calculateVelocityStatistics(double static_vel_thresh,
                                                 double /*mahalanobis_thresh*/,
                                                 double max_variance_thresh,
                                                 double max_vel_for_scaling,
                                                 bool   use_ego_comp,
                                                 double ego_vx, double ego_vy)
{
    // 시각화를 위한 동적 점수는 부드럽게 감소하도록 유지
    for (auto& cell : grid_) {
        cell.dynamic_score *= 0.90;
        if (cell.dynamic_score < 0.01) cell.dynamic_score = 0.0;
    }

    auto& parts = particle_filter_->getParticles();

    // 동적/정적 판정을 위한 프레임 카운트 (Hysteresis 필터)
    const int need_on_frames  = 2; // 동적이 되기 위한 조건
    const int need_off_frames = 4; // 정적이 되기 위한 조건 (더 보수적으로)

    // 각 셀의 파티클들을 처리하는 람다 함수
    auto flush_cell = [&](int cell_idx, int start, int end) {
        if (cell_idx < 0 || cell_idx >= static_cast<int>(grid_.size())) return;
        auto& c = grid_[cell_idx];

        // 셀에 할당된 파티클이 너무 적으면 무조건 정적으로 판단
        if (end - start <= 2) {
            c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            if (c.stat_streak >= need_off_frames) c.is_dynamic = false;
            c.dyn_streak = 0;
            return;
        }

        // 1. 셀의 평균 속도 계산 (가중 평균)
        double w_sum=0, vx_sum=0, vy_sum=0;
        for (int j = start; j < end; ++j) {
            const auto& p = parts[j];
            const double w = p.weight;
            w_sum  += w;
            vx_sum += w * p.vx;
            vy_sum += w * p.vy;
        }
        if (w_sum <= 1e-9) {
             c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            if (c.stat_streak >= need_off_frames) c.is_dynamic = false;
            c.dyn_streak = 0;
            return;
        }
        c.mean_vx = vx_sum / w_sum;
        c.mean_vy = vy_sum / w_sum;

        // (디버깅/확장용) 속도 분산 계산
        double var_vx_sum = 0, var_vy_sum = 0;
        for (int j = start; j < end; ++j) {
            const auto& p = parts[j];
            const double w = p.weight;
            var_vx_sum += w * (p.vx - c.mean_vx) * (p.vx - c.mean_vx);
            var_vy_sum += w * (p.vy - c.mean_vy) * (p.vy - c.mean_vy);
        }
        c.var_vx = var_vx_sum / w_sum;
        c.var_vy = var_vy_sum / w_sum;

        // 2. 동적 후보 판단 (절대 속도 임계값 기준)
        double speed = std::hypot(c.mean_vx, c.mean_vy);
        if (use_ego_comp) {
            speed = std::hypot(c.mean_vx - ego_vx, c.mean_vy - ego_vy);
        }

        const bool is_occupied = (c.m_occ > 0.60);
        const bool has_speed   = (speed > static_vel_thresh);

        const bool dyn_candidate = is_occupied && has_speed;

        // 3. Hysteresis 필터 적용
        if (dyn_candidate) {
            c.dyn_streak  = std::min<std::uint8_t>(255, c.dyn_streak + 1);
            c.stat_streak = 0;
        } else {
            c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            c.dyn_streak  = 0;
        }

        // 최종 상태 결정
        if (!c.is_dynamic && c.dyn_streak  >= need_on_frames) c.is_dynamic = true;
        if ( c.is_dynamic && c.stat_streak >= need_off_frames) c.is_dynamic = false;

        // 시각화를 위한 동적 점수 계산
        const double target = c.is_dynamic ? std::min(1.0, speed / std::max(1e-6, max_vel_for_scaling)) : 0.0;
        const double alpha  = 0.6; // 부드러운 변화를 위한 가중치
        c.dynamic_score = alpha * target + (1.0 - alpha) * c.dynamic_score;
    };

    // 정렬된 파티클 배열을 순회하며 셀 단위로 flush_cell 함수 실행
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