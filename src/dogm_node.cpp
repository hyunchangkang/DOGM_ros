#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <memory>
#include <algorithm>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>

#include "dogm_ros/dynamic_grid_map.h"
#include "dogm_ros/structures.h" // mmWaveCloudType 정의를 위해 추가

// [핵심 수정] 새로운 3단계(녹->보->주) 색상 보간 함수
void interpolateColor(double velocity, double& r, double& g, double& b) {
    // 색상 정의 (0.0 ~ 1.0 범위)
    const double green[3] = {124/255.0, 252/255.0, 154/255.0}; // #7CFC9A
    const double purple[3] = {138/255.0, 43/255.0, 226/255.0}; // #8A2BE2
    const double orange[3] = {255/255.0, 165/255.0, 0/255.0};   // #FFA500

    velocity = std::abs(velocity);

    if (velocity < 0.5) { // 0.0 ~ 0.5 m/s: 녹색 -> 보라색
        double scale = velocity / 0.5;
        r = (1.0 - scale) * green[0] + scale * purple[0];
        g = (1.0 - scale) * green[1] + scale * purple[1];
        b = (1.0 - scale) * green[2] + scale * purple[2];
    } else if (velocity < 1.5) { // 0.5 ~ 1.5 m/s: 보라색 -> 주황색
        double scale = (velocity - 0.5) / 1.0;
        r = (1.0 - scale) * purple[0] + scale * orange[0];
        g = (1.0 - scale) * purple[1] + scale * orange[1];
        b = (1.0 - scale) * purple[2] + scale * orange[2];
    } else { // 1.5 m/s 이상: 주황색 고정
        r = orange[0];
        g = orange[1];
        b = orange[2];
    }
}
class DogmNode {
public:
    DogmNode() : nh_(), pnh_("~") {
        loadParams();

        grid_map_ = std::make_unique<DynamicGridMap>(
            grid_size_, grid_res_, num_particles_,
            process_noise_pos_, process_noise_vel_
        );

        grid_pub_   = nh_.advertise<nav_msgs::OccupancyGrid>(grid_topic_, 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);
        radar_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/dogm/radar_velocity_viz", 1);

        scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(lidar_topic_, 1, &DogmNode::scanCb, this);
        radar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(radar_topic_, 1, &DogmNode::radarCb, this);

        if (use_ego_comp_) {
            odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &DogmNode::odomCb, this);
        }

        const double hz = (filter_hz_ > 0.5 ? filter_hz_ : 20.0);
        timer_ = nh_.createTimer(ros::Duration(1.0 / hz), &DogmNode::updateLoop, this);

        ROS_INFO_STREAM("DOGM with Radar visualization node started.");
    }

private:
    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
        last_scan_ = msg;
        has_scan_  = true;
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!msg) return;
        ego_vx_ = msg->twist.twist.linear.x;
        ego_vy_ = msg->twist.twist.linear.y;
    }

    void radarCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // [수정 1] Ptr(일반 포인터) 타입의 스마트 포인터를 새로 할당하여 초기화합니다.
        pcl::PointCloud<mmWaveCloudType>::Ptr current_radar_cloud(new pcl::PointCloud<mmWaveCloudType>());

        // [수정 1] ROS 메시지를 Ptr이 가리키는 객체에 직접 변환하여 저장합니다.
        // const가 아니므로 fromROSMsg 함수 사용이 가능합니다.
        pcl::fromROSMsg(*msg, *current_radar_cloud);

        if (current_radar_cloud->points.empty()) {
            return; // 데이터가 없으면 즉시 종료
        }

        // --- RViz 시각화 ---
        visualization_msgs::Marker points_viz;
        points_viz.header.frame_id = base_frame_; // params.yaml의 base_frame 사용
        points_viz.header.stamp = ros::Time::now();
        points_viz.ns = "radar_velocity_status";
        points_viz.id = 0;
        points_viz.type = visualization_msgs::Marker::POINTS;
        points_viz.action = visualization_msgs::Marker::ADD;
        points_viz.pose.orientation.w = 1.0;
        points_viz.scale.x = 0.04;
        points_viz.scale.y = 0.04;
        points_viz.lifetime = ros::Duration(0.1); // 마커가 잠시 후 사라지도록 설정

        // [핵심 수정] 속도 기반 컬러맵 적용
        const double MAX_VEL_FOR_COLORMAP = 1.5; // 컬러맵의 최대 속도 기준 (m/s)

        for (const auto& pt : current_radar_cloud->points) {
            geometry_msgs::Point p;
            p.x = pt.x; p.y = pt.y; p.z = 0.1; // Lidar와 겹치지 않게 살짝 위로
            points_viz.points.push_back(p);

            std_msgs::ColorRGBA color;
            color.a = 1.0;

            // 속도를 정규화 (0.0 ~ 1.0)
            double normalized_velocity = std::abs(pt.velocity) / MAX_VEL_FOR_COLORMAP;
            
            // 정규화된 속도를 기반으로 R, G, B 값을 계산
            double r, g, b;
            interpolateColor(normalized_velocity, r, g, b);
            color.r = r;
            color.g = g;
            color.b = b;
            
            points_viz.colors.push_back(color);
        }
        radar_viz_pub_.publish(points_viz);
    }

    void updateLoop(const ros::TimerEvent&) {
        if (!has_scan_) return;

        ros::Time now = ros::Time::now();
        double dt = (now - last_update_time_).toSec();
        if (dt <= 0.0) return;
        last_update_time_ = now;

        auto& pf = grid_map_->getParticleFilter();

        // [수정 2] Radar 데이터는 DOGM 로직에 사용하지 않으므로 인자를 하나만 전달합니다.
        grid_map_->generateMeasurementGrid(last_scan_);

        pf.predict(dt, persistence_prob_,
                   velocity_damping_threshold_, velocity_damping_factor_,
                   max_velocity_);

        pf.sortParticlesByGridCell(*grid_map_);
        pf.updateWeights(grid_map_->getMeasurementGrid());
        grid_map_->updateOccupancy(birth_prob_);

        auto new_borns = grid_map_->generateNewParticles(newborn_vel_stddev_,
                                                         dynamic_birth_ratio_,
                                                         dynamic_newborn_vel_stddev_);
        pf.resample(new_borns);

        grid_map_->calculateVelocityStatistics(static_velocity_threshold_,
                                               mahalanobis_threshold_,
                                               max_variance_threshold_,
                                               max_vel_for_scaling_,
                                               use_ego_comp_,
                                               ego_vx_, ego_vy_);

        nav_msgs::OccupancyGrid grid_msg;
        grid_map_->toOccupancyGridMsg(grid_msg, base_frame_);
        grid_pub_.publish(grid_msg);

        visualization_msgs::MarkerArray marker_msg;
        grid_map_->toMarkerArrayMsg(marker_msg, base_frame_, show_velocity_arrows_);
        marker_pub_.publish(marker_msg);

        has_scan_ = false;
    }

    void loadParams() {
        // radar_topic 파라미터를 추가하여 유연성 확보
        pnh_.param("lidar_topic", lidar_topic_, std::string("/scan"));
        pnh_.param("radar_topic", radar_topic_, std::string("/ti_mmwave/radar_scan_pcl_0"));
        pnh_.param("grid_topic", grid_topic_, std::string("/dogm/grid"));
        pnh_.param("marker_topic", marker_topic_, std::string("/dogm/markers"));
        pnh_.param("base_frame", base_frame_, std::string("base_link"));

        pnh_.param("grid_size", grid_size_, 3.0);
        pnh_.param("grid_resolution", grid_res_, 0.15);
        pnh_.param("filter_update_rate", filter_hz_, 20.0);
        pnh_.param("persistence_prob", persistence_prob_, 0.99);
        pnh_.param("birth_prob", birth_prob_, 0.03);
        pnh_.param("num_particles", num_particles_, 5000);
        pnh_.param("process_noise_pos", process_noise_pos_, 0.05);
        pnh_.param("process_noise_vel", process_noise_vel_, 2.5);
        pnh_.param("max_velocity", max_velocity_, 2.0);
        pnh_.param("newborn_vel_stddev", newborn_vel_stddev_, 0.5);
        pnh_.param("dynamic_birth_ratio", dynamic_birth_ratio_, 0.6);
        pnh_.param("dynamic_newborn_vel_stddev", dynamic_newborn_vel_stddev_, 4.0);
        pnh_.param("velocity_damping_threshold", velocity_damping_threshold_, 0.2);
        pnh_.param("velocity_damping_factor", velocity_damping_factor_, 0.8);
        pnh_.param("static_velocity_threshold", static_velocity_threshold_, 0.6);
        pnh_.param("mahalanobis_threshold", mahalanobis_threshold_, 9.21);
        pnh_.param("max_variance_threshold", max_variance_threshold_, 0.4);
        pnh_.param("max_velocity_for_scaling", max_vel_for_scaling_, 0.5);
        pnh_.param("show_velocity_arrows", show_velocity_arrows_, true);
        pnh_.param("use_ego_comp", use_ego_comp_, true);
        pnh_.param("odom_topic", odom_topic_, std::string("/odom"));
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber scan_sub_, odom_sub_, radar_sub_;
    ros::Publisher  grid_pub_, marker_pub_, radar_viz_pub_;
    ros::Timer      timer_;

    std::unique_ptr<DynamicGridMap> grid_map_;

    // Params
    std::string lidar_topic_, radar_topic_, grid_topic_, marker_topic_, base_frame_, odom_topic_;
    double grid_size_, grid_res_, filter_hz_, persistence_prob_, birth_prob_;
    int    num_particles_;
    double process_noise_pos_, process_noise_vel_, max_velocity_, newborn_vel_stddev_;
    double dynamic_birth_ratio_, dynamic_newborn_vel_stddev_, velocity_damping_threshold_, velocity_damping_factor_;
    double static_velocity_threshold_, mahalanobis_threshold_, max_variance_threshold_, max_vel_for_scaling_;
    bool show_velocity_arrows_, use_ego_comp_;

    // State
    sensor_msgs::LaserScan::ConstPtr last_scan_;
    // [수정 1] Radar 콜백 내에서만 사용할 것이므로 멤버 변수로 둘 필요가 없습니다.
    // pcl::PointCloud<mmWaveCloudType>::ConstPtr last_radar_cloud_;
    bool has_scan_{false};
    // bool has_radar_{false}; // 사용하지 않으므로 주석 처리
    ros::Time last_update_time_;
    double ego_vx_{0.0}, ego_vy_{0.0};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dogm_node");
    DogmNode node;
    ros::spin();
    return 0;
}