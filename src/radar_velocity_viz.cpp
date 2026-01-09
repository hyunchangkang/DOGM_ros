#include "dogm_ros/radar_velocity_viz.h"
#include <pcl/common/transforms.h>

// Color mapping: Violet (low speed) -> Green -> Yellow -> Orange -> Red (high speed)
// Blue excluded to avoid confusion with static cells
void RadarVizNode::interpolateColor(double velocity, double& r, double& g, double& b)
{
    velocity = std::abs(velocity);

    const double min_vel = 0.0;
    const double max_vel = radar_viz_color_max_vel_;
    velocity = std::max(min_vel, std::min(velocity, max_vel)); 

    // Normalize to 0.0 - 1.0
    double normalized_vel = (max_vel - min_vel > 1e-3) ? ((velocity - min_vel) / (max_vel - min_vel)) : 0.0;

    // HSV Hue: 280° (violet) -> 0° (red), skipping blue at 240°
    double hue = 280.0 - (normalized_vel * 280.0);
    double saturation = 1.0; 
    double value = 1.0;      

    // Convert HSV to RGB
    int i = static_cast<int>(hue / 60.0) % 6;
    double f = (hue / 60.0) - i;
    double p = value * (1.0 - saturation);
    double q = value * (1.0 - f * saturation);
    double t = value * (1.0 - (1.0 - f) * saturation);

    switch(i) {
        case 0: r = value; g = t; b = p; break; 
        case 1: r = q; g = value; b = p; break; 
        case 2: r = p; g = value; b = t; break; 
        case 3: r = p; g = q; b = value; break; 
        case 4: r = t; g = p; b = value; break; 
        case 5: r = value; g = p; b = q; break; 
        default: r = g = b = 1.0; break; 
    }
}


// Constructor
RadarVizNode::RadarVizNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
    loadParams(); 

    radar_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(radar_viz_topic_, 1);

    if (use_radar_) {
        // Subscribe to both radar topics
        radar_sub_1_ = nh_.subscribe<sensor_msgs::PointCloud2>(radar_topic_1_, 1, &RadarVizNode::radar1Cb, this);
        radar_sub_2_ = nh_.subscribe<sensor_msgs::PointCloud2>(radar_topic_2_, 1, &RadarVizNode::radar2Cb, this);

        ROS_INFO("Radar Velocity Visualization Node started.");
        ROS_INFO("Subscribing to Radar 1: %s (Frame: %s)", radar_topic_1_.c_str(), radar_frame_1_.c_str());
        ROS_INFO("Subscribing to Radar 2: %s (Frame: %s)", radar_topic_2_.c_str(), radar_frame_2_.c_str());
    } else {
        ROS_WARN("Radar visualization is disabled by param 'use_radar'.");
    }
}

// Loads parameters from the parameter server
void RadarVizNode::loadParams()
{
    pnh_.param("use_radar", use_radar_, true);
    
    pnh_.param("radar_topic_1", radar_topic_1_, std::string("/ti_mmwave/radar_scan_pcl_0"));
    pnh_.param("radar_topic_2", radar_topic_2_, std::string("/ti_mmwave/radar_scan_pcl_1"));
    pnh_.param("radar_frame_1", radar_frame_1_, std::string("radar_1"));
    pnh_.param("radar_frame_2", radar_frame_2_, std::string("radar_2"));

    pnh_.param("radar_viz_topic", radar_viz_topic_, std::string("/dogm/radar_velocity_viz")); 
    pnh_.param("base_frame", base_frame_, std::string("base_link")); 
    pnh_.param("radar_viz_lifetime", radar_viz_lifetime_, 0.2);
    pnh_.param("radar_viz_color_max_vel", radar_viz_color_max_vel_, 1.5); 
    pnh_.param("viz_buffer_size", viz_buffer_size_, 5);
}

// Callback for Radar 1
void RadarVizNode::radar1Cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    processRadar(msg, radar_frame_1_);
}

// Callback for Radar 2
void RadarVizNode::radar2Cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    processRadar(msg, radar_frame_2_);
}

// Common processing function
void RadarVizNode::processRadar(const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& sensor_frame)
{
    if (msg->data.empty()) return;

    pcl::PointCloud<mmWaveCloudType>::Ptr cloud_transformed(new pcl::PointCloud<mmWaveCloudType>());
    pcl::PointCloud<mmWaveCloudType>::Ptr cloud_raw(new pcl::PointCloud<mmWaveCloudType>());
    
    pcl::fromROSMsg(*msg, *cloud_raw);

    // Transform radar cloud to base_frame using TF
    try {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_.lookupTransform(base_frame_, sensor_frame, ros::Time(0), ros::Duration(0.1));

        // Use Eigen Matrix for PCL custom point type transformation
        Eigen::Matrix4f mat;
        pcl_ros::transformAsMatrix(transform_stamped.transform, mat);

        pcl::transformPointCloud(*cloud_raw, *cloud_transformed, mat);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "[RadarViz] TF Exception: %s. Skipping this scan.", ex.what());
        return;
    }

    // Buffer management: keep recent N scans
    cloud_buffer_.push_back(cloud_transformed);

    while (cloud_buffer_.size() > static_cast<size_t>(viz_buffer_size_))
    {
        cloud_buffer_.pop_front();
    }

    // Visualization: publish colored point markers
    if (cloud_buffer_.empty()) return;

    visualization_msgs::Marker points_viz;
    points_viz.header.stamp = ros::Time::now();
    points_viz.header.frame_id = base_frame_;
    points_viz.ns = "radar_velocity_status";
    points_viz.id = 0;
    points_viz.type = visualization_msgs::Marker::POINTS;
    points_viz.action = visualization_msgs::Marker::ADD;
    points_viz.pose.orientation.w = 1.0; 
    points_viz.scale.x = 0.04;
    points_viz.scale.y = 0.04;
    points_viz.lifetime = ros::Duration(radar_viz_lifetime_); 

    for (const auto& cloud : cloud_buffer_)
    {
        for (const auto& pt : cloud->points) {
            geometry_msgs::Point p;
            p.x = pt.x; 
            p.y = pt.y; 
            p.z = pt.z;
            points_viz.points.push_back(p);

            // Color based on velocity magnitude
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            double r, g, b;
            interpolateColor(pt.velocity, r, g, b); 
            color.r = r; color.g = g; color.b = b;
            points_viz.colors.push_back(color);
        }
    }
    
    radar_viz_pub_.publish(points_viz);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_velocity_viz_node"); 
    RadarVizNode node;                                 
    ros::spin();                                       
    return 0;
}