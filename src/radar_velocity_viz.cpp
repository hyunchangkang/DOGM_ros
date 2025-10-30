#include "dogm_ros/radar_velocity_viz.h" // Include the header file we just created

// Function to interpolate color based on velocity magnitude using a rainbow spectrum
// Violet (low speed) -> Blue -> Green -> Yellow -> Orange -> Red (high speed)
void RadarVizNode::interpolateColor(double velocity, double& r, double& g, double& b)
{
    // Use absolute velocity (speed) for color mapping
    velocity = std::abs(velocity);

    // Define the speed range for the full spectrum
    const double min_vel = 0.0;
    // Use the parameter for the max velocity of the color scale
    const double max_vel = radar_viz_color_max_vel_;
    velocity = std::max(min_vel, std::min(velocity, max_vel)); // Clamp velocity to the range [min_vel, max_vel]

    // Normalize velocity to 0.0 - 1.0 within the range
    double normalized_vel = (max_vel - min_vel > 1e-3) ? ((velocity - min_vel) / (max_vel - min_vel)) : 0.0;

    // Map normalized velocity (0-1) to HSV color space (Hue: 240 (blue/violet) to 0 (red))
    double hue = 240.0 * (1.0 - normalized_vel);
    double saturation = 1.0; // Full saturation
    double value = 1.0;      // Full brightness

    // Convert HSV to RGB (simplified version)
    int i = static_cast<int>(hue / 60.0) % 6;
    double f = (hue / 60.0) - i;
    double p = value * (1.0 - saturation);
    double q = value * (1.0 - f * saturation);
    double t = value * (1.0 - (1.0 - f) * saturation);

    switch(i) {
        case 0: r = value; g = t; b = p; break; // Red dominant
        case 1: r = q; g = value; b = p; break; // Yellow-Green dominant
        case 2: r = p; g = value; b = t; break; // Green dominant
        case 3: r = p; g = q; b = value; break; // Cyan-Blue dominant
        case 4: r = t; g = p; b = value; break; // Blue dominant
        case 5: r = value; g = p; b = q; break; // Magenta-Violet dominant
        default: r = g = b = 1.0; break; // Should not happen
    }
}


// Constructor
RadarVizNode::RadarVizNode() : nh_(), pnh_("~")
{
    loadParams(); // Load parameters first

    // Setup publisher for visualization markers
    radar_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(radar_viz_topic_, 1);

    // Setup subscriber for radar point cloud data
    radar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(radar_topic_, 1, &RadarVizNode::radarCb, this);

    ROS_INFO("Radar Velocity Visualization Node started.");
    ROS_INFO("Subscribing to radar topic: %s", radar_topic_.c_str());
    ROS_INFO("Publishing visualization to: %s", radar_viz_topic_.c_str());
}

// Loads parameters from the parameter server
void RadarVizNode::loadParams()
{
    // Get parameters using the private NodeHandle (~)
    pnh_.param("radar_topic", radar_topic_, std::string("/ti_mmwave/radar_scan_pcl_0")); // Default radar topic
    pnh_.param("radar_viz_topic", radar_viz_topic_, std::string("/dogm/radar_velocity_viz")); // Default viz topic
    pnh_.param("base_frame", base_frame_, std::string("base_link")); // Default frame_id
    pnh_.param("radar_viz_lifetime", radar_viz_lifetime_, 0.5); // Default lifetime in seconds
    pnh_.param("radar_viz_color_max_vel", radar_viz_color_max_vel_, 1.5); // Default max speed for red color
}

// Callback function for incoming radar point clouds
void RadarVizNode::radarCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<mmWaveCloudType>::Ptr current_radar_cloud(new pcl::PointCloud<mmWaveCloudType>());
    pcl::fromROSMsg(*msg, *current_radar_cloud);

    // Exit if no points in the cloud
    if (current_radar_cloud->points.empty()) {
        // Optional: Publish an empty marker to clear previous ones if needed
        // visualization_msgs::Marker clear_marker;
        // clear_marker.header = msg->header; // Use incoming message header
        // clear_marker.header.frame_id = base_frame_; // Ensure frame is correct
        // clear_marker.ns = "radar_velocity_status";
        // clear_marker.id = 0;
        // clear_marker.action = visualization_msgs::Marker::DELETEALL;
        // radar_viz_pub_.publish(clear_marker);
        return;
    }

    // --- Track Min/Max Velocity in this scan (Optional, for logging) ---
    float min_vel = std::numeric_limits<float>::max();
    float max_vel = std::numeric_limits<float>::lowest();
    // --- End Min/Max Tracking ---

    // --- Prepare Visualization Marker (POINTS type) ---
    visualization_msgs::Marker points_viz;
    points_viz.header = msg->header; // Use header from incoming message for timestamp
    points_viz.header.frame_id = base_frame_; // Set the correct frame ID
    points_viz.ns = "radar_velocity_status"; // Namespace for RViz display
    points_viz.id = 0; // Use a consistent ID
    points_viz.type = visualization_msgs::Marker::POINTS;
    points_viz.action = visualization_msgs::Marker::ADD;
    points_viz.pose.orientation.w = 1.0; // No rotation
    points_viz.scale.x = 0.05; // Point size (adjust as needed)
    points_viz.scale.y = 0.05;
    points_viz.lifetime = ros::Duration(radar_viz_lifetime_); // Set lifetime from parameter
    // --- End Marker Preparation ---

    // Iterate through each point in the radar cloud
    for (const auto& pt : current_radar_cloud->points) {
        // Add point position to the marker
        geometry_msgs::Point p;
        p.x = pt.x; p.y = pt.y; p.z = 0.1; // Slightly above ground plane
        points_viz.points.push_back(p);

        // Add color based on velocity using the rainbow mapping
        std_msgs::ColorRGBA color;
        color.a = 1.0; // Fully opaque
        double r, g, b;
        interpolateColor(pt.velocity, r, g, b); // Calculate color
        color.r = r; color.g = g; color.b = b;
        points_viz.colors.push_back(color);

        // --- Update Min/Max Velocity (Optional) ---
        if (pt.velocity < min_vel) min_vel = pt.velocity;
        if (pt.velocity > max_vel) max_vel = pt.velocity;
        // --- End Update Min/Max ---
    }
    // Publish the visualization marker
    radar_viz_pub_.publish(points_viz);

    // --- Print Min/Max Velocity to console (Optional, throttled) ---
    // ROS_INFO_THROTTLE(1.0, "Radar Viz: Min Vel=%.2f, Max Vel=%.2f", min_vel, max_vel);
    // --- End Print Min/Max ---
}

// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_velocity_viz_node"); // Initialize ROS node
    RadarVizNode node;                                 // Create the node object
    ros::spin();                                       // Keep the node running
    return 0;
}