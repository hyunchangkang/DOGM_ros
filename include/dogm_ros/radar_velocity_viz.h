#ifndef RADAR_VELOCITY_VIZ_H
#define RADAR_VELOCITY_VIZ_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <pcl_conversions/pcl_conversions.h> // Include for PCL conversion
#include <limits> // Include for numeric_limits

// Include structure definitions (adjust path if necessary)
#include "dogm_ros/structures.h"

// Class for the Radar Velocity Visualization Node
class RadarVizNode
{
public:
    // Constructor
    RadarVizNode();

private:
    // Callback function for incoming radar point clouds
    void radarCb(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Loads parameters from the parameter server
    void loadParams();

    // Helper function to map velocity to color
    void interpolateColor(double velocity, double& r, double& g, double& b);

    // ROS NodeHandles
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Subscriber and Publisher
    ros::Subscriber radar_sub_;
    ros::Publisher radar_viz_pub_;

    // Parameters
    std::string radar_topic_;         // Topic to subscribe radar data from
    std::string radar_viz_topic_;     // Topic to publish visualization markers to
    std::string base_frame_;          // Frame ID for the markers
    double radar_viz_lifetime_;       // How long markers should persist in RViz (seconds)
    double radar_viz_color_max_vel_; // Speed (m/s) corresponding to the 'reddest' color

};

#endif // RADAR_VELOCITY_VIZ_H