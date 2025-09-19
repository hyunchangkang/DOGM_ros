# DOGM ROS Package - Local Testing Guide

## Quick Setup for Local Testing

This package has been optimized for local testing with fake laser data.

### 1. Build the package
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Make scripts executable
```bash
chmod +x ~/catkin_ws/src/dogm_ros/scripts/*.py
```

### 3. Run the test
```bash
# Launch everything (DOGM + fake data + RViz)
roslaunch dogm_ros test_dogm.launch

# Or without RViz
roslaunch dogm_ros test_dogm.launch rviz:=false
```

### 4. Monitor performance
```bash
# Check update rates
rostopic hz /dogm/grid
rostopic hz /dogm/velocity_markers
rostopic hz /scan

# Check TF tree
rosrun tf view_frames
evince frames.pdf

# Monitor CPU usage
htop
```

## What to Expect

- **OccupancyGrid** (`/dogm/grid`): Now shows continuous probability values (0-100) instead of binary occupied/free
- **Velocity Markers** (`/dogm/velocity_markers`): Dynamic obstacles should show colored arrows
- **Fake Scan**: Creates moving obstacles to test dynamic detection
- **Performance**: Reduced to 5000 particles and 5Hz for better responsiveness

## Troubleshooting

1. **No map updates**: Check TF tree and scan topic
2. **Slow performance**: Further reduce `num_particles` in `config/params.yaml`
3. **No dynamic detection**: Adjust `mahalanobis_dist_thresh` and `birth_prob`

## Key Changes Made

1. **Performance**: Reduced particles from 200k to 5k, update rate from 20Hz to 5Hz
2. **Visualization**: OccupancyGrid now shows continuous probabilities
3. **Fake Data**: Enhanced laser scan with moving obstacles
4. **TF**: Improved transform lookup for better reliability
5. **Grid Size**: Reduced from 50m to 20m for faster processing

## File Structure
```
dogm_ros/
├── src/               # Main C++ source files
│   ├── dogm_node.cpp         # Main ROS node
│   ├── dynamic_grid_map.cpp  # Grid map implementation
│   ├── dynamic_grid_map.h    # Grid map header
│   ├── particle_filter.cpp  # Particle filter
│   ├── particle_filter.h    # Particle filter header
│   └── structures.h          # Data structures
├── scripts/           # Python test scripts
│   ├── fake_scan.py          # Fake laser publisher
│   └── static_tf.py          # Static TF publisher
├── launch/           # Launch files
│   ├── dogm.launch          # Original launch
│   └── test_dogm.launch     # Test launch with fake data
├── config/           # Configuration
│   └── params.yaml          # Parameters (optimized)
└── rviz/            # RViz configuration
    └── dogm.rviz           # Visualization setup
```
