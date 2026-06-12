
#include <configuration.h>

RoadmapConfig::RoadmapConfig()
{

  success_ = false;
}

RoadmapConfig::~RoadmapConfig()
{
}

// read predicitve configuration paramter from paramter server
bool RoadmapConfig::initialize() //const std::string& node_handle_name
{
  ros::NodeHandle nh;

  retrieveParameter(nh, "roadmap/debug_output", debug_output_, true);
  retrieveParameter(nh, "roadmap/map_package_name", map_package_name_);
  retrieveParameter(nh, "roadmap/map_file_name", map_file_name_);
  retrieveParameter(nh, "roadmap/update_frequency", update_frequency_, 10.);

  retrieveParameter(nh, "roadmap/spline/fit_clothoid", fit_clothoid_);
  retrieveParameter(nh, "roadmap/spline/spline_sample_distance", spline_sample_distance_);
  retrieveParameter(nh, "roadmap/spline/minimum_waypoint_distance", minimum_waypoint_distance_);
  retrieveParameter(nh, "roadmap/spline/clothoid_point_per_xm", clothoid_point_per_xm_);

  retrieveParameter(nh, "roadmap/scale", scale_);

  retrieveParameter(nh, "roadmap/external_waypoint_topic", external_waypoint_topic_, std::string());

  success_ = true;

  if (debug_output_)
    ROS_WARN("[Roadmap]: Configuration initialized");

  return true;
}
