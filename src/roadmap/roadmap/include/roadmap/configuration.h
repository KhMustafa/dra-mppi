#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "roadmap/helpers.h"

#include <ros/ros.h>
#include <string>
#include <vector>

class RoadmapConfig
{

  /**
   * @brief Class for retrieving configuration parameters
   *
   */

public:
  RoadmapConfig();
  ~RoadmapConfig();

  /**
   * @brief intialize:  check parameters on parameter server and read from there
   * @param node_handle_name: node handler initialize from name, as parameter set inside that name
   * @return true all parameter initialize successfully else false
   */
  bool initialize();

  // High-level Parameters
  bool debug_output_;

  std::string map_package_name_;
  std::string map_file_name_;

  double update_frequency_;

  // Spline settings
  bool fit_clothoid_;

  double spline_sample_distance_;
  double minimum_waypoint_distance_;
  double clothoid_point_per_xm_;

  // Visual settings
  double scale_;

  // Topics
  std::string external_waypoint_topic_;

  bool success_;

public:
  /* Retrieve paramater, if it doesn't exist return false */
  template <class T>
  static bool retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value)
  {

    if (!nh.getParam(name, value))
    {
      ROS_WARN_STREAM(" Parameter " << name << " not set on node " << ros::this_node::getName().c_str());
      return false;
    }
    else
    {
      return true;
    }
  }

  /* Retrieve parameter, if it doesn't exist use the default */
  template <class T>
  static void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value, const T &default_value)
  {

    if (!retrieveParameter(nh, name, value))
    {
      ROS_WARN_STREAM(" Setting " << name << " to default value: " << default_value);
      value = default_value;
    }
  }
};

#endif
