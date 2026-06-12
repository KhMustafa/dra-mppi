#ifndef SPLINE_CONVERTER_H
#define SPLINE_CONVERTER_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <Eigen/Eigen>

// spline libraries
#include <spline.h>
#include <Clothoid.h>

#include "configuration.h"
#include "types.h"
#include "helpers.h"

// Whens earching for the closest point on the path, this variable indicates the distance that the algorithm searches behind the current spline point.
#define MAX_STEP_BACK_TOLERANCE 0.1f

/** Struct for a waypoint with distance */
struct Waypoint
{
    double x, y, theta, s;

    Waypoint()
    {
    }

    Waypoint(double _x, double _y, double _theta, double _s)
        : x(_x), y(_y), theta(_theta), s(_s)
    {
    }
    Waypoint(double _x, double _y, double _theta)
        : x(_x), y(_y), theta(_theta), s(-1.)
    {
    }
};

class SplineConverter
{

    /**
     * @brief Converts waypoints into spline representations
     */

public:
    SplineConverter(){};

    void Initialize(ros::NodeHandle &nh, RoadmapConfig *config);

public:
    Map converted_map_;

    /** @brief Convert the waypoints of a map object into spline representations */
    Map &ConvertMap(Map &map);

    /**
     * @brief Visualize a map
     *
     * @param map the map object
     * @param converted_map is this an output map? Will change the topic and style of visualizations.
     */
    void VisualizeInputData(Map &map);

    void VisualizeMap();

private:
    RoadmapConfig *config_{nullptr}; /** parameters */

    /** Two classes for visualization of the map */
    // std::unique_ptr<RosTools::ROSMarkerPublisher> input_map_markers_;
    // std::unique_ptr<RosTools::ROSMarkerPublisher> output_map_markers_;
    // std::unique_ptr<RosTools::ROSMarkerPublisher> arrow_markers_;

    /**
     * @brief Fit a spline to a particular lane
     * @see types.h
     * @param lane A lane (road, sidewalk, etc. )
     */
    void FitSplineOnLane(Lane &lane);

    void FitSplineOnWaypoints(const std::vector<Waypoint> &waypoints, std::vector<Waypoint> &waypoints_out, Lane &lane);

    /**
     * @brief Convert a set of input waypoints to the format for fitting the cubic spline
     *
     * @param waypoints the input waypoints
     * @param x output x vector
     * @param y output y vector
     * @param s output s vector
     */
    void ConvertWaypointsToVectors(const std::vector<Waypoint> &waypoints, std::vector<double> &x, std::vector<double> &y, std::vector<double> &s);

    /**
     * @brief Fit a Clothoid over the given waypoints
     *
     * @param waypoints the input waypoints
     * @param x output x vector
     * @param y output y vector
     * @param s output s vector
     */
    void FitClothoid(const std::vector<Waypoint> &waypoints, std::vector<double> &x, std::vector<double> &y, std::vector<double> &s);

    /**
     * @brief Fit a cubic spline over a set of waypoints (x, y, s)
     *
     * @param lane The lane object that contains the spline
     * @param waypoints_out waypoints sampled over the fitted spline
     */
    void FitCubicSpline(Lane &lane, std::vector<Waypoint> &waypoints_out);

    /**
     * @brief Visualize the spline fitted over a lane
     *
     * @param lane the lane to visualize
     */
    void VisualizeLaneSpline(const Lane &lane);
};
#endif