#include "spline_converter.h"

#include <ros_tools/visuals.h>

void SplineConverter::Initialize(ros::NodeHandle &nh, RoadmapConfig *config)
{
    (void)nh;

    config_ = config;

    ROADMAP_WARN("Initialized Spline Converter");
}

Map &SplineConverter::ConvertMap(Map &map)
{
    if (map.ways.size() == 0)
    {
        ROADMAP_WARN("Tried to convert an empty map (returning)")
        return converted_map_; // Needs to also give an error
    }

    ROADMAP_INFO_STREAM("Fitting splines over " << map.ways.size() << " ways");
    // VisualizeMap(map);

    // First fit splines on all defined ways
    converted_map_ = map;
    for (Way &way : converted_map_.ways)
    {
        for (auto &lane : way.lanes)
            FitSplineOnLane(lane);
    }

    // Then define additional ways by translating (i.e., for a sidewalk)

    // VisualizeMap(map, true);

    if (config_->debug_output_)
    {
        int total_waypoints = 0;
        for (auto &way : converted_map_.ways)
        {
            for (auto &lane : way.lanes)
            {
                for (auto &node : lane.nodes)
                {
                    (void)node;
                    total_waypoints++;
                }
            }
        }
        ROADMAP_INFO_STREAM("Done converting map (" << total_waypoints << " waypoints)");
    }

    return converted_map_;
}

void SplineConverter::VisualizeMap()
{
    ROADMAP_INFO("Visualizing the map");
    bool plot_arrows = false;
    bool plot_cubes = true;

    auto &output_map_visuals = VISUALS.getPublisher("output_map");

    auto &arrow_visuals = VISUALS.getPublisher("arrows");
    RosTools::ROSPointMarker &arrow = arrow_visuals.getNewPointMarker("ARROW"); // Note: is expensive to DRAW, only show when debugging.

    double scale = 0.2 * config_->scale_;

    for (auto &way : converted_map_.ways)
    {
        for (size_t i = 0; i < way.lanes.size(); i++) // For all lanes in this way
        {
            const Lane &lane = way.lanes[i];

            if (lane.type == roadmap_msgs::RoadPolyline::LANECENTER_FREEWAY)
                continue;

            RosTools::ROSMultiplePointMarker &cube = output_map_visuals.getNewMultiplePointMarker("CUBE"); // Batch rendering (same color and scale)

            if (plot_cubes)
                cube.setScale(scale, scale, scale);

            const Node *prev_node = nullptr;
            for (const Node &node : lane.nodes)
            {
                if (plot_cubes)
                    cube.setColorInt(lane.type, 20, RosTools::Colormap::VIRIDIS);

                if (prev_node && plot_arrows)
                {

                    // For fitted points we plot arrows to indicate the direction
                    arrow.setScale(3 * scale, scale);

                    arrow.setColor((double)lane.type / (double)20);

                    double orientation = std::atan2(node.y - prev_node->y, node.x - prev_node->x);
                    arrow.setOrientation(orientation);
                    arrow.addPointMarker(Eigen::Vector3d(
                        node.x,
                        node.y,
                        0.1));
                }
                if (plot_cubes)
                {
                    cube.addPointMarker(Eigen::Vector3d(
                        node.x,
                        node.y,
                        0.1));
                }
                prev_node = &node;
            }

            VisualizeLaneSpline(lane);

            if (plot_cubes)
                cube.finishPoints();
        }
    }

    // For multiple point markers we need to enforce the draw call
    output_map_visuals.publish();
    if (plot_arrows)
        arrow_visuals.publish();
}

void SplineConverter::VisualizeInputData(Map &map)
{
    ROADMAP_INFO("Visualizing input data");
    auto &input_map_visuals = VISUALS.getPublisher("input_map");
    RosTools::ROSPointMarker &unique_cubes = input_map_visuals.getNewPointMarker("CUBE"); // Unique per color

    double scale;

    for (auto &way : map.ways)
    {
        for (size_t i = 0; i < way.lanes.size(); i++) // For all lanes in this way
        {
            const Lane &lane = way.lanes[i];

            RosTools::ROSMultiplePointMarker &cube = input_map_visuals.getNewMultiplePointMarker("CUBE"); // Batch rendering (same color and scale)

            bool is_first_node = true;
            for (const Node &node : lane.nodes)
            {
                scale = 0.75 * config_->scale_;

                if (is_first_node) // Color the first node black (with the unique cubes)
                {
                    unique_cubes.setColor(0, 0, 0);

                    scale *= 1.5;
                    unique_cubes.setScale(scale, scale, scale);
                    unique_cubes.addPointMarker(Eigen::Vector3d(
                        node.x,
                        node.y,
                        0.1));
                    is_first_node = false;
                    continue;
                }
                else
                {
                    cube.setColor((double)lane.type / (double)20);
                    cube.setScale(scale, scale, scale);

                    cube.addPointMarker(Eigen::Vector3d(
                        node.x,
                        node.y,
                        0.1));
                }
            }

            cube.finishPoints();
        }
    }

    // For multiple point markers we need to enforce the draw call

    input_map_visuals.publish();
}

void SplineConverter::VisualizeLaneSpline(const Lane &lane)
{
    auto &output_map_visuals = VISUALS.getPublisher("output_map");

    RosTools::ROSLine &line = output_map_visuals.getNewLine();
    line.setColorInt(lane.type, 20, RosTools::Colormap::VIRIDIS); //(double)lane.type / (double)20);
                                                                  // line.setColor(0.5, 0.0, 0.0);

    line.setScale(0.1);
    line.setOrientation(0.0);

    geometry_msgs::Point cur, prev;

    for (size_t i = 0; i < lane.nodes.size(); i++)
    {
        // Do not plot the middle of the roads (just the edges)
        if (lane.type == roadmap_msgs::RoadPolyline::LANECENTER_FREEWAY)
            continue;

        const Node &node = lane.nodes[i];
        cur.x = node.x;
        cur.y = node.y;

        if (i > 0)
        {
            line.addLine(prev, cur);
        }

        prev = cur;
    }
}

void SplineConverter::FitSplineOnLane(Lane &lane)
{

    // Construct x, y, theta vectors
    std::vector<Waypoint> waypoints;
    waypoints.reserve(lane.nodes.size());

    for (Node &node : lane.nodes)
    {
        // Retrieve waypoints
        waypoints.emplace_back(node.x,
                               node.y,
                               node.theta);
    }

    // Fit a spline over the waypoints
    std::vector<Waypoint> spline_waypoints;
    FitSplineOnWaypoints(waypoints, spline_waypoints, lane);

    // Modify the way object
    lane.nodes.clear();
    for (Waypoint &waypoint : spline_waypoints)
        lane.nodes.push_back(Node(waypoint.x, waypoint.y, waypoint.theta));
}

void SplineConverter::FitSplineOnWaypoints(const std::vector<Waypoint> &waypoints, std::vector<Waypoint> &waypoints_out, Lane &lane)
{
    // First we obtain a set of waypoints (either directly or by fitting a clothoid)
    std::vector<double> x, y, s;
    if (config_->fit_clothoid_)
        FitClothoid(waypoints, x, y, s);
    else
        ConvertWaypointsToVectors(waypoints, x, y, s);

    // Define the splines
    tk::spline ref_path_x, ref_path_y;
    ref_path_x.set_points(s, x);
    ref_path_y.set_points(s, y);
    lane.length = s.back();
    lane.AssignSpline(ref_path_x, ref_path_y); // Assign this spline to the way object

    // Then we fit cubic splines on these waypoints
    FitCubicSpline(lane, waypoints_out);
}

void SplineConverter::FitClothoid(const std::vector<Waypoint> &waypoints, std::vector<double> &x, std::vector<double> &y, std::vector<double> &s)
{
    ROADMAP_INFO("Reference Path: Generating path with clothoid interpolation...");

    double length = 0;
    double k, dk, L;
    int n_clothoid = 0;
    double last_s = 0.;

    std::vector<double> X, Y;
    int max_clothoid_size = std::ceil(500. / config_->clothoid_point_per_xm_);
    X.reserve(max_clothoid_size);
    Y.reserve(max_clothoid_size);

    s.push_back(0);

    // For each set of waypoints in the trajectory
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
        // Build a clothoid and get the X and Y waypoints
        Clothoid::buildClothoid(waypoints[i].x, waypoints[i].y, waypoints[i].theta,
                                waypoints[i + 1].x, waypoints[i + 1].y, waypoints[i + 1].theta,
                                k, dk, L);

        n_clothoid = std::max((int)std::ceil(L / config_->clothoid_point_per_xm_), 2);
        X.resize(n_clothoid);
        Y.resize(n_clothoid);
        Clothoid::pointsOnClothoid(waypoints[i].x, waypoints[i].y, waypoints[i].theta, k, dk, L, n_clothoid, X, Y);

        if (i == 0)
        {
            // For the first one insert the full clothoid
            x.insert(x.end(), X.begin(), X.end()); // Only take the first n_clothoid points!
            y.insert(y.end(), Y.begin(), Y.end());
        }
        else
        {
            // Afterwards, insert all except for the duplicate initial point
            X.erase(X.begin() + 0);
            Y.erase(Y.begin() + 0);
            x.insert(x.end(), X.begin(), X.end());
            y.insert(y.end(), Y.begin(), Y.end());
        }

        length += L;

        // For each point in the clothoid
        for (int j = 1; j < n_clothoid; j++)
        {
            s.push_back(last_s + L / (n_clothoid - 1) * j);
        }
        last_s = s.back();
    }
}

void SplineConverter::ConvertWaypointsToVectors(const std::vector<Waypoint> &waypoints, std::vector<double> &x, std::vector<double> &y, std::vector<double> &s)
{
    ROADMAP_INFO("Generating spline without clothoid interpolation");
    assert(waypoints.size() > 1);

    double length = 0.;
    double added_length = 0.;
    double L;

    s.push_back(0);
    x.push_back(waypoints[0].x);
    y.push_back(waypoints[0].y);

    for (size_t i = 1; i < waypoints.size(); i++)
    {
        // Compute the distance between the current waypoint and the last used waypoint
        L = std::sqrt(std::pow(waypoints[i].x - waypoints[i - 1].x, 2.) + std::pow(waypoints[i].y - waypoints[i - 1].y, 2.));

        added_length += L;
        length += L;

        // If the distance is sufficiently large, add the waypoint
        if (added_length > config_->minimum_waypoint_distance_ || i == waypoints.size() - 1)
        {
            added_length = 0.;
            x.push_back(waypoints[i].x);
            y.push_back(waypoints[i].y);
            s.push_back(length);
        }
    }

    // In the case of just 2 waypoints, we need to add an additional waypoint to have a spline representation
    if (x.size() == 2)
    {
        // Copy into the 2nd waypoint
        x.push_back(x.back());
        y.push_back(y.back());
        s.push_back(s.back());

        // Average for the middle waypoint
        x[1] = (x[0] + x[2]) / 2.;
        y[1] = (y[0] + y[2]) / 2.;
        s[1] = (s[0] + s[2]) / 2.;
    }
}

void SplineConverter::FitCubicSpline(Lane &lane, std::vector<Waypoint> &waypoints_out)
{
    assert(lane.spline_fit);
    double length = lane.length;

    double spline_sample_dist = std::min(config_->spline_sample_distance_, length); // length = 70
    int n_spline_pts = ceil(length / spline_sample_dist);                           // 70  / 10 = 7
    waypoints_out.resize(n_spline_pts);
    double s_cur = 0;
    for (int i = 0; i < n_spline_pts; i++)
    {
        waypoints_out[i].s = s_cur;
        waypoints_out[i].x = lane.spline_x(s_cur);
        waypoints_out[i].y = lane.spline_y(s_cur);
        waypoints_out[i].theta = std::atan2(lane.spline_y.deriv(1, s_cur), lane.spline_x.deriv(1, s_cur));

        s_cur += spline_sample_dist;
    }

    // Check if we are not at our destination yet
    /** @note: Bugged */
    // double error = std::sqrt(std::pow(waypoints_out.back().x - lane.spline_x.m_y_.back(), 2) + std::pow(waypoints_out.back().y - lane.spline_y.m_y_.back(), 2));
    // if (error > 0.01)
    // {
    //     // Add a final waypoint
    //     LOG_VALUE("x", lane.spline_x.m_x_.back());
    //     LOG_VALUE("y", lane.spline_y.m_y_.back());
    //     LOG_VALUE("theta", waypoints_out.back().theta);
    //     waypoints_out.emplace_back(
    //         lane.spline_x.m_x_.back(),
    //         lane.spline_y.m_y_.back(),
    //         waypoints_out.back().theta);
    // }

    ROADMAP_INFO("Cubic Spline Fitted");
}