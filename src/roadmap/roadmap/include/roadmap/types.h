#ifndef __TYPES_H__
#define __TYPES_H__

#include "roadmap_msgs/RoadPolylineArray.h"
#include "roadmap_msgs/RoadPolyline.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include <ros_tools/convertions.h>
#include <ros_tools/spline.h>

// Radius of Earth
#define GLOBE_RADIUS 6371.0e3

/**
 * @brief Struct describing a waypoint node (x, y, theta)
 */
struct Node
{

    double x, y, theta;

    Node() {}

    Node(double _x, double _y, double _theta)
        : x(_x), y(_y), theta(_theta)
    {
    }

    // Is skewed!
    Node(double _lat, double _long, double base_latitude, double base_longitude, double _theta)
    {
        x = GLOBE_RADIUS * (_long - base_longitude) * std::cos(0.);
        y = GLOBE_RADIUS * (_lat - base_latitude);
        theta = _theta;
    }
};

/**
 * @brief Struct describing a lane
 *
 */
struct Lane
{
    int id, type;            /** ID and the type of lane as int */
    std::vector<Node> nodes; /** Nodes that construct this lane */

    // Variables after spline fitting
    bool spline_fit;               /** Has a spline been fitted over this lane? */
    double length;                 /** Length of the lane */
    tk::spline spline_x, spline_y; /** Cubic spline objects */

    /**
     * @brief Construct a new Lane object
     *
     * @param _nodes nodes of the centerlane
     * @param offset offset with respect to the centerlane for this lane
     * @param _type type of lane (see road_msgs/RoadPolyline)
     * @param _id current ID to add this lane on (and is increased after usage)
     */
    Lane(const std::vector<Node> &_nodes, double offset, int _type, int &_id)
        : type(_type)
    {
        spline_fit = false;
        id = _id;
        _id++;

        // Offset the nodes
        nodes.reserve(_nodes.size());
        for (auto &node : _nodes)
        {
            // Create a node at the offset from the center node
            nodes.emplace_back(
                node.x + offset * std::cos(node.theta - M_PI_2),
                node.y + offset * std::sin(node.theta - M_PI_2),
                node.theta);
        }
    }

    void Reverse()
    {
        std::reverse(nodes.begin(), nodes.end());

        for (size_t n = 0; n < nodes.size(); n++)
            nodes[n].theta += M_PI; // Reverse the direction
    }

    /**
     * @brief Aassign a spline to this lane
     */
    void AssignSpline(const tk::spline &_spline_x, const tk::spline &_spline_y)
    {
        spline_x = _spline_x;
        spline_y = _spline_y;
        spline_fit = true;
    }
};

/**
 * @brief A way struct that may contain multiple lanes (e.g., road, sidewalk, etc.)
 *
 */
struct Way
{
    std::vector<Lane> lanes;          /** Lanes on this way */
    std::vector<Node> nodes;          /** Nodes defining the center lane */
    double plus_offset, minus_offset; /** Current aggregatied offset in lane construction */

    Way()
    {
        plus_offset = 0.;
        minus_offset = 0.;
    }

    void Reverse()
    {

        std::reverse(nodes.begin(), nodes.end());
        for (size_t n = 0; n < nodes.size(); n++)
            nodes[n].theta += M_PI;

        for (size_t l = 0; l < lanes.size(); l++)
            lanes[l].Reverse();
    }

    /**
     * @brief Add a node to the center line of this way
     *
     * @param node the node to add
     */
    void AddNode(const Node &node)
    {
        nodes.emplace_back(node);
    }

    /**
     * @brief Add a road by copying the current data and offsetting it.
     * Can only be called before the spline is fit!
     *
     * @param _type type of lane to add
     * @param width width of the lane
     * @param two_way is it a two way lane?
     * @param id ID to start at
     */
    void AddLane(const std::string &_type, double width, bool two_way, int &id)
    {
        // The first lane we add needs to offset at the end, the rest at the start
        if (plus_offset != 0.)
        {
            plus_offset += width / 2.;

            if (two_way)
                minus_offset -= width / 2.;
        }

        if (!_type.compare("road"))
        {
            if (two_way)
            {
                lanes.emplace_back(nodes, plus_offset, roadmap_msgs::RoadPolyline::LANECENTER_FREEWAY, id); // Main road is ON the reference
                lanes.emplace_back(nodes, minus_offset - width, roadmap_msgs::RoadPolyline::LANECENTER_FREEWAY, id);

                lanes.emplace_back(nodes, plus_offset - width / 2., roadmap_msgs::RoadPolyline::ROADLINE_BROKENSINGLEWHITE, id);

                lanes.emplace_back(nodes, plus_offset + width / 2., roadmap_msgs::RoadPolyline::ROADEDGEBOUNDARY, id);
                lanes.emplace_back(nodes, minus_offset - 3. / 2. * width, roadmap_msgs::RoadPolyline::ROADEDGEBOUNDARY, id);
            }
            else
            {
                lanes.emplace_back(nodes, plus_offset, roadmap_msgs::RoadPolyline::LANECENTER_FREEWAY, id);

                lanes.emplace_back(nodes, minus_offset - width / 2., roadmap_msgs::RoadPolyline::ROADEDGEBOUNDARY, id);
                lanes.emplace_back(nodes, plus_offset + width / 2., roadmap_msgs::RoadPolyline::ROADEDGEBOUNDARY, id);
            }
        }
        else if (!_type.compare("crosswalk"))
        {
            lanes.emplace_back(nodes, plus_offset, roadmap_msgs::RoadPolyline::CROSSWALK, id);
        }
        else if (!_type.compare("sidewalk")) // In some cases, we want a new way object, offset from the current one
        {
            lanes.emplace_back(nodes, plus_offset, roadmap_msgs::RoadPolyline::LANECENTER_SURFACESTREET, id);
            if (two_way)
                lanes.emplace_back(nodes, minus_offset, roadmap_msgs::RoadPolyline::LANECENTER_SURFACESTREET, id);
        }

        // For the first added lane
        if (plus_offset == 0.)
        {
            if (two_way)
            {
                plus_offset += width / 2.;
                minus_offset -= 3. * width / 2.;
            }
            else
            {
                plus_offset += width / 2.;
                minus_offset -= width / 2.;
            }
        }
    }
};

/**
 * @brief Struct to define the map.
 */
struct Map
{
    std::vector<Way> ways;

    Map()
    {
    }

    /**  @brief Clear the map  */
    void Clear() { ways.clear(); }

    /** @brief Reverse the direction of the reference path */
    void Reverse()
    {
        for (size_t w = 0; w < ways.size(); w++)
        {
            // std::cout << ways[w].nodes.back().y << std::endl;
            ways[w].Reverse();
            // std::cout << ways[w].nodes.back().y << std::endl;
        }
    }

    /**
     * @brief Load the data of this map object into a ros message.
     *
     * @param msg the output message
     */
    void ToMsg(roadmap_msgs::RoadPolylineArray &msg)
    {
        msg.road_polylines.reserve(ways.size());

        // Load the ways one by one
        for (Way &way : ways)
        {
            for (Lane &lane : way.lanes)
            {
                roadmap_msgs::RoadPolyline line_msg;
                line_msg.type = lane.type;
                line_msg.id = lane.id;

                geometry_msgs::Point point;
                for (Node &node : lane.nodes)
                {
                    point.x = node.x;
                    point.y = node.y;
                    line_msg.coords.push_back(point);
                }

                msg.road_polylines.push_back(line_msg);
            }
        }

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
    }

    /**
     * @brief Load the data of the reference path in this map into a ros message.
     *
     * @param msg the output message
     */
    void ToMsg(nav_msgs::Path &msg)
    {
        // Save the road as reference trajectory

        // Load the ways one by one
        for (Way &way : ways)
        {
            for (Lane &lane : way.lanes)
            {
                // Look for the reference trajectory
                if (lane.type == roadmap_msgs::RoadPolyline::LANECENTER_FREEWAY)
                {
                    msg.poses.reserve(lane.nodes.size());

                    // geometry_msgs::Pose pose_msg;
                    // line_msg.type = lane.type;
                    // line_msg.id = lane.id;

                    // geometry_msgs::Point point;
                    geometry_msgs::PoseStamped pose_msg;
                    pose_msg.header.stamp = ros::Time::now();
                    pose_msg.header.frame_id = "map";

                    for (Node &node : lane.nodes)
                    {
                        pose_msg.pose.position.x = node.x;
                        pose_msg.pose.position.y = node.y;

                        pose_msg.pose.orientation = RosTools::angleToQuaternion(node.theta);

                        msg.poses.push_back(pose_msg);
                    }
                    msg.header.stamp = ros::Time::now();
                    msg.header.frame_id = "map";
                    return;
                }
            }
        }
    }
};

#endif // __TYPES_H__