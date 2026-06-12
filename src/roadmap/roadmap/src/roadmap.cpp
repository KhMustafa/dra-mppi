#include "roadmap.h"

#include <ros_tools/visuals.h>

Roadmap::Roadmap()
{
    // Initialize the configuration
    config_.reset(new RoadmapConfig());
    config_->initialize();

    reader_.reset(new Reader(config_.get()));

    VISUALS.init(&nh_);

    // Subscribers
    waypoints_sub_ = nh_.subscribe(config_->external_waypoint_topic_, 1, &Roadmap::WaypointCallback, this); // Subscriber for waypoints (forwarded to the reader)
    reset_sub_ = nh_.subscribe("/lmpcc/reset_environment", 1, &Roadmap::ResetCallback, this);
    reverse_sub_ = nh_.subscribe("/roadmap/reverse", 1, &Roadmap::ReverseCallback, this);

    // Debug: listens to rviz point for translating the map
    offset_sub_ = nh_.subscribe("roadmap/offset", 1, &Roadmap::OffsetCallback, this); // Subscriber for waypoints (forwarded to the reader)

    // Publishers
    map_pub_ = nh_.advertise<roadmap_msgs::RoadPolylineArray>("roadmap/road_polylines", 1);
    reference_pub_ = nh_.advertise<nav_msgs::Path>("roadmap/reference", 1);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/roadmap/goal", 1);

    // Then convert the read waypoints to splines
    spline_converter_.Initialize(nh_, config_.get());

    ReadFromFile();

    is_reversed_ = false;

    timer_ = nh_.createTimer(ros::Duration(1.0 / config_->update_frequency_), &Roadmap::Poll, this);
    ROADMAP_WARN("Initialization completed");
}

// Have two maps (input / output)
void Roadmap::ReadFromFile()
{
    ROADMAP_INFO("Reading map from file");

    reader_->Read();
    ConvertMap();
}

void Roadmap::WaypointCallback(const nav_msgs::Path &msg)
{
    ROADMAP_INFO("Received waypoints");

    reader_->WaypointCallback(msg);
    ConvertMap();
    reference_pub_.publish(ref_msg_); // publish
}

void Roadmap::OffsetCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    reader_->OffsetCallback(msg);
    ReadFromFile();
}

void Roadmap::ReverseCallback(const std_msgs::Empty &msg)
{
    (void)msg;

    is_reversed_ = !is_reversed_;

    config_.reset(new RoadmapConfig());
    config_->initialize();

    reader_.reset(new Reader(config_.get()));

    // Then convert the read waypoints to splines
    spline_converter_ = SplineConverter();
    spline_converter_.Initialize(nh_, config_.get());

    reader_->Read();

    if (is_reversed_)
        reader_->GetMap().Reverse(); // Reverse the map

    ConvertMap();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = spline_converter_.converted_map_.ways[0].nodes.back().x;
    pose_msg.pose.position.y = spline_converter_.converted_map_.ways[0].nodes.back().y;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    goal_pub_.publish(pose_msg);

    ROADMAP_INFO("Roadmap reverse completed");
}

void Roadmap::ResetCallback(const std_msgs::Empty &msg)
{
    (void)msg;
    /* // Initialize the configuration
     config_.reset(new RoadmapConfig());
     config_->initialize();

     reader_.reset(new Reader(config_.get()));

     // Then convert the read waypoints to splines
     spline_converter_.Initialize(nh_, config_.get());

     ReadFromFile();

     ROADMAP_WARN("Roadmap reset completed");*/
}

void Roadmap::ConvertMap()
{
    spline_converter_.ConvertMap(reader_->GetMap());

    road_msg_.road_polylines.clear();
    ref_msg_.poses.clear();
    reader_->GetMap().ToMsg(road_msg_);               // Load map data into the message
    spline_converter_.converted_map_.ToMsg(ref_msg_); // Load reference path data into the message
}

void Roadmap::Poll(const ros::TimerEvent &event)
{
    (void)event;
    ROADMAP_INFO("====== START LOOP ======");

    runs_++;

    map_pub_.publish(road_msg_);
    reference_pub_.publish(ref_msg_);

    ROADMAP_INFO("Published polylines and reference");

    spline_converter_.VisualizeInputData(reader_->GetMap());
    spline_converter_.VisualizeMap();

    ROADMAP_INFO("======= END LOOP =======");
}
