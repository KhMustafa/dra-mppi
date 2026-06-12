
/** @brief: This node bundles obstacles into a single message */

#include <ros/ros.h>

#include <derived_object_msgs/ObjectArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>

std::vector<std::string> _dynamic_topic_list, _static_topic_list;

std::vector<ros::Subscriber> _dynamic_subs, _static_subs;
ros::Publisher _bundled_object_pub;

derived_object_msgs::ObjectArray _bundled_objects;

int _dynamic_obstacles_expected;
int _static_obstacles_expected;
int _obstacles_expected;

double _dynamic_radius;
double _static_radius;
std::vector<int> _static_size_list;

// Count obstacles
int received_obstacles = 0;
std::vector<bool> _received_list;

void odometryToObject(nav_msgs::Odometry::ConstPtr msg, derived_object_msgs::Object &object)
{
    object.pose = msg->pose.pose;
    object.twist = msg->twist.twist;
    object.header = msg->header;
}

void poseWithCovarianceStampedToObject(geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg, derived_object_msgs::Object &object)
{
    object.pose = msg->pose.pose;
    object.twist.linear.x = 0.;
    object.twist.linear.y = 0.;
    object.twist.linear.z = 0.;

    object.header = msg->header;
}

derived_object_msgs::Object &addOrGetObject(int id)
{
    for (auto &object : _bundled_objects.objects)
    {
        if (id == (int)object.id)
        {
            // ROS_INFO("Vicon Bundle: Received existing object");
            return object;
        }
    }

    // ROS_INFO("Vicon Bundle: Received new object");

    _bundled_objects.objects.emplace_back();
    _bundled_objects.objects.back().id = id;
    return _bundled_objects.objects.back();
}

void increaseObstacleCount(int id)
{
    if (!_received_list[id])
    {
        _received_list[id] = true;
        received_obstacles++;
    }

    if (received_obstacles == _obstacles_expected)
    {
        // ROS_INFO("Vicon Bundle: Publishing bundled objects");

        _bundled_object_pub.publish(_bundled_objects);

        received_obstacles = 0;
        _received_list = std::vector<bool>(_obstacles_expected, false);
    }
}

void dynamicObstacleOdomCallback(int id, nav_msgs::Odometry::ConstPtr msg)
{
    auto &object = addOrGetObject(id); // Get the object with this ID (or make it)
    odometryToObject(msg, object);     // Convert the msg to an object msg type

    object.shape.dimensions.resize(2); // Add dimensions (for dynamic objects: disc with radius 0.4)
    object.shape.type = object.shape.CYLINDER;
    object.shape.dimensions[object.shape.CYLINDER_RADIUS] = _dynamic_radius;
    object.shape.dimensions[object.shape.CYLINDER_HEIGHT] = 2.2;

    increaseObstacleCount(id); // We received one more object, publish if all objects were received
}

/** @see dynamicObstacleOdomCallback */
void staticObstacleOdomCallback(int id, geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    auto &object = addOrGetObject(id);
    poseWithCovarianceStampedToObject(msg, object);

    // Todo: Box
    object.shape.dimensions.resize(3);
    object.shape.type = object.shape.BOX;

    object.shape.dimensions[0] = _static_radius;
    object.shape.dimensions[1] = _static_radius * _static_size_list[id - _dynamic_obstacles_expected];

    object.pose.position.z = 1.;
    object.shape.dimensions[2] = 2.; // Height is defaulted to 2m

    increaseObstacleCount(id);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_bundle_node");

    auto nh_private = std::make_shared<ros::NodeHandle>("~");
    nh_private->getParam("dynamic_objects/topics", _dynamic_topic_list);
    nh_private->getParam("static_objects/topics", _static_topic_list);
    _dynamic_obstacles_expected = _dynamic_topic_list.size();
    _static_obstacles_expected = _static_topic_list.size();
    _obstacles_expected = _dynamic_obstacles_expected + _static_obstacles_expected;

    nh_private->getParam("dynamic_objects/radius", _dynamic_radius);
    nh_private->getParam("static_objects/radius", _static_radius);
    nh_private->getParam("static_objects/sizes", _static_size_list);

    _received_list = std::vector<bool>(_obstacles_expected, false);

    auto nh = std::make_shared<ros::NodeHandle>("");
    std::string print_statement = "Bundling " + std::to_string(_dynamic_obstacles_expected) + " dynamic obstacles: ";
    for (size_t i = 0; i < _dynamic_topic_list.size(); i++)
    {
        auto &topic = _dynamic_topic_list[i];
        _dynamic_subs.push_back(nh->subscribe<nav_msgs::Odometry>(
            topic + "/odometry/filtered",
            1,
            std::bind(dynamicObstacleOdomCallback, i, std::placeholders::_1)));
        print_statement += topic + " ";
    }

    print_statement += "and " + std::to_string(_static_obstacles_expected) + " static obstacles: ";
    for (size_t i = 0; i < _static_topic_list.size(); i++)
    {
        auto &topic = _static_topic_list[i];
        _static_subs.push_back(nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            "/vicon/" + topic,
            1,
            std::bind(staticObstacleOdomCallback, _dynamic_obstacles_expected + i, std::placeholders::_1)));
        print_statement += topic + " ";
    }
    ROS_INFO_STREAM(print_statement);

    if (_dynamic_radius <= 0.0)
    {
        ROS_ERROR("Obstacle Bundle Node: Radius of dynamic objects was less than 0. Exiting...");
        return 1;
    }

    if (_static_radius <= 0.0)
    {
        ROS_ERROR("Obstacle Bundle Node: Radius of static objects was less than 0. Exiting...");
        return 1;
    }

    if ((int)_static_size_list.size() < _static_obstacles_expected)
    {
        ROS_ERROR("Obstacle Bundle Node: Not enough sizes provided for static objects. Exiting...");
        return 1;
    }

    _bundled_object_pub = nh->advertise<derived_object_msgs::ObjectArray>("vicon_util/dynamic_objects", 1);

    ros::spin();
}