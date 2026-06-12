
/** @brief: This node publishes fake vicon messages to be able to test outside of the lab */

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>

std::vector<ros::Publisher> _spoof_pubs;
std::vector<std::string> _topic_list;
std::vector<std::string> _message_types;

ros::Timer _timer;

void loop(const ros::TimerEvent &)
{
    geometry_msgs::PoseStamped spoof_pose_msg;
    spoof_pose_msg.header.stamp = ros::Time::now();
    spoof_pose_msg.header.frame_id = "map";
    spoof_pose_msg.pose.position.x = 0.0;
    spoof_pose_msg.pose.position.y = 0.0;
    spoof_pose_msg.pose.position.z = 0.0;
    spoof_pose_msg.pose.orientation.x = 0.0;
    spoof_pose_msg.pose.orientation.y = 0.0;
    spoof_pose_msg.pose.orientation.z = 0.0;
    spoof_pose_msg.pose.orientation.w = 1.0;

    geometry_msgs::PoseWithCovarianceStamped spoof_pose_with_covariance_msg;
    spoof_pose_with_covariance_msg.header.stamp = ros::Time::now();
    spoof_pose_with_covariance_msg.header.frame_id = "map";
    spoof_pose_with_covariance_msg.pose.pose.position.x = 0.0;
    spoof_pose_with_covariance_msg.pose.pose.position.y = 0.0;
    spoof_pose_with_covariance_msg.pose.pose.position.z = 0.0;
    spoof_pose_with_covariance_msg.pose.pose.orientation.x = 0.0;
    spoof_pose_with_covariance_msg.pose.pose.orientation.y = 0.0;
    spoof_pose_with_covariance_msg.pose.pose.orientation.z = 0.0;
    spoof_pose_with_covariance_msg.pose.pose.orientation.w = 1.0;
    double identity_multiplier = 1.;
    spoof_pose_with_covariance_msg.pose.covariance[0] = identity_multiplier;
    spoof_pose_with_covariance_msg.pose.covariance[7] = identity_multiplier;
    spoof_pose_with_covariance_msg.pose.covariance[14] = identity_multiplier;
    spoof_pose_with_covariance_msg.pose.covariance[21] = identity_multiplier;
    spoof_pose_with_covariance_msg.pose.covariance[28] = identity_multiplier;
    spoof_pose_with_covariance_msg.pose.covariance[35] = identity_multiplier;

    for (size_t i = 0; i < _topic_list.size(); i++)
    {
        auto &pub = _spoof_pubs[i];

        spoof_pose_msg.pose.position.x += 2;
        spoof_pose_with_covariance_msg.pose.pose.position.x += 2;

        if (_message_types[i] == "geometry_msgs/PoseWithCovarianceStamped")
            pub.publish(spoof_pose_with_covariance_msg);
        else
            pub.publish(spoof_pose_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spoof_vicon_node");

    auto nh_private = std::make_shared<ros::NodeHandle>("~");
    auto nh = std::make_shared<ros::NodeHandle>("");

    nh_private->getParam("topics", _topic_list);
    nh_private->getParam("message_types", _message_types);

    if (_topic_list.empty())
    {
        ROS_ERROR("No topics provided to spoof_vicon_node. Exiting...");
        return 1;
    }

    std::string print_statement = "Spoofing vicon topics: ";
    for (size_t i = 0; i < _topic_list.size(); i++)
    {
        auto &topic = _topic_list[i];
        if (_message_types[i] == "geometry_msgs/PoseWithCovarianceStamped")
            _spoof_pubs.push_back(nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("vicon/" + topic, 1));
        else
            _spoof_pubs.push_back(nh->advertise<geometry_msgs::PoseStamped>("vicon/" + topic, 1));

        print_statement += topic + " ";
    }
    ROS_INFO_STREAM(print_statement);

    _timer = nh->createTimer(ros::Duration(1.0 / 100.), &loop);

    ros::spin();
}