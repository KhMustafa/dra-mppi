
/** @brief: This node visualizes the vicon scene */

#include <ros/ros.h>

#include <ros_tools/convertions.h>
#include <ros_tools/visuals.h>

#include <derived_object_msgs/ObjectArray.h>

ros::Subscriber _object_sub;

void objectCallback(const derived_object_msgs::ObjectArray::ConstPtr &msg)
{
    auto &dynamic_publisher = VISUALS.getPublisher("vicon_util/visuals/dynamic_objects");
    auto &dynamic_marker = dynamic_publisher.getNewModelMarker();

    auto &static_publisher = VISUALS.getPublisher("vicon_util/visuals/static_objects");
    auto &static_marker = static_publisher.getNewPointMarker("CUBE");
    static_marker.setColor(0., 0., 0., 0.6);

    for (auto &object : msg->objects)
    {
        if (object.shape.type == object.shape.BOX)
        {
            static_marker.setScale(object.shape.dimensions[0], object.shape.dimensions[1], object.shape.dimensions[2]);
            static_marker.addPointMarker(object.pose);
        }
        else
        {
            if (object.id == 0)
                continue; // The first object is the robot

            double angle = RosTools::quaternionToAngle(object.pose.orientation) + M_PI_2;

            dynamic_marker.setOrientation(RosTools::angleToQuaternion(angle));
            dynamic_marker.setColorInt(object.id, 1., RosTools::Colormap::BRUNO);
            dynamic_marker.addPointMarker(object.pose);
        }
    }

    dynamic_publisher.publish();
    static_publisher.publish();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualize_scene");

    auto nh_private = std::make_shared<ros::NodeHandle>("~");
    auto nh = std::make_shared<ros::NodeHandle>("");

    _object_sub = nh->subscribe<derived_object_msgs::ObjectArray>(
        "vicon_util/dynamic_objects",
        1,
        std::bind(objectCallback, std::placeholders::_1));

    VISUALS.init(nh.get());

    ros::spin();
}