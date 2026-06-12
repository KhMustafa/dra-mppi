#!/usr/bin/env python
import os
import rospy
import subprocess

# NOTE: This script is used to launch multiple EKFs for different Vicon topics


def launch_ekf(input, output, publish_tf):

    source_path = f"{os.path.dirname(os.path.realpath(__file__))}/../../../devel/setup.bash"

    run_command = f"source {source_path}"
    run_command += f" && roslaunch vicon_util ekf.launch input_topic:={input} output_topic:={output} publish_tf:={publish_tf} &"
    rospy.loginfo(run_command)
    subprocess.call(["bash", "-c", run_command])


def node_function():

    # Initialize the ROS node
    rospy.init_node("launcher_node", anonymous=True)

    topic_list = rospy.get_param("~input_topics", ["drone1", "dynamic_object1"])
    robot_topic = rospy.get_param("~robot_topic", "drone1")
    for topic in topic_list:
        launch_ekf(topic, topic, topic == robot_topic)

    rospy.loginfo(f"Launched EKFs for {topic_list}")
    rospy.spin()


if __name__ == "__main__":
    try:
        node_function()
    except rospy.ROSInterruptException:
        pass
