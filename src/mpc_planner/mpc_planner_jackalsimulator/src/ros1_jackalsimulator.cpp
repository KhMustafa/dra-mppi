#include <mpc_planner_jackalsimulator/ros1_jackalsimulator.h>

#include <mpc_planner/visualizer.h>

#include <mpc_planner/data_preparation.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>
#include <ros_tools/data_saver.h>

#include <std_msgs/Empty.h>
#include <ros_tools/profiling.h>



using namespace MPCPlanner;

JackalPlanner::JackalPlanner(ros::NodeHandle &nh)
{
    LOG_INFO("Started Jackal Planner");
    
    Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings")); // Initialize the configuration

    // _data.robot_area = {Disc(0., CONFIG["robot_radius"].as<double>())}; // Zero offset single disc
    _data.robot_area = defineRobotArea(CONFIG["robot"]["length"].as<double>(),
                                       CONFIG["robot"]["width"].as<double>(),
                                       CONFIG["n_discs"].as<int>());

    _visualizer = std::make_unique<Visualizer>(); // Initialize the visualizer

    initializeSubscribersAndPublishers(nh); // Initialize the ROS interface

    

    startEnvironment(); // Start the simulation

    _reconfigure = std::make_unique<JackalsimulatorReconfigure>(); // Initialize RQT reconfigure

    RosTools::Instrumentor::Get().BeginSession("mpc_planner_jackalsimulator");

    // Start the control loop
    _timer = nh.createTimer(
        ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()),
        &JackalPlanner::loop,
        this);

    LOG_DIVIDER();
   
}

JackalPlanner::~JackalPlanner()
{
    LOG_INFO("Stopped Jackal Planner");
    BENCHMARKERS.print();
    RosTools::Instrumentor::Get().EndSession();
}

void JackalPlanner::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
{
    LOG_INFO("initializeSubscribersAndPublishers");

    /** @note Topics are mapped in the launch file! */
    _state_sub = nh.subscribe<nav_msgs::Odometry>(
        "/input/state", 5,
        boost::bind(&JackalPlanner::stateCallback, this, _1));

    _state_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/input/state_pose", 5,
        boost::bind(&JackalPlanner::statePoseCallback, this, _1));


    _path_sub = nh.subscribe<nav_msgs::Path>(
        "/input/reference_path", 1,
        boost::bind(&JackalPlanner::pathCallback, this, _1));

    _obstacle_sim_sub = nh.subscribe<mpc_planner_msgs::ObstacleArray>(
        "/input/obstacles", 1,
        boost::bind(&JackalPlanner::obstacleCallback, this, _1));

    _cmd_pub = nh.advertise<geometry_msgs::Twist>(
        "/output/command", 1);

    _pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/output/pose", 1);

    _collisions_sub = nh.subscribe<std_msgs::Float64>(
        "/feedback/collisions", 1,
        boost::bind(&JackalPlanner::collisionCallback, this, _1));

    _mppi_planned_trajectory_sub = nh.subscribe<nav_msgs::Path>(
        "/mppi_planned_trajectory", 1,
        boost::bind(&JackalPlanner::plannedTrajectoryCallback, this, _1));

    _path_pub   = nh.advertise<visualization_msgs::Marker>("/visualization_path", 1);
    _points_pub = nh.advertise<visualization_msgs::Marker>("/visualization_points", 1);

    // Environment Reset
    _reset_simulation_pub = nh.advertise<std_msgs::Empty>("/lmpcc/reset_environment", 1);
    _reset_simulation_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    _reset_ekf_client = nh.serviceClient<robot_localization::SetPose>("/set_pose");

    // Pedestrian simulator
    _ped_horizon_pub = nh.advertise<std_msgs::Int32>("/pedestrian_simulator/horizon", 1);
    _ped_integrator_step_pub = nh.advertise<std_msgs::Float32>("/pedestrian_simulator/integrator_step", 1);
    _ped_clock_frequency_pub = nh.advertise<std_msgs::Float32>("/pedestrian_simulator/clock_frequency", 1);
    _ped_start_client = nh.serviceClient<std_srvs::Empty>("/pedestrian_simulator/start");

    // guidance planner service
    // Create a service
    // _guidance_service = nh.advertiseService("/guidance_planner", &JackalPlanner::guidancesCallback, this);
    //LOG_INFO("Guidance planner service ready.");
}

void JackalPlanner::startEnvironment()
{

    LOG_INFO("Starting pedestrian simulator");
    for (int i = 0; i < 20; i++)
    {
        std_msgs::Int32 horizon_msg;
        horizon_msg.data = CONFIG["N"].as<int>();
        _ped_horizon_pub.publish(horizon_msg);

        std_msgs::Float32 integrator_step_msg;
        integrator_step_msg.data = CONFIG["integrator_step"].as<double>();
        _ped_integrator_step_pub.publish(integrator_step_msg);

        std_msgs::Float32 clock_frequency_msg;
        clock_frequency_msg.data = CONFIG["control_frequency"].as<double>();
        _ped_clock_frequency_pub.publish(clock_frequency_msg);

        std_srvs::Empty empty_msg;
        if (_ped_start_client.call(empty_msg))
            break;
        else
        {
            LOG_INFO_THROTTLE(3, "Waiting for pedestrian simulator to start");
            ros::Duration(1.0).sleep();
        }
    }
    _enable_output = CONFIG["enable_output"].as<bool>();

    // Initialize simulation utilities
    _timeout_timer.setDuration(60.);
    _timeout_timer.start();
    for (int i = 0; i < CAMERA_BUFFER; i++)
    {
        _x_buffer[i] = 0.;
        _y_buffer[i] = 0.;
    }

    LOG_INFO("Environment ready.");
}

bool JackalPlanner::objectiveReached()
{
    // Simple conditions for resetting the simulation
    return _state.get("x") > 50.; //    Straight
    // return RosTools::distance(_state.getPos(), Eigen::Vector2d(24., 24.)) < 4.0; // Diagonal
}


void JackalPlanner::loop(const ros::TimerEvent &event)
{
    
    (void)event;
    visualizePath();

    _data.planning_start_time = std::chrono::system_clock::now();

    LOG_DEBUG("============= Loop =============");
    
    publishPose();
    publishCamera();    
    LOG_DEBUG("============= End Loop =============");
}

void JackalPlanner::stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
   
    _state.set("x", msg->pose.pose.position.x);
    _state.set("y", msg->pose.pose.position.y);
    _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
    _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));

    if (std::abs(msg->pose.pose.orientation.x) > (M_PI / 8.) || std::abs(msg->pose.pose.orientation.y) > (M_PI / 8.))
    {
        LOG_WARN("Detected flipped robot. Resetting.");
        reset(false); // Reset without success
    }
    
}

void JackalPlanner::statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    
    _state.set("x", msg->pose.position.x);
    _state.set("y", msg->pose.position.y);
    _state.set("psi", msg->pose.orientation.z);
    _state.set("v", msg->pose.position.z); // Encoded here in this case

    if (std::abs(msg->pose.orientation.x) > (M_PI / 8.) || std::abs(msg->pose.orientation.y) > (M_PI / 8.))
    {
        LOG_ERROR("Detected flipped robot. Resetting.");
        reset(false); // Reset without success
    }
    
}


bool JackalPlanner::isPathTheSame(const nav_msgs::Path::ConstPtr &msg)
{
    // Check if the path is the same
    if (_data.reference_path.x.size() != msg->poses.size())
        return false;

    // Check up to the first two points
    int num_points = std::min(2, (int)_data.reference_path.x.size());
    for (int i = 0; i < num_points; i++)
    {
        if (!_data.reference_path.pointInPath(i, msg->poses[i].pose.position.x, msg->poses[i].pose.position.y))
            return false;
    }
    return true;
}

void JackalPlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    LOG_DEBUG("Path callback");

    if (isPathTheSame(msg))
        return;

    _data.reference_path.clear();

    for (auto &pose : msg->poses)
    {
        _data.reference_path.x.push_back(pose.pose.position.x);
        _data.reference_path.y.push_back(pose.pose.position.y);
    }
    _data.reference_path.psi.push_back(0.0);
}

void JackalPlanner::plannedTrajectoryCallback(const nav_msgs::Path::ConstPtr &msg)
{
    //LOG_WARN("Planned trajectory callback");
    _mppi_plan.clear();

    for (auto &pose : msg->poses)
    {
       _mppi_plan.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
    }
    //std::cout << "visualize" << std::endl;
    _visualizer->visualize(_state, _data, _mppi_plan);

}

void JackalPlanner::obstacleCallback(const mpc_planner_msgs::ObstacleArray::ConstPtr &msg)
{
    _data.dynamic_obstacles.clear();
    //std::cout << "obstacle callback" << std::endl;

    for (auto &obstacle : msg->obstacles)
    {
        // Save the obstacle (ID, position, orientation, radius)
        _data.dynamic_obstacles.emplace_back(
            obstacle.id,
            Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y),
            RosTools::quaternionToAngle(obstacle.pose),
            CONFIG["obstacle_radius"].as<double>());
        auto &dynamic_obstacle = _data.dynamic_obstacles.back();

        if (obstacle.probabilities.size() == 0) // No Predictions!
        {
            std::cout << "no predictions" << std::endl;
            continue;
        }

        // Save the prediction
        if (obstacle.probabilities.size() == 1) // One mode
        {
            //std::cout << "single mode" << std::endl;
            dynamic_obstacle.prediction = Prediction(PredictionType::GAUSSIAN, obstacle.probabilities.size());

            const auto &mode = obstacle.gaussians[0];
            for (size_t k = 0; k < mode.mean.poses.size(); k++)
            {
                // Add prediction at time step k (position, orientation, major/minor bivariate Gaussian size)
                dynamic_obstacle.prediction.modes[0].emplace_back(
                    Eigen::Vector2d(mode.mean.poses[k].pose.position.x, mode.mean.poses[k].pose.position.y),
                    RosTools::quaternionToAngle(mode.mean.poses[k].pose.orientation),
                    mode.major_semiaxis[k],
                    mode.minor_semiaxis[k]);
            }

            if (mode.major_semiaxis.back() == 0. || !CONFIG["probabilistic"]["enable"].as<bool>()) // If uncertainty is zero
                dynamic_obstacle.prediction.type = PredictionType::DETERMINISTIC;
            else
                dynamic_obstacle.prediction.type = PredictionType::GAUSSIAN;
        }
        else
        {   
            dynamic_obstacle.prediction = Prediction(PredictionType::NONGAUSSIAN, obstacle.probabilities.size());
            
            // std::cout << "number of modes first entry is: " << obstacle.probabilities.size() << std::endl;

            for (size_t mode = 0; mode < obstacle.probabilities.size(); mode++)
            {
                //std::cout << "number of poses is: " << obstacle.gaussians[mode].mean.poses.size() << std::endl;
                for (size_t k = 0; k < obstacle.gaussians[mode].mean.poses.size(); k++)
                {
                    // Add prediction at time step k (position, orientation, major/minor bivariate Gaussian size)
                    // std::cout << "dynamic obstacle prediction modes is: " << dynamic_obstacle.prediction.modes.size() << std::endl;
                    dynamic_obstacle.prediction.modes[mode].emplace_back(
                        Eigen::Vector2d(obstacle.gaussians[mode].mean.poses[k].pose.position.x, obstacle.gaussians[mode].mean.poses[k].pose.position.y),
                        RosTools::quaternionToAngle(obstacle.gaussians[mode].mean.poses[k].pose.orientation),
                        obstacle.gaussians[mode].major_semiaxis[k],
                        obstacle.gaussians[mode].minor_semiaxis[k]);
                }

            }
            
        }
    }
    // ensureObstacleSize(_data.dynamic_obstacles, _state); // Ensure that there are `max_obstacles` obstacles (possibly adding dummies)

    // if (CONFIG["probabilistic"]["propagate_uncertainty"].as<bool>())
    //     propagatePredictionUncertainty(_data.dynamic_obstacles);
}

void JackalPlanner::visualizePath() 
{
    visualization_msgs::Marker path_marker, points_marker;

    // Set common properties
    path_marker.header.frame_id = points_marker.header.frame_id = "map";  // Change as needed
    path_marker.header.stamp = points_marker.header.stamp = ros::Time::now();
    path_marker.ns = points_marker.ns = "reference_path";
    path_marker.action = points_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = points_marker.pose.orientation.w = 1.0;

    // Green Line Strip (Path)
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.scale.x = 0.05;  // Line width
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;

    // Black Spheres (Discrete Points)
    points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    points_marker.scale.x = points_marker.scale.y = points_marker.scale.z = 0.1; // Dot size
    points_marker.color.r = 0.0;
    points_marker.color.g = 0.0;
    points_marker.color.b = 0.0;
    points_marker.color.a = 1.0;

    // Add points
    for (size_t i = 0; i < _data.reference_path.x.size(); ++i) {
        geometry_msgs::Point p;
        p.x = _data.reference_path.x[i];
        p.y = _data.reference_path.y[i];
        p.z = 0.0;  // Assuming a 2D path

        path_marker.points.push_back(p);
        points_marker.points.push_back(p);
    }

    // Publish markers
    _path_pub.publish(path_marker);
    _points_pub.publish(points_marker);
}

void JackalPlanner::visualize() // Function to visualize anything in this wrapper
{
    
    auto &publisher = VISUALS.getPublisher("angle");
    auto &line = publisher.getNewLine();

    line.addLine(Eigen::Vector2d(_state.get("x"), _state.get("y")),
                 Eigen::Vector2d(_state.get("x") + 1.0 * std::cos(_state.get("psi")), _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
    publisher.publish();
    
}

void JackalPlanner::reset(bool success)
{   
    LOG_WARN("Resetting");

    _reset_simulation_client.call(_reset_msg); // Reset simulation
    _reset_ekf_client.call(_reset_pose_msg);
    _reset_simulation_pub.publish(std_msgs::Empty());

    for (int i = 0; i < CAMERA_BUFFER; i++)
    {
        _x_buffer[i] = 0.;
        _y_buffer[i] = 0.;
    }

    ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()).sleep();    
}

void JackalPlanner::collisionCallback(const std_msgs::Float64::ConstPtr &msg)
{
    _data.intrusion = (float)(msg->data);

    if (_data.intrusion > 0.)
        LOG_INFO_THROTTLE(500., "Collision detected (Intrusion: " << _data.intrusion << ")");
}

void JackalPlanner::publishPose() // Used for the social forces model to avoid the robot
{
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = _state.get("x");
    pose.pose.position.y = _state.get("y");
    pose.pose.orientation = RosTools::angleToQuaternion(_state.get("psi"));

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    _pose_pub.publish(pose);
    
}

void JackalPlanner::publishCamera() // For a smoothened RViz camera
{
    
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = ros::Time::now();

    if ((msg.header.stamp - _prev_stamp) < ros::Duration(0.5 / CONFIG["control_frequency"].as<double>()))
        return;

    _prev_stamp = msg.header.stamp;

    msg.header.frame_id = "map";
    msg.child_frame_id = "camera";

    // Smoothen the camera
    for (int i = 0; i < CAMERA_BUFFER - 1; i++)
    {
        _x_buffer[i] = _x_buffer[i + 1];
        _y_buffer[i] = _y_buffer[i + 1];
    }
    _x_buffer[CAMERA_BUFFER - 1] = _state.get("x");
    _y_buffer[CAMERA_BUFFER - 1] = _state.get("y");
    double camera_x = 0., camera_y = 0.;
    for (int i = 0; i < CAMERA_BUFFER; i++)
    {
        camera_x += _x_buffer[i];
        camera_y += _y_buffer[i];
    }
    msg.transform.translation.x = camera_x / (double)CAMERA_BUFFER;
    msg.transform.translation.y = camera_y / (double)CAMERA_BUFFER;
    msg.transform.translation.z = 0.0;
    msg.transform.rotation.x = 0;
    msg.transform.rotation.y = 0;
    msg.transform.rotation.z = 0;
    msg.transform.rotation.w = 1;

    _camera_pub.sendTransform(msg);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    ros::NodeHandle nh;
    auto jackal_planner = std::make_shared<JackalPlanner>(nh);
    VISUALS.init(&nh);

    ros::spin();

    BENCHMARKERS.print();

    return 0;
}