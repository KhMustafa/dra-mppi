#include "reader.h"

#include <ros_tools/paths.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>

void Reader::Read()
{
    std::string map_file = getPackagePath(config_->map_package_name_) + "/" + config_->map_file_name_; // ros::package::getPath(config_->map_package_name_) + "/" + config_->map_file_name_;
    Read(map_file);
}

void Reader::Read(const std::string &file_name)
{
    if (from_callback_)
        return;

    ROADMAP_INFO("Reading map file");
    map_.Clear();

    // If the path does not contain the package, add it
    std::string map_file;
    if (file_name.find("//" + config_->map_package_name_ + "//") != std::string::npos)
        map_file = getPackagePath(config_->map_package_name_) + "/" + file_name;
    else
        map_file = file_name;

    ROADMAP_INFO_STREAM("\tFile: " << map_file);

    // Read the file with the correct file extension
    if (map_file.substr(map_file.find_last_of(".") + 1) == "xml")
        ReadXML(map_file);
    else if (map_file.substr(map_file.find_last_of(".") + 1) == "osm")
        ReadOSM(map_file);
    else
        ReadYAML(map_file);
}

void Reader::Read(const std::string &&file_name)
{
    std::string map_file = getPackagePath(config_->map_package_name_) + "/" + file_name;
    LOG_VALUE("map file", map_file);
    Read(map_file);
}

void Reader::ReadXML(const std::string &file)
{
    rapidxml::file<> xmlFile(file.c_str()); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    // Ways connect the nodes
    int id = 0;
    for (rapidxml::xml_node<> *way = doc.first_node("way"); way; way = way->next_sibling("way"))
    {
        // Create a new way and add its nodes (nodes with ref attribute)
        Way new_way;
        double angle = RosTools::quaternionToAngle(offset_);
        Eigen::MatrixXd R = RosTools::rotationMatrixFromHeading(-angle);

        for (rapidxml::xml_node<> *node = way->first_node("nd"); node; node = node->next_sibling("nd"))
        {
            Eigen::Vector2d new_pos = R * Eigen::Vector2d(atof(node->first_attribute("x")->value()), atof(node->first_attribute("y")->value()));

            // int id = atoi(node->first_attribute("ref")->value());
            new_way.AddNode(Node(offset_.position.x + new_pos(0),
                                 offset_.position.y + new_pos(1),
                                 atof(node->first_attribute("theta")->value()) + angle));
        }

        for (rapidxml::xml_node<> *lane = way->first_node("lane"); lane; lane = lane->next_sibling("lane"))
        {
            // Optional two way parameters (default = false)
            bool two_way = false;
            if (lane->first_attribute("two_way")->value())
                two_way = bool(atoi(lane->first_attribute("two_way")->value()));

            // Create a lane for each lane tag
            new_way.AddLane(std::string(lane->first_attribute("type")->value()),
                            atof(lane->first_attribute("width")->value()),
                            two_way,
                            id);
        }

        map_.ways.push_back(new_way);
        ROADMAP_INFO("XML Read.");
    }
}

void Reader::ReadYAML(const std::string &file)
{
    (void)file;
    LOG_HOOK();
    // // std::cout << std::string("rosparam load ") + file << std::endl;
    // if (system(std::string("rosparam load " + file).c_str()))
    //     return;

    // // Assume one Way object as reference path

    // ros::NodeHandle nh;

    // std::vector<double> x, y, theta;
    // RoadmapConfig::retrieveParameter(nh, "global_path/x", x);
    // RoadmapConfig::retrieveParameter(nh, "global_path/y", y);
    // RoadmapConfig::retrieveParameter(nh, "global_path/theta", theta);
    // assert(x.size() == y.size());
    // assert(y.size() == theta.size());

    // // Create a way object
    // Way new_way;
    // double angle = RosTools::quaternionToAngle(offset_);
    // Eigen::MatrixXd R = RosTools::rotationMatrixFromHeading(-angle); // Rotate to the offset

    // for (size_t i = 0; i < x.size(); i++)
    // {
    //     Eigen::Vector2d new_pos = R * Eigen::Vector2d(x[i], y[i]);

    //     new_way.AddNode(Node(offset_.position.x + new_pos(0), offset_.position.y + new_pos(1), theta[i] + angle)); // Create nodes
    // }
    // int id = 0;
    // // Add a regular road and sidewalk by default
    // new_way.AddLane("road", 4.0, true, id);
    // new_way.AddLane("sidewalk", 2.0, true, id);
    // map_.ways.push_back(new_way);
    // ROADMAP_INFO("YAML Read.");
}

void Reader::ReadOSM(const std::string &file)
{
    rapidxml::file<> xmlFile(file.c_str()); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    map_.Clear();

    // Read the OSM
    rapidxml::xml_node<> *osm = doc.first_node("osm");

    // Estimate the average latitude and longitude
    double base_latitude = 0.;
    double base_longitude = 0.;
    int num_nodes = 0;
    for (rapidxml::xml_node<> *node = osm->first_node("node"); node; node = node->next_sibling("node"))
    {
        base_latitude += atof(node->first_attribute("lat")->value());
        base_longitude += atof(node->first_attribute("lon")->value());
        num_nodes++;
    }
    base_latitude /= (double)num_nodes;
    base_longitude /= (double)num_nodes;

    // First read all the nodes
    std::map<long, Node> nodes; /** A map from ID to node */
    for (rapidxml::xml_node<> *node = osm->first_node("node"); node; node = node->next_sibling("node"))
    {

        long node_id = atol(node->first_attribute("id")->value());
        nodes[node_id] = Node(atof(node->first_attribute("lat")->value()),
                              atof(node->first_attribute("lon")->value()),
                              base_latitude,
                              base_longitude,
                              0.);

        // std::cout << nodes[node_id].x << ", " << nodes[node_id].y << std::endl;
    }
    // Then read the ways and connect them to the nodes
    int id;
    for (rapidxml::xml_node<> *way = osm->first_node("way"); way; way = way->next_sibling("way"))
    {
        Way new_way;

        // Add its nodes
        for (rapidxml::xml_node<> *node = way->first_node("nd"); node; node = node->next_sibling("nd"))
        {
            // Add the current node to the way
            long node_id = atol(node->first_attribute("ref")->value());

            if (nodes.find(node_id) == nodes.end())
                std::cout << "node not found..." << std::endl;
            else
            {
                new_way.AddNode(nodes[node_id]);
                std::cout << "Adding: " << nodes[node_id].x << ", " << nodes[node_id].y << std::endl;
            }
        }
        // Check its tags
        bool is_road = true;
        std::string type;
        for (rapidxml::xml_node<> *tag = way->first_node("tag"); tag; tag = tag->next_sibling("tag"))
        {
            if (!std::string(tag->first_attribute("k")->value()).compare("highway"))
                type = "road";

            // Remove buildings
            if ((!std::string(tag->first_attribute("k")->value()).compare("building")) && (!std::string(tag->first_attribute("k")->value()).compare("yes")))
            {
                is_road = false;
                break;
            }
        }

        // Only proceed if we are reading a road
        if (!is_road)
            continue;

        new_way.AddLane(type, 4.0, false, id);

        map_.ways.push_back(new_way);
    }

    // for (rapidxml::xml_node<> *lane = way->first_node("lane"); lane; lane = lane->next_sibling("lane"))
    // {
    //     // Optional two way parameters (default = false)
    //     bool two_way = false;
    //     if (lane->first_attribute("two_way")->value())
    //         two_way = bool(atoi(lane->first_attribute("two_way")->value()));

    //     // Create a lane for each lane tag
    //     new_way.AddLane(std::string(lane->first_attribute("type")->value()),
    //                     atof(lane->first_attribute("width")->value()),
    //                     two_way,
    //                     id);
    // }

    // map_.ways.push_back(new_way);
    // ROADMAP_INFO("XML Read.");
}

void Reader::WaypointCallback(const nav_msgs::Path &msg)
{
    from_callback_ = true;

    // if (config_->activate_debug_output_)
    ROADMAP_INFO("Received " << std::floor(0.5 * msg.poses.size()) << " Waypoints");

    // Assume one Way object as reference path
    map_.Clear();

    // Create a way object
    Way new_way;
    std::cout << "number of waypoints is: " << msg.poses.size() << std::endl;
    for (size_t i = 0; i < msg.poses.size(); i += 2)
    {
        new_way.AddNode(Node(msg.poses[i].pose.position.x,
                             msg.poses[i].pose.position.y,
                             RosTools::quaternionToAngle(msg.poses[i].pose)));
    }

    int id = 0;
    // Add a regular road and sidewalk by default (road is not two way! or otherwise centered offcentre)
    new_way.AddLane("road", 4.0, true, id);
    new_way.AddLane("sidewalk", 2.0, true, id);
    map_.ways.push_back(new_way);

    ROADMAP_INFO("Waypoints Saved.");
}

void Reader::OffsetCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    offset_ = msg.pose.pose;
}
