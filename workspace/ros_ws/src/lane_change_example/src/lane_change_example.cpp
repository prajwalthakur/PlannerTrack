#include "lane_change_example/lane_change_example.h"

////////////////////////////////////////////////////////////////////////////////

LaneChangeExample::LaneChangeExample():Node("lnae_change_example_node",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) 
{

    RCLCPP_INFO(this->get_logger(),"Lane change example node started");
    
    
    std::string config_path;
    this->get_parameter("configuration_file", config_path);
    YAML::Node root = YAML::LoadFile(config_path);

    // load the params
    mRosParams = root["/**"]["ros__parameters"];
    RCLCPP_INFO(this->get_logger(),"mRosParams loaded");

    mEgoVehId = mRosParams["laneChangeExample"]["egoVehId"].as<int>();
    mnum_vehiclesicles  = mRosParams["simulation"]["num_vehicles"].as<int>();
    mCreateLaneObj = std::make_shared<CreateLanes>(mRosParams);
    RCLCPP_INFO(this->get_logger(),"params loaded");

    // YAML::Node sim_config = mRosParams["simulation"];
    // mSimTimeStep = sim_config["simTimeStep"].as<double>();
    // mStatePublisherTimeStep = sim_config["statePublisherTimeStep"].as<double>();
    // mFixedFrame = sim_config["fixedFrame"].as<std::string>();
    // RCLCPP_INFO(this->get_logger(),"mSimTimeStep");
    // mTfBroadCaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    

}

////////////////////////////////////////////////////////////////////////////////

void LaneChangeExample::on_activate()
{
    addVehicles();
    
    RCLCPP_INFO(this->get_logger(), " on activate");
    for(int idx=1;idx<=mnum_vehiclesicles; ++idx)
    {
            mControlPublisher[mVehKeyCollection.at(idx)]= create_publisher<project_utils::msg::EigenVector>("/vehicle_" + std::to_string(idx) + "/control_command", 10);        
            mStateSubsriber[mVehKeyCollection.at(idx)] =\
            create_subscription<project_utils::msg::EigenVector>("/vehicle_" + std::to_string(idx) + "/state",10,
                            [this, idx](const project_utils::msg::EigenVector::SharedPtr msg ){this->state_sub_callback(msg,mVehKeyCollection.at(idx));});
    
    }

    rclcpp::QoS qos(1);
    qos.transient_local();   
    qos.best_effort();

    mLanePublisher = create_publisher<visualization_msgs::msg::Marker>(
        "/lane_marker", qos);

    mCreateLaneObj->createStLanes(mLeftWaypoints,mRightWaypoints);
    //createLane();
    mLanePublisherTimer = this->create_wall_timer(std::chrono::duration<double>(5.0),[this](){this->createLane();});
}

////////////////////////////////////////////////////////////////////////////////

void LaneChangeExample::createLane()
{
    auto publish_lane = [&](const std::vector<std::vector<double>>& waypoints,
                            int id,
                            float r, float g, float b)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = now();
        marker.ns = "lane";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.15;   // line width

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        marker.points.resize(waypoints.size());

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            geometry_msgs::msg::Point p;

            if (waypoints[i].size() < 2)
                continue;   // safety check

            p.x = waypoints[i][0];
            p.y = waypoints[i][1];
            p.z = 0.0;

            marker.points[i] = p;
        }

        mLanePublisher->publish(marker);
    };

    publish_lane(mLeftWaypoints, 0, 1.0, 1.0, 0.0);   // left lane
    publish_lane(mRightWaypoints, 1, 1.0, 1.0, 0.0);  // right lane
}


////////////////////////////////////////////////////////////////////////////////

void LaneChangeExample::addVehicles()
{
    YAML::Node simConfig = mRosParams["simulation"];
    RCLCPP_INFO(this->get_logger(),"adding vehicles in sim");
    for(int i=1;i<=mnum_vehiclesicles;++i)
    {
        
        std::string veh_key = "vehicle" + std::to_string(i) + "_param";
        if (!mRosParams[veh_key])
        {
            RCLCPP_ERROR(this->get_logger(), "Missing config for %s", veh_key.c_str());
            continue;
        }
        YAML::Node config = mRosParams[veh_key];
        // pass simulation config and vehicle configs yaml node.
        ptSharedPtr<VehicleModel> veh  = std::make_shared<SingleTrackDynModel>(simConfig,config,i);
        mVehicleCollection[veh->id()] = veh;
        //std::cerr <<veh->id().value() <<" ";
        mVehKeyCollection[veh->id().value()] = veh->id();
    }

}

////////////////////////////////////////////////////////////////////////////////

void LaneChangeExample::state_sub_callback( const project_utils::msg::EigenVector::SharedPtr& msg,
    const UniqueId& id)
{
    StateVector state = Eigen::Map<Eigen::VectorXd>(msg->data.data(), msg->data.size());
    const auto numStateControl = mVehicleCollection[id]->getNumStatesInputs();
    if(numStateControl.first != state.size())
    {
            // Might need to obtained other states depending on the situation and feedback policy
            // For now it should be equal to the defined number of states
            RCLCPP_ERROR_STREAM(this->get_logger(),
                "Received state feedback " << state.size()
                << " and defined number of states "
                << numStateControl.first
                << " does not match");

    }
    mVehicleCollection[id]->setState(state);
    
}

////////////////////////////////////////////////////////////////////////////////
