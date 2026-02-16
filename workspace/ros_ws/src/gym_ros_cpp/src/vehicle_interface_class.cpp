#include "gym_ros_cpp/vehicle_interface_class.h"

////////////////////////////////////////////////////////////////////////////////

VehicleInterface::VehicleInterface():Node("vehicle_interface_node",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) 
{

    RCLCPP_INFO(this->get_logger(),"vehicle interface node started");
    
    
    std::string config_path;
    this->get_parameter("configuration_file", config_path);
    YAML::Node root = YAML::LoadFile(config_path);

    // load the params
    mRosParams = root["/**"]["ros__parameters"];
    YAML::Node sim_config = mRosParams["simulation"];
    mSimTimeStep = sim_config["simTimeStep"].as<double>();
    mStatePublisherTimeStep = sim_config["statePublisherTimeStep"].as<double>();
    mFixedFrame = sim_config["fixedFrame"].as<std::string>();
    mnum_vehiclesicles  = sim_config["num_vehicles"].as<int>();

    RCLCPP_INFO(this->get_logger(),"mSimTimeStep");
    mTfBroadCaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

}

////////////////////////////////////////////////////////////////////////////////

void VehicleInterface::broadcastTransform(std::string& vehicleName, const stPose& state )
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = mFixedFrame;
    t.child_frame_id = vehicleName + "/base_link";

    t.transform.translation.x = state.xCoord;
    t.transform.translation.y = state.yCoord;
    t.transform.translation.z = state.zCoord;

    tf2::Quaternion q;
    q.setRPY(0, 0, state.yaw);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    mTfBroadCaster->sendTransform(t);

}

////////////////////////////////////////////////////////////////////////////////

void VehicleInterface::state_pub_timer_callback() 
{
    // Timer callback logic here
    RCLCPP_INFO(this->get_logger(), " state Publisher Timer triggered");
    for(auto& [key, veh] : mVehicleCollection)
    {

        std::string vehicleName = "vehicle_" + std::to_string(veh->id().value());
        StateVector state = veh->getState(); 
        stPose statePose = veh->getStatePose();
        double steering_angle = statePose.steeringAngle;


        project_utils::msg::EigenVector msg;
        msg.data = std::vector<double>(state.data(), state.data() + state.size());
        mStatePublisher[key]->publish(msg);
        

        sensor_msgs::msg::JointState joint_msg;
        joint_msg.name = {
            vehicleName + "/front_left_hinge_to_wheel",
            vehicleName + "/front_right_hinge_to_wheel"
        };
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.position = {steering_angle, steering_angle};
        mJointStatePublisher[key]->publish(joint_msg);
        broadcastTransform(vehicleName, statePose);
    }
}

////////////////////////////////////////////////////////////////////////////////

void VehicleInterface::state_update_timer_callback()
{
    RCLCPP_INFO(this->get_logger(), " State Update Timer triggered");
    for(auto& [key, veh] : mVehicleCollection)
    {
        //if(mCommandedControlMap.find(key) != mCommandedControlMap.end())
        veh->step();
    }
}

////////////////////////////////////////////////////////////////////////////////

void VehicleInterface::control_sub_callback(
    const project_utils::msg::EigenVector::SharedPtr& msg,
    const UniqueId& id)
{
    //int idx = msg->data.idx();
    //if(mVehKeyCollection.find(idx)!=mVehKeyCollection.end())
    InputVector control= Eigen::Map<Eigen::VectorXd>(msg->data.data(), msg->data.size());
    mCommandedControlMap[id] =control;   
    mVehicleCollection[id]->updateCommandedControl(control);
}

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief VehicleInterface activation function to initialize 
 * m_vehicle, m_integrator, publisher and subscriber
 */
void VehicleInterface::on_activate() 
{
    addVehicles();
    RCLCPP_INFO(this->get_logger(), " on activate");
    for(int idx=1;idx<=mnum_vehiclesicles;++idx)
    {
        mStatePublisher[mVehKeyCollection.at(idx)] = create_publisher<project_utils::msg::EigenVector>("/vehicle_" + std::to_string(idx) + "/state", 10);
        mControlSubscriber[mVehKeyCollection.at(idx)] = \
            create_subscription<project_utils::msg::EigenVector>("/vehicle_" + std::to_string(idx) + "/control_command",10,
                            [this, idx](const project_utils::msg::EigenVector::SharedPtr msg ){this->control_sub_callback(msg,mVehKeyCollection.at(idx));});
    
        mJointStatePublisher[mVehKeyCollection.at(idx)] = create_publisher<sensor_msgs::msg::JointState>("/vehicle_" + std::to_string(idx) + "/joint_states", 10);
    }
    

    mStateUpdateTimer = this->create_wall_timer(std::chrono::duration<double>(mSimTimeStep),[this](){this->state_update_timer_callback();});
    mStatePubTimer = this->create_wall_timer(std::chrono::duration<double>(mStatePublisherTimeStep),[this](){this->state_pub_timer_callback();});
    
}

////////////////////////////////////////////////////////////////////////////////

void VehicleInterface::addVehicles()
{
    YAML::Node simConfig = mRosParams["simulation"];
    RCLCPP_INFO(this->get_logger(),"adding vehicles");
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
        ptSharedPtr<VehicleModel> veh  = std::make_shared<SingleTrackDynModel>(simConfig,config);
        mVehicleCollection[veh->id()] = veh;
        //std::cerr <<veh->id().value() <<" ";
        mVehKeyCollection[veh->id().value()] = veh->id();
    
    }

}