#include "vehicle_interface/vehicle_interface_class.h"

////////////////////////////////////////////////////////////////////////////////

VehicleInterface::VehicleInterface():Node("vehicle_interface_node",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) 
{

    RCLCPP_INFO(this->get_logger(),"vehicle interface node started");
    
    
    std::string config_path;
    this->get_parameter("configuration_file", config_path);
    YAML::Node root = YAML::LoadFile(config_path);

    // load the params
    mRosParams = root["/**"]["ros__parameters"];
    //this->declare_parameter<double>("simulation.simTimeStep", 0.01);
    //this->declare_parameter<double>("simulation.statePublisherTimeStep", 0.05);
    YAML::Node sim_config = mRosParams["simulation"];
    mSimTimeStep = sim_config["simTimeStep"].as<double>();
    mStatePublisherTimeStep = sim_config["statePublisherTimeStep"].as<double>();
    RCLCPP_INFO(this->get_logger(),"mSimTimeStep");
}

////////////////////////////////////////////////////////////////////////////////

void VehicleInterface::state_pub_timer_callback() 
{
    // Timer callback logic here
    RCLCPP_INFO(this->get_logger(), " state Publisher Timer triggered");
    for(auto& [key, veh] : mVehicleCollection)
    {
        project_utils::msg::EigenVector msg;
        StateVector state = veh->getState(); 
        msg.data = std::vector<double>(state.data(), state.data() + state.size());
        mStatePublisher[key]->publish(msg);
    }
}

////////////////////////////////////////////////////////////////////////////////

void VehicleInterface::state_update_timer_callback()
{
    RCLCPP_INFO(this->get_logger(), " State Update Timer triggered");
    for(auto& [key, veh] : mVehicleCollection)
    {
        if(mCommandedControlMap.find(key) != mCommandedControlMap.end())
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
        std::cerr <<veh->id().value() <<" ";
        mVehKeyCollection[veh->id().value()] = veh->id();
    }

}