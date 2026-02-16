#pragma once
#include <rclcpp/rclcpp.hpp>
#include <project_utils/msg/eigen_vector.hpp>
#include <chrono>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
#include "vehicleModel/VehicleModelCollection.h"

class VehicleInterface: public rclcpp::Node
{
    public:
        VehicleInterface();
        void on_activate();
    private:
        void addVehicles();
    private: 
        int mnum_vehiclesicles{2};
        ptUnorderedMap<UniqueId,ptSharedPtr<VehicleModel>> mVehicleCollection;
        ptUnorderedMap<long int , UniqueId> mVehKeyCollection;
        ptUnorderedMap<UniqueId,InputVector> mCommandedControlMap;
        ptUnorderedMap<UniqueId, rclcpp::Publisher<project_utils::msg::EigenVector>::SharedPtr> mStatePublisher;
        ptUnorderedMap<UniqueId,  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> mJointStatePublisher;
        ptUnorderedMap<UniqueId, rclcpp::Subscription<project_utils::msg::EigenVector>::SharedPtr> mControlSubscriber;

        rclcpp::TimerBase::SharedPtr mStateUpdateTimer;
        rclcpp::TimerBase::SharedPtr mStatePubTimer;

        void state_pub_timer_callback(); 
        void state_update_timer_callback();
        void broadcastTransform(std::string& vehicleName, const stPose& state );
        void control_sub_callback(const project_utils::msg::EigenVector::SharedPtr& msg, const UniqueId& id);
        
        YAML::Node mRosParams;
        double mSimTimeStep;
        double mStatePublisherTimeStep;
        std::string mFixedFrame;
        std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadCaster{nullptr};

};
