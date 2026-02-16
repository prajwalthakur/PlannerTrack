#pragma once
#include <rclcpp/rclcpp.hpp>
#include <project_utils/msg/eigen_vector.hpp>
#include <chrono>
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
#include "vehicleModel/VehicleModelCollection.h"
#include "lane_change_example/CreateLane.h"

class LaneChangeExample: public rclcpp::Node
{
    public:
        LaneChangeExample();
        void on_activate();
    private:
        void state_sub_callback(const project_utils::msg::EigenVector::SharedPtr& msg, const UniqueId& id);
        //void veh_controller_update_callback();
        //void update_other_veh_control();
       // void update_ego_veh_control();
        void addVehicles();
        void createLane();
    private:
        YAML::Node mRosParams;
        int mEgoVehId{1};
        int mnum_vehiclesicles{1};
        ptUnorderedMap<UniqueId,ptSharedPtr<VehicleModel>> mVehicleCollection;
        ptUnorderedMap<long int , UniqueId> mVehKeyCollection;
    
        ptUnorderedMap<UniqueId, rclcpp::Publisher<project_utils::msg::EigenVector>::SharedPtr> mControlPublisher;
        ptUnorderedMap<UniqueId, rclcpp::Subscription<project_utils::msg::EigenVector>::SharedPtr> mStateSubsriber;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mLanePublisher{nullptr};
        ptSharedPtr<CreateLanes> mCreateLaneObj{nullptr};
        std::vector<std::vector<double>> mLeftWaypoints;
        std::vector<std::vector<double>> mRightWaypoints;
        rclcpp::TimerBase::SharedPtr mLanePublisherTimer{nullptr};

};