#pragma once
#include "controller_mode.hpp"
#include "trajectory_follower_base/trajectory_follower_base_collection.h"
#include "dummy_lateral_controller/DummyLateralController.hpp"
#include "dummy_longitudinal_controller/DummyLongitudinalController.hpp"
#include "mpl_rclcpp_utils/Parameter.hpp"
#include "mpl_rclcpp_utils/PollingSubscriber.hpp"
#include "project_utils/msg/control.hpp"
#include "project_utils/msg/control_horizon.hpp"
#include "project_utils/msg/longitudinal.hpp"
#include "project_utils/msg/trajectory.hpp"

#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
using project_utils::msg::Float64Stamped;

namespace mpl::control
{
    using trajectory_follower::LateralHorizon;
    using trajectory_follower::LateralOutput;
    using trajectory_follower::LongitudinalHorizon;
    using trajectory_follower::LongitudinalOutput;
    namespace trajectory_follower_node
    {
        using project_utils::msg::ControlHorizon;
        namespace trajectory_follower = ::mpl::control::trajectory_follower;
        namespace mpl_utils = ::mpl_rclcpp_utils;
        //\classController
        /*
        * \brief The node class used for generating longitudinal control commands (velocity/acceleration) and lateral control commands
        */
        class Controller : public rclcpp::Node
        {
            public: 
                // Constructor
                explicit Controller(const rclcpp::NodeOptions& nodeOptions );
                // Destructor
                virtual ~Controller() {};
            private:
                // compute control command, publish periodically
                // compute the input data for the controller to work on.
                std::optional<trajectory_follower::InputData> createInputData(rclcpp::Clock& clock);
                // Call the controller
                void callbackTimerControl();
                // Check if the node has all the data to run the controller, if not skip the call to compute the control
                // and print the log message
                bool processData(rclcpp::Clock& clock);
                // Returns true if the computed control outputs are stale.
                bool isTimeOut(const trajectory_follower::LongitudinalOutput& lonOut, const trajectory_follower::LateralOutput& latOut);
                //LateralControllerMode getLateralControllerMode(const std::string& algoName) const;
                //LongitudinalControllerMode getLongitudinalControllerMode(const std::string& algoName) const;
                void publishDebugMarker([[maybe_unused]] const trajectory_follower::InputData& inputData,
                                        [[maybe_unused]] const trajectory_follower::LateralOutput& latOut, 
                                        [[maybe_unused]] const trajectory_follower::LongitudinalOutput& longOut) const;
                /**
                * @brief merge lateral and longitudinal horizons
                * @details If one of the commands has only one control, repeat the control to match the other
                *          horizon. If each horizon has different time intervals, resample them to match the size
                *          with the greatest common divisor.
                * @param lateral_horizon lateral horizon
                * @param longitudinal_horizon longitudinal horizon
                * @param stamp stamp
                * @return merged control horizon
                */
                static std::optional<ControlHorizon> mergeLatLonHorizon(
                    const LateralHorizon& lateralHorizon, const LongitudinalHorizon& longitudinalHorizon,
                    const rclcpp::Time& timeStamp);
                

        
                // Publish the processing time
                void publishProcessingTime(const double tMs , 
                    [[maybe_unused]] const rclcpp::Publisher<Float64Stamped>::SharedPtr pub);
                
            private:
                rclcpp::TimerBase::SharedPtr mTimerControl;
                double mTimeoutThrSec;
                double mCyclicMessageTimeoutThrSec;
                bool mEnableControlCmdHorizonPub{false};
                std::optional<LongitudinalOutput> mLongitudinalOutput{std::nullopt};
                std::shared_ptr<trajectory_follower::LongitudinalControllerBase> mLongitudinalController;
                std::shared_ptr<trajectory_follower::LateralControllerBase> mLateralController;

                // Subscriber
                mpl_utils::InternalProcessPollingSubscriber<project_utils::msg::Trajectory> mSubRefPath{this,"~/input/reference_trajectory"};
                mpl_utils::InternalProcessPollingSubscriber<nav_msgs::msg::Odometry> mSubOdometry{this, "~/input/current_odometry"};
                
                mpl_utils::InternalProcessPollingSubscriber<project_utils::msg::SteeringReport> mSubSteering{this, "~/input/current_steering"};
                mpl_utils::InternalProcessPollingSubscriber<geometry_msgs::msg::AccelWithCovarianceStamped> mSubAccel{this, "~/input/current_accel"};

              
                // Publishers
                rclcpp::Publisher<project_utils::msg::Control>::SharedPtr mControlCmdPub;
                rclcpp::Publisher<Float64Stamped>::SharedPtr mPubProcessingTimeLatMs;
                rclcpp::Publisher<Float64Stamped>::SharedPtr mPubProcessingTimeLonMs ; 
                rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mDebugMarkerPub ;
                rclcpp::Publisher<project_utils::msg::ControlHorizon>::SharedPtr mControlCmdHorizonPub;

                project_utils::msg::Trajectory::ConstSharedPtr  mCurrentTrajectoryPtr; 
                nav_msgs::msg::Odometry::ConstSharedPtr mCurrentOdometryPtr;
                project_utils::msg::SteeringReport::ConstSharedPtr mCurrentSteeringPtr; 
                geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr mCurrentAccelPtr; 
       
       
                static constexpr double loggerThrottleInterval  = 5000; //in ms -> 5 sec
       
        };



    }//namespace trajectory_follower_node

}//namespace mpl::control