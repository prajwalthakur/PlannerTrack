#pragma once

// #include "trajectory_follower_base/lateral_controller_base.h"
#include "trajectory_follower_base/trajectory_follower_base_collection.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "project_utils/msg/longitudinal.hpp"


#include "project_utils/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <optional>
#include <memory>
#include <vector>


using mpl::control::trajectory_follower::InputData;
using mpl::control::trajectory_follower::LongitudinalControllerBase;
using mpl::control::trajectory_follower::LongitudinalOutput;
using project_utils::msg::Longitudinal;
using project_utils::msg::Trajectory;
using project_utils::msg::TrajectoryPoint;


namespace dummy_longitudinal_controller
{

    class DummyLongitudinalController : public LongitudinalControllerBase
    {

        public:
            /// \param node Reference to the node used only for the component and parameter initialization.
            explicit DummyLongitudinalController([[maybe_unused]] rclcpp::Node & node);
        private:
            /**
            * @brief compute control command for path follow with a constant control period
            */
            bool isReady([[maybe_unused]] const InputData & input_data) override;

            LongitudinalOutput run(const InputData & input_data) override;
        private:
            rclcpp::Clock::SharedPtr mClock;

        
    };

}//namespace dummy_longitudinal_controller