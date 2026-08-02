#pragma once 
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include "mppi_controller/utils/ros_namespace.hpp"
#include "mppi_controller/parameters.hpp"
#include "mppi_controller/utils/logger_utils.hpp"
#include "mppi_controller/utils/geometry_utils.hpp"
#include "mppi_controller/utils/types.hpp"
#include "mppi_controller/utils/nav2_utils/path_utils.hpp"
#include "mppi_controller/utils/nav2_utils/robot_utils.hpp"

namespace controller::mppi_controller::utils
{
    using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;
    using PathSegment = std::pair<PathIterator,PathIterator>;

    class PathHandler
    {
        public:
            virtual ~PathHandler()=default;
            /**
             * @brief Initialize parameters
             * @param parent Lifecycle node pointer
             * @param logger Node logging interface
             * @param plugin_name Name of the plugin
             * @param costmap_ros Costmap2DROS object
             * @param tf Shared ptr of TF2 buffer
             */
            virtual void initialize(Parameters* parameter, 
                Logger& , 
                const std::shared_ptr<CostMapRos> ,
                const std::string& ,
                const std::string& ,
                std::shared_ptr<tf2_ros::Buffer> )=0;
            /**
             * @brief Set new reference plan
             * @param Path Path to use
             */
            virtual void setPlan(const nav_msgs::msg::Path& path)=0;
            /**
             * @brief Determines the portion of the global plan to be used for local control.
             * This function locates the start and end iterators of the global plan segment
             * that is relevant for controller computation based on the robot's current pose and local costmap.
             * @param pose Robot pose in odom frame
             * @return PathSegment A pair of iterators defining the start and end of the
             *         selected plan segment.
             */  
            virtual PathSegment findPlanSegment(const geometry_msgs::msg::PoseStamped& pose)=0;
            /**
                * @brief Transforms a predefined segment of the global plan into the costmap global frame.
                * @param closest_point Iterator to the starting pose of the path segment.
                * @param pruned_plan_end Iterator to the ending pose of the path segment.
                * @return nav_msgs::msg::Path The transformed local plan segment in the costmap global frame.
            */
            virtual nav_msgs::msg::Path transformLocalPlan(
                const PathIterator & closest_point,
                const PathIterator & pruned_plan_end) = 0;
            /**
             * @brief Get the global goal pose transformed to the costmap global frame
             * @param stamp Time to get the goal pose at
             * @return Transformed goal pose
             */
            virtual geometry_msgs::msg::PoseStamped getTransformedGoal(
                const builtin_interfaces::msg::Time & stamp) = 0;
};

}//controller::mppi_controller::utils;

namespace mppi_utils = controller::mppi_controller::utils;