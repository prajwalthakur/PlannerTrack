#pragma once 
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "mppi_controller/utils/nav2_utils/path_handler.hpp"
#include "mppi_controller/utils/nav2_utils/path_utils.hpp"
#include "mppi_controller/utils/geometry_utils.hpp"
#include "mppi_controller/utils/nav2_utils/robot_utils.hpp"
#include "mppi_controller/utils/logger_utils.hpp"
#include "mppi_controller/parameters.hpp"
#include "mppi_controller/utils/ros_namespace.hpp"
#include "mppi_controller/utils/geometry_utils.hpp"
#include "mppi_controller/utils/nav2_utils/path_utils.hpp"
#include "mppi_controller/utils/nav2_utils/robot_utils.hpp"

namespace controller::mppi_controller::utils
{

    /**
    * @class FeasiblePathHandler
    * @brief This plugin manages the global plan by clipping it to the local
    * segment, typically bounded by the local costmap size
    * and transforming the resulting path into the odom frame.
    */
    class FeasiblePathHandler : public mppi_utils::PathHandler
    {

        public:

            /**
             * @brief Construct a new Feasible Path Handler object
            */
            FeasiblePathHandler() = default;

            /**
             * @brief Destroy the Feasible Path Handler object
             */
            ~FeasiblePathHandler() =default;
            /**
             * @brief Initialize parameters
             * @param parent Lifecycle node pointer
             * @param logger Node logging interface
             * @param plugin_name Name of the plugin
             * @param costmap_ros Costmap2DROS object
             * @param tf Shared ptr of TF2 buffer
             */
             void initialize(Parameters* parameter, 
                Logger& logger, 
                const std::shared_ptr<CostMapRos> costMapRos,
                const std::string& parentName,
                const std::string& name,
                std::shared_ptr<tf2_ros::Buffer> tf) override;
            
             /**
             * @brief Set new reference plan
             * @param Path Path to use
             */
            void setPlan(const nav_msgs::msg::Path & path) override;
            /**
             * @brief Find the closest point to the robot on the (possibly looping) racing
             * plan and update mGlobalPlan to a fixed-size lookahead window starting there,
             * wrapping back to the start of the lap once the robot passes the last point.
             * Operates on its own padded buffer (mGlobalPlanPadded), independent of
             * mGlobalPlanUpToConstraint/findPlanSegment/transformLocalPlan, so it isn't
             * affected by their front-pruning.
             * @param pose Robot pose in odom frame
             * @return The lookahead window transformed into the costmap global frame,
             *         ready to use as the plan to follow.
             */
            nav_msgs::msg::Path setPathForRacing(const geometry_msgs::msg::PoseStamped & pose);
            /**
             * @brief Determines the portion of the global plan to be used for local control.
             * This function locates the start and end iterators of the global plan segment
             * that is relevant for controller computation based on the robot's current pose and local costmap.
             * @param pose Robot pose in odom frame
             * @return PathSegment A pair of iterators defining the start and end of the
             *         selected plan segment.
             */
            PathSegment findPlanSegment(
                const geometry_msgs::msg::PoseStamped & pose) override;           
            /**
            * @brief Transforms a predefined segment of the global plan into the costmap global frame.
            * @param closest_point Iterator to the starting pose of the path segment.
            * @param pruned_plan_end Iterator to the ending pose of the path segment.
            * @return nav_msgs::msg::Path The transformed local plan segment in the costmap global frame.
            */
            nav_msgs::msg::Path transformLocalPlan(
                const PathIterator & closest_point,
                const PathIterator & pruned_plan_end) override;
            /**
             * @brief Get the global goal pose transformed to the costmap global frame
             * @param stamp Time to get the goal pose at
             * @return Transformed goal pose
             */
            geometry_msgs::msg::PoseStamped getTransformedGoal(
                const builtin_interfaces::msg::Time & stamp) override;

            /**
             * @brief Gets the global plan
             * @return The global plan
             */
            nav_msgs::msg::Path getPlan() {return mGlobalPlan;}   
        protected:
            /**
            * @brief Transform a pose to the global reference frame
            * @param pose Current pose
            * @return output poose in global reference frame
            */
            geometry_msgs::msg::PoseStamped transformToGlobalPlanFrame(
                const geometry_msgs::msg::PoseStamped & pose);

            /**
             * @brief Transform a [begin, end) segment of a plan into the costmap global
             * frame, stopping early if a pose falls outside the costmap bounds. Shared by
             * transformLocalPlan() and setPathForRacing().
             * @param begin Iterator to the starting pose of the path segment.
             * @param end Iterator to the ending pose of the path segment.
             * @return nav_msgs::msg::Path The transformed segment in the costmap global frame.
             */
            nav_msgs::msg::Path transformPlanToCostmapFrame(
                const PathIterator & begin, const PathIterator & end);

            /**
             * Get the greatest extent of the costmap in meters from the center.
             * @return max of distance from center in meters to edge of costmap
             */
            double getCostmapMaxExtent() const;
            /**
            * @brief Check if the robot pose is within the set inversion tolerances
            * @param robot_pose Robot's current pose to check
            * @return bool If the robot pose is within the set inversion tolerances
            */
            bool isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose);
            /**
            * @brief Prune a path to only interesting portions
            * @param plan Plan to prune
            * @param end Final path iterator
            */
            void prunePlan(nav_msgs::msg::Path & plan, const PathIterator end);

            // skipping the following function, check nav2 #TODO:
            // validateParameterUpdatesCallback()
            // updateParametersCallback()
        private:
            Parameters* mParameters{nullptr};
            Logger mLogger;

            std::shared_ptr<CostMapRos> mCostmapRos{nullptr};

            std::string mParentName;
            std::string mName;

            double mTransformTolerance{0.0};

            bool mRejectUnitPath{false};
            double mMaxRobotPoseSearchDist{0.0};
            double mPruneDistance{0.0};

            bool mEnforcePathInversion{false};
            bool mEnforcePathRotation{false};

            double mInversionXYTolerance{0.0};
            double mInversionYawTolerance{0.0};

            double mMinimumRotationAngle{0.0};

            nav_msgs::msg::Path mGlobalPlan;
            nav_msgs::msg::Path mGlobalPlanUpToConstraint;
            nav_msgs::msg::Path mGlobalPlanOriginal;
            geometry_msgs::msg::PoseStamped mGlobalPose;

            unsigned int mConstraintLocale{0u};

            // Racing/looping-lap support (setPathForRacing()): mGlobalPlanPadded is its
            // own copy of mGlobalPlanOriginal, padded once per new plan with a copy of its
            // own first mLookAheadNumPoints poses appended to the end, so a fixed-size
            // search window starting anywhere in [0, lap size) never runs past the end of
            // the vector. It is kept separate from mGlobalPlanUpToConstraint, which
            // findPlanSegment()/transformLocalPlan() prune from the front every cycle --
            // sharing a single vector between the two caused the circular window to run
            // out and stall once that pruning caught up with the padding. mPrevClosestPoint
            // is always kept folded back into [0, lap size) between calls.
            nav_msgs::msg::Path mGlobalPlanPadded;
            bool mRacingPathPadded{false};
            size_t mPrevClosestPoint{0};
            size_t mLookAheadNumPoints{20};

            std::shared_ptr<tf2_ros::Buffer> mTfBuffer{nullptr};

    };




}//controller::mppi_controller::utils;

namespace mppi_utils = controller::mppi_controller::utils;