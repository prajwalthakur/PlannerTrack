#pragma once
#include <string>
#include <algorithm>
#include <string>
#include <cctype>
#include <limits>
#include "mppi_controller/critics/critic_data.hpp"
#include "mppi_controller/models/path.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/utils/ros_namespace.hpp"
#include "mppi_controller/utils/pose2d.hpp"
#include "mppi_controller/utils/geometry_utils.hpp"
/**
 * \file
 * \brief Path/trajectory helpers used by the critics: furthest-reached
 * path point, per-point costmap validity, and pose/yaw alignment against a
 * target.
 */
namespace controller::mppi_controller::utils
{

    /// \brief Case-insensitive string equality (used by \ref mppi_critics_utils::getCritic "critics_utils::getCritic").
    inline bool compareStringIgnoreCase(const std::string& a, const std::string& b)
    {
        return a.size() == b.size() &&
            std::equal(a.begin(), a.end(), b.begin(),
                [](char c1, char c2)
                {
                    return std::tolower(c1) == std::tolower(c2);
                });
    }

    /**
    * @brief Compare to trajectory points to find closest path point along integrated distances
    * @param vec Vect to check
    * @return dist Distance to look for
    * @return init Starting index to indec from
    */
    inline size_t findClosestPathPt(const std::vector<float>& vec, const float dist, const size_t init = 0u)
    {
        float distm1 = init==0u? 0.0f : vec[init];
        float disti = 0.0f;
        const size_t size = vec.size();
        for(size_t i=init+1; i < size;++i)
        {
            disti = vec[i];
            if(disti > dist)
            {
                // distm1   dist    dist_i
                // dist is closer to which point
                if(i > 0 && dist - distm1  < disti - dist)
                    return i-1;
                return i;
            }
            distm1 = disti;
        }
        return size-1;
    }

    /**
    * @brief Evaluate furthest point idx of data.path which is
    * nearest to some trajectory in data.trajectories
    * @param data Data to use
    * @return Idx of furthest path point reached by a set of trajectories
    */
    inline size_t findPathFurthestReachedPoint(const mppi::CriticData& data)
    {   


        models::Trajectories* trajectories = data.model->trajectories();
        models::Path* path = data.model->path();

        size_t traj_cols = trajectories->x.cols();
        // get the final x,y of all the trajectories
        const auto traj_x = trajectories->x.col(traj_cols-1); //.col() returns a lightweight object that references a column inside the matrix.
        const auto traj_y = trajectories->y.col(traj_cols-1);

        // path to follow
        const auto& path_x = path->x; // x-coordinates for the robot to follow
        const auto& path_y = path->y; // y-coordinates for the robot to follow

        size_t max_id_by_trajectories = 0; // max-id on path reached by the trajectory
         

        size_t n_rows =  traj_x.rows();
        size_t n_cols =  path_x.cols();
        for(size_t i=0; i<n_rows ; ++i)
        {
            const auto dxs = path_x - traj_x(i);
            const auto dys = path_y - traj_y(i);

            Eigen::Index idx;
            [[maybe_unused]] float min_val =  (dxs*dxs + dys*dys).minCoeff(&idx);

            max_id_by_trajectories  = std::max(max_id_by_trajectories, static_cast<size_t>(idx));

            if(max_id_by_trajectories == n_cols-1)
            {
                break;
            }
        }

        return max_id_by_trajectories;
    }

    /**
    * @brief evaluate path costs
    * @param data Data to use
    */
    inline void setPathFurthestPointIfNotSet(mppi::CriticData& data)
    {
        if(!data.furthest_reached_path_point)
        {
            data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
        }
    }
    
    inline void findPathCosts(mppi::CriticData& data, std::shared_ptr<CostMapRos> costmap_ros)
    {
        auto* costmap = costmap_ros->getCostmap();
        unsigned int map_x, map_y;
        models::Path* path = data.model->path();
        const size_t path_segments_count = path->x.size()-1;
        data.path_pts_valid = std::vector<bool>(path_segments_count,false);
        const bool tracking_unkown = costmap_ros->getLayeredCostmap()->isTrackingUnknown();
        auto& valid = data.path_pts_valid.value();
        data.invalid_pts_count = 0;
        for(size_t idx=0; idx < path_segments_count; ++idx)
        {
            if(!costmap->worldToMap(path->x(idx), path->y(idx), map_x, map_y))
            {
                valid[idx] = false;
                
                continue;
            }

            switch(costmap->getCost(map_x,map_y))
            {
                case (LETHAL_OBSTACLE):
                {
                    valid[idx] = false;
                    data.invalid_pts_count++;
                    continue;
                }
                case (INSCRIBED_INFLATED_OBSTACLE):
                {
                    valid[idx] = false;
                    data.invalid_pts_count++;
                    continue;
                }
                case (NO_INFORMATION):
                {
                    valid[idx] = tracking_unkown ? true : false;
                    if(valid[idx]==false)
                        data.invalid_pts_count++;
                    continue;
                }
                default:
                {
                    valid[idx] =  true ;
                    continue;
                }
            }


        }

    }

    /**
    * @brief evaluate path costs if it is not set
    * @param data Data to use
    */
    inline void setPathCostsIfNotSet(mppi::CriticData& data, std::shared_ptr<CostMapRos> costmap_ros)
    {
        if(!data.path_pts_valid)
            findPathCosts(data, costmap_ros);
    }

    /**
    * @brief Select the yaw orientation (θ or θ + π) that is closest to a reference yaw.
    *
    * Each candidate yaw has two valid orientations (forward and reverse).
    * This function picks the one that minimizes angular difference to the reference.
    *
    * @param reference_yaws   Reference yaw angles (e.g., trajectory final yaws).
    * @param candidate_yaws   Candidate yaw angles (e.g., path directions).
    *
    * @return Eigen::ArrayXf  Candidate yaws with chosen orientation (closest to reference).
    */
    inline Eigen::ArrayXf selectClosestYawOrientation(
        const Eigen::Ref<const Eigen::ArrayXf>& reference_yaws,
        const Eigen::Ref<const Eigen::ArrayXf>& candidate_yaws)
    {
        const auto angular_diff =
            mppi_utils::shortestAngularDistance(reference_yaws, candidate_yaws).abs();

        const auto use_original = (angular_diff < M_PIF_2);

        const auto flipped =
            (candidate_yaws + M_PIF).unaryExpr([](float x) {
                return mppi_utils::normalizeAngles(x);
            });

        return use_original.select(candidate_yaws, flipped);
    }

    /**
    * @brief Select the yaw orientation (θ or θ + π) that is closest to a reference yaw.
    *
    * Each candidate yaw has two valid orientations (forward and reverse).
    * This function picks the one that minimizes angular difference to the reference.
    *
    * @param reference_yaw   Reference yaw angle (e.g., goal final yaw).
    * @param candidate_yaws   Candidate yaw angles (e.g., path directions).
    *
    * @return Eigen::ArrayXf  Candidate yaws with chosen orientation (closest to reference).
    */
    inline auto selectClosestYawOrientation(
    const float reference_yaw,
    const Eigen::Ref<const Eigen::ArrayXf> & candidate_yaws)
    {
        const auto angular_diff =
            mppi_utils::shortestAngularDistance(reference_yaw, candidate_yaws).abs();

        const auto use_original = (angular_diff < M_PIF_2);

        const auto flipped =
            (candidate_yaws + M_PIF).unaryExpr([](float x) {
                return mppi_utils::normalizeAngles(x);
            });

        return use_original.select(candidate_yaws, flipped);
    }

    /**
    * @brief Select the yaw orientation (θ or θ + π) that is closest to a reference yaw (Scalar version).
    */
    inline float selectClosestYawOrientation(const float& reference_yaw, const float& candidate_yaw)
    {
        const float angular_diff = std::abs(mppi_utils::shortestAngularDistance(reference_yaw, candidate_yaw));

        if (angular_diff < M_PIF_2) {
            return candidate_yaw;
        }

        return mppi_utils::normalizeAngles(candidate_yaw + M_PIF);
    }
    /**
    * @brief evaluate angle from pose (have angle) to point (no angle)
    * @param pose pose
    * @param point_x Point to find angle relative to X axis
    * @param point_y Point to find angle relative to Y axis
    * @param forward_preference If reversing direction is valid
    * @return Angle between two points
    */
    inline float computePoseToPointAlignmentErrorForward( const geometry_msgs::msg::Pose & pose, float goal_x, float goal_y, bool forward_preference)
    {
            float pose_x = pose.position.x;
            float pose_y = pose.position.y;
            float pose_yaw = tf2::getYaw(pose.orientation);

            float yaw = atan2f(goal_y - pose_y, goal_x - pose_x);
            // If no preference for forward, return smallest angle either in heading or 180 of heading
            if (!forward_preference) 
            {
                return std::min(
                fabs(mppi_utils::shortestAngularDistance(yaw, pose_yaw)),
                fabs(mppi_utils::shortestAngularDistance(yaw, mppi_utils::normalizeAngles(pose_yaw + M_PIF))));
            }

            return fabs(mppi_utils::shortestAngularDistance(yaw, pose_yaw));

    }

    /**
    * @brief Compute heading alignment error between pose and a target point direction.
    *
    * Computes the direction from pose → point, resolves its 180° ambiguity
    * using a reference yaw (pointYaw), and returns the absolute angular
    * difference w.r.t. the pose's current heading.
    *
    * @param pose        Current pose (position + orientation).
    * @param point_x     Target point x-coordinate.
    * @param point_y     Target point y-coordinate.
    * @param pointYaw    Reference yaw to resolve direction ambiguity.
    *
    * @return Absolute angular error (radians).
    */
    inline float computePoseToPointAlignmentError(
        const geometry_msgs::msg::Pose& pose,
        double point_x, double point_y, float pointYaw)
    {
        const float pose_x = pose.position.x;
        const float pose_y = pose.position.y;
        const float pose_yaw = static_cast<float>(tf2::getYaw(pose.orientation));

        const float yaw =
            atan2f(static_cast<float>(point_y) - pose_y,
                static_cast<float>(point_x) - pose_x);

        const float yawCorrected =
            mppi_utils::selectClosestYawOrientation(pointYaw, yaw);

        return std::fabs(
            mppi_utils::shortestAngularDistance(pose_yaw, yawCorrected));
    }

}//namespace controller::mppi_controller::utils
namespace mppi_utils = controller::mppi_controller::utils;