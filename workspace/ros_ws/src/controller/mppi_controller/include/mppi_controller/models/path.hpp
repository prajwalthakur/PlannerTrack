#pragma once
#include "mppi_controller/utils/types.hpp"
#include "mppi_controller/utils/pose2d.hpp"

namespace controller::mppi_controller::models
{
    struct Path
    {
        virtual ~Path() = default;

        virtual void reset(size_t size)
        {
            x.setZero(size);
            y.setZero(size);
            yaws.setZero(size);
            path_pose_2d.resize(size);
            path_integrated_distances.resize(size);
            path_pts_valid.reset();
            geom_path_initialized = false;
        }
        void computePathGeometry();

        inline void setGeometricPath()
        {
            computePathGeometry();
            geom_path_initialized = true;
        }
        std::shared_ptr<std_msgs::msg::Header> mHeader{nullptr};
        mppi_mt::ArrayX x;
        mppi_mt::ArrayX y;
        mppi_mt::ArrayX yaws;

        std::vector<mppi_utils::Pose2D> path_pose_2d;
        std::vector<float> path_integrated_distances;
        std::optional<std::vector<bool>> path_pts_valid{std::nullopt};
        bool geom_path_initialized{false};
    };

    /**
     * @brief compute the geometric path
    */
    inline void Path::computePathGeometry() // Removed the pointer argument
    {
        size_t N = static_cast<size_t>(this->x.rows());
        if(N == 0) return;

        // Direct access to member variables
        path_pose_2d.resize(N);
        path_integrated_distances.resize(N, 0.0f);

        // Initialize the first pose (Index 0)
        path_pose_2d[0].x = x(0);
        path_pose_2d[0].y = y(0);
        path_pose_2d[0].theta = yaws(0);
        path_integrated_distances[0] = 0.0f;

        for (size_t i = 1; i < N; i++) 
        {
            // Populate the pose vector so it matches the arrays
            path_pose_2d[i].x = x(i);
            path_pose_2d[i].y = y(i);
            path_pose_2d[i].theta = yaws(i);

            float dx = x(i) - x(i - 1);
            float dy = y(i) - y(i - 1);

            path_integrated_distances[i] =
                path_integrated_distances[i - 1] + sqrtf(dx * dx + dy * dy);
        }
    }

}

namespace models = controller::mppi_controller::models;