#include "mppi_controller/critics/common/path_align_critic.hpp"
#include "mppi_controller/utils/utils.hpp"

/** \file
 * \brief \c mppi_critic::ObstacleCritic "ObstacleCritic" implementation.
 * \warning Not part of the build (see the class doc comment) -- and even
 * includes the wrong header (`path_align_critic.hpp`, which does not
 * declare `ObstacleCritic`), so this file would not compile as-is if added
 * to `CMakeLists.txt`.
 */

void mppi_critic::ObstacleCritic::initialize()
{
    auto getParam = mParameter->getParamGetter(mName);
    getParam(mPower, "cost_power", 1);
    getParam(mWeight, "cost_weight", 10.0f);
    getParam(mThresholdToConsider, "threshold_to_consider", 0.5f);
    getParam(mOffsetFromFurthest, "offset_from_furthest", 20);
    getParam(mTrajectoriesPointStep,"trajectory_point_step",4);
    getParam(mMaxPathOccupancyRatio, "max_path_occupancy_ratio", 0.07f);
    getParam(mUsePathOrientation, "use_path_orientations", false);
    mLogger.info("pathAlign Critic instantiated with %d power and %f weight and %f angular threshold %s",mPower, mWeight, mThresholdToConsider,  mUsePathOrientation ? "enabled" : "disabled");

}

void mppi_critic::ObstacleCritic::score(CriticData& data)
{

    if(!mEnabled || data.model->state()->local_path_length < mThresholdToConsider)
        return;

    mppi_utils::setPathFurthestPointIfNotSet(data);

    const size_t pathSegmentsCount = data.furthest_reached_path_point.value();

    if(pathSegmentsCount < mOffsetFromFurthest)
    {
        return;
    }
    
    // Don't apply when dynamic obstacles are blocking significant proportions of the local path
    mppi_utils::setPathCostsIfNotSet(data, mCostmapRos);
    [[maybe_unused]] std::vector<bool>& pathPtsValid = data.path_pts_valid.value();
    
    float pathSegmentsFlt = static_cast<float>(pathSegmentsCount);
    if(data.invalid_pts_count/pathSegmentsFlt > mMaxPathOccupancyRatio && data.invalid_pts_count > 2.0f)
        return;


    
    models::Path* path = data.model->path();
    models::Trajectories* trajectories = data.model->trajectories();
    const size_t batchSize = trajectories->x.rows();
    if(!path->geom_path_initialized)
        path->setGeometricPath();
    

    int stridedTrajRows = trajectories->x.rows();
    int stridedTrajCols = floor((trajectories->x.cols() - 1) / mTrajectoriesPointStep) + 1;
    int outerStride     = stridedTrajRows * mTrajectoriesPointStep;
    
    const auto T_x = Eigen::Map<const mppi_mt::ArrayXX, 0,
        Eigen::Stride<-1, -1>>(
        trajectories->x.data(),
        stridedTrajRows, stridedTrajCols, Eigen::Stride<-1, -1>(outerStride, 1));
    
    const auto T_y = Eigen::Map<const mppi_mt::ArrayXX, 0,
        Eigen::Stride<-1, -1>>(
        trajectories->y.data(),
        stridedTrajRows, stridedTrajCols, Eigen::Stride<-1, -1>(outerStride, 1));
    
    const auto T_yaw = Eigen::Map<const Eigen::ArrayXXf, 0,
        Eigen::Stride<-1, -1>>(
        trajectories->yaws.data(), stridedTrajRows, stridedTrajCols,
        Eigen::Stride<-1, -1>(outerStride, 1));

    const auto trajSampledSize = T_x.cols();
    



    float  summedPathDist = 0.0f , dYaw = 0.0f;
    size_t numSamples = 0u;
    size_t pathPt = 0u;
    float trajIntegratedDistance = 0.0f;
    float dx = 0.0f, dy = 0.0f;
    mppi_mt::ArrayX cost(data.costs.rows());
    cost.setZero();
    
    const auto& path_integrated_distances = path->path_integrated_distances;
    const auto&  path_pts_valid =path->path_pts_valid;
    const auto& path_pose_2d = path->path_pose_2d;

    for(size_t batchIter = 0 ; batchIter < batchSize ; ++ batchIter)
    {
        summedPathDist=0.0f;
        numSamples = 0.0f;
        trajIntegratedDistance=0.0f;
        pathPt = 0u;
        for(int j = 1 ; j<trajSampledSize; ++j )
        {
            dx = T_x(batchIter,j) - T_x(batchIter,j-1);
            dy = T_y(batchIter,j) - T_y(batchIter,j-1);
            trajIntegratedDistance+= sqrtf(dx*dx + dy*dy);
            pathPt = mppi_utils::findClosestPathPt(path_integrated_distances, trajIntegratedDistance, pathPt);
            // The nearest path point to align to needs to be not in collision, else
            // let the obstacle critic take over in this region due to dynamic obstacles
            if(path_pts_valid[pathPt])
            {
                const auto & pose = path_pose_2d[pathPt];
                dx = pose.x - T_x(batchIter,j);
                dy = pose.y - T_y(batchIter,j);
                numSamples++;
                if(mUsePathOrientation)
                {
                    dYaw = mppi_utils::shortestAngularDistance(pose.theta, T_yaw(batchIter, j));
                    summedPathDist+=sqrtf(dx * dx + dy * dy + dYaw * dYaw);

                }
                else
                {
                    summedPathDist+=sqrtf(dx * dx + dy * dy + dYaw * dYaw);
                }
            }
        }
        if(numSamples>0u)
        {
            cost(batchIter) = summedPathDist/(static_cast<float>(numSamples));
        }
        else
        {
            cost(batchIter) = 0.0f;
        }
    }

    if (mPower > 1u)
    {
        data.costs += (cost * mWeight).pow(mPower).eval();
    } else 
    {
        data.costs += (cost * mWeight).eval();
    }

}