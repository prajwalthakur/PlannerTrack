#include "mppi_controller/critics/common/cost_critic.hpp"
#include "mppi_controller/utils/utils.hpp"

void mppi_critic::CostCritic::initialize()
{
    auto getParam = mParameter->getParamGetter(mName);
    getParam(mConsiderFootprint, "consider_footprint", false);
    getParam(mPower, "cost_power", 1);
    getParam(mWeight, "cost_weight", 10.0f);
    getParam(mCriticalCost, "critical_cost", 300.0f);
    getParam(mNearCollisionCost, "near_collision_cost", 253);
    getParam(mCollisionCost, "collision_cost", 1000000.0f);
    getParam(mNearGoalDistance, "near_goal_distance", 0.5f);
    getParam(mInflationLayerName, "inflation_layer_name", std::string(""));
    getParam(mTrajectoriesPointStep, "trajectory_point_step",2);
    getParam(mEnabled, "enabled", true);
    
    

    // 0: Completely free space.

    // 1–252: Inflated area (the "gradient" around obstacles).

    // 253: Inscribed inflated obstacle (the robot is touching the obstacle).

    // 254: Lethal obstacle (the center of the obstacle).

    // 255: No information (unknown space).
    mWeight /= 254.0f;

    // Normalize the weight when parameter is chaged dynamically

    // TODO(enable this): if using nav2 
    // auto weightDynamicCb = [&](
    //     const rclcpp::Parameter & weight){
    //         mWeight = weight.as_double() / 254.0f;};
    // mParameter->addParamCallback(mName + ".cost_weight", weightDynamicCb);
    

    mCollisionChecker.setCostmap(mCostmap);
    mPossibleCollisionCost = findCircumscribedCost(mCostmapRos);

    if (mPossibleCollisionCost < 1.0f) {
        mLogger.error(
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
    }

    if (mCostmapRos->getUseRadius() == mConsiderFootprint) {
        mLogger.warn(
      "Inconsistent configuration in collision checking. Please verify the robot's shape settings "
      "in both the costmap and the cost critic.");
    if (mCostmapRos->getUseRadius()) {
        mLogger.error("Considering footprint in CostCritic but no robot footprint provided in the "
              "costmap (robot radius used instead). Disable considering footprint.");
        throw std::runtime_error("Considering footprint in CostCritic but no robot footprint provided in the "
              "costmap (robot radius used instead). Disable considering footprint.");
    }
  }

  if (mNearCollisionCost > 253) {
    mLogger.warn( "Near collision cost is set higher than INSCRIBED_INFLATED_OBSTACLE");
  }

  mLogger.info(
    "Cost critic instantiated with %d power and %f / %f weights. "
    "Critic will collision check based on %s cost.",
    mPower, mCriticalCost, mWeight, mConsiderFootprint ?
    "footprint" : "circular");
}

// check if the cost corresponds to be in collision with the obstacle
bool  mppi_critic::CostCritic::inCollision(float cost, float x, float y, float theta)
{
    float scoreCost = cost;
    if(mConsiderFootprint && (cost >= mPossibleCollisionCost || mPossibleCollisionCost < 1.0f))
    {
        scoreCost = static_cast<float>(mCollisionChecker.footprintCostAtPose(
                    static_cast<double>(x), static_cast<double>(y), static_cast<double>(theta),
                    mCostmapRos->getRobotFootprint()));
    }

    switch (static_cast<unsigned char>(scoreCost))
    {
        case (LETHAL_OBSTACLE):
            return true;
        case (INSCRIBED_INFLATED_OBSTACLE):
            return mConsiderFootprint ? false : true;
        case (NO_INFORMATION):
            return mIsTrackingUnknown ? false : true;
    }
    return false;
}


float mppi_critic::CostCritic::findCircumscribedCost(std::shared_ptr<CostMapRos> costmapRos)
{

    double result = -1.0;
    // Radius of smallest circle enclosing robot footprint.
    const double circumRadius = costmapRos->getLayeredCostmap()->getCircumscribedRadius();

    // check if the costmap has an inflation layer
    const auto inflationLayer  = InflationLayer::getInflationLayer(costmapRos,mInflationLayerName);
    

    // if(inflationLayer !=nullptr)
    //     mLogger.info("circum radius %.3f, inflation radius %.3f circumscribedcost %.3f", mCircumscribedRadius, inflationLayer->getInflationRadius(), mCircumscribedCost);


   // mLogger.info("mCircumscribedRadius %.3f, circumRadius radius %.3f", mCircumscribedRadius, circumRadius);

    if (std::abs(static_cast<float>(circumRadius) - mCircumscribedRadius) < 1e-5f) {
        return mCircumscribedCost;
    }

    double inflationRadius = 0.0;
    if(inflationLayer !=nullptr)
    {
        const double resolution = costmapRos->getCostmap()->getResolution();
        inflationRadius = inflationLayer->getInflationRadius();
        if(inflationRadius < circumRadius)
        {
            mLogger.warn("The inflation radius (%f) is smaller than the circumscribed radius (%f) "
                    "If this is an SE2-collision checking plugin, it cannot use costmap potential "
                    "field to speed up collision checking by only checking the full footprint "
                    "when robot is within possibly-inscribed radius of an obstacle. This may "
                    "significantly slow down planning times!",inflationRadius, circumRadius);
            result =0.0f;
            return result;
        }
        // Radius of smallest circle enclosing robot footprint
        result = inflationLayer->computeCost(circumRadius/resolution);
    }
    else
    {
        mLogger.warn(
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times and not avoid anything but absolute collisions!");
    }
    mCircumscribedRadius = static_cast<float>(circumRadius);
    mCircumscribedCost = static_cast<float>(result);
    mLogger.info("circum radius %.3f, inflation radius %.3f circumscribedcost %.3f", mCircumscribedRadius, inflationRadius, mCircumscribedCost);

    return  mCircumscribedCost;

}






bool mppi_critic::CostCritic::worldToMapFloat(float wx, float wy, unsigned int& mx, unsigned int& my)
{
    if(wx < mOriginX || wy < mOriginY)
    {
        return false;
    }
    mx = static_cast<unsigned int>( (wx - mOriginX)/ mResolution);
    my = static_cast<unsigned int>( (wy - mOriginY)/ mResolution);
    if(mx < mSizeX && my < mSizeY)
    {
        return true;
    }
    return false;
}


unsigned int  mppi_critic::CostCritic::getIndex(unsigned int mx, unsigned int my) const
{
    return my*mSizeX + mx;
}




void mppi_critic::CostCritic::score(CriticData& data)
{
    if(!mEnabled){
        return;
    }
    mLogger.info("calculating cost critic");
    auto* costmap = mCollisionChecker.getCostmap();

    // Add this guard
    if (!costmap) {
        mLogger.error("CostCritic: costmap pointer is null! Skipping.");
        return;
    }

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    // Setup cost information for various parts of the critic
    mIsTrackingUnknown = mCostmapRos->getLayeredCostmap()->isTrackingUnknown();
    //mLogger.info("COST CRITIC mIsTrackingUnknown %d",mIsTrackingUnknown);

    mOriginX = static_cast<float>(costmap->getOriginX());
    mOriginY = static_cast<float>(costmap->getOriginY());
    mResolution = static_cast<float>(costmap->getResolution());
    mSizeX = costmap->getSizeInCellsX();
    mSizeY = costmap->getSizeInCellsY();

    //mLogger.info("origin-X %.3f origin-y %.3f size-x %u size-y %u", mOriginX, mOriginY, mSizeX, mSizeY);
    if(mConsiderFootprint){
        // footprint may have changed since initialization if user has dynamic footprints
        mPossibleCollisionCost = findCircumscribedCost(mCostmapRos);
    }

    // If near the goal, don't apply the preferential term since the goal is near obstacles.
    // Preferential : preference to be away fro obstacles.
    bool nearGoal = false;
    if( data.model->state()->local_path_length < mNearGoalDistance){
        nearGoal = true;
    }

    mppi_mt::ArrayX repulsiveCost(data.costs.rows());
    repulsiveCost.setZero();
    bool allTrajectoriesCollide = true;
    
    auto& collisions = data.trajectories_in_collision;
    const bool trackCollisions = !collisions.empty();  

    
    //models::Path* path = data.model->path();
    models::Trajectories* trajectories = data.model->trajectories();
    //const int batchSize = trajectories->x.rows();
    //mLogger.info("batch size %ld numtimesteps %ld",trajectories->x.rows(),trajectories->x.cols());



    int stridedTrajRows = trajectories->x.rows();
    int stridedTrajCols = floor((trajectories->x.cols() - 1) / mTrajectoriesPointStep) + 1;
    int outerStride     = stridedTrajRows * mTrajectoriesPointStep;

    // strided x,y,yaw trajectories
    const auto trajX = Eigen::Map<const mppi_mt::ArrayXX, 0,
        Eigen::Stride<-1, -1>>(
        trajectories->x.data(),
        stridedTrajRows, stridedTrajCols, Eigen::Stride<-1, -1>(outerStride, 1));
    
    const auto trajY = Eigen::Map<const mppi_mt::ArrayXX, 0,
        Eigen::Stride<-1, -1>>(
        trajectories->y.data(),
        stridedTrajRows, stridedTrajCols, Eigen::Stride<-1, -1>(outerStride, 1));
    
    const auto trajYaw = Eigen::Map<const Eigen::ArrayXXf, 0,
        Eigen::Stride<-1, -1>>(
        trajectories->yaws.data(), stridedTrajRows, stridedTrajCols,
        Eigen::Stride<-1, -1>(outerStride, 1));

    for(int i=0; i<stridedTrajRows; ++i){
        bool  trajectoryCollide = false;
        float poseCost = 0.0f;
        float & trajCost = repulsiveCost(i);

        for(int j=0; j< stridedTrajCols; ++j){
            float Tx = trajX(i,j);
            float Ty = trajY(i,j);
            unsigned int xI = 0u, yI = 0u;
            // The getCost doesn't use orientation
            // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
            // So the center point has more information than the footprint
            
            if(!worldToMapFloat(Tx,Ty,xI,yI)){
                // (Tx,Ty) point is outside the costmap region
                poseCost = 255.0f; // No information 
            }else{
                // Center point gives rich gradient
                // depending on the distance from the obstacle cell, we get the costs
                poseCost = static_cast<float>(costmap->getCost(getIndex(xI,yI)));
                if(poseCost <1.0f){
                    continue; // In free space
                }
            }

            if(inCollision(poseCost,Tx,Ty,trajYaw(i,j))) {
                // if any point of the tracjectory is-in collision then give very high penalty, 
                // and no need to check further points in trajectory.
                trajCost = mCollisionCost; 
                trajectoryCollide = true;
                if(trackCollisions) {
                    collisions[i]=true;
                }
                break;
            }
            // Let near-collision trajectory points be punished severely
            // Note that we collision check based on the footprint actual,
            // but score based on the center-point cost regardless   
            // by default mNearCollisionCost is 253, meaning within inscribed_radius (check costamp documentation on ros)
            if(poseCost >= static_cast<float>(mNearCollisionCost)){
                trajCost += mCriticalCost;
            } else if(!nearGoal){ // Generally prefer trajectories further from obstacles
                trajCost +=poseCost;
            }
        }
        allTrajectoriesCollide &=trajectoryCollide;
    }

    if(mPower >1u){
        data.costs += (repulsiveCost*(mWeight/static_cast<float>(stridedTrajCols))).pow(mPower);
    }else{
        data.costs += (repulsiveCost*(mWeight/static_cast<float>(stridedTrajCols)));

    }
    data.fail_flag = allTrajectoriesCollide;
    
}
