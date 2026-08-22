#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
namespace controller::mppi_controller::critic
{
    /**
    * @brief Penalizes rollout points that land on occupied/high-cost
    * costmap cells, using either the robot's full footprint or a simpler
    * circular radius check depending on \ref mConsiderFootprint. This is
    * the critic that actually keeps rollouts out of obstacles.
    */
    class CostCritic: public CriticFunction
    {
        public:
            CostCritic()=default;
            ~CostCritic() override =default;
            void initialize() override;
            /**
            * @brief Accumulate a cost for each rollout point that lands on
            * an occupied/high-cost costmap cell (collision or near-collision).
            *
            * @param data [in,out] critic data; collision cost is added to `data.costs`
            */
            void score(CriticData& data) override;

        protected:
            /**
                * @brief Checks if cost represents a collision
                * @param cost Point cost at pose center
                * @param x X of pose
                * @param y Y of pose
                * @param theta theta of pose
                * @return bool if in collision
                */
            bool inCollision(float cost, float x, float y, float theta);
        
            /**
                * @brief Find the min cost of the inflation decay function for which the robot MAY be
                * in collision in any orientation
                * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
                * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
                * since some element of the robot could be in collision
            */
            float findCircumscribedCost(std::shared_ptr<CostMapRos> costmapRos);

            /**
            * @brief An implementation of worldToMap fully using floats
            * @param wx Float world X coord
            * @param wy Float world Y coord
            * @param mx unsigned int map X coord
            * @param my unsigned into map Y coord
            * @return if successful
            */
            bool worldToMapFloat(float wx, float wy, unsigned int& mx, unsigned int& my);

            /**
                * @brief A local implementation of getIndex
                * @param mx unsigned int map X coord
                * @param my unsigned into map Y coord
                * @return Index (row major indexing)
            */
            unsigned int getIndex(unsigned int mx, unsigned int my) const;
        protected:
            
            

            CollisionChecker mCollisionChecker{nullptr};
            
            float mWeight{0};
            float mPossibleCollisionCheck{0.0f};
            float mPossibleCollisionCost{0.0f};
            float mCircumscribedRadius{0.0f};
            float mCircumscribedCost{0.0f};
            float mCollisionCost{0.0f};
            float mCriticalCost{0.0f};
            float mNearGoalDistance{0.0};
            float mOriginX, mOriginY, mResolution;

            bool mConsiderFootprint{true};
            bool mIsTrackingUnknown{true};

            unsigned int mPower{0};
            unsigned int mNearCollisionCost{253};
            unsigned int mTrajectoriesPointStep;
            unsigned int mSizeX, mSizeY;
            
            
            
            std::string mInflationLayerName;
    };

}//namespace controller::mppi_controller

