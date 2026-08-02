#pragma once
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <memory>
#include "mppi_controller/input_data.hpp"
#include "mppi_controller/utils/types.hpp"
#include "mppi_controller/parameters.hpp"
#include "mppi_controller/output_data.hpp"
#include "mppi_controller/models/model_factory.hpp"
#include "mppi_controller/utils/logger_utils.hpp"
#include "mppi_controller/critics/critic_manager.hpp"
#include "mppi_controller/models/optimizer_settings.hpp"
#include "mppi_controller/critics/critic_data.hpp"
#include "mppi_controller/models/control_sequence.hpp"
namespace controller::mppi_controller
{

    
    namespace models =  ::controller::mppi_controller::models;
    class Model;
    class Optimizer
    {
        public:
            using CostmapRos = nav2_costmap_2d::Costmap2DROS;
            using CostMap =  nav2_costmap_2d::Costmap2D;
    
            // Constructor
            Optimizer() = default;

            //Optimizer(std::shared_ptr<controller::mppi_controller::Parameters> mppi_parameters, const std::string& name);

            //Optimizer(std::shared_ptr<controller::mppi_controller::Parameters> mppi_parameters, std::shared_ptr<CostmapRos> costMapRos, const std::string& name);
            
            Optimizer(std::shared_ptr<controller::mppi_controller::Parameters> mppi_parameters, std::shared_ptr<CostmapRos> costMapRos, Logger logger, const std::string& name, const rclcpp::Clock::SharedPtr& clock);

            // Destructor
            ~Optimizer()=default;

            void onConfigure();
            
            controller::mppi_controller::OutputData computeControl(InputData& ,  CostMap* );
            controller::mppi_controller::OutputData computeControl(InputData& );
            
            void prepare(const InputData& );

            void generateNoisedTrajectory();
            void updateStateVelocities();
            void integrateStateVelocities();
            void evalTrajectoriesScores();
            void updateControlSequence();
            mppi_mt::ArrayXX getOptimizedTrajectory();

            // Accessors for post-optimization visualization data
            const models::Trajectories * getTrajectories() const { return mModel ? mModel->trajectories() : nullptr; }
            const mppi_mt::ArrayX & getCosts() const { return mCosts; }
            const std::vector<bool> & getCollisionFlags() const { return mCriticsData.trajectories_in_collision; }


        private:
            void configure();
            void configure(const InputData& inputData);
            void computeControlInternal(InputData& , controller::mppi_controller::OutputData& );
            void optimize();
            void calculateControlFromSequenceAsTwist();
            void setOffset(float, float );
            void shiftControlSequence();
            geometry_msgs::msg::TwistStamped getControlCommand();
            void reset(bool resetDynamicSpeedLimits=false);
        private:
            // name
            std::string mName;
            // logger
            Logger mLogger;
            // store the costs of the trajectories
            // batch_sizex1
            mppi_mt::ArrayX mCosts;
            // goal 
            geometry_msgs::msg::PoseStamped mGoal;
            // dt
            float mModelDt{0.0f};
            // clock
            rclcpp::Clock::SharedPtr mClock{nullptr};
            // path to follow flag
            bool mPathFollow{true};


            // pointers
            std::shared_ptr<CostmapRos> mCostMapRosPtr{nullptr};
            
            std::shared_ptr<Parameters> mMppiCtrlParamPtr{nullptr};

            std::unique_ptr<models::Model> mModel{nullptr};
            std::unique_ptr<CriticsManager> mCriticsManager{nullptr};

            std::shared_ptr<models::OptimizerSettings> mOptimizerSettings{nullptr};

            // Critic Data
            mppi::CriticData mCriticsData = {nullptr, mGoal, mCosts, mModelDt,false,0,std::nullopt,std::nullopt,{}, nullptr};

    };

} // namespace controller::mppi_controller
