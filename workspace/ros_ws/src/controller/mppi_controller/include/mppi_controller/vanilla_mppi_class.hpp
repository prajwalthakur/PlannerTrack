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
    /**
     * \brief The MPPI (Model Predictive Path Integral) trajectory optimizer:
     * samples a batch of noised control-sequence rollouts through \ref
     * models::Model "Model", scores each against the active \ref
     * CriticsManager critics, and updates the nominal control sequence as
     * their cost-weighted average.
     *
     * One \ref computeControl call = one iteration of: \ref prepare
     * (bookkeeping) -> \ref optimize (\ref generateNoisedTrajectory ->
     * \ref updateStateVelocities -> \ref integrateStateVelocities ->
     * \ref evalTrajectoriesScores -> \ref updateControlSequence) ->
     * \ref getControlCommand.
     */
    class Optimizer
    {
        public:
            using CostmapRos = nav2_costmap_2d::Costmap2DROS;
            using CostMap =  nav2_costmap_2d::Costmap2D;

            Optimizer() = default;

            //Optimizer(std::shared_ptr<controller::mppi_controller::Parameters> mppi_parameters, const std::string& name);

            //Optimizer(std::shared_ptr<controller::mppi_controller::Parameters> mppi_parameters, std::shared_ptr<CostmapRos> costMapRos, const std::string& name);

            Optimizer(std::shared_ptr<controller::mppi_controller::Parameters> mppi_parameters, std::shared_ptr<CostmapRos> costMapRos, Logger logger, const std::string& name, const rclcpp::Clock::SharedPtr& clock);

            ~Optimizer()=default;

            /// \brief Load \ref mOptimizerSettings from ROS params and construct the model/critics manager.
            void onConfigure();

            /// \brief Run one full optimization cycle against \p CostMap and return the resulting control command.
            controller::mppi_controller::OutputData computeControl(InputData& ,  CostMap* );
            /// \brief Overload of \ref computeControl using the costmap already owned by \ref mCostMapRosPtr.
            controller::mppi_controller::OutputData computeControl(InputData& );

            /// \brief Per-cycle bookkeeping before \ref optimize -- goal/costmap-lock setup, offset/shift of the previous control sequence.
            void prepare(const InputData& );

            /// \brief Sample a batch of randomly-perturbed control sequences around the nominal one.
            void generateNoisedTrajectory();
            /// \brief Roll each noised control sequence through \ref models::Model "Model" to get per-rollout state velocities.
            void updateStateVelocities();
            /// \brief Integrate the rolled-out velocities into full state trajectories (pose over time) for each rollout.
            void integrateStateVelocities();
            /// \brief Score every rollout trajectory against the active critics via \ref mCriticsManager.
            void evalTrajectoriesScores();
            /// \brief Update the nominal control sequence as the cost-weighted average of this cycle's rollouts.
            void updateControlSequence();
            /// \brief Get the optimized (nominal) control sequence as a plain array.
            mppi_mt::ArrayXX getOptimizedTrajectory();

            // Accessors for post-optimization visualization data
            /// \brief All rollout trajectories from the last \ref optimize call, for visualization (or `nullptr` before the model is configured).
            const models::Trajectories * getTrajectories() const { return mModel ? mModel->trajectories() : nullptr; }
            /// \brief Per-rollout costs from the last \ref evalTrajectoriesScores call.
            const mppi_mt::ArrayX & getCosts() const { return mCosts; }
            /// \brief Per-rollout in-collision flags from the last critic evaluation.
            const std::vector<bool> & getCollisionFlags() const { return mCriticsData.trajectories_in_collision; }


        private:
            /// \brief Construct \ref mModel and \ref mCriticsManager from \ref mOptimizerSettings (no input data needed yet).
            void configure();
            /// \brief Finish configuration once the first \ref InputData is available (e.g. model dimensions that depend on it).
            void configure(const InputData& inputData);
            /// \brief Shared body of both \ref computeControl overloads: \ref prepare, \ref optimize, then \ref getControlCommand.
            void computeControlInternal(InputData& , controller::mppi_controller::OutputData& );
            /// \brief Run the sample/score/update loop (\ref generateNoisedTrajectory through \ref updateControlSequence), retrying with relaxed constraints on total failure.
            void optimize();
            /// \brief Convert the current control-sequence head into a \c geometry_msgs::msg::Twist.
            void calculateControlFromSequenceAsTwist();
            /// \brief Set the linear/angular velocity offset applied when shifting the control sequence.
            void setOffset(float, float );
            /// \brief Shift the control sequence one step forward in time (drop the executed step, extend the tail), for warm-starting the next cycle.
            void shiftControlSequence();
            /// \brief Extract this cycle's commanded twist from the (possibly just-shifted) control sequence.
            geometry_msgs::msg::TwistStamped getControlCommand();
            /// \brief Reset the nominal control sequence and rollout buffers (e.g. on a new goal/replan).
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
