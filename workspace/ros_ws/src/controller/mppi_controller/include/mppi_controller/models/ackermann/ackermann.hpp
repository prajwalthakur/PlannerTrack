#pragma once
#include "mppi_controller/models/ackermann/ackermann_include.hpp"
#include "mppi_controller/utils/geometry_utils.hpp"
namespace controller::mppi_controller::models
{
    class AckermannControlConstraints;
    class AckermannSamplingStd;
    class AckermannOptimizerSettings;
    class AckermannPath;
    class AckermannTrajectories;
    class AckermannState;
    class AckermannNoiseGenerator;
    /**
     * \brief \ref Model implementation for an Ackermann-steered (bicycle)
     * vehicle: a `{vx, wz}` control per rollout/timestep, integrated
     * kinematically into `(x, y, yaw)` rollout trajectories.
     *
     * Composes the Ackermann-specific state/trajectories/path/control-
     * sequence/constraints/noise-generator pieces (see the forward
     * declarations above and `ackermann_include.hpp`) behind the abstract
     * \ref Model interface, mirroring the base/plugin split of \ref
     * plugin_architecture (this codebase's only current \ref Model, but
     * kept behind the interface so \c Optimizer stays model-agnostic).
     */
    class AckermannModel : public Model
    {

        public:
            AckermannModel()=default;
            ~AckermannModel() override =default;

            void onConfigure(std::shared_ptr<Parameters> parameters, std::shared_ptr<OptimizerSettings> settings, const std::string& name, Logger logger) final;

            void integrateStateVelocities() final;

            /// \brief Integrate one rollout's `{vx, wz}` \p sequence into an `(x, y, yaw)` \p trajectory via the bicycle kinematics.
            void integrateStateVelocities(mppi_mt::ArrayX3& trajectory, const mppi_mt::ArrayX2& sequence) const;

            void setParameters(Parameters& parameters) final;
            
            void setState(const InputData& inputData) final;
            
            void generateNoisedTrajectory() final;

            void updateStateVelocities() final;

            void updateControlSequence(mppi_mt::ArrayX&) final;
            
            const OptimizerSettings& settings() const override;

            std::string getModelType() const override;
            
            
            void reset() override ;

            models::State* state() const override;

            models::Trajectories* trajectories() const override;

            models::Path* path() const override;

            //models::ControlSequence* controlSequence() const final;

            mppi_mt::ArrayXX getOptimizedTrajectory() final;

            void calculateControlFromSequenceAsTwist() final;

            geometry_msgs::msg::TwistStamped getControlCommand() const final;

            void shiftControlSequence() final;

            /// \brief This vehicle's minimum turning radius (see \ref AckermannControlConstraints).
            float getMinTurningRadius();

            /// \brief Clamp the noised control sequence to \ref AckermannControlConstraints bounds.
            void applyConstraints();


        private:
            /// \brief Construct \ref mControlConstraints / \ref mSamplingStdPtr / \ref mPath / \ref mTrajectories / \ref mState / \ref mNoiseGenerator / \ref mControlSequence from settings.
            void configure(std::shared_ptr<Parameters> parameters, std::shared_ptr<OptimizerSettings> settings);
            /// \brief Convert a \c nav_msgs::msg::Path into \ref mPath's tensor form.
            void toTensor(const nav_msgs::msg::Path& path);
            /// \brief Apply \ref applyConstraints to the current control sequence.
            void applyControlSequenceConstraints();

            /**
            * @brief Apply Savisky-Golay filter to optimal trajectory
            * @param control_sequence Sequence to apply filter to
            * @param control_history Recent set of controls for edge-case handling
            * @param Settings Settings to use
            */
            void savitskyGolayFilter( models::AckermannControlSequence & control_sequence,
                std::array<models::AckermannControl, 4> & control_history,
                const models::OptimizerSettings & settings);
        public:
            
            std::unique_ptr<AckermannControlConstraints> mControlConstraints{nullptr};
            std::unique_ptr<AckermannSamplingStd> mSamplingStdPtr{nullptr};
            std::unique_ptr<AckermannPath> mPath{nullptr};
            std::unique_ptr<AckermannTrajectories> mTrajectories{nullptr};
            std::unique_ptr<AckermannState> mState{nullptr};
            std::unique_ptr<AckermannNoiseGenerator> mNoiseGenerator{nullptr};
            std::unique_ptr<AckermannControlSequence> mControlSequence{nullptr};
            std::shared_ptr<Parameters> mParameters{nullptr};
            std::shared_ptr<OptimizerSettings> mSettings{nullptr};
        private:
            std::shared_ptr<std_msgs::msg::Header> mHeader{nullptr};
            // mppi_mt::ArrayX mCosts;
            geometry_msgs::msg::TwistStamped mLastCmdSpeed;
            int mControlDim{2}; //vx,wz
            Logger mLogger;
            // Control history
            std::array<models::AckermannControl, 4> mControlHistory;


    };
}//namespace controller::mppi_controller::models
namespace models = controller::mppi_controller::models;