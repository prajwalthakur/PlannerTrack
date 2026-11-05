/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/dynamic_model/dynamic_model.hpp"
#include "project_utils/main.hpp"

#include <Eigen/Dense>

#include <array>

#include <nav_msgs/msg/odometry.hpp>
#include <project_utils_msgs/msg/steering_report.hpp>

#include <yaml-cpp/yaml.h>

//////////////////////////////////////////////////////////////////////////
/**
 * \class SingleTrackAugDynModel
 * \brief \ref SingleTrackDynStateModel augmented with the longitudinal
 * acceleration as a state, driven by jerk.
 *
 * Same physics as \ref SingleTrackDynStateModel (load-normalized,
 * friction-limited tire model, longitudinal load transfer, kinematic blend
 * at low speed); the only change is that the acceleration is integrated
 * from a jerk input instead of being commanded directly. This makes the
 * jerk weight and jerk penalty of the Diff-WMPC cost meaningful
 * (project_docs/learning_based_controls/steps.md, §1.3).
 *
 * \verbatim
      State & Input
   x = [x, y, yaw, vx, vy, yaw_rate, steer, acc]^T      (numStates = 8)
   u = [jerk, sv]^T                                     (numControlInputs = 2)

      Dynamics: as SingleTrackDynStateModel with acc taken from the state, plus
   acc_dot = jerk

      Config (dynamics_params): the keys of SingleTrackDynStateModel;
   initAcc is the initial acceleration STATE; initJerk (optional, default 0)
   and initSv are the initial inputs.

      Actuator limits (optional): clip_control_in_sim (default false). If true,
   the agent's vehicle_limits (max_jerk, max_steer_rate, max_steer, max_accel,
   max_decel; handed in by VehicleModelFactory as dynamics_params.vehicle_limits)
   are enforced in simulation:
     jerk  -> clip to [-max_jerk, max_jerk]
     sv    -> clip to [-max_steer_rate, max_steer_rate]
     steer -> end stop at [-max_steer, max_steer]
     acc   -> end stop at [-max_decel, max_accel]
   End stop: the rate is zero while the state sits at a limit and pushes
   outward (in xdot), and the state is projected back inside the limits after
   every RK4 sub-step (RK4 overshoots the stop by up to one sub-step otherwise).

      Command mode (optional): command_mode = jerk (default) | acceleration.
   acceleration: u[0] is an acceleration command a_cmd, tracked by a
   first-order drive actuator with time constant accelTimeConstant [s]:
     jerk = (a_cmd - acc) / accelTimeConstant
   The rest (jerk clipping, end stops, physics) is unchanged; with
   clip_control_in_sim, a_cmd is clipped to [-max_decel, max_accel] first.
   This lets acceleration-commanded controllers (pid_controller, via
   \ref packAccelSteerRate) drive the plant; initJerk is ignored and the
   initial command is initAcc (zero jerk).
 * \endverbatim
 *
 * \note \ref packAccelSteerRate throws in jerk mode: an acceleration command
 * cannot be turned into a jerk without the simulator's current acceleration.
 * Set command_mode: acceleration for acceleration-commanded ROS controllers
 * (e.g. pid_controller).
 */

/// \brief State of \ref SingleTrackAugDynModel.
struct AugStateStruct
{
	double x;
	double y;
	double z{0.0};
	double yaw;
	double vx;
	double vy;
	double yaw_rate;
	double steer;
	double acc;
};

//////////////////////////////////////////////////////////////////////////

/// \brief Input of \ref SingleTrackAugDynModel: jerk and steering rate.
struct AugInputStruct
{
	double jerk;
	double sv;
};

//////////////////////////////////////////////////////////////////////////

class SingleTrackAugDynModel : public DynamicModel
{
	using BaseType = DynamicModel;

  public:
	/// \brief Control (input) layout u = [jerk, steer_rate]: index of each control.
	enum Control : int { kJerk = 0, kSteerRate, kNumControls };
	/// \brief Name of each control, indexed by \ref Control.
	static constexpr std::array<const char *, kNumControls> kControlNames{{"jerk", "steer_rate"}};
	/// \brief State layout x = [x, y, yaw, vx, vy, yaw_rate, steer, acc]: index of each state.
	enum StateIndex : int { kX = 0, kY, kYaw, kVx, kVy, kYawRate, kSteer, kAcc, kNumStateIndices };
	/// \brief Name of each state, indexed by \ref StateIndex.
	static constexpr std::array<const char *, kNumStateIndices> kStateNames{
	    {"x", "y", "yaw", "vx", "vy", "yaw_rate", "steer", "acc"}};
	static constexpr int kNumStates = kNumStateIndices;
	static constexpr int kNumInputs = kNumControls;
	/// \brief Meaning of u[0]: jerk, or an acceleration command (see class doc).
	enum class CommandMode { kJerk, kAcceleration };

	SingleTrackAugDynModel() = default;
	~SingleTrackAugDynModel() = default;

	/// \throws std::invalid_argument if numStates != 8 or numControlInputs != 2, if
	/// clip_control_in_sim is true without valid vehicle_limits, or if command_mode is
	/// unknown / accelTimeConstant is not > 0.
	void initialze(YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId & id) override;
	void step() override;
	void updateCommandedControl(const InputVector & u) override;
	const StateVector & getState() const override;
	void setState(const StateVector &) override;
	void reset() override;
	/// \brief Current commanded input [jerk, sv] (or [a_cmd, sv] in acceleration mode).
	const StateVector & getInput() const;
	/// \brief Set the commanded input directly (bypassing \ref updateCommandedControl).
	void setInput(const InputVector &);
	stPose getStatePose() const override;
	/**
	 * \brief Continuous-time state derivative (see class doc), integrated by \ref step.
	 * With clip_control_in_sim the input is clipped and the steer/acc rates stop at
	 * their end stops, exactly as \ref step applies them.
	 * \param state `[x, y, yaw, vx, vy, yaw_rate, steer, acc]`.
	 * \param input `[jerk, sv]`, or `[a_cmd, sv]` in acceleration mode.
	 * \return State derivative, same layout as \p state.
	 */
	StateVector xdot(const StateVector & state, const InputVector & input) const override;
	void createIntegrator() override;
	void setupRos(const rclcpp::Node::SharedPtr & node, const std::string & ns,
	    const std::string & fixedFrame) override;
	void publishStates() const override;
	/// \brief `[acc, steeringRate]` in acceleration mode.
	/// \throws std::logic_error in jerk mode: the model then takes jerk, not acceleration.
	InputVector packAccelSteerRate(double acc, double steeringRate) const override;

  private:
	/// \brief Apply \ref mInputVector as the currently-commanded input.
	void updateCommandedControl();
	/// \brief Build the numerical integrator from YAML config.
	void createIntegrator(YAML::Node & simConfig, YAML::Node & vehConfig);
	/// \brief Read clip_control_in_sim and, if set, the vehicle_limits.
	void loadActuatorLimits();
	/// \brief Read command_mode and, in acceleration mode, accelTimeConstant.
	void loadCommandMode();
	/// \brief Input clipped to its limits: u[0] to the jerk limit (acceleration mode:
	/// to [-max_decel, max_accel]), sv to the steering-rate limit. Unchanged if clipping is off.
	InputVector clipInput(const InputVector & input) const;
	/// \brief Steer and acc projected into their limits (unchanged if clipping is off).
	void projectToLimits(StateVector & state) const;
	StateVector StateToVector(const AugStateStruct &) const;
	AugStateStruct VectorToState(const StateVector &) const;
	InputVector InputToVector(const AugInputStruct &) const;
	AugInputStruct VectorToInput(const InputVector &) const;

  private:
	int NX{kNumStates};
	int NU{kNumInputs};
	double mInitXPose;
	double mInitYPose;
	double mInitYaw;
	double mInitYawRate;
	double mInitVx;
	double mInitVy;
	double mInitSf;
	double mInitAcc;
	double mInitJerk{0.0};
	double mInitSv;
	double mMass;
	double mIz;
	double mLf;
	double mLr;
	double mCf;
	double mCr;
	double mMu{1.0};
	double mHcg{0.0};
	// Actuator limits (clip_control_in_sim); positive magnitudes.
	bool mClipControl{false};
	double mMaxJerk{0.0};
	double mMaxSteerRate{0.0};
	double mMaxSteer{0.0};
	double mMaxAccel{0.0};
	double mMaxDecel{0.0};
	CommandMode mCommandMode{CommandMode::kJerk};
	double mAccelTimeConstant{0.0};  // [s], acceleration mode only
	double mIntStep{0.0};  // RK4 sub-step [s]
	double mSimStep{0.0};  // one step() [s]
	AugInputStruct mInputStruct;
	AugStateStruct mStateStruct;
	InputVector mInputVector;
	StateVector mStateVector;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomPub;
	rclcpp::Publisher<project_utils_msgs::msg::SteeringReport>::SharedPtr mSteeringPub;
};

//////////////////////////////////////////////////////////////////////////
