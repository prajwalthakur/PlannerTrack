#include "trajectory_follower_node/controller_node.hpp"

namespace
{
    template<typename T>
    std::vector<T> resampleHorizonByZeroOrderHold(
        const std::vector<T>& origHorizon, const double origTimeStepMs,
        const double newTimeStepMs
    )
    {
        std::vector<T> resampledHorizon{};
        const size_t stepFactor = static_cast<size_t>(origTimeStepMs/newTimeStepMs);
        const size_t resampledSize = origHorizon.size()*stepFactor;
        resampledHorizon.reserve(resampledSize);
        for(const auto& cmd : origHorizon)
        {
            for(size_t i=0; i < stepFactor;++i)
            {
                resampledHorizon.push_back(cmd);
            }
        }
        return resampledHorizon;
    }
} // namespace 

////////////////////////////////////////////////////////////////////////////////

namespace mpl::control::trajectory_follower_node
{
    Controller::Controller(const rclcpp::NodeOptions& nodeOptions):
        Node("controller",nodeOptions)
        {
            using std::placeholders::_1;
            double defaultVal = 0.0;
            const double ctrlPeriodSec = mpl_rclcpp_utils::get_or_declare_parameter<double>(*this,"ctrl_period_sec",defaultVal);
            mTimeoutThrSec = mpl_rclcpp_utils::get_or_declare_parameter<double>(*this,"timeout_thr_sec",defaultVal);
            mCyclicMessageTimeoutThrSec = mpl_rclcpp_utils::get_or_declare_parameter<double>(*this,"cyclic_message_timeout_thr_sec",defaultVal);
            // NOTE: It is possible that using control_horizon could be expected to enhance performance,
            // but it is not a formal interface topic, only an experimental one.
            // So it is disabled by default.
            mEnableControlCmdHorizonPub = mpl_rclcpp_utils::get_or_declare_parameter<bool>(*this,"enable_control_cmd_horizon_pub", false);
            const auto lateralControllerMode = getLateralControllerMode(mpl_rclcpp_utils::get_or_declare_parameter<std::string>(*this,"lateral_controller_mode", "pure_pursuit"));
            switch (lateralControllerMode)
            {
                case LateralControllerMode::MPC :
                {
                    //mLateralController = std::make_shared<mpc_lateral_controller::MpcLateralController>(*this);
                    break;
                }
                case LateralControllerMode::PURE_PURSUIT :
                {
                    //mLateralController =  std::make_shared<pure_pursuit::PurePursuitController>(*this);
                    break;
                }
                case LateralControllerMode::DUMMY_LATERAL:
                {
                    mLateralController = std::make_shared<dummy_lateral_controller::DummyLateralController>(*this);
                    break;
                }
                default:
                {
                    throw std::domain_error("[LateralController] invalid algorithm");
                }
            }
            
            
            const auto longitudinalControllerMode = getLongitudinalControllerMode(mpl_rclcpp_utils::get_or_declare_parameter<std::string>(*this,"longitudinal_controller_mode","pid"));
            switch (longitudinalControllerMode){
                case LongitudinalControllerMode::PID:
                {
                    //mLongitudinalController = std::make_shared<pid_longitudinal_controller::PIDLongController>(*this);
                    break;
                }
                case LongitudinalControllerMode::DUMMY_LONGITUDINAL:
                {
                    mLongitudinalController = std::make_shared<dummy_longitudinal_controller::DummyLongitudinalController>(*this);
                    break;
                }
                default:
                {
                    throw std::domain_error("[LongitudinalController] invalid algorithm");
                }
            }
            mControlCmdPub = create_publisher<project_utils::msg::Control>(
                "~/output/control_cmd",rclcpp::QoS{1}.transient_local());
            mPubProcessingTimeLatMs = create_publisher<Float64Stamped>("~/lateral/debug/processing_time_ms", 1);
            mPubProcessingTimeLonMs = create_publisher<Float64Stamped>("~/longitudinal/debug/processing_time_ms", 1);
            mDebugMarkerPub = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_marker",rclcpp::QoS{1});
            if(mEnableControlCmdHorizonPub)
            {
                mControlCmdHorizonPub = create_publisher<project_utils::msg::ControlHorizon>("~/debug/control_cmd_horizon",1);
            }
            
            // Timer
            {
                // std::ratio<1, 1000>
                const auto preiodNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double, std::ratio<1, 1>>(ctrlPeriodSec));
                        
                mTimerControl = rclcpp::create_timer(
                    this, get_clock(), preiodNs, [this](){this->callbackTimerControl();});
            }

        }
    
    ////////////////////////////////////////////////////////////////////////////////

    bool Controller::processData(rclcpp::Clock& clock)
    {
        bool isReady = true;
        // Logger if the data is not available, print every loggerThrottleInterval ms
        const auto& logData = [&clock, this](const std::string& dataType)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), clock, loggerThrottleInterval, "waiting for %s data",dataType.c_str());
        };

        // try to get data from subscriber buffer
        const auto& getData = [&logData](auto & dest, auto & sub, const std::string dataType ="")
        {
            const auto temp = sub.takeData();
            if(temp)
            {
                dest = temp;
                return true;
            }
            if(!dataType.empty())
            {
                logData(dataType); 
            }
            return false;
        };

        isReady&= getData(mCurrentAccelPtr,mSubAccel,"acceleration");
        isReady&= getData(mCurrentSteeringPtr,mSubSteering,"steering");
        isReady&= getData(mCurrentTrajectoryPtr,mSubRefPath,"trajectory");
        isReady&= getData(mCurrentOdometryPtr, mSubOdometry, "odometry");
        return isReady;
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    bool Controller::isTimeOut(const trajectory_follower::LongitudinalOutput& longOut,
        const trajectory_follower::LateralOutput& latOut)
        {
            const auto now = this->now();
            if((now-latOut.mControlCmd.stamp).seconds() > mTimeoutThrSec)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(), loggerThrottleInterval,
                "Lateral control command too old, control-cmd will not be published.");
                return true;
            }
            if((now-longOut.mControlCmd.stamp).seconds() > mTimeoutThrSec)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(), loggerThrottleInterval,
                "Longitudinal control command too old, control-cmd will not be published.");
                return true;
            }        
        
            return false;
        }

    ////////////////////////////////////////////////////////////////////////////////

    std::optional<trajectory_follower::InputData> Controller::createInputData(rclcpp::Clock& clock)
    {
        if(!processData(clock))
            return std::nullopt;
        trajectory_follower::InputData inputData;
        inputData.mCurrentTrajectory = *mCurrentTrajectoryPtr;
        inputData.mCurrentOdometry = *mCurrentOdometryPtr;
        inputData.mCurrentSteering = *mCurrentSteeringPtr;
        inputData.mCurrentAcc = *mCurrentAccelPtr;
        return inputData;
    }

    ////////////////////////////////////////////////////////////////////////////////

    void Controller::callbackTimerControl()
    {
        project_utils::msg::Control controlCmdOut;
        controlCmdOut.stamp = this->now();

        // 1.Create Input data
        const auto inputData = createInputData(*get_clock());
        if(!inputData)
        {
            RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), loggerThrottleInterval, "Control is skipped since input data is not ready.");
            return;
        }

        //2. check if controllers are ready
        const bool isLatReady = mLateralController->isReady(*inputData);
        const bool isLongReady = mLongitudinalController->isReady(*inputData);
        if(!isLatReady || !isLongReady)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), loggerThrottleInterval,
            "Control is skipped since lateral and/or longitudinal controllers are not ready to run.");
        return;
        }

        //3 . run controllers
        const auto latOut = mLateralController->run(*inputData);
        const auto longOut = mLongitudinalController->run(*inputData);

        //4 . sync with other controller
        //TODO: uderstand
        mLongitudinalController->sync(latOut.mSyncData);
        mLateralController->sync(longOut.mSyncData);

        if(isTimeOut(longOut,latOut)) return;

        // 4.5 Update diagonistics
        //TODO:

        // 5 publish control command
        controlCmdOut.lateral = latOut.mControlCmd;
        controlCmdOut.longitudinal = longOut.mControlCmd;
        mControlCmdPub->publish(controlCmdOut);

        //6. publish debug
        publishDebugMarker(*inputData,latOut,longOut);

        //7. publish experimental topic
        if(mEnableControlCmdHorizonPub)
        {
            const auto controlHorizon = 
                mergeLatLonHorizon(latOut.mControlCmdHorizon,longOut.mControlCmdHorizon, this->now());
            if(controlHorizon.has_value())
            {
                mControlCmdHorizonPub->publish(controlHorizon.value());
            }
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    
    void Controller::publishDebugMarker(
        [[maybe_unused]] const trajectory_follower::InputData& inputData,
        [[maybe_unused]] const trajectory_follower::LateralOutput& latOut,
        [[maybe_unused]] const trajectory_follower::LongitudinalOutput& longOut
    ) const
    {
        // visualization_msgs::msg::MarkerArray debug_marker_array{};
        // {
        //     auto marker = autoware_utils::create_default_marker(
        //     "map", this->now(), "steer_converged", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        //     autoware_utils::create_marker_scale(0.0, 0.0, 1.0),
        //     autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.99));
        //     marker.pose = inputData.mCurrentOdometry.pose.pose;

        //     std::stringstream ss;
        //     const double current = inputData.current_steering.steering_tire_angle;
        //     const double cmd = latOut.mControlCmd.steering_tire_angle;
        //     const double diff = current - cmd;
        //     ss << "current:" << current << " cmd:" << cmd << " diff:" << diff
        //     << (latOut.sync_data.is_steer_converged ? " (converged)" : " (not converged)");
        //     marker.text = ss.str();

        //     debug_marker_array.markers.push_back(marker);
        // }

        // debug_marker_pub_->publish(debug_marker_array);
    }

    ////////////////////////////////////////////////////////////////////////////////

    void Controller::publishProcessingTime(
    const double t_ms, [[maybe_unused]] const rclcpp::Publisher<Float64Stamped>::SharedPtr pub)
    {
        Float64Stamped msg{};
        msg.stamp = this->now();
        msg.data = t_ms;
        //pub->publish(msg);
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    std::optional<ControlHorizon> Controller::mergeLatLonHorizon(
    const LateralHorizon & lateralHorizon, const LongitudinalHorizon & longitudinalHorizon,
    const rclcpp::Time & stamp)
    {
        if (lateralHorizon.mControls.empty() || longitudinalHorizon.mControls.empty()) 
        {
            return std::nullopt;
        }

        project_utils::msg::ControlHorizon controlHorizon{};
        controlHorizon.stamp = stamp;

        // If either of the horizons has only one control, repeat the control to match the other horizon.
        if (lateralHorizon.mControls.size() == 1) 
        {
            controlHorizon.time_step_ms = longitudinalHorizon.mTimeStepMs;
            const auto lateral = lateralHorizon.mControls.front();
            for (const auto & longitudinal : longitudinalHorizon.mControls) 
            {
                project_utils::msg::Control control;
                control.longitudinal = longitudinal;
                control.lateral = lateral;
                control.stamp = stamp;
                controlHorizon.controls.push_back(control);
            }
            return controlHorizon;
        }
        if (longitudinalHorizon.mControls.size() == 1) 
        {
            controlHorizon.time_step_ms = lateralHorizon.mTimeStepMs;
            const auto longitudinal = longitudinalHorizon.mControls.front();
            for (const auto & lateral : lateralHorizon.mControls) 
            {
                project_utils::msg::Control control;
                control.longitudinal = longitudinal;
                control.lateral = lateral;
                control.stamp = stamp;
                controlHorizon.controls.push_back(control);
            }
            return controlHorizon;
        }

        // If both horizons have multiple controls, align the time steps and zero-order hold the controls.
        
        // calculate greatest common divisor of time steps
        const auto gcd_double = [](const double a, const double b) 
        {
            const double precision = 1e9;
            const int int_a = static_cast<int>(round(a * precision));
            const int int_b = static_cast<int>(round(b * precision));
            return static_cast<double>(std::gcd(int_a, int_b)) / precision;
        };

        const double timeStepMs =
            gcd_double(lateralHorizon.mTimeStepMs, longitudinalHorizon.mTimeStepMs);
        
        controlHorizon.time_step_ms = timeStepMs;

        const auto lateralControls = resampleHorizonByZeroOrderHold(
            lateralHorizon.mControls, lateralHorizon.mTimeStepMs, timeStepMs);
        const auto longitudinalControls = resampleHorizonByZeroOrderHold(
            longitudinalHorizon.mControls, longitudinalHorizon.mTimeStepMs, timeStepMs);

        if (lateralControls.size() != longitudinalControls.size()) 
        {
            return std::nullopt;
        }

        const size_t num_steps = lateralControls.size();
        for (size_t i = 0; i < num_steps; ++i) 
        {
            project_utils::msg::Control control{};
            control.stamp = stamp;
            control.lateral = lateralControls.at(i);
            control.longitudinal = longitudinalControls.at(i);
            controlHorizon.controls.push_back(control);
        }
        
        return controlHorizon;
    }

} // namespace mpl::control::trajectory_follower_node

