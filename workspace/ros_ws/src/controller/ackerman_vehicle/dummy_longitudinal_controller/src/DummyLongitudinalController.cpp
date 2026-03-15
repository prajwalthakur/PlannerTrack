#include "dummy_longitudinal_controller/DummyLongitudinalController.hpp"

namespace dummy_longitudinal_controller
{
    DummyLongitudinalController::DummyLongitudinalController([[maybe_unused]] rclcpp::Node & node)
    :mClock(node.get_clock())
    {

    }

    ////////////////////////////////////////////////////////////////////////////////

    bool DummyLongitudinalController::isReady([[maybe_unused]] const InputData & input_data)
    {
        return true;
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    LongitudinalOutput DummyLongitudinalController::run(const InputData & input_data)
    {

        [[maybe_unused]] auto current_pose_ = input_data.mCurrentOdometry.pose.pose;
        [[maybe_unused]] auto trajectory_ = input_data.mCurrentTrajectory;
        [[maybe_unused]] auto current_odometry_ = input_data.mCurrentOdometry;
        [[maybe_unused]] auto current_steering_ = input_data.mCurrentSteering;
        LongitudinalOutput output;
        project_utils::msg::Longitudinal  cmd_;
        cmd_.stamp         =    mClock->now();
        cmd_.velocity      =    0.0;
        cmd_.acceleration  =    0.0;

        output.mControlCmd =    cmd_;
        return output;
    }

}//namespace dummy_longitudinal_controller