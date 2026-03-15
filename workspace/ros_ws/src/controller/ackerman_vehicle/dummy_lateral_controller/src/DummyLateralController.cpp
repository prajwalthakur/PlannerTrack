#include "dummy_lateral_controller/DummyLateralController.hpp"

namespace dummy_lateral_controller
{
    DummyLateralController::DummyLateralController([[maybe_unused]] rclcpp::Node & node)
    :
    mClock(node.get_clock())
    {

    }

    ////////////////////////////////////////////////////////////////////////////////

    bool DummyLateralController::isReady([[maybe_unused]] const InputData & input_data)
    {
        return true;
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    LateralOutput DummyLateralController::run(const InputData & input_data)
    {

        [[maybe_unused]] auto current_pose_ = input_data.mCurrentOdometry.pose.pose;
        [[maybe_unused]] auto trajectory_ = input_data.mCurrentTrajectory;
        [[maybe_unused]] auto current_odometry_ = input_data.mCurrentOdometry;
        [[maybe_unused]] auto current_steering_ = input_data.mCurrentSteering;
        LateralOutput output;
        project_utils::msg::Lateral  cmd_;
        cmd_.stamp = mClock->now();
        cmd_.steering_tire_angle = 0.0;
        output.mControlCmd = cmd_;
        return output;
    }

}//namespace dummy_lateral_controller