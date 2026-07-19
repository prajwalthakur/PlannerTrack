#include "dummy_lateral_controller/dummy_lateral_controller.hpp"

namespace dummy_lateral_controller
{
    DummyLateralController::DummyLateralController([[maybe_unused]] rclcpp::Node & node)
    {
        mClock = node.get_clock();
        mLogger = mpl::rclcpp_utils::Logger(node.get_logger(),"Lateral_Controller");
        mLogger.info("Dummy lateral controller is initiated");
    }

    ////////////////////////////////////////////////////////////////////////////////

    bool DummyLateralController::isReady([[maybe_unused]] const InputData & input_data)
    {
        return true;
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    LateralOutput DummyLateralController::run([[maybe_unused]] const InputData & input_data)
    {

        mLogger.info("running lateral controller");
        [[maybe_unused]] auto current_pose_ = input_data.mCurrentOdometry.pose.pose;
        mLogger.info("running lateral controller current pose of robot %.3f, %.3f, %.3f",current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
        
        //[[maybe_unused]] auto trajectory_ = input_data.mCurrentTrajectory;
        //[[maybe_unused]] auto current_odometry_ = input_data.mCurrentOdometry;
        //[[maybe_unused]] auto current_steering_ = input_data.mCurrentSteering;
        LateralOutput output;
        project_utils_msgs::msg::Lateral  cmd_;
        cmd_.stamp = mClock->now();
        cmd_.steering_tire_angle = 0.0;
        cmd_.steering_tire_rotation_rate=0.0;
        output.mControlCmd = cmd_;
        return output;
    }

}//namespace dummy_lateral_controller