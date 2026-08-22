#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

/**
 * \file
 * \brief Standalone scratch node for exercising a bare
 * `nav2_costmap_2d::Costmap2DROS` in isolation (manually driving its
 * lifecycle transitions, without a lifecycle manager). Not listed in
 * `CMakeLists.txt`'s `LIB_SOURCES` -- not part of the build; `mppi_controller`
 * gets its own `Costmap2DROS` from \c MPPIController instead.
 */
class CostmapNode : public rclcpp::Node
{
public:
  CostmapNode() : Node("costmap_node")
  {
    // 1. Costmap MUST have a TF buffer to work
    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 2. Initialize the Costmap2DROS object
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "local_costmap", 
        std::string(this->get_namespace()), 
        "local_costmap");

    // 3. Trigger Lifecycle transitions
    // In a real app, use a Lifecycle Manager, but for a standalone node:
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ros_->on_activate(rclcpp_lifecycle::State());
    
    RCLCPP_INFO(this->get_logger(), "Costmap node initialized and activated");
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
//   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CostmapNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

