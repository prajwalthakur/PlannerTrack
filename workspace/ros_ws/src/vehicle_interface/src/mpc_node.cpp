
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <project_utils/msg/eigen_vector.hpp>
#include <blasfeo_d_aux_ext_dep.h>
#include "hpipm_d_ocp_qp_ipm.h"
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp.h"
#include "hpipm_d_ocp_qp_sol.h"
#include "hpipm_timing.h"
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
#include "vehicleModel/VehicleModelCollection.h"


class MpcNode : public rclcpp::Node
{
public:
  MpcNode() : Node("mpc_node")
  {
    RCLCPP_INFO(this->get_logger(), "MPC Node started!");

    // Example of using Eigen
    Eigen::Matrix2d mat;
    mat << 1, 2,
           3, 4;
    RCLCPP_INFO(this->get_logger(), "Eigen matrix:\n%f %f\n%f %f",
                mat(0, 0), mat(0, 1), mat(1, 0), mat(1, 1));
    project_utils::msg::EigenVector gg;
    // Example HPIPM usage (pseudo-code)
    // struct d_ocp_qp ocp_qp;
    // struct d_ocp_qp_ipm_arg arg;
    // ...
    // d_ocp_qp_ipm_solve(...);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcNode>());
  rclcpp::shutdown();
  return 0;
}
