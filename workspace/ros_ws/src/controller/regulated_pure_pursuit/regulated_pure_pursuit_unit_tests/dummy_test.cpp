#include "ament_index_cpp/get_package_share_directory.hpp"
#include "interpolation_utils/spline_interpolation_points_2d.hpp"
#include "project_utils/logger.hpp"
#include "project_utils/parameter.hpp"
#include "project_utils_testing/test_fixture.hpp"

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace
{
std::string pkg_path = ament_index_cpp::get_package_share_directory("mpcc_controller");
std::string param_file = pkg_path + "/test_config/params.yaml";
}  // namespace
class DummyTest : public mplTestFixture
{
  public:
	DummyTest()
	{
		if (!rclcpp::ok()) {
			rclcpp::init(0, nullptr);
		}
	}
};

mplTest(DummyTest, testDummy)
{
    EXPECT_TRUE(true);  // if constructor runs → pass
}

