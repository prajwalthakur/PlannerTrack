// Adapted from Autoware Contributors


#include "mpl_rclcpp_utils/Parameter.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

TEST(TestParameter, GetOrDeclare)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("foo", 12345);
  options.append_parameter_override("bar", "str");
  std::string str ="";
  int defVal = 0;
  const auto node = std::make_shared<rclcpp::Node>("param_get_or_declare", options);
  const auto p1 = mpl_rclcpp_utils::get_or_declare_parameter<int>(*node, "foo",defVal);
  const auto p2 = mpl_rclcpp_utils::get_or_declare_parameter<int>(*node, "foo",defVal);
  const auto p3 = mpl_rclcpp_utils::get_or_declare_parameter<std::string>(*node, "bar",str);
  const auto p4 = mpl_rclcpp_utils::get_or_declare_parameter<std::string>(*node, "bar",str);
  EXPECT_EQ(p1, 12345);
  EXPECT_EQ(p2, 12345);
  EXPECT_EQ(p3, "str");
  EXPECT_EQ(p4, "str");
}

TEST(TestParameter, Update)
{
  const std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("foo", 12345),
    rclcpp::Parameter("bar", "str"),
  };
  int foo = 0;
  std::string bar = "default";
  std::string baz = "default";

  EXPECT_TRUE(mpl_rclcpp_utils::update_param(params, "foo", foo));
  EXPECT_TRUE(mpl_rclcpp_utils::update_param(params, "bar", bar));
  EXPECT_EQ(foo, 12345);
  EXPECT_EQ(bar, "str");

  EXPECT_FALSE(mpl_rclcpp_utils::update_param(params, "baz", baz));
  EXPECT_EQ(baz, "default");
}