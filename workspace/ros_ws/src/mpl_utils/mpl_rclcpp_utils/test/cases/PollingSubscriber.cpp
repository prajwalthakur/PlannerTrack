// Adapted from Autoware Contributors

#include "mpl_rclcpp_utils/PollingSubscriber.hpp"

#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

TEST(TestPollingSubscriber, InitialValues)
{
  const auto node = std::make_shared<rclcpp::Node>("test_initial_values");

  const auto latest_sub = mpl_rclcpp_utils::InternalProcessPollingSubscriber<
    std_msgs::msg::String, mpl_rclcpp_utils::polling_policy::Latest>::
    createSubscription(node.get(), "/test/initial_latest", 1);
  EXPECT_EQ(latest_sub->lastTakenDataTimeStamp(), std::nullopt);
  EXPECT_EQ(latest_sub->takeData(), nullptr);

  const auto newest_sub = mpl_rclcpp_utils::InternalProcessPollingSubscriber<
    std_msgs::msg::String, mpl_rclcpp_utils::polling_policy::Newest>::
    createSubscription(node.get(), "/test/initial_newest", 1);
  EXPECT_EQ(newest_sub->lastTakenDataTimeStamp(), std::nullopt);
  EXPECT_EQ(newest_sub->takeData(), nullptr);

  const auto all_sub = mpl_rclcpp_utils::InternalProcessPollingSubscriber<
    std_msgs::msg::String, mpl_rclcpp_utils::polling_policy::All>::
    createSubscription(node.get(), "/test/initial_all", 1);
  EXPECT_EQ(all_sub->lastTakenDataTimeStamp(), std::nullopt);
  EXPECT_TRUE(all_sub->takeData().empty());
}

TEST(TestPollingSubscriber, PubSub)
{
  const auto pub_node = std::make_shared<rclcpp::Node>("pub_node");
  const auto sub_node = std::make_shared<rclcpp::Node>("sub_node");

  const auto pub = pub_node->create_publisher<std_msgs::msg::String>("/test/text", 1);
  const auto sub = mpl_rclcpp_utils::InternalProcessPollingSubscriber<
    std_msgs::msg::String>::createSubscription(sub_node.get(), "/test/text", 1);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  std::thread thread([&executor] { executor.spin(); });
  while (rclcpp::ok() && !executor.is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std_msgs::msg::String pub_msg;
  pub_msg.data = "foo-bar";
  pub->publish(pub_msg);

  const auto sub_msg = sub->takeData();
  EXPECT_NE(sub_msg, nullptr);
  EXPECT_EQ(sub_msg->data, pub_msg.data);

  const auto timestamp = sub->lastTakenDataTimeStamp();
  EXPECT_TRUE(timestamp.has_value());

  rclcpp::Duration duration = sub_node->now() - timestamp.value();
  EXPECT_LE(duration.seconds(), 1.0);

  executor.cancel();
  thread.join();
}

TEST(TestPollingSubscriber, LatestTimestampRetention)
{
  const auto pub_node = std::make_shared<rclcpp::Node>("pub_node_latest");
  const auto sub_node = std::make_shared<rclcpp::Node>("sub_node_latest");

  const auto pub = pub_node->create_publisher<std_msgs::msg::String>("/test/latest_retention", 1);
  const auto sub = mpl_rclcpp_utils::InternalProcessPollingSubscriber<
    std_msgs::msg::String, mpl_rclcpp_utils::polling_policy::Latest>::
    createSubscription(sub_node.get(), "/test/latest_retention", 1);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  std::thread thread([&executor] { executor.spin(); });
  while (rclcpp::ok() && !executor.is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Publish a message
  std_msgs::msg::String pub_msg;
  pub_msg.data = "test-message";
  pub->publish(pub_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // First takeData: message received, timestamp set
  const auto msg1 = sub->takeData();
  EXPECT_NE(msg1, nullptr);
  const auto ts1 = sub->lastTakenDataTimeStamp();
  EXPECT_TRUE(ts1.has_value());

  // Second takeData: no new message, but Latest policy retains previous data and timestamp
  const auto msg2 = sub->takeData();
  EXPECT_EQ(msg2, msg1);  // Same message as before
  const auto ts2 = sub->lastTakenDataTimeStamp();
  EXPECT_TRUE(ts2.has_value());
  EXPECT_EQ(ts2, ts1);  // Timestamp is retained

  executor.cancel();
  thread.join();
}

TEST(TestPollingSubscriber, NewestTimestampClear)
{
  const auto pub_node = std::make_shared<rclcpp::Node>("pub_node_newest");
  const auto sub_node = std::make_shared<rclcpp::Node>("sub_node_newest");

  const auto pub = pub_node->create_publisher<std_msgs::msg::String>("/test/newest_clear", 1);
  const auto sub = mpl_rclcpp_utils::InternalProcessPollingSubscriber<
    std_msgs::msg::String, mpl_rclcpp_utils::polling_policy::Newest>::
    createSubscription(sub_node.get(), "/test/newest_clear", 1);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  std::thread thread([&executor] { executor.spin(); });
  while (rclcpp::ok() && !executor.is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Publish a message
  std_msgs::msg::String pub_msg;
  pub_msg.data = "test-message";
  pub->publish(pub_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // First takeData: message received, timestamp set
  const auto msg1 = sub->takeData();
  EXPECT_NE(msg1, nullptr);
  const auto ts1 = sub->lastTakenDataTimeStamp();
  EXPECT_TRUE(ts1.has_value());

  // Second takeData: no new message, Newest policy returns nullptr and clears timestamp
  const auto msg2 = sub->takeData();
  EXPECT_EQ(msg2, nullptr);
  const auto ts2 = sub->lastTakenDataTimeStamp();
  EXPECT_EQ(ts2, std::nullopt);  // Timestamp is cleared

  executor.cancel();
  thread.join();
}

TEST(TestPollingSubscriber, AllTimestampClear)
{
  const auto pub_node = std::make_shared<rclcpp::Node>("pub_node_all");
  const auto sub_node = std::make_shared<rclcpp::Node>("sub_node_all");

  const auto pub = pub_node->create_publisher<std_msgs::msg::String>("/test/all_clear", 1);
  const auto sub = mpl_rclcpp_utils::InternalProcessPollingSubscriber<
    std_msgs::msg::String, mpl_rclcpp_utils::polling_policy::All>::
    createSubscription(sub_node.get(), "/test/all_clear", rclcpp::QoS{10});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  std::thread thread([&executor] { executor.spin(); });
  while (rclcpp::ok() && !executor.is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Publish a message
  std_msgs::msg::String pub_msg;
  pub_msg.data = "test-message";
  pub->publish(pub_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // First takeData: message received, timestamp set
  const auto msgs1 = sub->takeData();
  EXPECT_FALSE(msgs1.empty());
  const auto ts1 = sub->lastTakenDataTimeStamp();
  EXPECT_TRUE(ts1.has_value());

  // Second takeData: no new message, All policy returns empty vector and clears timestamp
  const auto msgs2 = sub->takeData();
  EXPECT_TRUE(msgs2.empty());
  const auto ts2 = sub->lastTakenDataTimeStamp();
  EXPECT_EQ(ts2, std::nullopt);  // Timestamp is cleared

  executor.cancel();
  thread.join();
}