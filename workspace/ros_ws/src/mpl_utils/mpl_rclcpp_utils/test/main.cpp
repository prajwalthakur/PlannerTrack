// Copyright 2025 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

class RclcppEnvironment : public testing::Environment
{
public:
  RclcppEnvironment(int argc, char ** argv) : argc(argc), argv(argv) {}
  void SetUp() override { rclcpp::init(argc, argv); }
  void TearDown() override { rclcpp::shutdown(); }

private:
  int argc;
  char ** argv;
};

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new RclcppEnvironment(argc, argv));
  return RUN_ALL_TESTS();
}