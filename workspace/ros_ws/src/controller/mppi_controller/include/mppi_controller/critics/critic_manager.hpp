#pragma once
#include "mppi_controller/critics/critic_data.hpp"
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/critics/critics_factory.hpp"
#include "mppi_controller/utils/logger_utils.hpp"
#include "mppi_controller/utils/ros_namespace.hpp"
#include "mppi_controller/utils/types.hpp"

#include "project_utils_msgs/msg/critics_stats.hpp"  //TODO:

#include <string>
#include <vector>

namespace controller::mppi_controller
{
class CriticsManager
{
  public:
	// Constructor
	CriticsManager(const rclcpp::Clock::SharedPtr & clock);
	// Destructor
	~CriticsManager() = default;

	void onConfigure(Parameters * parameter, const std::string & name,
	    std::shared_ptr<CostMapRos> costMapRos, Logger logger);

	void evalTrajectoriesScores(CriticData & data);

	void loadCritics();

  private:
	rclcpp::Clock::SharedPtr mClock{nullptr};
	std::string mName;
	Logger mLogger;
	// bool mVisualize{false};
	bool mPublishCriticData{false};
	std::vector<std::pair<std::string, mppi_mt::ArrayX>> mCriticCosts;
	std::vector<std::string> mCrticsNames;
	std::shared_ptr<CostMapRos> mCostmapRos;
	Parameters * mParameters;
	std::vector<std::unique_ptr<mppi_critic::CriticFunction>> mCriticsCollection;

	// publisher
	// rclcpp::Publisher<project_utils_msgs::msg::CriticsStats>::SharedPtr mCriticsEffectPub;
	// PublisherT<project_utils_msgs::msg::CriticsStats> mCriticsEffectPub;
};
}  // namespace controller::mppi_controller

namespace mppi = controller::mppi_controller;