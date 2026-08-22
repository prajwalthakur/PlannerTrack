#include "mppi_controller/critics/critic_manager.hpp"

/** \file
 * \brief \c CriticsManager implementation: loading critics by name and
 * running them all against a rollout batch.
 */

mppi::CriticsManager::CriticsManager(const rclcpp::Clock::SharedPtr & clock) : mClock{clock}
{
}

void mppi::CriticsManager::onConfigure(Parameters * parameter, const std::string & name,
    std::shared_ptr<CostMapRos> costMapRos, Logger logger)
{
	mName = name;
	mCostmapRos = costMapRos;
	mParameters = parameter;
	mLogger = logger;
	mLogger.info("going to load critics");
	auto getParam = mParameters->getParamGetter(mName);
	// getParam(mVisualize,"visualize",false);
	getParam(mPublishCriticData, "publish_critics_stats", false);
	getParam(mCrticsNames, "critics", std::vector<std::string>{});
	loadCritics();
}

void mppi::CriticsManager::loadCritics()
{
	mCriticsCollection.clear();

	for (std::string & criticName : mCrticsNames) {
		if (mppi_utils::compareStringIgnoreCase(criticName, "none")) return;
		std::unique_ptr<mppi_critic::CriticFunction> critic_ =
		    mppi_critics_utils::getCritic(criticName);
		mCriticsCollection.push_back(std::move(critic_));
		Logger crticLogger(mLogger, criticName);
		mCriticsCollection.back()->onConfigure(
		    mName, mName + "." + criticName, mCostmapRos, mParameters, crticLogger);
		mLogger.info("critic  %s loaded", criticName.c_str());
	}

	// if (mVisualize)
	// {
	//     // mCriticsEffectPub =
	//     mParentNode->create_publisher<project_utils_msgs::msg::CriticsStats>(
	//     // "~/critics_stats");
	//     // mCriticsEffectPub->on_activate();
	// }
}

void mppi::CriticsManager::evalTrajectoriesScores(CriticData & data)
{
	size_t numCritics = mCriticsCollection.size();
	mLogger.info("number of critics %ld", numCritics);
	auto statMsg = std::make_unique<project_utils_msgs::msg::CriticsStats>();

	if (mPublishCriticData) {
		data.trajectories_in_collision.assign(data.costs.size(), false);
		mCriticCosts.clear();
		mCriticCosts.reserve(numCritics);

		statMsg->critics.reserve(numCritics);
		statMsg->changed.reserve(numCritics);
		statMsg->costs_sum.reserve(numCritics);
	}
	for (size_t i = 0; i < numCritics; ++i) {
		if (data.fail_flag) break;

		// store costs before critic evaluatin
		mppi_mt::ArrayX costsBefore;
		if (mPublishCriticData) {
			costsBefore = data.costs;
		}

		mCriticsCollection[i]->score(data);

		// Cal statistics if visulaization is enabled
		if (mPublishCriticData) {
			statMsg->critics.push_back(mCrticsNames[i]);
			// cal sum of costs added by this individual critic
			mppi_mt::ArrayX costDiff = data.costs - costsBefore;
			float costSum = costDiff.sum();
			statMsg->costs_sum.push_back(costSum);
			statMsg->changed.push_back(costSum != 0.0f);
			mCriticCosts.emplace_back(mCrticsNames[i], std::move(costDiff));
			std::string criticName = mCrticsNames[i];
			mLogger.info("critic Name %s cost %.3f", criticName.c_str(), costSum);
		}
	}
	if (mPublishCriticData) {
		statMsg->header.stamp = mClock->now();  // TODO:
		// mCriticsEffectPub->publish(std::move(statMsg));
		data.critic_stat_msg = std::move(statMsg);
	}
}
