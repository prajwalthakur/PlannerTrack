// Author Prajwal Thakur
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

#include "agent_sim/vehicle_interface_class.hpp"

#include "project_utils/geometry_utils.hpp"

//////////////////////////////////////////////////////////////////////////

AgentInterface::AgentInterface() : rclcpp::Node("vehicle_interface_node")
{
	RCLCPP_INFO(get_logger(), "AgentInterface node created");
}

// ─────────────────────────────────────────────────────────────────────────────

//////////////////////////////////////////////////////////////////////////

void AgentInterface::onConfigure()
{
	mParameters = std::make_unique<mpl::rclcpp_utils::Parameters>(*this);
	mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	mStaticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
	mLogger = mpl::rclcpp_utils::Logger(get_logger(), "veh-interface-logger");
	mVehicleFactory = std::make_unique<VehicleModelFactory>(mLogger);
	auto get = mParameters->getParamGetter("");

	// ── Simulation timing + world frame ────────────────────────────────────────
	get(mSimTimeStep, "sim_time_step", 0.01);
	get(mStatePublisherTimeStep, "agent_interface.state_publisher_dt", 0.05);
	get(mVisDt, "lidar.vis_dt", 0.1);
	get(mFixedFrame, "agent_interface.fixed_frame", std::string("odom"));
	get(mNumAgents, "agents.num_agents", 0);
	get(mSimConfigFile, "sim_config_file", std::string(""));
	get(mAgentsConfigFile, "agents_config_file", std::string(""));

	mSimConfig = YAML::LoadFile(mSimConfigFile);
	mAgentsConfig = YAML::LoadFile(mAgentsConfigFile);

	// ── Static obstacles ───────────────────────────────────────────────────────
	int num_obstacles = 0;
	get(num_obstacles, "obstacles.num_obstacles", 0);
	for (int i = 1; i <= num_obstacles; ++i) {
		const std::string base = "obstacles.obstacle_" + std::to_string(i);
		std::vector<double> pos{0.0, 0.0, 0.0, 0.0};
		double r = 0.3;
		get(pos, base + ".position", pos);
		get(r, base + ".radius", 0.3);
		mLogger.info("obstacle %d, x %.3f y %.3f", i, pos[0], pos[1]);
		ShapeDescriptor obstacle;
		obstacle.kind = ShapeDescriptor::Kind::Circle;
		obstacle.circle.cx = pos[0];
		obstacle.circle.cy = pos[1];
		obstacle.circle.radius = r;
		mObstacleCircles.push_back(obstacle);
	}
}

// ─────────────────────────────────────────────────────────────────────────────

//////////////////////////////////////////////////////////////////////////

void AgentInterface::onActivate()
{
	addAgents();
	publishStaticTFs();

	mStateUpdateTimer = create_wall_timer(
	    std::chrono::duration<double>(mSimTimeStep), [this]() { stateUpdateTimerCallback(); });

	mStatePubTimer = create_wall_timer(std::chrono::duration<double>(mStatePublisherTimeStep),
	    [this]() { statePubTimerCallback(); });

	mLidarTimer = create_wall_timer(
	    std::chrono::duration<double>(mVisDt), [this]() { lidarTimerCallback(); });

	RCLCPP_INFO(get_logger(), "AgentInterface activated with %d agents", mNumAgents);
}

// ─────────────────────────────────────────────────────────────────────────────
// Read per-vehicle params, create state model + geometry, wire up ROS I/O
// ─────────────────────────────────────────────────────────────────────────────

//////////////////////////////////////////////////////////////////////////

void AgentInterface::addAgents()
{
	auto get = mParameters->getParamGetter("");
	mAgents.resize(mNumAgents + 1);
	int i = 1;
	for (const auto & entry : mAgentsConfig["agents"]) {
		const std::string agentName = entry.first.as<std::string>();
		const YAML::Node & agentNode = entry.second;
		auto id = UniqueId("agent", i);
		auto & agent = mAgents[i];
		agent.id = id;
		agent.model = mVehicleFactory->create(mSimConfig, agentNode, id);
		agent.controlSub =
		    create_subscription<EigenVector>("/agent_" + std::to_string(i) + "/control", 1,
		        [this, idx = i](EigenVector::SharedPtr msg) {
			        const InputVector u = Eigen::Map<const Eigen::VectorXd>(
			            msg->data.data(), static_cast<Eigen::Index>(msg->data.size()));
			        mAgents[idx].model->updateCommandedControl(u);
		        });

		agent.odomPub =
		    create_publisher<nav_msgs::msg::Odometry>("/agent_" + std::to_string(i) + "/odom", 10);

		agent.scanPub = create_publisher<sensor_msgs::msg::LaserScan>(
		    "/agent_" + std::to_string(i) + "/scan", 10);

		const YAML::Node sensorParams = agentNode["sensor_params"];
		if (sensorParams["angle_min"]) agent.lidarAngleMin = sensorParams["angle_min"].as<float>();
		if (sensorParams["angle_max"]) agent.lidarAngleMax = sensorParams["angle_max"].as<float>();
		if (sensorParams["range_min"]) agent.lidarRangeMin = sensorParams["range_min"].as<float>();
		if (sensorParams["range_max"]) agent.lidarRangeMax = sensorParams["range_max"].as<float>();

		i++;
	}
}

// ─────────────────────────────────────────────────────────────────────────────

//////////////////////////////////////////////////////////////////////////

void AgentInterface::publishStaticTFs()
{
	std::vector<geometry_msgs::msg::TransformStamped> tfs;
	for (int i = 1; i <= mNumAgents; ++i) {
		geometry_msgs::msg::TransformStamped tf;
		tf.header.stamp = now();
		tf.header.frame_id = "agent_" + std::to_string(i) + "_base_link";
		tf.child_frame_id = "agent_" + std::to_string(i) + "_lidar_link";
		tf.transform.rotation.w = 1.0;
		tfs.push_back(tf);
	}
	mStaticTfBroadcaster->sendTransform(tfs);
}

// ─────────────────────────────────────────────────────────────────────────────

//////////////////////////////////////////////////////////////////////////

void AgentInterface::broadcastAgentTF(int agentNum, const stPose & pose)
{
	geometry_msgs::msg::TransformStamped tf;
	tf.header.stamp = now();
	tf.header.frame_id = mFixedFrame;
	tf.child_frame_id = "agent_" + std::to_string(agentNum) + "_base_link";
	tf.transform.translation.x = pose.xCoord;
	tf.transform.translation.y = pose.yCoord;
	tf.transform.translation.z = pose.zCoord;
	tf2::Quaternion q;
	// mLogger.info("agent id %d, and yaw %.3f",agentNum, pose.yaw);
	q.setRPY(0.0, 0.0, pose.yaw);
	tf.transform.rotation.x = q.x();
	tf.transform.rotation.y = q.y();
	tf.transform.rotation.z = q.z();
	tf.transform.rotation.w = q.w();
	mTfBroadcaster->sendTransform(tf);
}

// ─────────────────────────────────────────────────────────────────────────────

//////////////////////////////////////////////////////////////////////////

void AgentInterface::stateUpdateTimerCallback()
{

	WorldSnapshot worldSnapShot;
	worldSnapShot.agents.reserve(mAgents.size());
	for (auto & ag : mAgents) {
		if (ag.model) {
			worldSnapShot.agents.push_back(&ag.model->geometry());
		}
	}
	worldSnapShot.staticObstacles = mObstacleCircles;

	for (auto & ag : mAgents) {
		if (ag.model) {
			ag.model->step(worldSnapShot);
		}
	}
}

// ─────────────────────────────────────────────────────────────────────────────

//////////////////////////////////////////////////////////////////////////

void AgentInterface::statePubTimerCallback()
{
	const rclcpp::Time stamp = now();
	for (auto & ag : mAgents) {
		if (!ag.model) {
			continue;
		}

		const int agentNum = static_cast<int>(ag.id.value());
		const stPose pose = ag.model->getStatePose();
		const StateVector & sv = ag.model->getState();

		// Odometry
		nav_msgs::msg::Odometry odom;
		odom.header.stamp = stamp;
		odom.header.frame_id = mFixedFrame;
		odom.child_frame_id = "agent_" + std::to_string(agentNum) + "_base_link";
		odom.pose.pose.position.x = pose.xCoord;
		odom.pose.pose.position.y = pose.yCoord;
		odom.pose.pose.position.z = pose.zCoord;
		odom.pose.pose.orientation = mpl::geometry_utils::createQuaternionFromYaw(pose.yaw);
		odom.twist.twist.linear.x = (sv.size() > 3) ? sv(3) : 0.0;
		ag.odomPub->publish(odom);

		// TF
		broadcastAgentTF(agentNum, pose);
	}
}

// ─────────────────────────────────────────────────────────────────────────────

//////////////////////////////////////////////////////////////////////////

void AgentInterface::lidarTimerCallback()
{
	const rclcpp::Time stamp = now();
	for (auto & ag : mAgents) {
		if (!ag.model || !ag.scanPub) {
			continue;
		}

		const std::vector<float> & ranges = ag.model->sensor().getReadings();
		const int agentNum = static_cast<int>(ag.id.value());

		sensor_msgs::msg::LaserScan scan;
		scan.header.stamp = stamp;
		scan.header.frame_id = "agent_" + std::to_string(agentNum) + "_lidar_link";
		scan.angle_min = ag.lidarAngleMin;
		scan.angle_max = ag.lidarAngleMax;
		scan.angle_increment = (ranges.size() > 1)
		    ? (ag.lidarAngleMax - ag.lidarAngleMin) / static_cast<float>(ranges.size())
		    : 0.0f;
		scan.time_increment = 0.0f;
		scan.scan_time = static_cast<float>(mVisDt);
		scan.range_min = ag.lidarRangeMin;
		scan.range_max = ag.lidarRangeMax;
		scan.ranges = ranges;
		ag.scanPub->publish(scan);
	}
}

// ─────────────────────────────────────────────────────────────────────────────

//////////////////////////////////////////////////////////////////////////
