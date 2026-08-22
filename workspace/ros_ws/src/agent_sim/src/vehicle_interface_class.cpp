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

/** \file
 * \brief \ref AgentInterface implementation: agent construction, the
 * fixed-rate simulation/publish/lidar timers, and TF/marker/scan
 * publishing.
 */

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
	mMarkerPub = create_publisher<visualization_msgs::msg::MarkerArray>("/agent_markers", 10);
	// map_server publishes /map transient_local (latched) so this still
	// picks up the map even though map_server's lifecycle activation can
	// finish before or after this subscription is created.
	rclcpp::QoS mapQos(1);
	mapQos.transient_local().reliable();
	mMapSub = create_subscription<nav_msgs::msg::OccupancyGrid>(
	    "/map", mapQos, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { mapCallback(msg); });
	mLogger = mpl::rclcpp_utils::Logger(get_logger(), "veh-interface-logger");
	mVehicleFactory = std::make_unique<VehicleModelFactory>(mLogger);
	auto get = mParameters->getParamGetter("");

	// ── Simulation timing + world frame ────────────────────────────────────────
	get(mSimTimeStep, "sim_time_step", 0.01);
	get(mStatePublisherTimeStep, "agent_interface.state_publisher_dt", 0.05);
	get(mVisDt, "lidar.vis_dt", 0.1);
	get(mFixedFrame, "agent_interface.fixed_frame", std::string("odom"));
	get(mNumAgents, "agent_interface.num_agents", 0);
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
		// create state publisher
		agent.model = mVehicleFactory->create(
		    mSimConfig, agentNode, id, shared_from_this(), mFixedFrame, true);
		agent.controlSub =
		    create_subscription<EigenVectorStamped>("/agent_" + std::to_string(i) + "/control", 1,
		        [this, idx = i](EigenVectorStamped::SharedPtr msg) {
			        const InputVector u = Eigen::Map<const Eigen::VectorXd>(
			            msg->data.data(), static_cast<Eigen::Index>(msg->data.size()));
			        mAgents[idx].model->updateCommandedControl(u);
		        });

		agent.scanPub = create_publisher<sensor_msgs::msg::LaserScan>(
		    "/agent_" + std::to_string(i) + "/scan", 10);

		agent.resetSub = create_subscription<std_msgs::msg::Empty>(
		    "/agent_" + std::to_string(i) + "/reset", 1,
		    [this, idx = i](std_msgs::msg::Empty::SharedPtr) {
			    mAgents[idx].model->resetToInitialState();
		    });

		const YAML::Node sensorParams = agentNode["sensor_params"];
		if (sensorParams["angle_min"]) agent.lidarAngleMin = sensorParams["angle_min"].as<float>();
		if (sensorParams["angle_max"]) agent.lidarAngleMax = sensorParams["angle_max"].as<float>();
		if (sensorParams["range_min"]) agent.lidarRangeMin = sensorParams["range_min"].as<float>();
		if (sensorParams["range_max"]) agent.lidarRangeMax = sensorParams["range_max"].as<float>();

		if (agentNode["color"]) {
			const auto rgba = agentNode["color"].as<std::vector<float>>();
			for (size_t c = 0; c < agent.color.size() && c < rgba.size(); ++c) {
				agent.color[c] = rgba[c];
			}
		}

		i++;
	}
}

// ─────────────────────────────────────────────────────────────────────────────

//////////////////////////////////////////////////////////////////////////

void AgentInterface::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
	mMapGrid.width = msg->info.width;
	mMapGrid.height = msg->info.height;
	mMapGrid.resolution = msg->info.resolution;
	mMapGrid.originX = msg->info.origin.position.x;
	mMapGrid.originY = msg->info.origin.position.y;
	mMapGrid.data = msg->data;
	mLogger.info("occupancy grid map received: %u x %u @ %.3f m/cell", mMapGrid.width,
	    mMapGrid.height, mMapGrid.resolution);
}

// ─────────────────────────────────────────────────────────────────────────────

//////////////////////////////////////////////////////////////////////////

void AgentInterface::publishStaticTFs()
{
	std::vector<geometry_msgs::msg::TransformStamped> tfs;
	for (int i = 1; i <= mNumAgents; ++i) {
		geometry_msgs::msg::TransformStamped tf;
		tf.header.stamp = now();
		tf.header.frame_id = "agent_" + std::to_string(i) + "/base_link";
		tf.child_frame_id = "agent_" + std::to_string(i) + "/lidar_link";
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
	tf.child_frame_id = "agent_" + std::to_string(agentNum) + "/base_link";
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
	worldSnapShot.mapGrid = mMapGrid.valid() ? &mMapGrid : nullptr;

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
	visualization_msgs::msg::MarkerArray markers;
	for (auto & ag : mAgents) {
		if (!ag.model) {
			continue;
		}
		// Model-specific publishing (odom, steering feedback, etc.) -- each
		// dynamics plugin owns this via the publishers it set up in
		// setupRos(), since not every agent type publishes the same things.
		ag.model->publishStates();

		const int agentNum = static_cast<int>(ag.id.value());
		const stPose pose = ag.model->getStatePose();

		// TF
		broadcastAgentTF(agentNum, pose);

		// Geometry visualization -- a CUBE sized from the agent's own
		// RectangularGeometry, anchored at its base_link frame's origin
		// (identity pose here) so TF -- already broadcast above -- does the
		// world placement instead of duplicating pose math into the marker.
		const ShapeDescriptor desc = ag.model->geometry().describe();
		if (desc.kind == ShapeDescriptor::Kind::Rectangle) {
			visualization_msgs::msg::Marker marker;
			marker.header.stamp = stamp;
			marker.header.frame_id = "agent_" + std::to_string(agentNum) + "/base_link";
			marker.ns = "agents";
			marker.id = agentNum;
			marker.type = visualization_msgs::msg::Marker::CUBE;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = desc.rect.length;
			marker.scale.y = desc.rect.width;
			marker.scale.z = 0.2;
			marker.color.r = ag.color[0];
			marker.color.g = ag.color[1];
			marker.color.b = ag.color[2];
			marker.color.a = ag.color[3];
			markers.markers.push_back(marker);
		}

		// Collision footprint visualization
		// SPHERE marker with scale.x != scale.y renders as an ellipse.
		// Semi-transparent and a different color from the true-shape CUBE
		// above: this is a safety-margin overlay, not the rendered body.
		const ShapeDescriptor footprintDesc = ag.model->collisionFootprint().describe();
		if (footprintDesc.kind == ShapeDescriptor::Kind::Ellipse) {
			visualization_msgs::msg::Marker marker;
			marker.header.stamp = stamp;
			marker.header.frame_id = "agent_" + std::to_string(agentNum) + "/base_link";
			marker.ns = "agents_footprint";
			marker.id = agentNum;
			marker.type = visualization_msgs::msg::Marker::SPHERE;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = footprintDesc.ellipse.majorAxisLength;
			marker.scale.y = footprintDesc.ellipse.minorAxisLength;
			marker.scale.z = 0.05;
			marker.color.r = 1.0f;
			marker.color.g = 0.3f;
			marker.color.b = 0.1f;
			marker.color.a = 0.35f;
			markers.markers.push_back(marker);
		}
	}
	mMarkerPub->publish(markers);
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
		scan.header.frame_id = "agent_" + std::to_string(agentNum) + "/lidar_link";
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
