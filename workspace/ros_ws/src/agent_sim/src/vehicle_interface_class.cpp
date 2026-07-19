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
	get(mFixedFrame, "agent_interface.fixed_frame", std::string("odom"));
	get(mNumAgents, "agents.num_agents", 0);
	get(mSimConfigFile, "sim_config_file", std::string(""));
	get(mAgentsConfigFile, "agents_config_file", std::string(""));

	mSimConfig = YAML::LoadFile(mSimConfigFile);
	mAgentsConfig = YAML::LoadFile(mAgentsConfigFile);

	// ── PID gains ─────────────────────────────────────────────────────────────
	// get(mKpV, "pid.kp_v", 2.0);
	// get(mKiV, "pid.ki_v", 0.0);
	// get(mKdV, "pid.kd_v", 0.05);
	// get(mKpW, "pid.kp_w", 2.0);
	// get(mKiW, "pid.ki_w", 0.0);
	// get(mKdW, "pid.kd_w", 0.05);
	// get(mAccMax, "pid.acc_max", 1.0);
	// get(mAlphaMax, "pid.alpha_max", 3.5);

	// // ── Lidar params ───────────────────────────────────────────────────────────
	// get(mVisDt, "lidar.vis_dt", 0.1f);
	// get(mLidarParams.num_beams, "lidar.num_beams", 360);
	// get(mLidarParams.angle_min, "lidar.angle_min", -static_cast<float>(M_PI));
	// get(mLidarParams.angle_max, "lidar.angle_max", static_cast<float>(M_PI));
	// get(mLidarParams.range_min, "lidar.range_min", 0.05f);
	// get(mLidarParams.range_max, "lidar.range_max", 10.0f);

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

	// ── Corridor walls ─────────────────────────────────────────────────────────
	// int num_lines = 0;
	// get(num_lines, "corridor.num_lines", 0);
	// for (int i = 1; i <= num_lines; ++i) {
	// 	const std::string base = "corridor.line_" + std::to_string(i);
	// 	std::vector<double> st{0.0, 0.0}, en{0.0, 0.0};
	// 	get(st, base + ".start", st);
	// 	get(en, base + ".end", en);
	// 	mWallSegments.push_back({static_cast<float>(st[0]), static_cast<float>(st[1]),
	// 	    static_cast<float>(en[0]), static_cast<float>(en[1])});
	// }

	// Lane segments
	// int num_lanes = 0;
	// get(num_lanes, "Lanes.num_lanes", 0);
	// for (int i = 1; i <= num_lanes; ++i) {
	// 	const std::string base = "Lanes.lane_" + std::to_string(i);
	// 	std::vector<double> st{0.0, 0.0}, end{0.0, 0.0};
	// 	double width = 0.0;
	// 	get(st, base + ".start", st);
	// 	get(end, base + ".end", end);
	// 	get(width, base + ".width", width);
	// 	mGraphLoc.addLanes({static_cast<float>(st[0]), static_cast<float>(st[1]),
	// 	    static_cast<float>(end[0]), static_cast<float>(end[1]), static_cast<float>(width)});
	// }
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

	// mLidarTimer = create_wall_timer(
	//     std::chrono::duration<double>(mVisDt), [this]() { lidarTimerCallback(); });

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
		i++;
	}
	// 	// ── State model ────────────────────────────────────────────────────────
	// 	ag.stateModel = std::make_shared<SingleTrackDynStateModel>(p, ag.id);
	// 	ag.stateModel->createIntegrator();

	// 	// ── Geometry + footprint ───────────────────────────────────────────────
	// 	double length = 0.5, width = 0.3, majorAxis = 0.6, minorAxis = 0.3;
	// 	get(length, base + ".length", 0.5);
	// 	get(width, base + ".width", 0.3);
	// 	get(majorAxis, base + ".collision_foot_print_major_axis", 0.6);
	// 	get(minorAxis, base + ".collision_foot_print_minor_axis", 0.3);

	// 	ag.geomModel = std::make_shared<RectangularGeometry>(length, width);
	// 	ag.footprint = std::make_shared<EllipseCollisionFootPrint>(majorAxis, minorAxis);
	// 	ag.geomModel->setCollisionFootPrint(ag.footprint);

	// 	ag.hl = static_cast<float>(length / 2.0);
	// 	ag.hw = static_cast<float>(width / 2.0);

	// 	const stPose pose0 = ag.stateModel->getStatePose();
	// 	ag.x = static_cast<float>(pose0.xCoord);
	// 	ag.y = static_cast<float>(pose0.yCoord);
	// 	ag.phi = static_cast<float>(pose0.yaw);

	// 	// ── PID initialisation ────────────────────────────────────────────────
	// 	ag.pid_vx = {mKpV, mKiV, mKdV, -mAccMax, mAccMax};
	// 	ag.pid_wz = {mKpW, mKiW, mKdW, -mAlphaMax, mAlphaMax};

	// 	// ── Velocity-reference subscriber: /agent_N/control (DiffControl) ────
	// 	// MPPI publishes: longitudinal.velocity = v_cmd,
	// 	//                 lateral.steering_tire_rotation_rate = ω_cmd
	// 	ag.velRefSub = create_subscription<DiffControl>("/agent_" + std::to_string(i) + "/control",
	// 	    10, [this, idx = i - 1](DiffControl::SharedPtr msg) {
	// 		    mAgents[idx].vx_ref = static_cast<double>(msg->longitudinal.velocity);
	// 		    mAgents[idx].wz_ref = static_cast<double>(msg->lateral.steering_tire_rotation_rate);
	// 	    });

	// 	// ── Odom publisher: /agent_N/odom ─────────────────────────────────────
	// 	ag.odomPub =
	// 	    create_publisher<nav_msgs::msg::Odometry>("/agent_" + std::to_string(i) + "/odom", 10);

	// 	// ── JointState publisher: /agent_N/joint_states ───────────────────────
	// 	ag.jointStatePub = create_publisher<sensor_msgs::msg::JointState>(
	// 	    "/agent_" + std::to_string(i) + "/joint_states", 10);

	// 	// lane loc publisher: /agent_N/lane localization
	// 	ag.laneLocPub = create_publisher<project_utils_msgs::msg::LaneId>(
	// 	    "/agent_" + std::to_string(i) + "/laneloc", 10);

	// 	// ── LaserScan publisher: /agent_N/scan ────────────────────────────────
	// 	ag.scanPub = create_publisher<sensor_msgs::msg::LaserScan>(
	// 	    "/agent_" + std::to_string(i) + "/scan", 10);

	// 	RCLCPP_INFO(get_logger(), "Agent %d | L=%.2f W=%.2f | init=(%.2f,%.2f) | topics created", i,
	// 	    length, width, p.initXPose, p.initYPose);
	// }
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
	// Every agent's GeometricModel is a live object owned by its AgentModel --
	// this collects pointers into them, it does not copy pose data. Rebuilt
	// fresh each tick (cheap: N pointers) rather than cached, so it's always
	// exactly the current agent list.
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
