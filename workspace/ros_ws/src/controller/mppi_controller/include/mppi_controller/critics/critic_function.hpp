#pragma once 
#include <string>
#include <memory>
#include "mppi_controller/critics/critic_data.hpp"
#include "mppi_controller/utils/geometry_utils.hpp"
#include "mppi_controller/utils/ros_namespace.hpp"
#include "mppi_controller/parameters.hpp"
#include "mppi_controller/utils/logger_utils.hpp"
namespace controller::mppi_controller::critic
{

/**
 * @brief Utility for storing cost information
 */
struct CollisionCost
{
  float cost{0};
  bool using_footprint{false};
};

/**
 * @brief Abstract critic objective function to score trajectories
 */
class CriticFunction
    {

        public:
        /**
            * @brief Constructor for mppi::critics::CriticFunction
            */
        CriticFunction() = default;

        /**
        * @brief Destructor for mppi::critics::CriticFunction
        */
        virtual ~CriticFunction() = default;

        /**
        * @brief Configure critic on bringup
        * @param name Name of plugin
        * @param costmap_ros Costmap2DROS object of environment
        * @param dynamic_parameter_handler Parameter handler object
        */
        void onConfigure(
            const std::string& parentName,
            const std::string & name,
            std::shared_ptr<CostMapRos> costmap_ros,
            Parameters* parameter, Logger logger)
        {
            mName = name;
            mParentName = parentName;
            mCostmapRos = costmap_ros;
            mCostmap = mCostmapRos->getCostmap();
            mParameter = parameter;

            auto getParam = mParameter->getParamGetter(mName);
            getParam(mEnabled, "enabled", true);
            mLogger = logger;
            initialize();
        }

        /**
        * @brief Main function to score trajectory
        * @param data Critic data to use in scoring
        */
        virtual void score(CriticData & data) = 0;

        /**
            * @brief Initialize critic
            */
        virtual void initialize() = 0;

        /**
            * @brief Get name of critic
            */
        std::string getName()
        {
            return mName;
        }

        protected:
            bool mEnabled;
            std::string mName;
            std::string mParentName;
            std::shared_ptr<CostMapRos> mCostmapRos;
            CostMap* mCostmap;
            Parameters* mParameter;
            Logger mLogger;
            //rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
    };

}  // namespace mppi::critics

namespace mppi_critic = controller::mppi_controller::critic;