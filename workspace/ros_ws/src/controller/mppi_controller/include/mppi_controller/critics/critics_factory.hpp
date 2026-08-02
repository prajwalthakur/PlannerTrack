#pragma once
#include <memory.h>
#include "mppi_controller/critics/common/critic_collection.hpp"
#include "mppi_controller/utils/utils.hpp"
// list and initialize the critics
// return unique pointers to critics

namespace controller::mppi_controller::critics_utils
{
    // enum class CriticsType
    // {
    //     GoalCritic
    // };

    inline std::unique_ptr<mppi_critic::CriticFunction> getCritic(const std::string& criticName)
    {
        // std::string criticName_ = criticName;
        // std::transform(criticName_.begin(), criticName_.end(), criticName_.begin(),
        //             [](unsigned char c){ return std::tolower(c); });
        if(mppi_utils::compareStringIgnoreCase(criticName,"GoalCritic"))
        {
            return std::make_unique<mppi_critic::GoalCritic>() ;
        }
        if(mppi_utils::compareStringIgnoreCase(criticName,"GoalAngleCritic"))
        {
            return std::make_unique<mppi_critic::GoalAngleCritic>();
        }
        if(mppi_utils::compareStringIgnoreCase(criticName,"PathAlignCritic"))
        {
            return std::make_unique<mppi_critic::PathAlignCritic>();
        }
        if(mppi_utils::compareStringIgnoreCase(criticName, "PathFollowCritic"))
        {
            return std::make_unique<mppi_critic::PathFollowCritic>();
        }
        if(mppi_utils::compareStringIgnoreCase(criticName, "PreferForwardCritic"))
        {
            return std::make_unique<mppi_critic::PreferForwardCritic>();
        }
        if(mppi_utils::compareStringIgnoreCase(criticName,"PathAngleCritic"))
        {
            return std::make_unique<mppi_critic::PathAngleCritic>();
        }
        if(mppi::utils::compareStringIgnoreCase(criticName,"ConstraintCritic"))
        {
            return std::make_unique<mppi_critic::ConstraintCritic>();
        }
        if(mppi_utils::compareStringIgnoreCase(criticName,"TwirlingCritic"))
        {
            return std::make_unique<mppi_critic::TwirlingCritic>();
        }
        if(mppi_utils::compareStringIgnoreCase(criticName,"VelocityDeadbandCritic"))
        {
            return std::make_unique<mppi_critic::VelocityDeadbandCritic>();
        }
        if(mppi_utils::compareStringIgnoreCase(criticName,"CostCritic"))
        {
            return std::make_unique<mppi_critic::CostCritic>();
        }
        return nullptr;
    }
}//namespace controller::mppi_controller
namespace mppi_critics_utils = controller::mppi_controller::critics_utils;