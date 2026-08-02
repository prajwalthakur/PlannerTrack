#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
namespace controller::mppi_controller::critic
{
    /**
    * @brief Enum type for different modes of operation
    */
    enum class PathAngleMode
    {
        FORWARD_PREFERENCE = 0,
        NO_DIRECTIONAL_PREFERENCE = 1,
        CONSIDER_FEASIBLE_PATH_ORIENTATIONS = 2,
        NONE=3
    };

    inline PathAngleMode toPathAngleMode (int mode) 
    {
        switch (mode) 
        {
            case 0: return PathAngleMode::FORWARD_PREFERENCE;
            case 1: return PathAngleMode::NO_DIRECTIONAL_PREFERENCE;
            case 2: return PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS;
            default:
            {
                throw std::runtime_error("Invalid PathAngleMode");
                return PathAngleMode::NONE;
            }

        }
    };
    /**
    * @brief Method to convert mode enum to string for printing
    */
    inline std::string modeToStr(const PathAngleMode & mode)
    {
    if (mode == PathAngleMode::FORWARD_PREFERENCE) {
        return "Forward Preference";
    } else if (mode == PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS) {
        return "Consider Feasible Path Orientations";
    } else if (mode == PathAngleMode::NO_DIRECTIONAL_PREFERENCE) {
        return "No Directional Preference";
    } else {
        return "Invalid mode!";
    }
    }

    /**
    * @class mppi::critics::PathAngleCritic
    * @brief Critic objective function for aligning to path in cases of extreme misalignment
    * or turning
    */
    class PathAngleCritic: public CriticFunction
    {
        public:
            PathAngleCritic()=default;
            ~PathAngleCritic() override =default;
            void initialize() override;
            /**
            * @brief Evaluate cost related to trajectories path alignment
            *
            * @param costs [out] add reference cost values to this tensor
            */
            void score(CriticData& data) override;
        protected:
            unsigned int mPower{0};
            float mWeight{0};
            float mThresholdToConsider{0};
            size_t mOffsetFromFurthest{0};
            bool mReversingAllowed{true};
            float mMaxAngleToFurthest{0};
            PathAngleMode mMode{0};    
    };



}//namespace controller::mppi_controller

