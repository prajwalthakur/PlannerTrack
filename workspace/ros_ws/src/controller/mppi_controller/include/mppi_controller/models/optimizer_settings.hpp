#pragma once
#include "mppi_controller/parameters.hpp"
namespace controller::mppi_controller::models
{

    /// \brief \c Optimizer's tunable parameters (batch size, horizon length, MPPI temperature/gamma, retry limit, ...), loaded from ROS params.
    struct OptimizerSettings
    {

        public:
            /// \brief Load every setting from ROS params under \p name, falling back to the defaults shown below.
            void onConfigure(Parameters& parameter, const std::string name)
            {
                auto getParam = parameter.getParamGetter(name);

                getParam(model_dt,"model_dt",0.05f);
                getParam(batch_size,"batch_size", 1000);
                getParam(time_steps, "time_steps", 56);
                getParam(iteration_count,"iteration_count",1);
                getParam(temperature,"temperature",0.3f);
                getParam(gamma, "gamma", 0.015f);
                getParam(retry_attempt_limit,"retry_attempt_limit", 1);
                getParam(open_loop, "open_loop", false);
                getParam(path_follow, "path_follow", true);
                getParam(regenerate_noise, "regenerate_noise",false );
                
                //getParam(shift_control_sequence, "shift_control_sequence", true);
            }

            float model_dt{0.0f};
            float temperature{0.0f};
            float gamma{0.0f};
            unsigned int batch_size{0u};
            unsigned int time_steps{0u};
            unsigned int iteration_count{0u};
            bool shift_control_sequence{false};
            size_t retry_attempt_limit{0};
            bool open_loop{false};
            bool regenerate_noise{false};
            bool path_follow{true}; // if global path is provided to follow it.


    };


}//namespace controller::mppi_controller::models
namespace models = controller::mppi_controller::models;
