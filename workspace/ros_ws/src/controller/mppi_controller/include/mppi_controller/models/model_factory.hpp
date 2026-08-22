#pragma once
#include <string>
#include "mppi_controller/models/ackermann/ackermann.hpp"




namespace controller::mppi_controller::models
{

    /// \brief Which concrete \ref Model to construct.
    enum class ModelType
    {
        ACKERMANN=0,
        NONE
    };

    /// \brief Parse a \ref ModelType from its ROS-param name (e.g. `"Ackermann"`).
    ModelType getModelType(const std::string& modelType);

    /// \brief Construct the concrete \ref Model named by \p modelType (currently only `"Ackermann"` -> \ref AckermannModel).
    std::unique_ptr<Model> createModel(const std::string& modelType);

};//namespace controller::mppi_controller::models;

namespace models = controller::mppi_controller::models;
