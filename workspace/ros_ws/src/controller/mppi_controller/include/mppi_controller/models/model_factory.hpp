#pragma once
#include <string>
#include "mppi_controller/models/ackermann/ackermann.hpp"




namespace controller::mppi_controller::models
{

    enum class ModelType
    {
        ACKERMANN=0,
        NONE
    };

    ModelType getModelType(const std::string& modelType);

    std::unique_ptr<Model> createModel(const std::string& modelType);

};//namespace controller::mppi_controller::models;

namespace models = controller::mppi_controller::models;
