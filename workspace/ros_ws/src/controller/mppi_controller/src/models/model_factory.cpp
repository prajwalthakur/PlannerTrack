
#include "mppi_controller/models/model_factory.hpp"


namespace controller::mppi_controller::models
{
    ModelType getModelType(const std::string& modelType)
    {
        
        std::string modelType_ = modelType;
        std::transform(modelType_.begin(), modelType_.end(), modelType_.begin(),
                    [](unsigned char c){ return std::tolower(c); });
        if(modelType_ == "ackermann")
            return ModelType::ACKERMANN;
        else
            return ModelType::NONE;
    }


    std::unique_ptr<models::Model> createModel(const std::string& modelType)
    {
        ModelType type = getModelType(modelType);
        if(type==ModelType::ACKERMANN)
        {
            std::unique_ptr<models::Model> model =  std::make_unique<models::AckermannModel>();
            return model;
        }
        return nullptr;

    }
};//namespace controller::mppi_controller::models;
