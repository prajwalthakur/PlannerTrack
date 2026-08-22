#pragma once

#include "mppi_controller/models/model_factory.hpp"

/**
 * \file
 * \brief Downcast helper from the abstract \ref models::Model "Model" to
 * the concrete \ref models::AckermannModel "AckermannModel".
 *
 * \warning Not currently included anywhere, and not marked `inline` --
 * including it from more than one translation unit would violate the ODR
 * (multiple definitions of \c getConcreteModel at link time).
 */
namespace  controller::mppi_controller
{

    /// \brief Statically downcast \p model to \ref models::AckermannModel "AckermannModel" (unchecked -- caller must know the model is actually Ackermann).
    models::AckermannModel& getConcreteModel(models::Model& model)
    {
        return static_cast<models::AckermannModel&>(model);
    }


}//namespace  controller::mppi_controller