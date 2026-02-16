#pragma once
#include "vehicleModel/VehicleModel.h"
#include <yaml-cpp/yaml.h>
class SingleTrackDynModel : public VehicleModel
{
    using BaseType = VehicleModel;
    public:
        // Constructor
        SingleTrackDynModel()=default;
        SingleTrackDynModel(YAML::Node& simConfig, YAML::Node& vehConfig, long int value=-1);
        // Destructor
        ~SingleTrackDynModel()=default;
        // Step function
        void step() override;
        StateVector getState() const override;
        void setState(const StateVector &) override;
        stPose getStatePose() const override;
        bool checkNumStatesInputs(const StateVector& stateVec, const InputVector& inputVec) override;
        std::pair<int,int> getNumStatesInputs() override;
};