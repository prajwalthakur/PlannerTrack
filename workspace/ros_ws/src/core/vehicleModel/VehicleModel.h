#pragma once
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
#include "geometricModel/GeometricModelCollection.h"
#include  "stateModel/StateModelCollection.h"
class VehicleModel
{
    public:
        // Constructor
        VehicleModel(YAML::Node& simConfig, YAML::Node& vehConfig);
        // Destructor
        virtual ~VehicleModel()=default;
        // Get the id
        const UniqueId& id() const;
        // Update the control command        
        void updateCommandedControl( const InputVector& u );
        // Virtual functions
        virtual void step() = 0;
        virtual StateVector getState() const = 0;
        virtual void setState(const StateVector &) =0;
        virtual bool checkNumStatesInputs(const StateVector& stateVec, const InputVector& inputVec) =0;
        virtual stPose getStatePose() const =0;
        virtual std::pair<int,int> getNumStatesInputs()=0;
    protected:
        YAML::Node mSimConfig;
        YAML::Node mVehConfig;
        UniqueId mId;
        int mNumStates{0};
        int mNumControlInputs{0};
        ptSharedPtr<GeometricModel> mGeomModel{nullptr}; // Geometric Model, contains geometric shape and footprint
        ptSharedPtr<StateModel> mStateModel{nullptr}; // state space model of vehicle

};// VehicleModel