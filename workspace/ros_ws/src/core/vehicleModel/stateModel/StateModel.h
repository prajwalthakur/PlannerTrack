#pragma once
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"

////////////////////////////////////////////////////////////////////////////////

class StateModel
{
    public:
        // Constructor
        StateModel(YAML::Node& simConfig, YAML::Node& vehConfig, const UniqueId& id)
        :mSimConfig{simConfig},mVehConfig{vehConfig},mId{id}{};
        // Destructor
        virtual ~StateModel()=default;  
        // Update the control command
        virtual void updateCommandedControl( const InputVector& u ) = 0 ; 
        // Step forward through simulation
        virtual void step()=0;
        // Get the vehicle's state.
        virtual const StateVector& getState() const = 0;
        // Get the pose of th vehicle.
        virtual stPose getStatePose() const = 0;
        // Set the state of the vehicle. (Usefull for setting the states through feedback, KF, etc)
        virtual void setState(const StateVector &) =0;
        // Set the integrator for the state model.
        virtual void createIntegrator()=0;
    protected:
        ptSharedPtr<IntegratorClass> mIntegrator{nullptr};
        YAML::Node mSimConfig;
        YAML::Node mVehConfig;
        InputVector mCommandedControl;
        UniqueId mId;   
};