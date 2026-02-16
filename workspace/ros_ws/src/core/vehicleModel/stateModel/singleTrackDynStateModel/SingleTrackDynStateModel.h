
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "vehicleModel/stateModel/StateModel.h"

//////////////////////////////////////////////////////////////////////////

struct StateStruct{
    double x;
    double y;
    double z{0.0};
    double yaw;
    double vx;
    double sf;
};

//////////////////////////////////////////////////////////////////////////

struct InputStruct
{
        double acc;
        double sv;   
};

//////////////////////////////////////////////////////////////////////////

class SingleTrackDynStateModel : public StateModel, public std::enable_shared_from_this<SingleTrackDynStateModel>
{
    using BaseType = StateModel;
    public:
        // Constructor.
        explicit SingleTrackDynStateModel(YAML::Node& simConfig, YAML::Node& vehConfig, const UniqueId& id );
        // Destructor.
        ~SingleTrackDynStateModel() = default;
        // Step the simulation (updates the state space of the car)
        void step() override;
        // Update commanded control.
        void updateCommandedControl(const InputVector& u ) override;
        // Get the current state of the vehicle.
        const StateVector& getState() const override;
        // Set the state of the vehicle.
        void setState(const StateVector &) override;
        // Get the input.
        const StateVector& getInput() const;
        // Set the input.
        void setInput(const InputVector &);
        // Get the pose of the vehicle
        stPose getStatePose() const override;
        // Vehicle Dynamics
        StateVector xdot(const StateVector & , const InputVector &) const;
        void createIntegrator() override;
    private:
        void reset();
        void updateCommandedControl();
        void createIntegrator(YAML::Node& simConfig, YAML::Node& vehConfig);
        StateVector StateToVector(const StateStruct & ) const;
        StateStruct VectorToState(const StateVector &) const;
        InputVector InputToVector(const InputStruct &) const;
        InputStruct VectorToInput(const InputVector &) const;
    private:
        int NX;
        int NU;
        float T_fwd;
        double mInitXPose;
        double mInitYPose;
        double mInitYaw;
        double mInitVx;
        double mInitSf;
        double mInitSv;
        double mInitAcc;
        double mVehWheelBase;
        InputStruct mInputStruct;
        StateStruct mStateStruct;
        InputVector mInputVector;
        StateVector mStateVector;
};

//////////////////////////////////////////////////////////////////////////


