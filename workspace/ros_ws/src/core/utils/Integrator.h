#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include "core/CoreCollection.h"

class IntegratorClass
{
    public:

        explicit IntegratorClass(std::function<StateVector(const StateVector&, const InputVector&)> dynamics , 
                std::function<const StateVector&()> getState,
                std::function<void(const StateVector&)> setState,
                std::function<void(const InputVector&)> setInput,double integrationTimeStep);
        
        explicit IntegratorClass(std::function<StateVector(const StateVector&, const InputVector&)> dynamics , 
            std::function<const StateVector&()> getState,
            std::function<void(const StateVector&)> setState,
            std::function<void(const InputVector&)> setInput,double integrationTimeStep, double simTimeStep);
        
        void simNextState(const InputVector& ,double )const;
        void simNextState(const InputVector&) const;
        StateVector rk4Integrator(const StateVector& , const InputVector& ,double )const;
        StateVector efIntegrator(const StateVector& , const InputVector& ,double )const;

    private:   
        std::function<StateVector(const StateVector&, const InputVector&)> mDynamicsFunc;  
        std::function<void(const StateVector&)> mSetStateFunc;
        std::function<void(const InputVector&)> mSetInputFunc;
        std::function<const StateVector&()> mGetStateFunc;
        const double mIntegrationStepSize;   
        const double mSimStepSize{-1.0};
};
