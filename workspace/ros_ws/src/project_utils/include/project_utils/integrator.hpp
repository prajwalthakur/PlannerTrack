// Author Prajwal Thakur 
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include "project_utils/types.hpp"
class IntegratorClass
{
 
    public:

        explicit IntegratorClass(std::function<mt::StateVector(const mt::StateVector&, const mt::InputVector&)> dynamics , 
                std::function<const mt::StateVector&()> getState,
                std::function<void(const mt::StateVector&)> setState,
                std::function<void(const mt::InputVector&)> setInput,double integrationTimeStep);
        
        explicit IntegratorClass(std::function<mt::StateVector(const mt::StateVector&, const mt::InputVector&)> dynamics , 
            std::function<const mt::StateVector&()> getState,
            std::function<void(const mt::StateVector&)> setState,
            std::function<void(const mt::InputVector&)> setInput,double integrationTimeStep, double simTimeStep);
        
        void simNextState(const mt::InputVector& ,double )const;
        void simNextState(const mt::InputVector&) const;
        mt::StateVector rk4Integrator(const mt::StateVector& , const mt::InputVector& ,double )const;
        mt::StateVector efIntegrator(const mt::StateVector& , const mt::InputVector& ,double )const;

    private:   
        std::function<mt::StateVector(const mt::StateVector&, const mt::InputVector&)> mDynamicsFunc;  
        std::function<void(const mt::StateVector&)> mSetStateFunc;
        std::function<void(const mt::InputVector&)> mSetInputFunc;
        std::function<const mt::StateVector&()> mGetStateFunc;
        const double mIntegrationStepSize;   
        const double mSimStepSize{-1.0};
};
