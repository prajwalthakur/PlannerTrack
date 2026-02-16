#include "Integrator.h"

////////////////////////////////////////////////////////////////////////////////

IntegratorClass::IntegratorClass(std::function<StateVector(const StateVector&, const InputVector&)> dynamics , 
        std::function<const StateVector&()> getState,
        std::function<void(const StateVector&)> setState,
        std::function<void(const InputVector&)> setInput,double integrationTimeStep) :
        mDynamicsFunc(dynamics),
        mSetStateFunc(setState), 
        mSetInputFunc(setInput), 
        mGetStateFunc(getState), 
        mIntegrationStepSize(integrationTimeStep)
    {}

////////////////////////////////////////////////////////////////////////////////

IntegratorClass::IntegratorClass(
    std::function<StateVector(const StateVector&, const InputVector&)> dynamics , 
    std::function<const StateVector&()> getState,
    std::function<void(const StateVector&)> setState,
    std::function<void(const InputVector&)> setInput, 
    double integrationTimeStep, 
    double simTimeStep
    ) : 
    mDynamicsFunc(dynamics) , 
    mSetStateFunc(setState) , 
    mSetInputFunc(setInput), 
    mGetStateFunc(getState), 
    mIntegrationStepSize(integrationTimeStep), 
    mSimStepSize(simTimeStep)
    {}

////////////////////////////////////////////////////////////////////////////////

StateVector IntegratorClass::rk4Integrator(const StateVector& x, const InputVector& u,double ts) const
{

    StateVector k1 = mDynamicsFunc(x, u);
    StateVector k2 = mDynamicsFunc(x + (ts/2.)*k1,u);
    StateVector k3 = mDynamicsFunc(x + (ts/2.)*k2,u);
    StateVector k4 = mDynamicsFunc(x + (ts/2.)*k3,u);
    StateVector x_next = x + ts*(k1/6.+k2/3.+k3/3.+k4/6.);
    std::cerr << " x_next " << x_next << std::endl;
    return x_next;
}

////////////////////////////////////////////////////////////////////////////////

StateVector IntegratorClass::efIntegrator(const StateVector& x, const InputVector& u,double ts) const {
    StateVector k1 = mDynamicsFunc(x, u);
    StateVector x_next = x + ts*(k1);
    return x_next;
}

////////////////////////////////////////////////////////////////////////////////

void IntegratorClass::simNextState( const InputVector& u,double ts)const
{

    StateVector x_next = this->mGetStateFunc();
    const int integration_steps = (int)(ts/this->mIntegrationStepSize);
    for(int i=0;i<integration_steps;i++)
    {
        
        x_next=this->rk4Integrator(x_next,u,mIntegrationStepSize);
        
    }
    //RCLCPP_INFO(this->get_logger(), "rk4 called");
    this->mSetStateFunc(x_next);
    this->mSetInputFunc(u);
}

////////////////////////////////////////////////////////////////////////////////

void IntegratorClass::simNextState(const InputVector& u ) const
{
    StateVector x_next = this->mGetStateFunc();
    const int integration_steps = (int)(mSimStepSize/this->mIntegrationStepSize);
    for(int i=0;i<integration_steps;i++)
    {
        x_next=this->rk4Integrator(x_next,u,mIntegrationStepSize);
    }
    //RCLCPP_INFO(this->get_logger(), "rk4 called");
    std::cerr << " rk4 integrator called "  << " "; 
    this->mSetStateFunc(x_next);
    std::cerr << " lol "  << " "; 
    this->mSetInputFunc(u);
    
}

////////////////////////////////////////////////////////////////////////////////
